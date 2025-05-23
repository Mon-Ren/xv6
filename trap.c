#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "x86.h"
#include "traps.h"
#include "spinlock.h"

// 本文件处理在内核或用户执行期间发生的陷阱、中断和异常。
//
// 陷阱/中断处理概述：
// 1. IDT 设置 (`tvinit`, `idtinit`):
//    - 中断描述符表 (IDT) 是一个 CPU 结构，它将每个中断向量 (0-255) 映射到一个门描述符。
//    - `tvinit()` 用门描述符填充 `idt` 数组。每个门指向 `vectors.S` 中的一个汇编语言入口点（通过 `vectors[]` 数组）。
//    - 对于系统调用 (`T_SYSCALL`)，设置一个陷阱门，允许用户模式代码使用 `INT` 指令触发它。其他门是中断门。
//    - `idtinit()` 使用 `lidt` 指令将 `idt` 表的地址和大小加载到 IDT 寄存器 (IDTR) 中。这使得 IDT 生效。
//
// 2. 陷阱/中断发生：
//    - 当发生中断（例如，定时器、磁盘）或异常（例如，页错误、除以零），或执行 `INT` 指令时：
//      a. CPU 将信息压入当前栈（如果在内核中则为内核栈，如果在用户模式则切换到内核栈）。这包括 EFLAGS、CS、EIP，有时还有一个错误代码。
//      b. CPU 禁用进一步的中断（在当前 CPU 上）。
//      c. 它使用中断向量号在 IDT 中查找相应的门。
//      d. 它跳转到 IDT 门中指定的处理程序地址。
//
// 3. 汇编处理程序 (`vectors.S` - `alltraps`):
//    - IDT 条目指向 `vectors.S` 中的单个汇编存根（`vectors[]` 的一部分）。
//    - 这些存根通常将陷阱号和错误代码（如果 CPU 尚未压入）压入栈，然后跳转到一个通用的汇编例程 (`alltraps`)。
//    - `alltraps` 保存所有通用寄存器，为内核模式设置段寄存器，并在栈上创建一个 `struct trapframe`。然后它调用 C 函数 `trap()`。
//
// 4. C 处理程序 (`trap()` 在此文件中):
//    - `trap()` 接收一个指向 `struct trapframe` 的指针。
//    - 它使用 `tf->trapno` 来确定陷阱的原因。
//    - 它根据陷阱号分派到特定的处理程序或采取行动：
//      - 系统调用：调用 `syscall()` (参见 syscall.c)。
//      - 硬件中断（定时器、IDE、键盘、UART）：调用各自的中断处理程序（例如 `ideintr()`, `kbdintr()`）。在特定于设备的处理程序之后，调用 `lapiceoi()` 以向本地 APIC 发送中断结束信号。
//      - 页错误：在 xv6 中，来自用户空间的页错误（如果不是写时复制情景，这在基本 xv6 中未完全实现）通常表示用户程序中的错误或尝试访问无效内存。`trap()` 中的默认处理程序通常会终止此类进程。内核空间中的页错误是内核错误，会导致 panic。
//      - 其他异常/伪中断：被记录，并可能终止进程或导致系统 panic。
//    - 如果设置了 `myproc()->killed`，`trap()` 还处理强制进程退出，并确保进程在定时器中断时让出 CPU 以实现抢占式多任务处理。
//
// 5. 从陷阱返回：
//    - `trap()` 返回后，控制权交还给 `vectors.S` 中的 `alltraps`。
//    - `alltraps` 从陷阱帧中恢复保存的寄存器并执行 `iret`，这将控制权返回到中断/异常发生点，恢复 EFLAGS、CS 和 EIP，如果陷阱源于用户模式，则返回用户模式。

// 中断描述符表 (IDT)，由所有 CPU 共享。
// 它包含256个条目，每个条目定义了如何处理特定的中断向量。
struct gatedesc idt[256];
extern uint vectors[];  // 256个入口指针数组，在 vectors.S 中定义。每个指针指向一个汇编处理程序。
struct spinlock tickslock; // Lock to protect the global `ticks` counter.
uint ticks;                // Global counter incremented by timer interrupts, used for scheduling and sleep.

// 初始化中断描述符表 (IDT)。
// 此函数为所有256个可能的中断向量填充 `idt` 数组的门描述符。
// - 对于大多数向量，它设置一个中断门，指向 `vectors[]` (来自 vectors.S) 中相应的汇编例程，在内核模式 (SEG_KCODE<<3)下运行。
//   特权级别 (DPL) 为0，意味着只有内核模式可以通过INT指令触发这些中断（但它们通常由硬件/异常触发）。
// - 对于系统调用向量 (`T_SYSCALL`)，它设置一个陷阱门，DPL_USER（特权级别3），
//   允许用户模式代码使用 `INT T_SYSCALL` 指令触发此中断。
// 它还初始化 `tickslock`。
void
tvinit(void)
{
  int i;

  for(i = 0; i < 256; i++)
    SETGATE(idt[i], 0, SEG_KCODE<<3, vectors[i], 0); // Default: kernel privilege interrupt gate.
  // Special gate for system calls: DPL_USER allows user mode to trigger INT T_SYSCALL.
  SETGATE(idt[T_SYSCALL], 1, SEG_KCODE<<3, vectors[T_SYSCALL], DPL_USER);

  initlock(&tickslock, "time"); // Initialize the lock for the global ticks counter.
}

// 将 IDT 加载到 CPU 的 IDTR 寄存器中。
// 此函数在启动期间由每个 CPU 调用一次（BSP 在 main.c 中调用，AP 在 mpmain.c 中调用），
// 以使初始化的 IDT 对当前 CPU 生效。
void
idtinit(void)
{
  lidt(idt, sizeof(idt)); // Load IDT register with base address and size of idt table.
}

//PAGEBREAK: 41
// 从 `alltraps` (vectors.S) 中的汇编代码调用的通用陷阱处理函数。
// `tf` 是指向由 `alltraps` 在栈上构建的 `struct trapframe` 的指针。
// 此函数确定陷阱/中断的类型并相应地进行分派。
void
trap(struct trapframe *tf)
{
  if(tf->trapno == T_SYSCALL){
    if(myproc()->killed)
      exit();
    myproc()->tf = tf;
    syscall();
    if(myproc()->killed)
      exit();
    return;
  }

  switch(tf->trapno){
  case T_IRQ0 + IRQ_TIMER: // Timer interrupt from local APIC.
    if(cpuid() == 0){      // Only CPU 0 handles ticks for simplicity in xv6.
      acquire(&tickslock);
      ticks++;             // Increment global tick counter.
      wakeup(&ticks);      // Wake up any processes sleeping on the `ticks` channel (e.g., in sys_sleep).
      release(&tickslock);
    }
    lapiceoi(); // Signal End-Of-Interrupt to the local APIC.
    break;
  case T_IRQ0 + IRQ_IDE: // IDE disk controller interrupt.
    ideintr();           // Call IDE interrupt handler (ide.c).
    lapiceoi();          // Signal EOI to local APIC.
    break;
  case T_IRQ0 + IRQ_IDE+1: // Secondary IDE channel interrupt.
    // Bochs generates spurious IDE1 interrupts. xv6 doesn't use this channel.
    break;
  case T_IRQ0 + IRQ_KBD: // Keyboard controller interrupt.
    kbdintr();           // Call keyboard interrupt handler (kbd.c).
    lapiceoi();          // Signal EOI to local APIC.
    break;
  case T_IRQ0 + IRQ_COM1: // UART (serial port) interrupt.
    uartintr();          // Call UART interrupt handler (uart.c).
    lapiceoi();          // Signal EOI to local APIC.
    break;
  case T_IRQ0 + 7:        // Spurious interrupt from old PIC? (IRQ 7 can be spurious).
  case T_IRQ0 + IRQ_SPURIOUS: // Spurious interrupt vector from APIC.
    cprintf("cpu%d: spurious interrupt at %x:%x\n",
            cpuid(), tf->cs, tf->eip);
    lapiceoi(); // Signal EOI to local APIC.
    break;

  //PAGEBREAK: 13
  default: // All other traps (exceptions, unhandled interrupts).
    // If `myproc()` is null or the trap occurred in kernel mode ((tf->cs&3) == 0),
    // it's a kernel problem (e.g., kernel page fault, unexpected exception).
    if(myproc() == 0 || (tf->cs&3) == 0){
      // In kernel, it must be our mistake.
      cprintf("unexpected trap %d from cpu %d eip %x (cr2=0x%x)\n",
              tf->trapno, cpuid(), tf->eip, rcr2()); // rcr2() holds the faulting address for page faults.
      panic("trap"); // Kernel panics.
    }
    // If the trap occurred in user space, assume the user process misbehaved.
    // This includes page faults (e.g., accessing invalid memory, protection violation),
    // general protection faults, divide by zero, etc.
    cprintf("pid %d %s: trap %d err %d on cpu %d "
            "eip 0x%x addr 0x%x--kill proc\n",
            myproc()->pid, myproc()->name, tf->trapno,
            tf->err, cpuid(), tf->eip, rcr2());
    myproc()->killed = 1; // Mark the process to be killed.
  }

  // Force process exit if it has been killed and is in user space.
  // (If it is still executing in the kernel, for example, during a system call,
  // let it keep running until it attempts to return to user space, where `syscall()`
  // or this check here will handle the exit).
  if(myproc() && myproc()->killed && (tf->cs&3) == DPL_USER)
    exit();

  // Force process to give up CPU on clock tick if it was running in user space.
  // This implements preemptive multitasking.
  // If interrupts were potentially enabled while locks were held by the kernel,
  // a more complex check involving `nlock` might be needed here.
  if(myproc() && myproc()->state == RUNNING &&
     tf->trapno == T_IRQ0+IRQ_TIMER)
    yield(); // Process yields the CPU.

  // Check if the process has been killed since we yielded (e.g., by another CPU via kill syscall)
  // or if it was marked killed by the default trap handler above and is now about to return to user space.
  if(myproc() && myproc()->killed && (tf->cs&3) == DPL_USER)
    exit();
}
  // (If it is still executing in the kernel, let it keep running
  // until it gets to the regular system call return.)
  if(myproc() && myproc()->killed && (tf->cs&3) == DPL_USER)
    exit();

  // Force process to give up CPU on clock tick.
  // If interrupts were on while locks held, would need to check nlock.
  if(myproc() && myproc()->state == RUNNING &&
     tf->trapno == T_IRQ0+IRQ_TIMER)
    yield();

  // Check if the process has been killed since we yielded
  if(myproc() && myproc()->killed && (tf->cs&3) == DPL_USER)
    exit();
}
