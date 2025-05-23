#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "x86.h"
#include "traps.h"
#include "spinlock.h"

// This file handles traps, interrupts, and exceptions that occur during kernel or user execution.
//
// Trap/Interrupt Handling Overview:
// 1. IDT Setup (`tvinit`, `idtinit`):
//    - The Interrupt Descriptor Table (IDT) is a CPU structure that maps each interrupt
//      vector (0-255) to a gate descriptor.
//    - `tvinit()` populates the `idt` array with gate descriptors. Each gate points to an
//      assembly language entry point in `vectors.S` (via the `vectors[]` array).
//    - For system calls (`T_SYSCALL`), a trap gate is set up that allows user-mode
//      code to trigger it using the `INT` instruction. Other gates are interrupt gates.
//    - `idtinit()` loads the IDT register (IDTR) with the address and size of the `idt` table
//      using the `lidt` instruction. This makes the IDT active.
//
// 2. Trap/Interrupt Occurs:
//    - When an interrupt (e.g., timer, disk) or an exception (e.g., page fault, divide by zero)
//      occurs, or when an `INT` instruction is executed:
//      a. The CPU pushes information onto the current stack (kernel stack if already in kernel,
//         or switches to kernel stack if in user mode). This includes EFLAGS, CS, EIP, and
//         sometimes an error code.
//      b. The CPU disables further interrupts (on the current CPU).
//      c. It uses the interrupt vector number to look up the corresponding gate in the IDT.
//      d. It jumps to the handler address specified in the IDT gate.
//
// 3. Assembly Handler (`vectors.S` - `alltraps`):
//    - The IDT entries point to individual assembly stubs in `vectors.S` (part of `vectors[]`).
//    - These stubs typically push a trap number and an error code (if not already pushed by CPU)
//      onto the stack, then jump to a common assembly routine (`alltraps`).
//    - `alltraps` saves all general-purpose registers, sets up segment registers for kernel mode,
//      and creates a `struct trapframe` on the stack. It then calls the C function `trap()`.
//
// 4. C Handler (`trap()` in this file):
//    - `trap()` receives a pointer to the `struct trapframe`.
//    - It uses `tf->trapno` to determine the cause of the trap.
//    - It dispatches to specific handlers or takes actions based on the trap number:
//      - System calls: Calls `syscall()` (see syscall.c).
//      - Hardware interrupts (timer, IDE, keyboard, UART): Calls respective interrupt
//        handlers (e.g., `ideintr()`, `kbdintr()`). After the device-specific handler,
//        `lapiceoi()` is called to signal End-Of-Interrupt to the local APIC.
//      - Page Faults: In xv6, a page fault from user space (if not a copy-on-write scenario,
//        which isn't fully implemented in base xv6) usually indicates a bug in the user
//        program or an attempt to access invalid memory. The default handler in `trap()`
//        will typically kill such a process. A page fault in kernel space is a kernel bug
//        and causes a panic.
//      - Other exceptions/spurious interrupts: Logged, and potentially the process is killed
//        or the system panics.
//    - `trap()` also handles forcing process exit if `myproc()->killed` is set and ensures
//      processes yield the CPU on timer interrupts for preemptive multitasking.
//
// 5. Return from Trap:
//    - After `trap()` returns, control goes back to `alltraps` in `vectors.S`.
//    - `alltraps` restores saved registers from the trap frame and executes `iret`,
//      which returns control to the point where the interrupt/exception occurred,
//      restoring EFLAGS, CS, and EIP, and returning to user mode if the trap
//      originated there.

// Interrupt Descriptor Table (IDT), shared by all CPUs.
// It contains 256 entries, each defining how a specific interrupt vector is handled.
struct gatedesc idt[256];
extern uint vectors[];  // Array of 256 entry pointers, defined in vectors.S. Each points to an assembly handler.
struct spinlock tickslock; // Lock to protect the global `ticks` counter.
uint ticks;                // Global counter incremented by timer interrupts, used for scheduling and sleep.

// Initialize the Interrupt Descriptor Table (IDT).
// This function populates the `idt` array with gate descriptors for all 256 possible interrupt vectors.
// - For most vectors, it sets up an interrupt gate pointing to the corresponding assembly
//   routine in `vectors[]` (from vectors.S), running in kernel mode (SEG_KCODE<<3).
//   The privilege level (DPL) is 0, meaning only kernel mode can trigger these via INT instruction (but they are usually hardware/exception triggered).
// - For the system call vector (`T_SYSCALL`), it sets up a trap gate with DPL_USER (privilege level 3),
//   allowing user-mode code to trigger this interrupt using the `INT T_SYSCALL` instruction.
// It also initializes `tickslock`.
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

// Load the IDT into the CPU's IDTR register.
// This function is called once per CPU during startup (in main.c for BSP, mpmain.c for APs)
// to make the initialized IDT active for the current CPU.
void
idtinit(void)
{
  lidt(idt, sizeof(idt)); // Load IDT register with base address and size of idt table.
}

//PAGEBREAK: 41
// Common trap handler function called from assembly code in `alltraps` (vectors.S).
// `tf` is a pointer to the `struct trapframe` that was built on the stack by `alltraps`.
// This function determines the type of trap/interrupt and dispatches accordingly.
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
