// 本文件实现了与本地APIC（高级可编程中断控制器）交互的功能。
// 多处理器系统中的每个CPU都有其自己的LAPIC。
//
// 本地APIC (LAPIC) 的作用:
// - 接收和管理中断：LAPIC负责接收其CPU本地的中断。这包括：
//   - 定时器中断：由LAPIC自身的可编程定时器产生，用于抢占式多任务处理（调度）。
//   - 处理器间中断 (IPI)：由一个CPU发送到另一个CPU，用于诸如启动其他CPU、
//     TLB击落（在xv6中未完全实现）或信号传递等任务。
//   - 伪中断：意外中断，LAPIC可以配置为优雅地处理它们。
//   - 错误中断：如果LAPIC检测到内部错误，则由其自身产生。
// - 中断优先级：LAPIC可以对中断进行优先级排序。任务优先级寄存器 (TPR) 可用于屏蔽较低优先级的中断。
// - 中断分发（与I/O APIC协同）：对于由I/O APIC路由的外部硬件中断，LAPIC是目标CPU上的最终目的地。
// - 中断结束 (EOI)：中断处理程序必须向LAPIC发送EOI信号，以指示中断已处理完毕，
//   从而允许LAPIC传递新的中断。
//
// 关键寄存器（通过内存映射I/O访问，`lapic` volatile指针）：
// - ID: LAPIC ID寄存器（标识LAPIC/CPU）。
// - VER: 版本寄存器。
// - TPR: 任务优先级寄存器。
// - EOI: 中断结束寄存器。
// - SVR: 伪中断向量寄存器。
// - ESR: 错误状态寄存器。
// - ICRLO/ICRHI: 中断命令寄存器（用于发送IPI）。
// - TIMER, LINT0, LINT1, ERROR: 本地向量表 (LVT) 条目，用于配置定时器、本地引脚和错误中断。
// - TICR, TCCR, TDCR: 定时器初始计数、当前计数和分频配置寄存器。
//
// 有关详细信息，请参阅Intel系统编程指南第3A卷第10章（高级可编程中断控制器 (APIC)）。

#include "param.h"
#include "types.h"
#include "defs.h"
#include "date.h"
#include "memlayout.h"
#include "traps.h"
#include "mmu.h"
#include "x86.h"

// Local APIC registers, divided by 4 for use as uint[] indices.
#define ID      (0x0020/4)   // ID
#define VER     (0x0030/4)   // Version
#define TPR     (0x0080/4)   // Task Priority
#define EOI     (0x00B0/4)   // EOI
#define SVR     (0x00F0/4)   // Spurious Interrupt Vector
  #define ENABLE     0x00000100   // Unit Enable
#define ESR     (0x0280/4)   // Error Status
#define ICRLO   (0x0300/4)   // Interrupt Command
  #define INIT       0x00000500   // INIT/RESET
  #define STARTUP    0x00000600   // Startup IPI
  #define DELIVS     0x00001000   // Delivery status
  #define ASSERT     0x00004000   // Assert interrupt (vs deassert)
  #define DEASSERT   0x00000000
  #define LEVEL      0x00008000   // Level triggered
  #define BCAST      0x00080000   // Send to all APICs, including self.
  #define BUSY       0x00001000
  #define FIXED      0x00000000
#define ICRHI   (0x0310/4)   // Interrupt Command [63:32]
#define TIMER   (0x0320/4)   // Local Vector Table 0 (TIMER)
  #define X1         0x0000000B   // divide counts by 1
  #define PERIODIC   0x00020000   // Periodic
#define PCINT   (0x0340/4)   // Performance Counter LVT
#define LINT0   (0x0350/4)   // Local Vector Table 1 (LINT0)
#define LINT1   (0x0360/4)   // Local Vector Table 2 (LINT1)
#define ERROR   (0x0370/4)   // Local Vector Table 3 (ERROR)
  #define MASKED     0x00010000   // Interrupt masked
#define TICR    (0x0380/4)   // Timer Initial Count
#define TCCR    (0x0390/4)   // Timer Current Count
#define TDCR    (0x03E0/4)   // Timer Divide Configuration

// `lapic` 是一个指向本地APIC内存映射寄存器基地址的volatile指针。
// 它在 `mp.c` 中，在从ACPI/MP表确定LAPIC的物理地址后进行初始化。
volatile uint *lapic;  // Initialized in mp.c

//PAGEBREAK!
// 将值写入LAPIC寄存器的辅助函数。
// `index` 是寄存器偏移量（除以4以用作uint[]索引）。
// 写入后会执行 `lapic[ID]` 的读取操作，以确保写入在任何后续操作（提交写入）之前完成。
static void
lapicw(int index, int value)
{
  lapic[index] = value;
  lapic[ID];  // Wait for write to finish by reading from a LAPIC register.
}

// 为当前CPU初始化本地APIC。
// 此函数在每个CPU启动期间调用一次。
// 如果 `lapic` 为空（例如，在没有APIC的单处理器系统上，或者内存映射失败），则此函数不执行任何操作。
//
// 配置步骤：
// 1. 启用LAPIC单元：设置伪中断向量寄存器 (SVR) 以在软件上启用APIC单元，并指定伪中断的向量 (T_IRQ0 + IRQ_SPURIOUS)。
// 2. 配置定时器：
//    - 将定时器分频配置 (TDCR) 设置为X1（除以1，有效地使用总线时钟）。
//    - 配置LVT定时器条目 (TIMER) 为周期性的，并产生中断T_IRQ0 + IRQ_TIMER。
//    - 设置定时器的初始计数值 (TICR)。定时器从此值开始倒计时。当达到零时，产生中断并重新加载。
//      （值10000000是任意的，定义了定时器中断频率）。
// 3. 屏蔽本地中断引脚：通过在其LVT条目中设置MASKED位来禁用LINT0和LINT1（处理器上的本地中断引脚，通常连接到NMI或用于8259A PIC兼容性）。
// 4. 屏蔽性能计数器中断：如果支持（LAPIC版本 >= 4），则禁用性能计数器溢出中断。
// 5. 映射错误中断：配置LVT错误条目以在发生APIC错误时产生中断T_IRQ0 + IRQ_ERROR。
// 6. 清除错误状态：通过向错误状态寄存器 (ESR) 写入两次来清除它。
// 7. 确认挂起的中断：向EOI寄存器写入0以清除LAPIC中在初始化之前可能存在的任何挂起中断状态。
// 8. 同步仲裁ID：发送广播INIT Level De-Assert IPI。这有助于在具有多个LAPIC的系统中同步仲裁ID。
// 9. 启用中断接收：将任务优先级寄存器 (TPR) 设置为0，允许LAPIC传递任何中断（即无优先级屏蔽）。
//    这并*不*在CPU上全局启用中断（这是由 `sti` 完成的）。
void
lapicinit(void)
{
  if(!lapic) // Do nothing if LAPIC is not mapped.
    return;

  // Enable local APIC; set spurious interrupt vector.
  // The spurious vector is delivered if an interrupt arrives with no source or for other error conditions.
  lapicw(SVR, ENABLE | (T_IRQ0 + IRQ_SPURIOUS));

  // Configure the LAPIC timer:
  // The timer repeatedly counts down at bus frequency from lapic[TICR] and then issues an interrupt.
  // If xv6 cared more about precise timekeeping, TICR would be calibrated using an external time source.
  lapicw(TDCR, X1); // Set timer divide configuration (divide by 1).
  lapicw(TIMER, PERIODIC | (T_IRQ0 + IRQ_TIMER)); // Set timer to periodic mode, vector T_IRQ0 + IRQ_TIMER.
  lapicw(TICR, 10000000); // Set initial count for the timer (determines interrupt frequency).

  // Disable local interrupt lines (LINT0, LINT1) by masking them.
  // These are often connected to an 8259A PIC in legacy systems or can be NMI.
  lapicw(LINT0, MASKED);
  lapicw(LINT1, MASKED);

  // Disable performance counter overflow interrupts if the LAPIC supports them (version >= 4).
  if(((lapic[VER]>>16) & 0xFF) >= 4)
    lapicw(PCINT, MASKED);

  // Map the APIC error interrupt to vector T_IRQ0 + IRQ_ERROR.
  lapicw(ERROR, T_IRQ0 + IRQ_ERROR);

  // Clear the error status register (ESR). Requires two writes: first write allows
  // the register to be updated, second write clears it.
  lapicw(ESR, 0);
  lapicw(ESR, 0);

  // Acknowledge any outstanding interrupts that might have been pending in the LAPIC
  // before initialization. This clears the In-Service Register (ISR) for the highest
  // priority pending interrupt.
  lapicw(EOI, 0);

  // Send an Init Level De-Assert IPI to all APICs (including self) to synchronize arbitration IDs.
  // This is part of the MP initialization sequence.
  lapicw(ICRHI, 0); // Destination field is 0 for BCAST if physical destination mode.
  lapicw(ICRLO, BCAST | INIT | LEVEL); // Broadcast, INIT message, Level triggered.
  while(lapic[ICRLO] & DELIVS) // Wait for delivery status to clear (IPI sent).
    ;

  // Enable interrupts on the APIC by setting Task Priority Register (TPR) to 0.
  // A TPR of 0 means the APIC will attempt to deliver any interrupt regardless of its priority.
  // This does NOT enable interrupts on the processor itself (that's `sti`).
  lapicw(TPR, 0);
}

// 获取当前CPU的本地APIC ID。
// ID存储在LAPIC ID寄存器的高8位中。
// 如果LAPIC未映射（例如，单处理器），则返回0。
int
lapicid(void)
{
  if (!lapic)
    return 0;
  return lapic[ID] >> 24; // APIC ID is in bits 31:24 of the ID register.
}

// 通过向EOI（中断结束）寄存器写入来确认中断。
// 在处理由本地APIC传递的中断后，中断处理程序必须调用此函数。
// 它向LAPIC发出信号，表明现在可以传递下一个中断（如果存在挂起的中断并且其优先级高于或等于刚服务过的中断，具体取决于TPR设置）。
void
lapiceoi(void)
{
  if(lapic)
    lapicw(EOI, 0); // Write 0 to EOI register.
}

// 旋转指定的微秒数。
// 这是xv6中的一个占位符，并不提供实际校准的延迟。
// 在真实硬件上，这需要进行调整或使用校准过的定时器。
// 对于像Bochs或QEMU这样的模拟环境，CPU速度差异很大，
// 使得基于简单循环的延迟不可靠。
void
microdelay(int us)
{
  // This function is intentionally left blank in xv6.
  // A real implementation would involve a calibrated loop or using a high-resolution timer.
}

#define CMOS_PORT    0x70 // CMOS address port.
#define CMOS_RETURN  0x71 // CMOS data port.

// 启动应用处理器 (AP) 在 `addr` 地址执行代码。
// 此函数使用IPI实现“通用启动算法”，
// 如多处理器规范中所述。
// 它由引导处理器 (BSP) 调用以唤醒其他CPU。
// - `apicid`: 要启动的AP的LAPIC ID。
// - `addr`: AP应开始执行的物理地址（必须小于1MB且页对齐）。
//
// 步骤：
// 1. CMOS关机代码：将CMOS关机状态字节（偏移量0xF）设置为0x0A。
//    这告诉POST/BIOS正在进行热复位。
// 2. 热复位向量：将物理地址0x40:0x67 (0x467) 处的暖复位向量
//    指向AP的启动代码 (`addr`)。该向量在其段部分存储 `addr >> 4`，在其偏移部分存储0。
// 3. 发送INIT IPI：
//    - 配置中断命令寄存器 (ICR) 以向目标 `apicid` 发送INIT IPI（电平触发，置位）。
//    - 等待传递，然后通过发送另一个INIT IPI（电平触发，复位）来复位INIT信号。
//    - 按照规范包含延迟。
// 4. 发送STARTUP IPI (SIPI)：
//    - 配置ICR以向目标 `apicid` 发送STARTUP IPI，启动地址为 `addr`
//      （除以4096，因为SIPI使用 `addr` 的高位形成一个20位地址 `0VV000h`，其中VV是来自ICR的向量）。
//    - 根据Intel的算法，这会执行两次，尽管第二次可能会被某些硬件/仿真器忽略。
//    - 包含延迟。
// 收到SIPI后，AP应该唤醒，初始化自身，并在 `addr` 处开始执行（在xv6中指向 `entryother.S`）。
void
lapicstartap(uchar apicid, uint addr)
{
  int i;
  ushort *wrv;

  // "The BSP must initialize CMOS shutdown code to 0AH
  // and the warm reset vector (DWORD based at 40:67) to point at
  // the AP startup code prior to the [universal startup algorithm]."
  outb(CMOS_PORT, 0xF);  // offset 0xF is shutdown code
  outb(CMOS_PORT+1, 0x0A);
  wrv = (ushort*)P2V((0x40<<4 | 0x67));  // Warm reset vector
  wrv[0] = 0;
  wrv[1] = addr >> 4;

  // "Universal startup algorithm."
  // Send INIT (level-triggered) interrupt to reset other CPU.
  lapicw(ICRHI, apicid<<24);
  lapicw(ICRLO, INIT | LEVEL | ASSERT);
  microdelay(200);
  lapicw(ICRLO, INIT | LEVEL);
  microdelay(100);    // should be 10ms, but too slow in Bochs!

  // Send startup IPI (twice!) to enter code.
  // Regular hardware is supposed to only accept a STARTUP
  // when it is in the halted state due to an INIT.  So the second
  // should be ignored, but it is part of the official Intel algorithm.
  // Bochs complains about the second one.  Too bad for Bochs.
  for(i = 0; i < 2; i++){
    lapicw(ICRHI, apicid<<24);
    lapicw(ICRLO, STARTUP | (addr>>12));
    microdelay(200);
  }
}

#define CMOS_STATA   0x0a
#define CMOS_STATB   0x0b
#define CMOS_UIP    (1 << 7)        // RTC update in progress

#define SECS    0x00
#define MINS    0x02
#define HOURS   0x04
#define DAY     0x07
#define MONTH   0x08
#define YEAR    0x09

static uint
cmos_read(uint reg)
{
  outb(CMOS_PORT,  reg);
  microdelay(200);

  return inb(CMOS_RETURN);
}

static void
fill_rtcdate(struct rtcdate *r)
{
  r->second = cmos_read(SECS);
  r->minute = cmos_read(MINS);
  r->hour   = cmos_read(HOURS);
  r->day    = cmos_read(DAY);
  r->month  = cmos_read(MONTH);
  r->year   = cmos_read(YEAR);
}

// qemu seems to use 24-hour GWT and the values are BCD encoded
void
cmostime(struct rtcdate *r)
{
  struct rtcdate t1, t2;
  int sb, bcd;

  sb = cmos_read(CMOS_STATB);

  bcd = (sb & (1 << 2)) == 0;

  // make sure CMOS doesn't modify time while we read it
  for(;;) {
    fill_rtcdate(&t1);
    if(cmos_read(CMOS_STATA) & CMOS_UIP)
        continue;
    fill_rtcdate(&t2);
    if(memcmp(&t1, &t2, sizeof(t1)) == 0)
      break;
  }

  // convert
  if(bcd) {
#define    CONV(x)     (t1.x = ((t1.x >> 4) * 10) + (t1.x & 0xf))
    CONV(second);
    CONV(minute);
    CONV(hour  );
    CONV(day   );
    CONV(month );
    CONV(year  );
#undef     CONV
  }

  *r = t1;
  r->year += 2000;
}
