// 本文件实现了与I/O APIC（高级可编程中断控制器）交互的功能。
// I/O APIC负责将在多处理器（SMP）系统中来自设备（例如磁盘、键盘、网卡）的外部硬件中断路由到特定的CPU。
// 在具有APIC的单处理器系统中，它仍然管理外部中断，取代了旧式8259A PIC的功能。
//
// 关键概念:
// - 内存映射I/O (MMIO): I/O APIC通过内存映射寄存器进行访问。`IOAPIC` 定义了其默认物理地址。
// - 重定向表 (RTE): I/O APIC的核心。它是一个条目数组（通常为24个或更多），
//   其中每个条目对应一个外部中断线 (IRQ)。每个RTE为64位宽，使用两个32位寄存器（低位和高位部分）进行配置。
//   一个RTE指定:
//     - 中断向量: 将传递给CPU的向量号 (0-255)。
//     - 传递模式: 中断如何传递（例如，固定模式、最低优先级模式）。
//     - 目标模式: 物理模式（目标APIC ID）或逻辑模式（目标CPU集）。
//     - 触发模式: 边沿触发或电平触发。
//     - 极性: 高电平有效或低电平有效。
//     - 屏蔽位: 中断是被屏蔽（禁用）还是未屏蔽（启用）。
//     - 目标字段: 指定目标CPU或APIC ID。
// - 寄存器:
//   - `REG_ID`: I/O APIC ID寄存器。
//   - `REG_VER`: I/O APIC版本寄存器（也指示最大重定向条目数）。
//   - `REG_TABLE`: 重定向表条目的基地址。
//
// 有关详细规格，请参阅Intel的I/O APIC数据手册（例如，29056601.pdf）。
// 在基于APIC的系统中，此文件还有效地取代了 `picirq.c`。

#include "types.h"
#include "defs.h"
#include "traps.h"

#define IOAPIC  0xFEC00000   // Default physical address of I/O APIC (memory-mapped).

// I/O APIC Register Indices (for accessing via ioapic->reg and ioapic->data).
#define REG_ID     0x00  // Register index: ID Register (IOAPICID)
#define REG_VER    0x01  // Register index: Version Register (IOAPICVER)
#define REG_TABLE  0x10  // Register index: Redirection Table Entry base (IOREDTBL[0])

// Bits in the low 32 bits of a Redirection Table Entry (RTE).
#define INT_DISABLED   0x00010000  // Interrupt masked (disabled) if set.
#define INT_LEVEL      0x00008000  // Level-triggered if set (else edge-triggered).
#define INT_ACTIVELOW  0x00002000  // Active low if set (else active high).
#define INT_LOGICAL    0x00000800  // Destination Mode: Logical if set (else Physical).
                                   // Physical: Destination field is an APIC ID.
                                   // Logical: Destination field is a set of processors.

// 指向内存映射的I/O APIC寄存器的指针。
// 这是volatile类型，因为其内容可能异步更改。
// 在 `ioapicinit` 中初始化。
volatile struct ioapic *ioapic;

// 通过MMIO访问I/O APIC寄存器的结构。
// 要写入寄存器：先将索引写入 `reg`，然后将值写入 `data`。
// 要从寄存器读取：先将索引写入 `reg`，然后从 `data` 读取值。
struct ioapic {
  uint reg;    // 地址寄存器：选择要访问的内部寄存器。
  uint pad[3]; // 填充，以使 `data` 从 `reg` 开始16字节对齐。
  uint data;   // 数据寄存器：从选定的内部寄存器读/写数据。
};

// 读取I/O APIC寄存器 `reg` 的值。
static uint
ioapicread(int reg)
{
  ioapic->reg = reg;    // Select the register.
  return ioapic->data; // Read its data.
}

// 将 `data` 写入I/O APIC寄存器 `reg`。
static void
ioapicwrite(int reg, uint data)
{
  ioapic->reg = reg;   // Select the register.
  ioapic->data = data; // Write the data.
}

// 初始化I/O APIC。
// 此函数在内核启动期间调用一次（如果在MP环境中，则在main.c中调用；否则在LAPIC初始化之后调用）。
// 步骤：
// 1. 将I/O APIC的物理地址映射到 `ioapic` 虚拟指针。
// 2. 读取版本寄存器以确定中断重定向条目的最大数量 (`maxintr`)。
// 3. 读取I/O APIC ID（用于验证，尽管xv6后续不严格使用它）。
// 4. 初始化所有重定向表条目 (RTE)：
//    - 将它们标记为禁用 (`INT_DISABLED`)。
//    - 将中断向量设置为 `T_IRQ0 + i`（其中 `i` 是IRQ编号）。
//      这确保每个IRQ映射到 `trap.c` 中处理的唯一向量。
//    - 将它们设置为边沿触发、高电平有效（ISA总线的常见默认值）。
//    - 最初不将它们路由到任何CPU（RTE高位部分的目标字段设置为0）。
void
ioapicinit(void)
{
  int i, id, maxintr;

  ioapic = (volatile struct ioapic*)IOAPIC; // Map I/O APIC base address.
  maxintr = (ioapicread(REG_VER) >> 16) & 0xFF; // Get max IRQ input pins from version register.
  id = ioapicread(REG_ID) >> 24; // Get I/O APIC ID.
  if(id != ioapicid) // `ioapicid` is a global, usually set if MP config table specifies it.
    cprintf("ioapicinit: id %d isn't equal to ioapicid %d; not a MP\n", id, ioapicid);

  // Mark all interrupts initially as edge-triggered, active high, disabled,
  // and not routed to any CPUs.
  // Each RTE is 64 bits, accessed as two 32-bit registers.
  for(i = 0; i <= maxintr; i++){
    // Low part of RTE: vector, delivery mode, trigger mode, polarity, mask.
    ioapicwrite(REG_TABLE+2*i, INT_DISABLED | (T_IRQ0 + i));
    // High part of RTE: destination APIC ID (cleared to 0).
    ioapicwrite(REG_TABLE+2*i+1, 0);
  }
}

// 启用特定的硬件中断 `irq` 并将其路由到 `cpunum`。
// 在xv6中，`cpunum` 通常对应于目标CPU的LAPIC ID。
//
// IRQ的重定向表条目 (RTE) 配置：
// - 向量: `T_IRQ0 + irq`。这将硬件IRQ映射到 `trap.c` 中处理的特定中断向量。
// - 传递模式: 默认为固定模式 (最常见)。
// - 屏蔽位: 清除 (中断启用)。
// - 触发模式: 边沿触发 (默认)。
// - 极性: 高电平有效 (默认)。
// - 目标模式: 物理模式 (针对特定的APIC ID)。
// - 目标字段: `cpunum << 24` (APIC ID放置在RTE高位部分的高字节中)。
//
// 注意: xv6提供的代码中没有显式的 `ioapicdisable` 函数。禁用
// 将涉及在IRQ的RTE的低位部分设置 `INT_DISABLED` 位。
void
ioapicenable(int irq, int cpunum)
{
  // Mark interrupt edge-triggered, active high, enabled,
  // and routed to the given cpunum (which is its LAPIC ID).
  // Low part of RTE: vector, clear INT_DISABLED bit (implicitly enabling).
  ioapicwrite(REG_TABLE+2*irq, T_IRQ0 + irq);
  // High part of RTE: set destination APIC ID to cpunum.
  ioapicwrite(REG_TABLE+2*irq+1, cpunum << 24);
}
