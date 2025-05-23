// This file implements functions for interacting with the I/O APIC (Advanced
// Programmable Interrupt Controller). The I/O APIC is responsible for routing
// external hardware interrupts from devices (e.g., disk, keyboard, network card)
// to specific CPUs in a multiprocessor (SMP) system. In a uniprocessor system
// with an APIC, it still manages external interrupts, replacing the functionality
// of the older 8259A PICs.
//
// Key Concepts:
// - Memory-Mapped I/O (MMIO): The I/O APIC is accessed via memory-mapped
//   registers. `IOAPIC` defines its default physical address.
// - Redirection Table (RTE): The core of the I/O APIC. It's an array of
//   entries (typically 24 or more), where each entry corresponds to an
//   external interrupt line (IRQ). Each RTE is 64 bits wide and configured
//   using two 32-bit registers (low and high parts).
//   An RTE specifies:
//     - Interrupt Vector: The vector number (0-255) that will be delivered to the CPU.
//     - Delivery Mode: How the interrupt is delivered (e.g., Fixed, Lowest Priority).
//     - Destination Mode: Physical (target APIC ID) or Logical (target set of CPUs).
//     - Trigger Mode: Edge-triggered or Level-triggered.
//     - Polarity: Active high or Active low.
//     - Mask Bit: Whether the interrupt is masked (disabled) or unmasked (enabled).
//     - Destination Field: Specifies the target CPU(s) or APIC ID(s).
// - Registers:
//   - `REG_ID`: I/O APIC ID register.
//   - `REG_VER`: I/O APIC Version register (also indicates max redirection entries).
//   - `REG_TABLE`: Base address of the redirection table entries.
//
// See Intel's I/O APIC datasheets (e.g., 29056601.pdf) for detailed specifications.
// This file also effectively replaces `picirq.c` in APIC-based systems.

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

// Pointer to the memory-mapped I/O APIC registers.
// This is volatile because its contents can change asynchronously.
// Initialized in `ioapicinit`.
volatile struct ioapic *ioapic;

// Structure for accessing I/O APIC registers via MMIO.
// To write to a register: write index to `reg`, then value to `data`.
// To read from a register: write index to `reg`, then read value from `data`.
struct ioapic {
  uint reg;    // Address register: select which internal register to access.
  uint pad[3]; // Padding to align `data` to a 16-byte boundary from `reg`.
  uint data;   // Data register: read/write data from/to the selected internal register.
};

// Read the value of an I/O APIC register `reg`.
static uint
ioapicread(int reg)
{
  ioapic->reg = reg;    // Select the register.
  return ioapic->data; // Read its data.
}

// Write `data` to an I/O APIC register `reg`.
static void
ioapicwrite(int reg, uint data)
{
  ioapic->reg = reg;   // Select the register.
  ioapic->data = data; // Write the data.
}

// Initialize the I/O APIC.
// This function is called once during kernel startup (in main.c if MP, or after LAPIC init).
// Steps:
// 1. Map the I/O APIC's physical address to `ioapic` virtual pointer.
// 2. Read the version register to determine the maximum number of interrupt
//    redirection entries (`maxintr`).
// 3. Read the I/O APIC ID (for verification, though not strictly used by xv6 later).
// 4. Initialize all redirection table entries (RTEs):
//    - Mark them as disabled (`INT_DISABLED`).
//    - Set the interrupt vector to `T_IRQ0 + i` (where `i` is the IRQ number).
//      This ensures each IRQ maps to a unique vector handled in `trap.c`.
//    - Set them as edge-triggered, active high (common defaults for ISA bus).
//    - Route them to no CPUs initially (destination field in high part of RTE set to 0).
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

// Enable a specific hardware interrupt `irq` and route it to `cpunum`.
// `cpunum` in xv6 typically corresponds to the LAPIC ID of the target CPU.
//
// Configuration for the IRQ's Redirection Table Entry (RTE):
// - Vector: `T_IRQ0 + irq`. This maps the hardware IRQ to a specific
//   interrupt vector handled in `trap.c`.
// - Delivery Mode: Implied as Fixed (most common).
// - Mask Bit: Cleared (interrupt enabled).
// - Trigger Mode: Edge-triggered (default).
// - Polarity: Active high (default).
// - Destination Mode: Physical (targeting a specific APIC ID).
// - Destination Field: `cpunum << 24` (the APIC ID is placed in the upper byte
//   of the high part of the RTE).
//
// Note: There isn't an explicit `ioapicdisable` in xv6's provided code. Disabling
// would involve setting the `INT_DISABLED` bit in the low part of the RTE for the IRQ.
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
