# XV6 Boot Assembly Code (bootasm.S)
#
# This is the very first code that runs when an x86 CPU boots from the hard disk
# in an xv6 system. The BIOS loads the first 512 bytes (the boot sector, which
# contains this code and `bootmain.c` code) from the disk into physical memory
# at address 0x7c00. Execution starts here in 16-bit real mode.
#
# Its main responsibilities are:
# 1. Basic CPU Initialization: Disable interrupts, zero out segment registers.
# 2. Enable A20 Line: Ensure access to memory above 1MB.
# 3. Set up Global Descriptor Table (GDT): A minimal GDT is defined to allow
#    transition to protected mode. It defines flat code and data segments where
#    virtual addresses map directly to physical addresses (identity mapping for
#    the boot phase).
# 4. Transition to 32-bit Protected Mode: Sets the PE (Protection Enable) bit
#    in the CR0 control register.
# 5. Reload Segment Registers: Performs a long jump (`ljmp`) to a 32-bit code
#    segment to flush the CPU's instruction prefetch queue and properly load CS.
#    Then, sets up DS, ES, SS for 32-bit mode using selectors from the new GDT.
# 6. Set up Stack: Initializes the stack pointer (%esp).
# 7. Call C Code: Calls the `bootmain` function (in bootmain.c), which is
#    responsible for loading the xv6 kernel from disk.
# 8. Halt on Error: If `bootmain` were to return (it shouldn't), this code
#    triggers a Bochs breakpoint and then enters an infinite loop.

# Relevant header files defining constants and macros used below.
# `asm.h`: Defines macros like `SEG_NULLASM`, `SEG_ASM`.
# `memlayout.h`: Defines memory layout constants (not directly used here but related).
# `mmu.h`: Defines MMU-related constants like `CR0_PE`, `SEG_KCODE`, `SEG_KDATA`.
#include "asm.h"
#include "memlayout.h"
#include "mmu.h"

# Start the first CPU: switch to 32-bit protected mode, jump into C.
# The BIOS loads this code from the first sector of the hard disk into
# memory at physical address 0x7c00 and starts executing in real mode
# with %cs=0 %ip=7c00.

.code16                       # Tell assembler to generate 16-bit code.
.globl start                  # Make 'start' label globally visible (entry point).
start:
  cli                         # Disable interrupts. BIOS might have enabled them.

  # Zero data segment registers DS, ES, and SS.
  # In real mode, segment registers hold segment base addresses / 16.
  # Setting them to 0 ensures a known, flat memory model for now.
  xorw    %ax,%ax             # Set %ax to zero.
  movw    %ax,%ds             # Set Data Segment.
  movw    %ax,%es             # Set Extra Segment.
  movw    %ax,%ss             # Set Stack Segment.

  # Enable A20 address line.
  # Historically, A20 was gated (often tied to zero) for compatibility with older
  # software that relied on address wraparound at 1MB. Enabling A20 allows
  # access to memory above 1MB. This is done by manipulating the keyboard controller.
seta20.1:
  inb     $0x64,%al           # Read keyboard controller status port (0x64).
  testb   $0x2,%al            # Check bit 1 (Input buffer full). Wait until clear.
  jnz     seta20.1            # Loop if busy.

  movb    $0xd1,%al           # Command 0xd1: "write output port".
  outb    %al,$0x64           # Send command to keyboard controller.

seta20.2:
  inb     $0x64,%al           # Read status port again.
  testb   $0x2,%al            # Wait for input buffer full bit to clear.
  jnz     seta20.2            # Loop if busy.

  movb    $0xdf,%al           # Data 0xdf (Enable A20) to keyboard controller data port (0x60).
  outb    %al,$0x60           # Bit 1 of port 0x60 controls A20 gate.

  # Switch from real mode to 32-bit protected mode.
  # 1. Load Global Descriptor Table (GDT) Register (LGDT).
  #    `gdtdesc` points to a descriptor for our bootstrap GDT (`gdt`).
  #    This GDT defines flat segments (base 0, limit 4GB) so that virtual
  #    addresses effectively equal physical addresses during this transition.
  lgdt    gdtdesc
  # 2. Enable Protected Mode.
  #    Set the PE bit (Protection Enable, bit 0) in CR0 control register.
  movl    %cr0, %eax          # Get current CR0 value.
  orl     $CR0_PE, %eax       # Set PE bit.
  movl    %eax, %cr0          # Write back to CR0.

//PAGEBREAK!
  # Complete the transition to 32-bit protected mode.
  # A long jump (`ljmp`) is used here to:
  #  - Flush the CPU's instruction prefetch queue, which might contain 16-bit instructions.
  #  - Load %cs with a 32-bit code segment selector (`SEG_KCODE<<3`).
  #  - Set %eip to the address of `start32`.
  # The segment descriptors in `gdt` are set up with base 0 and limit 0xffffffff,
  # so the effective memory map remains an identity mapping for now.
  ljmp    $(SEG_KCODE<<3), $start32 # SEG_KCODE is 1 (index in GDT), so selector is 1*8 = 0x08.

.code32  # Tell assembler to generate 32-bit code from this point onwards.
start32:
  # Set up the protected-mode data segment registers (DS, ES, SS).
  # Load them with the selector for our data segment (SEG_KDATA<<3).
  # FS and GS are zeroed as they are not used by xv6 kernel initially.
  movw    $(SEG_KDATA<<3), %ax    # SEG_KDATA is 2, selector is 2*8 = 0x10.
  movw    %ax, %ds                # Load Data Segment register.
  movw    %ax, %es                # Load Extra Segment register.
  movw    %ax, %ss                # Load Stack Segment register.
  movw    $0, %ax                 # Zero out FS and GS.
  movw    %ax, %fs
  movw    %ax, %gs

  # Set up the stack pointer (%esp).
  # The stack grows downwards. `start` (0x7c00) is used as the initial top.
  # This means the stack will be located just below the loaded boot sector code.
  movl    $start, %esp
  # Call the C function `bootmain` (defined in bootmain.c).
  # `bootmain` is responsible for loading the ELF kernel from disk.
  call    bootmain

  # If bootmain returns (which it should not, as it's supposed to jump to the kernel),
  # it indicates an error.
  # Trigger a Bochs-specific magic breakpoint if running under Bochs debugger.
  # Then, enter an infinite loop to halt the CPU.
  movw    $0x8a00, %ax            # Bochs magic breakpoint values.
  movw    %ax, %dx
  outw    %ax, %dx
  movw    $0x8ae0, %ax
  outw    %ax, %dx
spin:
  jmp     spin                  # Infinite loop.

# Bootstrap Global Descriptor Table (GDT)
# This GDT is minimal, providing basic flat code and data segments.
# It allows virtual addresses to be the same as physical addresses.
.p2align 2                                # Ensure GDT is 4-byte aligned.
gdt:
  # Null Descriptor (required as the first entry in GDT).
  SEG_NULLASM
  # Code Segment Descriptor (Kernel Code):
  # Base=0, Limit=0xffffffff (4GB), Type=Execute/Read, Privilege=0 (Kernel).
  # SEG_ASM macro takes (type, base, limit). STA_X is Execute, STA_R is Read.
  SEG_ASM(STA_X|STA_R, 0x0, 0xffffffff)
  # Data Segment Descriptor (Kernel Data):
  # Base=0, Limit=0xffffffff (4GB), Type=Read/Write, Privilege=0 (Kernel).
  # STA_W is Write. (Read is implied for data segments).
  SEG_ASM(STA_W, 0x0, 0xffffffff)

# GDT Descriptor (GDTR content)
# This structure is what `lgdt` instruction loads.
# It contains the size of the GDT (limit) and its linear base address.
gdtdesc:
  .word   (gdtdesc - gdt - 1)             # Limit: Size of GDT minus 1.
  .long   gdt                             # Base: Linear address of the GDT.

