// Boot loader for xv6.
//
// This C code (`bootmain.c`) is called by `bootasm.S` after the CPU has been
// switched into 32-bit protected mode. `bootasm.S` and `bootmain.c` together
// constitute the boot block, which is loaded by the BIOS from the first sector
// of the boot disk.
//
// Responsibilities of `bootmain`:
// 1. Read Kernel from Disk: It reads the xv6 kernel, which is an ELF (Executable
//    and Linkable Format) executable, from the hard disk. xv6 assumes the kernel
//    image starts at sector 1 of the disk.
// 2. Parse ELF Header: It reads the ELF header of the kernel to identify its
//    structure and entry point. It performs a basic magic number check to
//    validate that it's a valid ELF file.
// 3. Load Kernel Segments: It iterates through the program headers (segments)
//    defined in the ELF file. For each loadable segment:
//    - It reads the segment's data from the disk (using `readseg`) into the
//      physical memory address specified in the program header (`ph->paddr`).
//    - If the segment's memory size (`ph->memsz`) is larger than its size in the
//      file (`ph->filesz`) (e.g., for .bss sections), the extra memory region
//      is zero-filled.
// 4. Jump to Kernel Entry Point: After successfully loading all kernel segments,
//    it retrieves the kernel's entry point address from the ELF header (`elf->entry`).
//    It then casts this address to a function pointer and calls it, effectively
//    transferring control to the loaded xv6 kernel. This call does not return.
//
// Disk I/O:
// - `readseg()`: Reads a segment of specified size from a given offset on disk
//   into a physical memory address.
// - `readsect()`: Reads a single 512-byte sector from disk.
// - `waitdisk()`: Waits for the disk controller to be ready for a command.
// These functions use Programmed I/O (PIO) to interact with the IDE disk controller.

#include "types.h"
#include "elf.h"
#include "x86.h"
#include "memlayout.h"

#define SECTSIZE  512

void readseg(uchar*, uint, uint); // Forward declaration for the segment reading function.

// Main function for the boot loader. Called by `bootasm.S`.
void
bootmain(void)
{
  struct elfhdr *elf;     // Pointer to the ELF header structure.
  struct proghdr *ph, *eph; // Pointers for iterating through program headers.
  void (*entry)(void);    // Function pointer for the kernel's entry point.
  uchar* pa;              // Physical address for loading segments.

  // The ELF header and program headers will be read into this memory location.
  // 0x10000 is a common scratch address used by boot loaders.
  elf = (struct elfhdr*)0x10000;

  // Read the first page (4096 bytes) from the disk. This should contain
  // the ELF header and potentially the first few program headers of the kernel.
  // The kernel image is assumed to start at offset 0 on the disk (which readseg
  // translates to sector 1, as sector 0 is the boot sector itself).
  readseg((uchar*)elf, 4096, 0);

  // Validate the ELF magic number.
  // If it's not a valid ELF file, return. `bootasm.S` has a loop to handle this failure.
  if(elf->magic != ELF_MAGIC)
    return; // Error: not an ELF executable.

  // Load each program segment from the ELF file into memory.
  // `elf->phoff` is the file offset of the first program header.
  // `elf->phnum` is the number of program headers.
  ph = (struct proghdr*)((uchar*)elf + elf->phoff); // Pointer to the first program header.
  eph = ph + elf->phnum; // Pointer to one past the last program header.
  for(; ph < eph; ph++){
    // `ph->paddr` is the physical address where this segment should be loaded.
    pa = (uchar*)ph->paddr;
    // `ph->off` is the offset of this segment within the ELF file.
    // `ph->filesz` is the size of this segment in the file.
    readseg(pa, ph->filesz, ph->off); // Read the segment from disk into memory.

    // `ph->memsz` is the size this segment should occupy in memory.
    // If `memsz > filesz`, the remaining part (typically .bss) must be zero-filled.
    if(ph->memsz > ph->filesz)
      stosb(pa + ph->filesz, 0, ph->memsz - ph->filesz); // Zero-fill the .bss section.
  }

  // Get the kernel's entry point address from the ELF header.
  // `elf->entry` contains the virtual address of the kernel's entry point.
  // In this boot phase, virtual addresses are effectively physical addresses.
  entry = (void(*)(void))(elf->entry);

  // Jump to the kernel's entry point. This function call does not return.
  // Control is transferred to the loaded xv6 kernel.
  entry();
}

// Wait for the IDE disk to be ready to accept a command.
// Polls the status register (port 0x1F7):
// - Bit 7 (BSY): Busy bit. Cleared when ready.
// - Bit 6 (DRDY): Drive Ready bit. Set when ready.
// Waits until (BSY == 0 and DRDY == 1), which means status is 0x40.
void
waitdisk(void)
{
  // Wait for disk ready (BSY clear, DRDY set).
  while((inb(0x1F7) & 0xC0) != 0x40) // 0xC0 is mask for BSY and DRDY bits.
    ; // Spin.
}

// Read a single 512-byte sector from the disk.
// - `dst`: Destination memory address to store the sector data.
// - `offset`: Sector number on disk (LBA - Logical Block Address).
// Uses PIO (Programmed I/O) to interact with the IDE controller.
void
readsect(void *dst, uint offset)
{
  // Issue the read command to the IDE controller.
  waitdisk(); // Wait for disk to be ready.
  outb(0x1F2, 1);   // Number of sectors to read: 1.
  // Send LBA28 sector address (offset):
  outb(0x1F3, offset & 0xFF);         // LBA bits 0-7.
  outb(0x1F4, (offset >> 8) & 0xFF);  // LBA bits 8-15.
  outb(0x1F5, (offset >> 16) & 0xFF); // LBA bits 16-23.
  // Drive/Head register:
  //  - 0xE0 sets LBA mode and selects master drive (disk 0).
  //  - `(offset >> 24) & 0x0F` provides LBA bits 24-27.
  outb(0x1F6, (offset >> 24) | 0xE0);
  outb(0x1F7, 0x20);  // Command 0x20: Read Sectors.

  // Read data from the disk.
  waitdisk(); // Wait for data to be ready in the disk's buffer.
  // `insl` reads `SECTSIZE/4` doublewords (32-bit values) from I/O port 0x1F0 (data port)
  // into the memory location `dst`.
  insl(0x1F0, dst, SECTSIZE/4);
}

// Read `count` bytes from the kernel image on disk into physical address `pa`.
// The kernel image starts at `offset` bytes from the beginning of the disk image
// (but after the boot sector, so `readseg` adds 1 to sector numbers).
// This function might read more than `count` bytes if `count` is not sector-aligned,
// as it reads in full sector units.
// - `pa`: Destination physical memory address.
// - `count`: Number of bytes to read.
// - `offset`: Byte offset within the kernel image file (not absolute disk sector).
void
readseg(uchar* pa, uint count, uint offset)
{
  uchar* epa;

  epa = pa + count;

  // Round down to sector boundary.
  pa -= offset % SECTSIZE;

  // Translate from bytes to sectors; kernel starts at sector 1.
  offset = (offset / SECTSIZE) + 1;

  // If this is too slow, we could read lots of sectors at a time.
  // We'd write more to memory than asked, but it doesn't matter --
  // we load in increasing order.
  for(; pa < epa; pa += SECTSIZE, offset++)
    readsect(pa, offset);
}
