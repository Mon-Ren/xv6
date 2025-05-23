#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "x86.h"

// This file contains the main C entry point for the xv6 kernel after the
// bootloader (`bootasm.S` and `bootmain.c`) has loaded the kernel into memory
// and transferred control to it. The initial entry point is typically an assembly
// routine (`entry.S`) which sets up a temporary page table and stack, then calls
// this `main()` function.
//
// `main()` is executed by the Bootstrap Processor (BSP). Its primary roles are:
// 1. Initialize Kernel Subsystems: It calls a series of `xxxinit()` functions
//    to initialize various core components of the kernel, such as memory
//    management, interrupt controllers, process management, file system, etc.
// 2. Start Application Processors (APs): If in a multiprocessor environment,
//    it calls `startothers()` to wake up and initialize other CPUs.
// 3. Create First User Process: It calls `userinit()` to set up the very first
//    user-mode process (`initcode`).
// 4. Start Scheduler: Finally, it calls `mpmain()` (which in turn calls `scheduler()`)
//    to begin scheduling and running processes on the BSP.
//
// The order of initializations is important as some subsystems depend on others
// being already set up.

static void startothers(void); // Function to start Application Processors (APs).
static void mpmain(void)  __attribute__((noreturn)); // Common per-CPU main function, never returns.
extern pde_t *kpgdir; // Kernel's global page directory (pointer).
extern char end[];    // Symbol defined by the linker, marking the end of kernel code/data/bss.
                      // Used by `kinit1` to know where free physical memory begins.

// Bootstrap Processor (BSP) starts running C code here.
// The initial environment (e.g., stack, page table) is set up by `entry.S`.
// This `main` function orchestrates the initialization of the entire kernel.
int
main(void)
{
  // Initialize physical page allocator (kalloc.c) for memory below 4MB.
  // `end` marks the end of the kernel image. `P2V(4*1024*1024)` is 4MB virtual address.
  kinit1(end, P2V(4*1024*1024));
  // Create and switch to the kernel's main page table (kvm.c).
  // This maps kernel code/data, physical memory, and I/O devices.
  kvmalloc();
  // Initialize multiprocessor support (mp.c). Detects other CPUs.
  mpinit();
  // Initialize the Local APIC for the current CPU (BSP) (lapic.c).
  // Manages local interrupts like the timer.
  lapicinit();
  // Initialize segment descriptors (GDT) (vm.c).
  // Sets up kernel and user code/data segments.
  seginit();
  // Disable the legacy 8259 Programmable Interrupt Controller (picirq.c).
  // APIC system (LAPIC + I/O APIC) will be used for interrupts.
  picinit();
  // Initialize the I/O APIC (ioapic.c).
  // Routes external hardware interrupts to CPUs.
  ioapicinit();
  // Initialize console device (console.c).
  consoleinit();
  // Initialize UART (serial port) (uart.c).
  uartinit();
  // Initialize the process table (`ptable`) (proc.c).
  pinit();
  // Initialize the Interrupt Descriptor Table (IDT) with trap/interrupt handlers (trap.c).
  tvinit();
  // Initialize the buffer cache for disk I/O (bio.c).
  binit();
  // Initialize the file table (`ftable`) for open file management (file.c).
  fileinit();
  // Initialize the IDE disk controller driver (ide.c).
  ideinit();
  // Start other CPUs (Application Processors). This must be done before
  // initializing the rest of physical memory because APs need memory for their stacks.
  startothers();
  // Initialize the rest of the physical page allocator (memory above 4MB up to PHYSTOP).
  kinit2(P2V(4*1024*1024), P2V(PHYSTOP));
  // Create and initialize the first user-space process ("initcode") (proc.c).
  userinit();
  // Complete this processor's (BSP's) setup and start the scheduler.
  // `mpmain` will not return.
  mpmain();
}

// Entry point for Application Processors (APs) after they are started by `startothers()`.
// This function is jumped to from assembly code in `entryother.S`.
// Each AP performs its own per-CPU initializations.
static void
mpenter(void)
{
  switchkvm(); // Switch to kernel page table (already set up by BSP).
  seginit();   // Initialize segment descriptors for this AP.
  lapicinit(); // Initialize this AP's Local APIC.
  mpmain();    // Call common CPU setup and start scheduler.
}

// Common per-CPU setup code, called by both BSP (from `main`) and APs (from `mpenter`).
// This function completes CPU-specific initialization and then starts the scheduler.
// It does not return.
static void
mpmain(void)
{
  cprintf("cpu%d: starting\n", cpuid()); // Announce CPU start.
  idtinit();       // Load the IDT register (IDTR) for this CPU. The IDT itself is shared.
  xchg(&(mycpu()->started), 1); // Atomically set this CPU's `started` flag to 1.
                               // This signals to `startothers()` that this AP is up.
  scheduler();     // Start the scheduling loop for this CPU. Never returns.
}

// `entrypgdir` is a temporary page directory used during early boot (by `entry.S` for BSP
// and `entryother.S` for APs) before the main kernel page table (`kpgdir`) is established by `kvmalloc`.
// It provides basic identity mapping for low memory and maps KERNBASE to physical 0.
pde_t entrypgdir[];  // Defined below.

// Start the non-boot Application Processors (APs).
// This function is called by the BSP from `main()`.
static void
startothers(void)
{
  extern uchar _binary_entryother_start[], _binary_entryother_size[];
  uchar *code;
  struct cpu *c;
  char *stack;

  // Write entry code to unused memory at 0x7000.
  // The linker has placed the image of entryother.S in
  // _binary_entryother_start.
  code = P2V(0x7000);
  memmove(code, _binary_entryother_start, (uint)_binary_entryother_size);

  for(c = cpus; c < cpus+ncpu; c++){
    if(c == mycpu())  // We've started already.
      continue;

    // Tell entryother.S what stack to use, where to enter, and what
    // pgdir to use. We cannot use kpgdir yet, because the AP processor
    // is running in low  memory, so we use entrypgdir for the APs too.
    stack = kalloc();
    *(void**)(code-4) = stack + KSTACKSIZE;
    *(void(**)(void))(code-8) = mpenter;
    *(int**)(code-12) = (void *) V2P(entrypgdir);

    lapicstartap(c->apicid, V2P(code));

    // wait for cpu to finish mpmain()
    while(c->started == 0)
      ;
  }
}

// `entrypgdir`: A simple page directory used during early kernel startup
// (by `entry.S` for the BSP and by `entryother.S` for APs before they switch
// to `kpgdir`).
// It must be page-aligned.
// It provides two 4MB page mappings using PTE_PS (Page Size Extension):
// 1. Identity map: Virtual Addresses [0, 4MB) map to Physical Addresses [0, 4MB).
//    This is needed for low memory access before `kvmalloc` sets up `kpgdir`.
// 2. Kernel base map: Virtual Addresses [KERNBASE, KERNBASE+4MB) map to
//    Physical Addresses [0, 4MB). This allows kernel code (linked at KERNBASE)
//    to run while still accessing low physical memory directly.
// Other parts of the address space are not mapped by `entrypgdir`.

__attribute__((__aligned__(PGSIZE))) // Ensure page alignment for the page directory.
pde_t entrypgdir[NPDENTRIES] = {
  // Map VA's [0, 4MB) to PA's [0, 4MB)
  // PDE index for 0 is 0. Value is PA=0, Present, Writable, Page Size Extension (4MB page).
  [0] = (0) | PTE_P | PTE_W | PTE_PS,
  // Map VA's [KERNBASE, KERNBASE+4MB) to PA's [0, 4MB)
  // PDE index for KERNBASE is KERNBASE>>PDXSHIFT. Value is PA=0, Present, Writable, Page Size Extension.
  [KERNBASE>>PDXSHIFT] = (0) | PTE_P | PTE_W | PTE_PS,
};

//PAGEBREAK!
// Blank page.
//PAGEBREAK!
// Blank page.
//PAGEBREAK!
// Blank page.

