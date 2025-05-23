// Physical memory allocator, intended to allocate
// memory for user processes, kernel stacks, page table pages,
// and pipe buffers. Allocates 4096-byte pages.
// It maintains a free list of physical pages and provides
// functions to allocate and free these pages.

#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "spinlock.h"

void freerange(void *vstart, void *vend);
extern char end[]; // first address after kernel loaded from ELF file
                   // defined by the kernel linker script in kernel.ld

// Represents a free physical page.
// The 'next' field points to the next free page in the list.
struct run {
  struct run *next;
};

// Manages the kernel's physical memory.
// - lock: A spinlock to protect access to the freelist, ensuring thread safety.
// - use_lock: A flag indicating whether the lock should be used. This is important during early boot stages.
// - freelist: A pointer to the head of a singly linked list of free physical pages.
struct {
  struct spinlock lock;
  int use_lock;
  struct run *freelist;
} kmem;

// Initialization happens in two phases.
// 1. main() calls kinit1() while still using entrypgdir to place just
// the pages mapped by entrypgdir on free list. During this phase, locking is disabled
// as only one CPU is active.
// 2. main() calls kinit2() with the rest of the physical pages
// after installing a full page table that maps them on all cores. Locking is enabled
// in this phase as multiple CPUs might access the allocator.
void
kinit1(void *vstart, void *vend)
{
  initlock(&kmem.lock, "kmem");
  kmem.use_lock = 0; // Locking not needed yet (single CPU, no interrupts)
  freerange(vstart, vend);
}

void
kinit2(void *vstart, void *vend)
{
  freerange(vstart, vend);
  kmem.use_lock = 1; // Enable locking for multi-processor safety
}

// Adds a range of physical memory to the free list.
// It iterates from vstart to vend, page by page, and calls kfree for each page.
// This function is used by kinit1 and kinit2 to initialize the free list.
void
freerange(void *vstart, void *vend)
{
  char *p;
  p = (char*)PGROUNDUP((uint)vstart);
  for(; p + PGSIZE <= (char*)vend; p += PGSIZE)
    kfree(p);
}
//PAGEBREAK: 21
// Frees a 4096-byte page of physical memory.
// The page's virtual address is v.
// This function adds the page to the kmem.freelist.
// It panics if the address is not page-aligned, is within kernel code/data,
// or is outside the valid physical memory range.
// Before adding to the freelist, the page is filled with junk data (1s)
// to help catch use-after-free bugs (dangling references).
void
kfree(char *v)
{
  struct run *r;

  if((uint)v % PGSIZE || v < end || V2P(v) >= PHYSTOP)
    panic("kfree");

  // Fill with junk to catch dangling refs.
  memset(v, 1, PGSIZE);

  if(kmem.use_lock)
    acquire(&kmem.lock);
  r = (struct run*)v; // Treat the page as a 'struct run' to link it.
  r->next = kmem.freelist; // Add the page to the beginning of the freelist.
  kmem.freelist = r;
  if(kmem.use_lock)
    release(&kmem.lock);
}

// Allocates one 4096-byte page of physical memory.
// It removes the first page from the kmem.freelist and returns its virtual address.
// Returns 0 if no memory is available.
// The allocated page is not initialized; its contents are undefined (usually junk from kfree).
char*
kalloc(void)
{
  struct run *r;

  if(kmem.use_lock)
    acquire(&kmem.lock);
  r = kmem.freelist; // Get the first free page.
  if(r)
    kmem.freelist = r->next; // Advance the freelist head.
  if(kmem.use_lock)
    release(&kmem.lock);
  return (char*)r; // Return the virtual address of the allocated page.
}

