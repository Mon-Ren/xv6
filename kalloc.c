// 物理内存分配器，旨在为用户进程、内核栈、页表页
// 和管道缓冲区分配内存。分配大小为4096字节的页。
// 它维护一个物理页的空闲列表，并提供
// 分配和释放这些页的函数。

#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "spinlock.h"

void freerange(void *vstart, void *vend);
extern char end[]; // first address after kernel loaded from ELF file
                   // defined by the kernel linker script in kernel.ld

// 代表一个空闲的物理页。
// 'next' 字段指向列表中的下一个空闲页。
struct run {
  struct run *next;
};

// 管理内核的物理内存。
// - lock: 一个自旋锁，用于保护对空闲列表的访问，确保线程安全。
// - use_lock: 一个标志，指示是否应使用锁。这在早期启动阶段很重要。
// - freelist: 指向空闲物理页单链表头部的指针。
struct {
  struct spinlock lock;
  int use_lock;
  struct run *freelist;
} kmem;

// 初始化分两个阶段进行。
// 1. main() 在仍使用 entrypgdir 时调用 kinit1()，仅将 entrypgdir 映射的页放入空闲列表。
//    在此阶段，锁定被禁用，因为只有一个CPU处于活动状态。
// 2. main() 在安装了映射所有核心的完整页表后，用其余物理页调用 kinit2()。
//    在此阶段启用锁定，因为多个CPU可能会访问分配器。
void
kinit1(void *vstart, void *vend)
{
  initlock(&kmem.lock, "kmem");
  kmem.use_lock = 0; // 尚不需要锁定 (单CPU，无中断)
  freerange(vstart, vend);
}

void
kinit2(void *vstart, void *vend)
{
  freerange(vstart, vend);
  kmem.use_lock = 1; // 为多处理器安全启用锁定
}

// 将一段物理内存范围添加到空闲列表。
// 它从 vstart 到 vend 逐页迭代，并为每个页调用 kfree。
// 此函数由 kinit1 和 kinit2 用于初始化空闲列表。
void
freerange(void *vstart, void *vend)
{
  char *p;
  p = (char*)PGROUNDUP((uint)vstart);
  for(; p + PGSIZE <= (char*)vend; p += PGSIZE)
    kfree(p);
}
//PAGEBREAK: 21
// 释放一个4096字节的物理内存页。
// 页的虚拟地址是 v。
// 此函数将页添加到 kmem.freelist。
// 如果地址未页对齐、位于内核代码/数据区内，
// 或超出有效物理内存范围，则会 panic。
// 在添加到空闲列表之前，该页填充垃圾数据 (1s)
// 以帮助捕获悬空引用（use-after-free）错误。
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
  r = (struct run*)v; // 将页视为 'struct run' 以便将其链接起来。
  r->next = kmem.freelist; // 将页添加到空闲列表的开头。
  kmem.freelist = r;
  if(kmem.use_lock)
    release(&kmem.lock);
}

// 分配一个4096字节的物理内存页。
// 它从 kmem.freelist 中移除第一个页，并返回其虚拟地址。
// 如果没有可用内存，则返回0。
// 分配的页未初始化；其内容未定义（通常是 kfree 留下的垃圾数据）。
char*
kalloc(void)
{
  struct run *r;

  if(kmem.use_lock)
    acquire(&kmem.lock);
  r = kmem.freelist; // 获取第一个空闲页。
  if(r)
    kmem.freelist = r->next; // 将空闲列表头向前移动。
  if(kmem.use_lock)
    release(&kmem.lock);
  return (char*)r; // 返回分配页的虚拟地址。
}

