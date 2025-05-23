// 本文件在 xv6 中实现文件描述符管理和文件操作。
//
// 文件描述符：
// 每个进程都有其自己的打开文件描述符数组 (`p->ofile[NOFILE]`)。
// 文件描述符是一个小整数，用作此数组的索引。
// `p->ofile` 中的每个条目可以指向全局文件表 (`ftable`) 中的一个 `struct file`。
// 这允许多个文件描述符（即使跨不同进程，如果通过 fork 共享）
// 引用同一个底层打开文件。
//
// 文件表 (`ftable`):
// 一个全局表 (`ftable.file`)，包含 `NFILE` 个 `struct file` 条目。
// 此表表示系统中当前所有打开的文件。
// 它受自旋锁 (`ftable.lock`) 保护。
//
// `struct file`:
// 表示一个打开的文件。关键字段：
// - `type`: 文件类型 (FD_NONE, FD_PIPE, FD_INODE, FD_DEVICE)。
// - `ref`: 引用计数（有多少文件描述符指向此 struct file）。
// - `readable`, `writable`: 权限。
// - `pipe`: 如果是管道，则指向 `struct pipe`。
// - `ip`: 如果是 inode（常规文件、目录、设备），则指向 `struct inode`。
// - `off`: 基于 inode 的文件的当前读/写偏移量。
//
// 设备文件：
// `devsw[NDEV]` 是一个设备开关数组，为每个设备提供读/写函数。
// FD_DEVICE 类型的 `struct file` 将使用 `ip->major` 作为 `devsw` 的索引。

#include "types.h"
#include "defs.h"
#include "param.h"
#include "fs.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "file.h"

// 设备开关结构数组。每个条目为特定的主设备号
// 提供读/写函数。
struct devsw devsw[NDEV];

// 全局打开文件表 (`ftable`)。
// 它包含一个 `NFILE` 个 `struct file` 条目的数组，表示系统中当前
// 所有打开的文件。
// `lock` 保护对此表的访问，确保文件条目的线程安全分配、
// 释放和修改。
struct {
  struct spinlock lock;
  struct file file[NFILE]; // Array of NFILE file structures.
} ftable;

// 初始化文件表锁。
// 在内核初始化期间调用一次。
void
fileinit(void)
{
  initlock(&ftable.lock, "ftable");
}

// 从全局 `ftable` 分配一个新的 `struct file`。
// 它扫描 `ftable` 以查找 `f->ref == 0`（未使用）的条目。
// 如果找到，它将 `f->ref` 增加到1（标记为已使用）并返回指针。
// 如果没有可用的空闲 `struct file` 条目，则返回0。
// 获取并释放 `ftable.lock` 以确保独占访问。
struct file*
filealloc(void)
{
  struct file *f;

  acquire(&ftable.lock);
  for(f = ftable.file; f < ftable.file + NFILE; f++){
    if(f->ref == 0){
      f->ref = 1;
      release(&ftable.lock);
      return f;
    }
  }
  release(&ftable.lock);
  return 0;
}

// 增加现有 `struct file` `f` 的引用计数。
// 这通常在复制文件描述符时使用（例如，通过 `dup()` 系统调用
// 或当子进程在 `fork()` 期间从其父进程继承文件描述符时）。
// 在增加之前确保 `f->ref` 是有效的。
// 获取并释放 `ftable.lock`。
struct file*
filedup(struct file *f)
{
  acquire(&ftable.lock);
  if(f->ref < 1)
    panic("filedup");
  f->ref++;
  release(&ftable.lock);
  return f;
}

// 关闭一个打开的文件 `f`（减少其引用计数）。
// 如果引用计数降至0，则文件被真正关闭：
//  - 其在 `ftable` 中的条目被标记为空闲 (`f->ref = 0`, `f->type = FD_NONE`)。
//  - 如果它是一个管道，则调用 `pipeclose()`。
//  - 如果它是一个基于 inode 的文件，则通过 `iput()` 释放内存中的 inode。
//    `iput()` 本身处理减少 inode 的引用计数，并且如果 `nlink` 也变为0，
//    则可能释放 inode 及其数据块。
// 如果 `f->type == FD_INODE`，此函数必须在事务 (`begin_op`/`end_op`) 内调用，
// 因为 `iput` 可能会执行磁盘操作。
// 获取并释放 `ftable.lock`。
void
fileclose(struct file *f)
{
  struct file ff;

  acquire(&ftable.lock);
  if(f->ref < 1)
    panic("fileclose");
  if(--f->ref > 0){
    release(&ftable.lock);
    return;
  }
  ff = *f;
  f->ref = 0;
  f->type = FD_NONE;
  release(&ftable.lock);

  if(ff.type == FD_PIPE)
    pipeclose(ff.pipe, ff.writable);
  else if(ff.type == FD_INODE){
    begin_op();
    iput(ff.ip);
    end_op();
  }
}

// 获取有关打开文件 `f` 的元数据（状态）。
// 如果 `f` 是基于 inode 的文件 (FD_INODE)，它会锁定 inode，
// 调用 `stati()` (来自 fs.c) 以填充 `struct stat` `st`，
// 然后解锁 inode。
// 成功返回0，如果 `f` 不是基于 inode 的文件，则返回-1。
int
filestat(struct file *f, struct stat *st)
{
  if(f->type == FD_INODE){
    ilock(f->ip);
    stati(f->ip, st);
    iunlock(f->ip);
    return 0;
  }
  return -1;
}

// 从打开的文件 `f` 读取数据到缓冲区 `addr`，共 `n` 字节。
// - 检查文件是否可读 (`f->readable`)。
// - 如果 `f` 是管道 (FD_PIPE)，则调用 `piperead()`。
// - 如果 `f` 是基于 inode 的文件 (FD_INODE，例如常规文件、目录、设备文件)：
//   - 锁定关联的 inode (`f->ip`)。
//   - 调用 `readi()` (来自 fs.c) 从 inode 的当前文件偏移量 `f->off` 读取数据。
//     如果 `f->ip` 是设备 inode，`readi` 本身会处理对设备驱动程序的调度。
//   - 如果 `readi()` 成功，`f->off` 会增加读取的字节数。
//   - 解锁 inode。
// 返回读取的字节数，错误则返回-1。
// 如果 `f->type` 未知，则会 panic。
int
fileread(struct file *f, char *addr, int n)
{
  int r;

  if(f->readable == 0)
    return -1;
  if(f->type == FD_PIPE)
    return piperead(f->pipe, addr, n);
  if(f->type == FD_INODE){
    ilock(f->ip);
    if((r = readi(f->ip, addr, f->off, n)) > 0)
      f->off += r;
    iunlock(f->ip);
    return r;
  }
  panic("fileread");
}

//PAGEBREAK!
// 将数据从缓冲区 `addr` 写入打开的文件 `f`，共 `n` 字节。
// - 检查文件是否可写 (`f->writable`)。
// - 如果 `f` 是管道 (FD_PIPE)，则调用 `pipewrite()`。
// - 如果 `f` 是基于 inode 的文件 (FD_INODE)：
//   - 写入操作通常会分解成较小的块，以不超过最大日志事务大小。
//     这对于崩溃恢复很重要。
//   - 对于每个块：
//     - `begin_op()` 开始一个日志事务。
//     - 锁定关联的 inode (`f->ip`)。
//     - 调用 `writei()` (来自 fs.c) 将数据写入 inode 的当前偏移量 `f->off`。
//       如果 `f->ip` 是设备 inode，`writei` 会处理对设备驱动程序的调度。
//     - 如果 `writei()` 成功，`f->off` 会增加。
//     - 解锁 inode。
//     - `end_op()` 提交日志事务。
//   - 如果所有字节都成功写入，则返回 `n`，否则返回-1（或如果写入的字节数较少）。
// 如果 `f->type` 未知，则会 panic。
int
filewrite(struct file *f, char *addr, int n)
{
  int r;

  if(f->writable == 0)
    return -1;
  if(f->type == FD_PIPE)
    return pipewrite(f->pipe, addr, n);
  if(f->type == FD_INODE){
    // write a few blocks at a time to avoid exceeding
    // the maximum log transaction size, including
    // i-node, indirect block, allocation blocks,
    // and 2 blocks of slop for non-aligned writes.
    // this really belongs lower down, since writei()
    // might be writing a device like the console.
    int max = ((MAXOPBLOCKS-1-1-2) / 2) * 512;
    int i = 0;
    while(i < n){
      int n1 = n - i;
      if(n1 > max)
        n1 = max;

      begin_op();
      ilock(f->ip);
      if ((r = writei(f->ip, addr + i, f->off, n1)) > 0)
        f->off += r;
      iunlock(f->ip);
      end_op();

      if(r < 0)
        break;
      if(r != n1)
        panic("short filewrite");
      i += r;
    }
    return i == n ? n : -1;
  }
  panic("filewrite");
}

