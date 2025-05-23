// This file implements file descriptor management and file operations in xv6.
//
// File Descriptors:
// Each process has its own array of open file descriptors (`p->ofile[NOFILE]`).
// A file descriptor is a small integer that indexes into this array.
// Each entry in `p->ofile` can point to a `struct file` in the global file table (`ftable`).
// This allows multiple file descriptors (even across different processes, if shared via fork)
// to refer to the same underlying open file.
//
// File Table (`ftable`):
// A global table (`ftable.file`) containing `NFILE` `struct file` entries.
// This table represents all currently open files in the system.
// It's protected by a spinlock (`ftable.lock`).
//
// `struct file`:
// Represents an open file. Key fields:
// - `type`: Type of file (FD_NONE, FD_PIPE, FD_INODE, FD_DEVICE).
// - `ref`: Reference count (how many file descriptors point to this struct file).
// - `readable`, `writable`: Permissions.
// - `pipe`: If it's a pipe, pointer to `struct pipe`.
// - `ip`: If it's an inode (regular file, directory, device), pointer to `struct inode`.
// - `off`: Current read/write offset for inode-based files.
//
// Device Files:
// `devsw[NDEV]` is an array of device switches, providing read/write functions for each device.
// A `struct file` of type FD_DEVICE will use `ip->major` to index into `devsw`.

#include "types.h"
#include "defs.h"
#include "param.h"
#include "fs.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "file.h"

// Array of device switch structures. Each entry provides read/write
// functions for a specific major device number.
struct devsw devsw[NDEV];

// The global open file table (`ftable`).
// It contains an array of `NFILE` `struct file` entries, representing all
// files currently open in the system.
// The `lock` protects access to this table, ensuring thread-safe allocation,
// deallocation, and modification of file entries.
struct {
  struct spinlock lock;
  struct file file[NFILE]; // Array of NFILE file structures.
} ftable;

// Initialize the file table lock.
// Called once during kernel initialization.
void
fileinit(void)
{
  initlock(&ftable.lock, "ftable");
}

// Allocate a new `struct file` from the global `ftable`.
// It scans `ftable` for an entry with `f->ref == 0` (unused).
// If found, it increments `f->ref` to 1 (marking it as used) and returns the pointer.
// Returns 0 if no free `struct file` entries are available.
// Acquires and releases `ftable.lock` to ensure exclusive access.
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

// Increment the reference count for an existing `struct file` `f`.
// This is typically used when a file descriptor is duplicated (e.g., via `dup()` system call
// or when a child process inherits file descriptors from its parent during `fork()`).
// Ensures that `f->ref` is valid before incrementing.
// Acquires and releases `ftable.lock`.
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

// Close an open file `f` (decrement its reference count).
// If the reference count drops to 0, the file is truly closed:
//  - Its entry in `ftable` is marked as free (`f->ref = 0`, `f->type = FD_NONE`).
//  - If it was a pipe, `pipeclose()` is called.
//  - If it was an inode-based file, the in-memory inode is released via `iput()`.
//    `iput()` itself handles decrementing the inode's reference count and potentially
//    freeing the inode and its data blocks if `nlink` also becomes 0.
// This function must be called within a transaction (`begin_op`/`end_op`) if `f->type == FD_INODE`
// because `iput` might perform disk operations.
// Acquires and releases `ftable.lock`.
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

// Get metadata (status) about an open file `f`.
// If `f` is an inode-based file (FD_INODE), it locks the inode,
// calls `stati()` (from fs.c) to populate the `struct stat` `st`,
// then unlocks the inode.
// Returns 0 on success, -1 if `f` is not an inode-based file.
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

// Read data from an open file `f` into buffer `addr` for `n` bytes.
// - Checks if the file is readable (`f->readable`).
// - If `f` is a pipe (FD_PIPE), calls `piperead()`.
// - If `f` is an inode-based file (FD_INODE, e.g., regular file, directory, device file):
//   - Locks the associated inode (`f->ip`).
//   - Calls `readi()` (from fs.c) to read data from the inode at the current file offset `f->off`.
//     `readi` itself handles dispatching to device drivers if `f->ip` is a device inode.
//   - If `readi()` succeeds, `f->off` is incremented by the number of bytes read.
//   - Unlocks the inode.
// Returns the number of bytes read, or -1 on error.
// Panics if `f->type` is unknown.
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
// Write data from buffer `addr` to an open file `f` for `n` bytes.
// - Checks if the file is writable (`f->writable`).
// - If `f` is a pipe (FD_PIPE), calls `pipewrite()`.
// - If `f` is an inode-based file (FD_INODE):
//   - Writes are often broken into smaller chunks to not exceed the maximum log transaction size.
//     This is important for crash recovery.
//   - For each chunk:
//     - `begin_op()` starts a log transaction.
//     - Locks the associated inode (`f->ip`).
//     - Calls `writei()` (from fs.c) to write data to the inode at current offset `f->off`.
//       `writei` handles dispatching to device drivers if `f->ip` is a device inode.
//     - If `writei()` succeeds, `f->off` is incremented.
//     - Unlocks the inode.
//     - `end_op()` commits the log transaction.
//   - Returns `n` if all bytes were written successfully, or -1 on error (or if fewer bytes were written).
// Panics if `f->type` is unknown.
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

