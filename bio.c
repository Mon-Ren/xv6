// Buffer cache implementation for xv6.
//
// The buffer cache (`bcache`) serves two main purposes:
// 1. Caching: It keeps copies of recently used disk blocks in memory,
//    reducing the need for slow disk reads.
// 2. Synchronization: It provides a synchronization point for disk blocks
//    that might be accessed by multiple processes concurrently. Only one
//    process can have a buffer locked at a time.
//
// Structure:
// - `bcache.buf`: An array of `NBUF` `struct buf` entries. Each `struct buf`
//   can hold the contents of one disk block.
// - `bcache.head`: A dummy `struct buf` that serves as the head of a
//   doubly-linked list. This list orders buffers by usage (Most Recently Used - MRU).
//   `bcache.head.next` points to the MRU buffer, and `bcache.head.prev` points
//   to the LRU (Least Recently Used) buffer.
// - `bcache.lock`: A spinlock protecting the `bcache` structure itself, primarily
//   the linked list pointers and buffer `refcnt` fields during lookup and recycling.
//
// `struct buf` fields:
// - `dev`, `blockno`: Device and block number this buffer represents.
// - `flags`: State flags:
//   - `B_VALID`: Indicates that `buf->data` contains valid data read from disk.
//   - `B_DIRTY`: Indicates that `buf->data` has been modified and needs to be written to disk.
// - `refcnt`: Reference count. If > 0, the buffer is in use.
// - `lock`: A sleeplock protecting the buffer's data (`buf->data`) and its `flags`.
//           A process must acquire this lock before using the buffer's data.
// - `data[BSIZE]`: The actual cached data of the disk block.
// - `prev`, `next`: Pointers for the MRU/LRU linked list.
// - `qnext`: Pointer for queuing buffers for disk I/O in the IDE driver.
//
// Interface:
// - `bread(dev, blockno)`: Get a locked buffer for a given disk block. Reads from disk if not cached or not valid.
// - `bwrite(buf)`: Mark a locked buffer as dirty. The actual write to disk is deferred (usually handled by `iderw` called from `log_write`).
// - `brelse(buf)`: Release a locked buffer. Decrements `refcnt`. If `refcnt` is 0, moves buffer to MRU list.
// - `bget(dev, blockno)`: Internal function to find/allocate a buffer.
//
// Usage Pattern:
//   struct buf *b = bread(dev, blockno); // Get and lock buffer
//   // ... read or modify b->data ...
//   if (modified)
//     bwrite(b); // Mark dirty if modified
//   brelse(b);   // Release buffer

#include "types.h"
#include "defs.h"
#include "param.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "fs.h"
#include "buf.h"

// The global buffer cache structure.
struct {
  struct spinlock lock; // Spinlock to protect bcache metadata (list, refcounts).
  struct buf buf[NBUF]; // Array of NBUF buffer headers.

  // Doubly-linked list of all buffers, ordered by most-recently-used (MRU).
  // `head.next` points to the MRU buffer.
  // `head.prev` points to the LRU buffer.
  struct buf head; // Dummy head of the MRU list.
} bcache;

// Initialize the buffer cache.
// This function is called once during kernel startup.
// It initializes the `bcache.lock` and sets up the doubly-linked list
// of buffers, initially making all buffers available (part of the MRU list).
// Each buffer also gets its own sleeplock initialized.
void
binit(void)
{
  struct buf *b;

  initlock(&bcache.lock, "bcache");

//PAGEBREAK!
  // Create linked list of buffers
  bcache.head.prev = &bcache.head;
  bcache.head.next = &bcache.head;
  for(b = bcache.buf; b < bcache.buf+NBUF; b++){
    b->next = bcache.head.next;
    b->prev = &bcache.head;
    initsleeplock(&b->lock, "buffer");
    bcache.head.next->prev = b;
    bcache.head.next = b;
  }
}

// Get a buffer for the specified disk block (`dev`, `blockno`).
// This is the core function for acquiring a buffer from the cache.
//
// Operation:
// 1. Acquire `bcache.lock` to safely inspect and modify the cache.
// 2. Search for an existing buffer:
//    Iterate through the `bcache`'s MRU list. If a buffer for the
//    given `dev` and `blockno` is found:
//    - Increment its `refcnt`.
//    - Release `bcache.lock`.
//    - Acquire the buffer's individual sleeplock (`b->lock`).
//    - Return the locked buffer.
// 3. If not cached (block not found in the list):
//    - Search for an unused buffer to recycle: Iterate through the MRU list
//      (starting from LRU end via `bcache.head.prev`) looking for a buffer
//      with `refcnt == 0` and not `B_DIRTY`. A dirty buffer, even if `refcnt == 0`,
//      cannot be immediately recycled as it holds changes not yet written to disk
//      (often managed by the logging system).
//    - If a recyclable buffer is found:
//      - Update its `dev`, `blockno`.
//      - Clear its `flags` (it's no longer B_VALID for the old content).
//      - Set `refcnt = 1`.
//      - Release `bcache.lock`.
//      - Acquire the buffer's sleeplock (`b->lock`).
//      - Return the locked buffer. (Its data is not yet valid).
// 4. If no buffer can be recycled (e.g., all are referenced or dirty), panic.
//
// Returns a *locked* `struct buf`. The caller is responsible for eventually
// calling `brelse` on it.
static struct buf*
bget(uint dev, uint blockno)
{
  struct buf *b;

  acquire(&bcache.lock);

  // Is the block already cached?
  for(b = bcache.head.next; b != &bcache.head; b = b->next){
    if(b->dev == dev && b->blockno == blockno){
      b->refcnt++;
      release(&bcache.lock);
      acquiresleep(&b->lock);
      return b;
    }
  }

  // Not cached; recycle an unused buffer.
  // Even if refcnt==0, B_DIRTY indicates a buffer is in use
  // because log.c has modified it but not yet committed it.
  for(b = bcache.head.prev; b != &bcache.head; b = b->prev){
    if(b->refcnt == 0 && (b->flags & B_DIRTY) == 0) {
      b->dev = dev;
      b->blockno = blockno;
      b->flags = 0;
      b->refcnt = 1;
      release(&bcache.lock);
      acquiresleep(&b->lock);
      return b;
    }
  }
  panic("bget: no buffers");
}

// Return a locked buffer (`struct buf`) containing the contents of the
// disk block specified by `dev` and `blockno`.
//
// Operation:
// 1. Calls `bget(dev, blockno)` to get a buffer (either from cache or a recycled one).
//    `bget` returns the buffer locked.
// 2. If the buffer's data is not valid (`(b->flags & B_VALID) == 0`),
//    it means the buffer is newly allocated for this block or its previous
//    content was invalidated. In this case, `iderw(b)` is called to
//    read the block's data from the disk into `b->data`. `iderw` will set B_VALID.
// 3. Returns the locked buffer, now guaranteed to have valid data.
struct buf*
bread(uint dev, uint blockno)
{
  struct buf *b;

  b = bget(dev, blockno);
  if((b->flags & B_VALID) == 0) {
    iderw(b);
  }
  return b;
}

// Mark buffer `b` as dirty, indicating its contents have been modified
// and need to be written to disk.
// The caller *must* hold `b->lock`.
// This function sets the `B_DIRTY` flag and then calls `iderw(b)`.
// `iderw` will eventually write the data to disk when it processes the
// IDE queue (or immediately if the queue is empty and disk is idle).
// Note: `bwrite` itself doesn't block waiting for the write to complete.
// The actual disk write is handled by the IDE driver and interrupt.
void
bwrite(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("bwrite");
  b->flags |= B_DIRTY;
  iderw(b);
}

// Release a locked buffer `b`.
// The caller *must* hold `b->lock` (which `brelse` will release).
//
// Operation:
// 1. Release the buffer's sleeplock (`b->lock`).
// 2. Acquire `bcache.lock` to safely modify buffer metadata and the MRU list.
// 3. Decrement `b->refcnt`.
// 4. If `b->refcnt` becomes 0 (meaning no other part of the kernel is using this buffer):
//    - Move the buffer to the front of the MRU list (`bcache.head.next`).
//      This indicates it was recently used and is a good candidate for caching.
// 5. Release `bcache.lock`.
//
// After `brelse`, the caller should not use the buffer pointer `b` anymore,
// as it might be recycled by another call to `bget`.
void
brelse(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("brelse");

  releasesleep(&b->lock);

  acquire(&bcache.lock);
  b->refcnt--;
  if (b->refcnt == 0) {
    // no one is waiting for it.
    b->next->prev = b->prev;
    b->prev->next = b->next;
    b->next = bcache.head.next;
    b->prev = &bcache.head;
    bcache.head.next->prev = b;
    bcache.head.next = b;
  }
  
  release(&bcache.lock);
}
//PAGEBREAK!
// Blank page.

