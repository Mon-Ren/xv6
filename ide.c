// Simple PIO-based (Programmed I/O, non-DMA) IDE (Integrated Drive Electronics)
// disk driver for xv6. This driver is responsible for communicating with an IDE
// hard disk controller to read and write blocks of data (sectors).
//
// Key aspects:
// - PIO: Uses CPU instructions (inb, outb, insl, outsl) to transfer data
//   to/from the IDE controller's I/O ports, one word or dword at a time.
//   This is simpler than DMA (Direct Memory Access) but more CPU intensive.
// - Interrupt-driven: Uses IDE interrupts to signal completion of disk operations.
// - Request Queue (`idequeue`): Manages a queue of `struct buf` (buffer cache buffers)
//   that are waiting for disk I/O. This allows for asynchronous operations.
// - Synchronization: Uses `idelock` (spinlock) to protect the request queue and
//   `idewait` to poll for controller status. Processes sleep on buffer locks
//   while waiting for their specific I/O to complete.
//
// Core functions:
// - `ideinit()`: Initializes the IDE controller and interrupt.
// - `iderw(struct buf *b)`: Main entry point for reads/writes. Queues a buffer
//   and starts the operation if the disk is idle.
// - `idestart(struct buf *b)`: Initiates a read or write command to the disk hardware.
// - `ideintr()`: The interrupt handler called when a disk operation completes.
//   It processes the result, wakes up waiting processes, and starts the next
//   queued operation.

#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "x86.h"
#include "traps.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "fs.h"
#include "buf.h"

#define SECTOR_SIZE   512
#define IDE_BSY       0x80
#define IDE_DRDY      0x40
#define IDE_DF        0x20
#define IDE_ERR       0x01

#define IDE_CMD_READ  0x20
#define IDE_CMD_WRITE 0x30
#define IDE_CMD_RDMUL 0xc4
#define IDE_CMD_WRMUL 0xc5

// `idequeue`: A singly-linked list (queue) of `struct buf` pointers.
// These are buffers from the buffer cache that are waiting for disk I/O.
// `idequeue` points to the head of the queue (the buffer currently being
// processed or the next one if the disk is idle).
// `b->qnext` in `struct buf` is used to link buffers in this queue.
// Access to `idequeue` must be protected by `idelock`.
static struct spinlock idelock; // Spinlock to protect the idequeue and disk status.
static struct buf *idequeue;   // Pointer to the first buffer in the disk request queue.

static int havedisk1; // Flag indicating if disk 1 is present (xv6 primarily uses disk 0).
static void idestart(struct buf*); // Forward declaration for idestart.

// Wait for the IDE disk controller to become ready (DRDY set, BSY clear).
// This function polls the IDE status register (0x1f7).
// - `checkerr`: If true, also checks for disk errors (DF or ERR bits set in status).
// Returns 0 on success, -1 if `checkerr` is true and an error is detected.
static int
idewait(int checkerr)
{
  int r;

  while(((r = inb(0x1f7)) & (IDE_BSY|IDE_DRDY)) != IDE_DRDY)
    ;
  if(checkerr && (r & (IDE_DF|IDE_ERR)) != 0)
    return -1;
  return 0;
}

// Initialize the IDE interface.
// This function is called once during kernel startup.
// - Initializes `idelock`.
// - Enables the IDE interrupt (IRQ_IDE) in the I/O APIC.
// - Waits for the disk controller to become ready using `idewait`.
// - Probes for disk 1 (though xv6 typically uses only disk 0).
// - Selects disk 0 as the default.
void
ideinit(void)
{
  int i;

  initlock(&idelock, "ide"); // Initialize the spinlock for IDE operations.
  ioapicenable(IRQ_IDE, ncpu - 1); // Enable IDE interrupt line.
  idewait(0); // Wait for the disk to be ready (ignoring errors initially).

  // Check if disk 1 (slave) is present.
  // Standard IDE probe: select disk 1, check status register.
  // Some controllers might return a non-zero status if disk 1 exists.
  outb(0x1f6, 0xe0 | (1<<4)); // Select disk 1 (master/slave bit + drive select).
  for(i=0; i<1000; i++){ // Poll status.
    if(inb(0x1f7) != 0){ // Non-zero status might indicate presence.
      havedisk1 = 1;
      break;
    }
  }

  // Switch back to disk 0 (master).
  outb(0x1f6, 0xe0 | (0<<4)); // Select disk 0.
}

// Start a disk request for buffer `b`.
// The caller *must* hold `idelock`.
// This function sets up the IDE controller's registers to read or write
// the sectors corresponding to `b->blockno`.
// - Calculates the starting sector number on disk.
// - Issues commands to IDE registers:
//   - 0x3f6: Device control (disable interrupts during command setup, though not strictly necessary here).
//   - 0x1f2: Sector count (number of sectors to transfer, BSIZE/SECTOR_SIZE).
//   - 0x1f3-0x1f5: LBA28 sector address (low, mid, high bytes).
//   - 0x1f6: Drive/Head register (selects disk 0 or 1, LBA mode, upper bits of LBA28).
//   - 0x1f7: Command register (IDE_CMD_READ or IDE_CMD_WRITE).
// - If it's a write operation (`b->flags & B_DIRTY`), data is sent from `b->data`
//   to the IDE data port (0x1f0) using `outsl`.
// - If it's a read, the command is issued, and data will be read in `ideintr` using `insl`.
static void
idestart(struct buf *b)
{
  if(b == 0)
    panic("idestart");
  if(b->blockno >= FSSIZE)
    panic("incorrect blockno");
  int sector_per_block =  BSIZE/SECTOR_SIZE;
  int sector = b->blockno * sector_per_block;
  int read_cmd = (sector_per_block == 1) ? IDE_CMD_READ :  IDE_CMD_RDMUL;
  int write_cmd = (sector_per_block == 1) ? IDE_CMD_WRITE : IDE_CMD_WRMUL;

  if (sector_per_block > 7) panic("idestart");

  idewait(0);
  outb(0x3f6, 0);  // generate interrupt
  outb(0x1f2, sector_per_block);  // number of sectors
  outb(0x1f3, sector & 0xff);
  outb(0x1f4, (sector >> 8) & 0xff);
  outb(0x1f5, (sector >> 16) & 0xff);
  outb(0x1f6, 0xe0 | ((b->dev&1)<<4) | ((sector>>24)&0x0f));
  if(b->flags & B_DIRTY){
    outb(0x1f7, write_cmd);
    outsl(0x1f0, b->data, BSIZE/4);
  } else {
    outb(0x1f7, read_cmd);
  }
}

// IDE interrupt handler. This function is called when the IDE controller
// signals completion of a disk operation (read or write).
//
// Operation:
// 1. Acquire `idelock` to safely access `idequeue` and buffer flags.
// 2. Get the buffer `b` that just completed (it's at the head of `idequeue`).
//    If `idequeue` is empty, it's a spurious interrupt; release lock and return.
// 3. Dequeue `b` from `idequeue`.
// 4. If the operation was a read (`!(b->flags & B_DIRTY)`):
//    - Wait for the disk to be ready (`idewait(1)` with error checking).
//    - Read data from the IDE data port (0x1f0) into `b->data` using `insl`.
// 5. Mark the buffer as valid (`b->flags |= B_VALID`) and no longer dirty
//    (`b->flags &= ~B_DIRTY` as the operation is complete).
// 6. Wake up any process sleeping on this buffer `b` (e.g., in `iderw`).
// 7. If there are more buffers in `idequeue`, start the next one using `idestart`.
// 8. Release `idelock`.
void
ideintr(void)
{
  struct buf *b;

  // First queued buffer is the active request.
  acquire(&idelock);

  if((b = idequeue) == 0){
    release(&idelock);
    return;
  }
  idequeue = b->qnext;

  // Read data if needed.
  if(!(b->flags & B_DIRTY) && idewait(1) >= 0)
    insl(0x1f0, b->data, BSIZE/4);

  // Wake process waiting for this buf.
  b->flags |= B_VALID;
  b->flags &= ~B_DIRTY;
  wakeup(b);

  // Start disk on next buf in queue.
  if(idequeue != 0)
    idestart(idequeue);

  release(&idelock);
}

//PAGEBREAK!
// Synchronize a buffer `b` with the disk. This is the main function called
// by other parts of the kernel (like `bread` and `bwrite` from bio.c)
// to initiate a disk read or write for a buffer.
//
// Operation:
// 1. Sanity checks:
//    - Caller must hold the buffer's sleeplock (`b->lock`).
//    - Buffer must require action (either B_DIRTY is set for a write, or B_VALID
//      is not set for a read).
//    - If accessing disk 1, it must be present (`havedisk1`).
// 2. Acquire `idelock` to manipulate `idequeue`.
// 3. Add buffer `b` to the end of `idequeue`.
// 4. If `idequeue` was empty before adding `b` (meaning `b` is now the only
//    request and the disk is idle), call `idestart(b)` to begin the operation.
// 5. Atomically sleep:
//    - The process sleeps on the channel `b` (the buffer itself), releasing `idelock`
//      atomically (see `sleep()` in proc.c). This allows other processes to queue
//      requests or the interrupt handler to run.
//    - The process will be awakened by `ideintr()` when this specific buffer's
//      operation is complete.
//    - Loop `while((b->flags & (B_VALID|B_DIRTY)) != B_VALID)`: This condition
//      means "sleep as long as the buffer is not valid, or if it is valid but
//      still dirty". When a read completes, B_VALID is set. When a write completes,
//      B_DIRTY is cleared and B_VALID is set. So, the loop continues until B_VALID
//      is set and B_DIRTY is clear.
// 6. Release `idelock` (reacquired by `sleep` upon wakeup).
//
// This function effectively blocks the calling process until its specific
// disk I/O for buffer `b` is completed.
void
iderw(struct buf *b)
{
  struct buf **pp;

  if(!holdingsleep(&b->lock))
    panic("iderw: buf not locked");
  if((b->flags & (B_VALID|B_DIRTY)) == B_VALID)
    panic("iderw: nothing to do");
  if(b->dev != 0 && !havedisk1)
    panic("iderw: ide disk 1 not present");

  acquire(&idelock);  //DOC:acquire-lock

  // Append b to idequeue.
  b->qnext = 0;
  for(pp=&idequeue; *pp; pp=&(*pp)->qnext)  //DOC:insert-queue
    ;
  *pp = b;

  // Start disk if necessary.
  if(idequeue == b)
    idestart(b);

  // Wait for request to finish.
  while((b->flags & (B_VALID|B_DIRTY)) != B_VALID){
    sleep(b, &idelock);
  }


  release(&idelock);
}
