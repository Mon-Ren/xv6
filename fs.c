// File system implementation. xv6 supports a simple file system structure.
// This file implements the core components of that file system, including
// management of blocks, inodes, directories, and path name resolution.
// It interacts with the buffer cache (bio.c) for disk I/O and the logging
// layer (log.c) for crash recovery.
//
// The file system can be visualized in layers:
//   +-------------------+
//   | Pathnames         | (e.g., /usr/rtm/xv6/fs.c, handled by namei, nameiparent)
//   +-------------------+
//   | Directories       | (Implementation of directories as special files containing dirent structures, dirlookup, dirlink)
//   +-------------------+
//   | Files (Inodes)    | (Inode allocation, metadata, data block mapping (bmap), readi, writei)
//   +-------------------+
//   | Log               | (Crash recovery for multi-step disk updates, begin_op, log_write, end_op)
//   +-------------------+
//   | Blocks (Buffer    | (Raw disk block allocation (balloc, bfree) and management via buffer cache)
//   |  Cache)           |
//   +-------------------+
//   | Disk (IDE Driver) | (Hardware interface for disk reads/writes)
//   +-------------------+
//
// This file (`fs.c`) primarily deals with the "Files", "Directories", "Blocks",
// and some aspects of "Pathnames". System call interfaces are in `sysfile.c`.

#include "types.h"
#include "defs.h"
#include "param.h"
#include "stat.h"
#include "mmu.h"
#include "proc.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "fs.h"
#include "buf.h"
#include "file.h"

#define min(a, b) ((a) < (b) ? (a) : (b))
static void itrunc(struct inode*);
// there should be one superblock per disk device, but we run with
// only one device.
// `sb` is a global variable holding the superblock for the mounted file system.
// The superblock contains critical metadata about the file system, such as its
// total size, number of data blocks, number of inodes, start of the log,
// and start of the inode table and block bitmap.
struct superblock sb; 

// Read the super block from the disk device `dev`.
// The superblock is always located at block 1 on the disk.
// This function reads block 1, copies its content into the global `sb` struct.
void
readsb(int dev, struct superblock *sb)
{
  struct buf *bp;

  bp = bread(dev, 1);
  memmove(sb, bp->data, sizeof(*sb));
  brelse(bp);
}

// Zero a block.
static void
bzero(int dev, int bno)
{
  struct buf *bp;

  bp = bread(dev, bno);
  memset(bp->data, 0, BSIZE);
  log_write(bp);
  brelse(bp);
}

// Blocks.

// Allocate a zeroed disk block from the free block bitmap.
// It iterates through the bitmap blocks (sb.bmapstart). For each bitmap block,
// it checks each bit. If a bit is 0, the corresponding data block is free.
// It marks the block as used (sets the bit to 1), writes the modified bitmap
// block to disk (via log_write for crash safety), zeroes out the newly allocated
// data block using `bzero`, and returns the block number.
// Panics if no free blocks are available.
static uint
balloc(uint dev)
{
  int b, bi, m;
  struct buf *bp;

  bp = 0;
  for(b = 0; b < sb.size; b += BPB){
    bp = bread(dev, BBLOCK(b, sb));
    for(bi = 0; bi < BPB && b + bi < sb.size; bi++){
      m = 1 << (bi % 8);
      if((bp->data[bi/8] & m) == 0){  // Is block free?
        bp->data[bi/8] |= m;  // Mark block in use.
        log_write(bp);
        brelse(bp);
        bzero(dev, b + bi);
        return b + bi;
      }
    }
    brelse(bp);
  }
  panic("balloc: out of blocks");
}

// Free a disk block `b` on device `dev`.
// It calculates the bitmap block containing the bit for block `b`,
// reads this bitmap block, clears the corresponding bit (marks block as free),
// and writes the modified bitmap block back to disk (via log_write).
// Panics if trying to free an already free block.
static void
bfree(int dev, uint b)
{
  struct buf *bp;
  int bi, m;

  bp = bread(dev, BBLOCK(b, sb));
  bi = b % BPB;
  m = 1 << (bi % 8);
  if((bp->data[bi/8] & m) == 0)
    panic("freeing free block");
  bp->data[bi/8] &= ~m;
  log_write(bp);
  brelse(bp);
}

// Inodes.
//
// An inode describes a single unnamed file.
// The inode disk structure holds metadata: the file's type,
// its size, the number of links referring to it, and the
// list of blocks holding the file's content.
//
// The inodes are laid out sequentially on disk at
// sb.startinode. Each inode has a number, indicating its
// position on the disk.
//
// The kernel keeps a cache of in-use inodes in memory
// to provide a place for synchronizing access
// to inodes used by multiple processes. The cached
// inodes include book-keeping information that is
// not stored on disk: ip->ref and ip->valid.
//
// An inode and its in-memory representation go through a
// sequence of states before they can be used by the
// rest of the file system code.
//
// * Allocation: an inode is allocated if its type (on disk)
//   is non-zero. ialloc() allocates, and iput() frees if
//   the reference and link counts have fallen to zero.
//
// * Referencing in cache: an entry in the inode cache
//   is free if ip->ref is zero. Otherwise ip->ref tracks
//   the number of in-memory pointers to the entry (open
//   files and current directories). iget() finds or
//   creates a cache entry and increments its ref; iput()
//   decrements ref.
//
// * Valid: the information (type, size, &c) in an inode
//   cache entry is only correct when ip->valid is 1.
//   ilock() reads the inode from
//   the disk and sets ip->valid, while iput() clears
//   ip->valid if ip->ref has fallen to zero.
//
// * Locked: file system code may only examine and modify
//   the information in an inode and its content if it
//   has first locked the inode.
//
// Thus a typical sequence is:
//   ip = iget(dev, inum)
//   ilock(ip)
//   ... examine and modify ip->xxx ...
//   iunlock(ip)
//   iput(ip)
//
// ilock() is separate from iget() so that system calls can
// get a long-term reference to an inode (as for an open file)
// and only lock it for short periods (e.g., in read()).
// The separation also helps avoid deadlock and races during
// pathname lookup. iget() increments ip->ref so that the inode
// stays cached and pointers to it remain valid.
//
// Many internal file system functions expect the caller to
// have locked the inodes involved; this lets callers create
// multi-step atomic operations.
//
// The icache.lock spin-lock protects the allocation of icache
// entries. Since ip->ref indicates whether an entry is free,
// and ip->dev and ip->inum indicate which i-node an entry
// holds, one must hold icache.lock while using any of those fields.
//
// An ip->lock sleep-lock protects all ip-> fields other than ref,
// dev, and inum.  One must hold ip->lock in order to
// read or write that inode's ip->valid, ip->size, ip->type, &c.
//
// `icache` is the in-memory cache of inodes. It holds up to `NINODE` inodes.
// `icache.lock` (spinlock) protects the integrity of the cache structure itself,
// particularly during allocation/deallocation of entries or when searching.
// Each `struct inode` in the cache has its own `lock` (sleeplock) to protect
// its metadata fields (type, size, nlink, addrs, valid flag).
struct {
  struct spinlock lock;      // Protects access to the icache entries array.
  struct inode inode[NINODE]; // Array of in-memory inode structures.
} icache;

// Initialize the inode cache and read the superblock.
// This function is called once during file system initialization.
// It initializes the `icache.lock` and the sleeplock for each inode in the cache.
// It then reads the superblock information from the specified device.
void
iinit(int dev)
{
  int i = 0;
  
  initlock(&icache.lock, "icache"); // Initialize the global icache lock.
  for(i = 0; i < NINODE; i++) { // Initialize the lock for each inode structure in the cache.
    initsleeplock(&icache.inode[i].lock, "inode");
  }

  readsb(dev, &sb); // Read the superblock from the disk.
  // Print superblock information for debugging.
  cprintf("sb: size %d nblocks %d ninodes %d nlog %d logstart %d\
 inodestart %d bmap start %d\n", sb.size, sb.nblocks,
          sb.ninodes, sb.nlog, sb.logstart, sb.inodestart,
          sb.bmapstart);
}

static struct inode* iget(uint dev, uint inum);

//PAGEBREAK!
// Allocate a new inode on device `dev` of the given `type`.
// It scans the inode blocks on disk (starting from `sb.inodestart`)
// for a free inode (type == 0 in `struct dinode`).
// Once a free on-disk inode (`struct dinode`) is found, it's marked with the
// given `type`, its other fields are zeroed, and the change is written to disk
// (via `log_write` for crash safety).
// Finally, it returns an in-memory, referenced (but unlocked) `struct inode`
// corresponding to this newly allocated on-disk inode, obtained via `iget`.
// Panics if no free inodes are available.
struct inode*
ialloc(uint dev, short type)
{
  int inum;
  struct buf *bp;
  struct dinode *dip;

  for(inum = 1; inum < sb.ninodes; inum++){
    bp = bread(dev, IBLOCK(inum, sb));
    dip = (struct dinode*)bp->data + inum%IPB;
    if(dip->type == 0){  // a free inode
      memset(dip, 0, sizeof(*dip));
      dip->type = type;
      log_write(bp);   // mark it allocated on the disk
      brelse(bp);
      return iget(dev, inum);
    }
    brelse(bp);
  }
  panic("ialloc: no inodes");
}

// Copy a modified in-memory inode (`ip`) back to its on-disk representation (`struct dinode`).
// This function must be called after any change to an inode's persistent fields
// (type, major/minor, nlink, size, addrs) to ensure the changes are written to disk.
// The inode cache in xv6 is write-through for metadata.
// The caller must hold `ip->lock` to ensure exclusive access to inode fields.
// The changes are written to disk via `log_write` for crash safety.
void
iupdate(struct inode *ip)
{
  struct buf *bp;
  struct dinode *dip;

  bp = bread(ip->dev, IBLOCK(ip->inum, sb));
  dip = (struct dinode*)bp->data + ip->inum%IPB;
  dip->type = ip->type;
  dip->major = ip->major;
  dip->minor = ip->minor;
  dip->nlink = ip->nlink;
  dip->size = ip->size;
  memmove(dip->addrs, ip->addrs, sizeof(ip->addrs));
  log_write(bp);
  brelse(bp);
}

// Find the inode with number `inum` on device `dev` in the inode cache (`icache`).
// If the inode is already cached, its reference count (`ip->ref`) is incremented.
// If not cached, a free entry in `icache` is found (or panic if cache is full),
// initialized with `dev` and `inum`, its reference count set to 1, and `ip->valid`
// marked as 0 (meaning its data hasn't been read from disk yet).
// This function returns a pointer to the in-memory `struct inode`.
// It does *not* lock the inode or read it from disk; `ilock` handles that.
// `icache.lock` is acquired to protect access to `icache` and inode `ref` counts.
static struct inode*
iget(uint dev, uint inum)
{
  struct inode *ip, *empty;

  acquire(&icache.lock);

  // Is the inode already cached?
  empty = 0;
  for(ip = &icache.inode[0]; ip < &icache.inode[NINODE]; ip++){
    if(ip->ref > 0 && ip->dev == dev && ip->inum == inum){
      ip->ref++;
      release(&icache.lock);
      return ip;
    }
    if(empty == 0 && ip->ref == 0)    // Remember empty slot.
      empty = ip;
  }

  // Recycle an inode cache entry.
  if(empty == 0)
    panic("iget: no inodes");

  ip = empty;
  ip->dev = dev;
  ip->inum = inum;
  ip->ref = 1;
  ip->valid = 0;
  release(&icache.lock);

  return ip;
}

// Increment reference count for ip.
// Returns ip to enable ip = idup(ip1) idiom.
struct inode*
idup(struct inode *ip)
{
  acquire(&icache.lock);
  ip->ref++;
  release(&icache.lock);
  return ip;
}

// Lock the given inode `ip`.
// This function acquires the inode's sleeplock (`ip->lock`).
// If the inode's data is not yet valid in memory (`ip->valid == 0`),
// it reads the inode's metadata from disk (from the `struct dinode`)
// and populates the in-memory `struct inode` fields (type, size, links, data blocks).
// After `ilock` returns, the inode is locked and its in-memory data is valid.
// Panics if `ip` is null or has a reference count less than 1.
void
ilock(struct inode *ip)
{
  struct buf *bp;
  struct dinode *dip;

  if(ip == 0 || ip->ref < 1)
    panic("ilock");

  acquiresleep(&ip->lock);

  if(ip->valid == 0){
    bp = bread(ip->dev, IBLOCK(ip->inum, sb));
    dip = (struct dinode*)bp->data + ip->inum%IPB;
    ip->type = dip->type;
    ip->major = dip->major;
    ip->minor = dip->minor;
    ip->nlink = dip->nlink;
    ip->size = dip->size;
    memmove(ip->addrs, dip->addrs, sizeof(ip->addrs));
    brelse(bp);
    ip->valid = 1;
    if(ip->type == 0)
      panic("ilock: no type");
  }
}

// Unlock the given inode.
void
iunlock(struct inode *ip)
{
  if(ip == 0 || !holdingsleep(&ip->lock) || ip->ref < 1)
    panic("iunlock");

  releasesleep(&ip->lock);
}

// Drop a reference to an in-memory inode `ip`.
// This is called when a part of the kernel is done with an inode (e.g., file close).
// It decrements `ip->ref`.
// If `ip->ref` drops to 0 *and* `ip->nlink` (on-disk link count) is 0:
//   The inode is no longer referenced in memory and has no directory entries
//   pointing to it. So, its data blocks are truncated (`itrunc`), its type is set
//   to 0 on disk (`iupdate`), and `ip->valid` is cleared. This effectively frees the inode.
// If `ip->ref` drops to 0 but `ip->nlink` is not 0, the in-memory inode entry can be
// recycled by `iget` if needed, but the on-disk inode remains.
// `ip->lock` is acquired to check `nlink` and potentially modify the inode.
// `icache.lock` is acquired to safely decrement `ip->ref`.
// All calls to `iput()` must be within a transaction block (`begin_op`/`end_op`)
// because `itrunc` and `iupdate` write to disk via the log.
void
iput(struct inode *ip)
{
  acquiresleep(&ip->lock);
  if(ip->valid && ip->nlink == 0){
    acquire(&icache.lock);
    int r = ip->ref;
    release(&icache.lock);
    if(r == 1){
      // inode has no links and no other references: truncate and free.
      itrunc(ip);
      ip->type = 0;
      iupdate(ip);
      ip->valid = 0;
    }
  }
  releasesleep(&ip->lock);

  acquire(&icache.lock);
  ip->ref--;
  release(&icache.lock);
}

// Common idiom: unlock, then put.
void
iunlockput(struct inode *ip)
{
  iunlock(ip);
  iput(ip);
}

//PAGEBREAK!
// Inode content
//
// The content (data) associated with each inode is stored
// in blocks on the disk. The first NDIRECT block numbers
// are listed in ip->addrs[].  The next NINDIRECT blocks are
// listed in block ip->addrs[NDIRECT].

// Return the disk block address of the `bn`-th data block for inode `ip`.
// `bn` is the logical block number within the file (0 for the first block, 1 for second, etc.).
// If the block does not yet exist and `bn` is a valid next block to allocate,
// `bmap` allocates a new data block using `balloc` and updates the inode's
// address list (`ip->addrs`) to include this new block.
// Handles both direct blocks (`bn < NDIRECT`) and singly indirect blocks
// (`NDIRECT <= bn < NDIRECT + NINDIRECT`).
// For indirect blocks, it reads the indirect block, allocates if necessary,
// then finds/allocates the target data block entry within it.
// Returns the physical disk block number.
// Panics if `bn` is out of range (exceeds max file size).
// Caller must hold `ip->lock`.
static uint
bmap(struct inode *ip, uint bn)
{
  uint addr, *a;
  struct buf *bp;

  if(bn < NDIRECT){
    if((addr = ip->addrs[bn]) == 0)
      ip->addrs[bn] = addr = balloc(ip->dev);
    return addr;
  }
  bn -= NDIRECT;

  if(bn < NINDIRECT){
    // Load indirect block, allocating if necessary.
    if((addr = ip->addrs[NDIRECT]) == 0)
      ip->addrs[NDIRECT] = addr = balloc(ip->dev);
    bp = bread(ip->dev, addr);
    a = (uint*)bp->data;
    if((addr = a[bn]) == 0){
      a[bn] = addr = balloc(ip->dev);
      log_write(bp);
    }
    brelse(bp);
    return addr;
  }

  panic("bmap: out of range");
}

// Truncate inode `ip`, discarding all its data blocks.
// This is called when an inode is being freed (e.g., in `iput` when `nlink` and `ref` are 0).
// It iterates through all direct blocks (`ip->addrs[0...NDIRECT-1]`) and frees them using `bfree`.
// If there's an indirect block (`ip->addrs[NDIRECT]`), it reads that block,
// frees all the data blocks pointed to by entries in the indirect block,
// and then frees the indirect block itself.
// Finally, it sets `ip->size` to 0 and updates the inode on disk using `iupdate`.
// Caller must hold `ip->lock`.
static void
itrunc(struct inode *ip)
{
  int i, j;
  struct buf *bp;
  uint *a;

  for(i = 0; i < NDIRECT; i++){
    if(ip->addrs[i]){
      bfree(ip->dev, ip->addrs[i]);
      ip->addrs[i] = 0;
    }
  }

  if(ip->addrs[NDIRECT]){
    bp = bread(ip->dev, ip->addrs[NDIRECT]);
    a = (uint*)bp->data;
    for(j = 0; j < NINDIRECT; j++){
      if(a[j])
        bfree(ip->dev, a[j]);
    }
    brelse(bp);
    bfree(ip->dev, ip->addrs[NDIRECT]);
    ip->addrs[NDIRECT] = 0;
  }

  ip->size = 0;
  iupdate(ip);
}

// Copy inode metadata from `ip` to a `struct stat` structure `st`.
// `struct stat` is used by the `stat` and `fstat` system calls.
// Caller must hold `ip->lock` to ensure consistent reading of inode fields.
void
stati(struct inode *ip, struct stat *st)
{
  st->dev = ip->dev;
  st->ino = ip->inum;
  st->type = ip->type;
  st->nlink = ip->nlink;
  st->size = ip->size;
}

//PAGEBREAK!
// Read data from inode `ip` into destination buffer `dst`.
// Starts reading at `off` bytes into the file, for `n` bytes.
// Caller must hold `ip->lock`.
// If `ip` refers to a device file (T_DEV), it calls the device's read function.
// Otherwise, for regular files/directories:
// - It checks for valid `off` and `n` against `ip->size`.
// - It iterates, calculating the logical block number for the current `off` using `bmap`.
// - Reads the physical block using `bread`, copies the relevant portion into `dst`.
// - Releases the buffer using `brelse`.
// Returns the number of bytes read, or -1 on error.
int
readi(struct inode *ip, char *dst, uint off, uint n)
{
  uint tot, m;
  struct buf *bp;

  if(ip->type == T_DEV){
    if(ip->major < 0 || ip->major >= NDEV || !devsw[ip->major].read)
      return -1;
    return devsw[ip->major].read(ip, dst, n);
  }

  if(off > ip->size || off + n < off)
    return -1;
  if(off + n > ip->size)
    n = ip->size - off;

  for(tot=0; tot<n; tot+=m, off+=m, dst+=m){
    bp = bread(ip->dev, bmap(ip, off/BSIZE));
    m = min(n - tot, BSIZE - off%BSIZE);
    memmove(dst, bp->data + off%BSIZE, m);
    brelse(bp);
  }
  return n;
}

// PAGEBREAK!
// Write data from source buffer `src` to inode `ip`.
// Starts writing at `off` bytes into the file, for `n` bytes.
// Caller must hold `ip->lock`.
// If `ip` refers to a device file (T_DEV), it calls the device's write function.
// Otherwise, for regular files/directories:
// - It checks for valid `off` and `n` (cannot exceed MAXFILE size).
// - It iterates, calculating the logical block number for the current `off` using `bmap`
//   (which allocates blocks if they don't exist).
// - Reads the physical block using `bread`.
// - Copies data from `src` into the buffer.
// - Marks the buffer for logging (`log_write`) and releases it (`brelse`).
// - If the write extends the file, `ip->size` is updated and `iupdate` is called.
// Returns the number of bytes written, or -1 on error.
int
writei(struct inode *ip, char *src, uint off, uint n)
{
  uint tot, m;
  struct buf *bp;

  if(ip->type == T_DEV){
    if(ip->major < 0 || ip->major >= NDEV || !devsw[ip->major].write)
      return -1;
    return devsw[ip->major].write(ip, src, n);
  }

  if(off > ip->size || off + n < off)
    return -1;
  if(off + n > MAXFILE*BSIZE)
    return -1;

  for(tot=0; tot<n; tot+=m, off+=m, src+=m){
    bp = bread(ip->dev, bmap(ip, off/BSIZE));
    m = min(n - tot, BSIZE - off%BSIZE);
    memmove(bp->data + off%BSIZE, src, m);
    log_write(bp);
    brelse(bp);
  }

  if(n > 0 && off > ip->size){
    ip->size = off;
    iupdate(ip);
  }
  return n;
}

//PAGEBREAK!
// Directories

int
namecmp(const char *s, const char *t)
{
  return strncmp(s, t, DIRSIZ);
}

// Look for a directory entry `name` within directory `dp`.
// `dp` must be a T_DIR inode and the caller must hold `dp->lock`.
// It reads directory entries (`struct dirent`) from `dp`'s data blocks.
// If an entry with a matching `name` is found:
//  - If `poff` is not null, `*poff` is set to the byte offset of this entry within the directory.
//  - An in-memory inode for the found entry is obtained using `iget` and returned.
// If the entry is not found, returns 0.
// Panics if `dp` is not a directory or if there's a read error.
// `struct dirent` contains the filename (up to DIRSIZ chars) and its inode number.
struct inode*
dirlookup(struct inode *dp, char *name, uint *poff)
{
  uint off, inum;
  struct dirent de;

  if(dp->type != T_DIR)
    panic("dirlookup not DIR");

  for(off = 0; off < dp->size; off += sizeof(de)){
    if(readi(dp, (char*)&de, off, sizeof(de)) != sizeof(de))
      panic("dirlookup read");
    if(de.inum == 0)
      continue;
    if(namecmp(name, de.name) == 0){
      // entry matches path element
      if(poff)
        *poff = off;
      inum = de.inum;
      return iget(dp->dev, inum);
    }
  }

  return 0;
}

// Write a new directory entry (name, inum) into the directory `dp`.
// `dp` must be a T_DIR inode and the caller must hold `dp->lock`.
// - First, it checks if `name` already exists in `dp` using `dirlookup`. If so, fails (-1).
// - Then, it looks for an empty directory entry slot (`de.inum == 0`).
// - If an empty slot is found, it populates it with `name` and `inum`.
// - The updated directory entry is written back to `dp`'s data block using `writei`.
// Returns 0 on success, -1 on failure (e.g., name exists, or write error).
// Panics if there's an error writing the new entry.
int
dirlink(struct inode *dp, char *name, uint inum)
{
  int off;
  struct dirent de;
  struct inode *ip;

  // Check that name is not present.
  if((ip = dirlookup(dp, name, 0)) != 0){
    iput(ip);
    return -1;
  }

  // Look for an empty dirent.
  for(off = 0; off < dp->size; off += sizeof(de)){
    if(readi(dp, (char*)&de, off, sizeof(de)) != sizeof(de))
      panic("dirlink read");
    if(de.inum == 0)
      break;
  }

  strncpy(de.name, name, DIRSIZ);
  de.inum = inum;
  if(writei(dp, (char*)&de, off, sizeof(de)) != sizeof(de))
    panic("dirlink");

  return 0;
}

//PAGEBREAK!
// Paths

// Copy the next path element from path into name.
// Return a pointer to the element following the copied one.
// The returned path has no leading slashes,
// so the caller can check *path=='\0' to see if the name is the last one.
// If no name to remove, return 0.
//
// Examples:
//   skipelem("a/bb/c", name) = "bb/c", setting name = "a"
//   skipelem("///a//bb", name) = "bb", setting name = "a"
//   skipelem("a", name) = "", setting name = "a"
//   skipelem("", name) = skipelem("////", name) = 0
//
static char*
skipelem(char *path, char *name)
{
  char *s;
  int len;

  while(*path == '/')
    path++;
  if(*path == 0)
    return 0;
  s = path;
  while(*path != '/' && *path != 0)
    path++;
  len = path - s;
  if(len >= DIRSIZ)
    memmove(name, s, DIRSIZ);
  else {
    memmove(name, s, len);
    name[len] = 0;
  }
  while(*path == '/')
    path++;
  return path;
}

// Look up and return the inode for a given `path`.
// This is the core path resolution function.
// - `path`: The path string to resolve.
// - `nameiparent`: If true, resolve to the parent directory of the final path element,
//   and copy the final element's name into `name`.
// - `name`: Buffer to store the final path element if `nameiparent` is true.
//
// Operation:
// - Starts from either the root directory (`/`) or the current working directory (`myproc()->cwd`).
// - Iteratively calls `skipelem` to get the next path component.
// - For each component, it locks the current directory inode (`ip`), checks if it's a directory,
//   then uses `dirlookup` to find the inode of the next component.
// - It unlocks and puts the current directory inode and moves to the next one.
// - If `nameiparent` is true and it's the last element, it returns the parent inode.
// - Otherwise, it returns the inode for the final element of the path.
// Returns the inode if found, or 0 on error (e.g., path component not found, not a directory).
// Must be called inside a transaction (`begin_op`/`end_op`) because it calls `iput`,
// which might modify disk structures (freeing an inode).
static struct inode*
namex(char *path, int nameiparent, char *name)
{
  struct inode *ip, *next;

  if(*path == '/')
    ip = iget(ROOTDEV, ROOTINO);
  else
    ip = idup(myproc()->cwd);

  while((path = skipelem(path, name)) != 0){
    ilock(ip);
    if(ip->type != T_DIR){
      iunlockput(ip);
      return 0;
    }
    if(nameiparent && *path == '\0'){
      // Stop one level early.
      iunlock(ip);
      return ip;
    }
    if((next = dirlookup(ip, name, 0)) == 0){
      iunlockput(ip);
      return 0;
    }
    iunlockput(ip);
    ip = next;
  }
  if(nameiparent){
    iput(ip);
    return 0;
  }
  return ip;
}

struct inode*
namei(char *path)
{
  char name[DIRSIZ];
  return namex(path, 0, name);
}

struct inode*
nameiparent(char *path, char *name)
{
  return namex(path, 1, name);
}
