// This file implements the kernel-side of system calls that are related to
// file and file system operations.
//
// These functions typically perform the following steps:
// 1. Argument Fetching and Validation:
//    - Use helper functions like `argint`, `argstr`, `argptr` (from syscall.c)
//      to retrieve arguments (e.g., file descriptors, paths, pointers, sizes)
//      from the user process's stack and memory.
//    - Validate these arguments (e.g., check if file descriptors are valid,
//      if pointers are within user space, if paths are reasonable).
// 2. Calling Lower-Level File System Functions:
//    - Invoke functions from `file.c` (for file descriptor and open file table
//      operations, like `filealloc`, `fileclose`, `fileread`, `filewrite`)
//      and `fs.c` (for inode and block-level operations, like `namei`,
//      `dirlink`, `create`, `readi`, `writei`).
// 3. Transaction Management:
//    - Many file system operations that modify on-disk structures (e.g., creating
//      a file, linking, unlinking) are wrapped in `begin_op()` and `end_op()`
//      to ensure atomicity and crash recovery via the logging system (log.c).
// 4. Return Value:
//    - Return an appropriate value to the user (e.g., number of bytes read/written,
//      a new file descriptor, 0 on success, or -1 on error).

#include "types.h"
#include "defs.h"
#include "param.h"
#include "stat.h"
#include "mmu.h"
#include "proc.h"
#include "fs.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "file.h"
#include "fcntl.h"

// Fetch the n-th word-sized system call argument, which is expected to be a
// file descriptor (fd).
// - `n`: The argument number (0-indexed).
// - `pfd`: If not null, the integer value of the fd is stored here.
// - `pf`: If not null, a pointer to the corresponding `struct file` from the
//         current process's open file table (`myproc()->ofile[fd]`) is stored here.
// Returns 0 on success.
// Returns -1 if:
//   - `argint` fails to fetch the fd value.
//   - The fd is out of bounds (less than 0 or greater than or equal to NOFILE).
//   - The process does not have a file open for that fd (`myproc()->ofile[fd] == 0`).
static int
argfd(int n, int *pfd, struct file **pf)
{
  int fd;
  struct file *f;

  if(argint(n, &fd) < 0)
    return -1;
  if(fd < 0 || fd >= NOFILE || (f=myproc()->ofile[fd]) == 0)
    return -1;
  if(pfd)
    *pfd = fd;
  if(pf)
    *pf = f;
  return 0;
}

// Allocate an unused file descriptor in the current process's open file table
// (`myproc()->ofile`) and assign the given `struct file *f` to it.
// This function effectively gives the process a handle (the fd) to an already
// opened file structure `f`.
// The `struct file *f` should already have its reference count incremented by the caller
// if it's a new reference (e.g., in `sys_open`), or it's being duplicated (`sys_dup`).
// `fdalloc` itself does not modify `f->ref`.
// Returns the allocated file descriptor (an integer) on success.
// Returns -1 if no free file descriptors are available in the process's table.
static int
fdalloc(struct file *f)
{
  int fd;
  struct proc *curproc = myproc();

  for(fd = 0; fd < NOFILE; fd++){
    if(curproc->ofile[fd] == 0){
      curproc->ofile[fd] = f;
      return fd;
    }
  }
  return -1;
}

// System call: dup(int fd)
// Duplicates an existing file descriptor.
// - Fetches the file descriptor argument using `argfd`.
// - Allocates a new file descriptor using `fdalloc`.
// - Increments the reference count of the underlying `struct file` using `filedup`.
// Returns the new file descriptor on success, -1 on error.
int
sys_dup(void)
{
  struct file *f;
  int fd;

  // Get the struct file for the given fd.
  if(argfd(0, 0, &f) < 0)
    return -1;
  // Allocate a new fd for this struct file.
  if((fd=fdalloc(f)) < 0)
    return -1;
  filedup(f); // Increment ref count of the struct file.
  return fd;  // Return new fd.
}

// System call: read(int fd, char *buf, int n)
// Reads up to `n` bytes from the file descriptor `fd` into buffer `buf`.
// - Fetches arguments: fd, user buffer pointer `p`, and count `n`.
// - Calls `fileread` (from file.c) to perform the read operation.
// Returns the number of bytes read, or -1 on error.
int
sys_read(void)
{
  struct file *f;
  int n;
  char *p;

  // Fetch fd and corresponding struct file*, user buffer pointer, and count.
  if(argfd(0, 0, &f) < 0 || argint(2, &n) < 0 || argptr(1, &p, n) < 0)
    return -1;
  return fileread(f, p, n);
}

// System call: write(int fd, char *buf, int n)
// Writes `n` bytes from buffer `buf` to the file descriptor `fd`.
// - Fetches arguments: fd, user buffer pointer `p`, and count `n`.
// - Calls `filewrite` (from file.c) to perform the write operation.
// Returns the number of bytes written, or -1 on error.
int
sys_write(void)
{
  struct file *f;
  int n;
  char *p;

  // Fetch fd and corresponding struct file*, user buffer pointer, and count.
  if(argfd(0, 0, &f) < 0 || argint(2, &n) < 0 || argptr(1, &p, n) < 0)
    return -1;
  return filewrite(f, p, n);
}

// System call: close(int fd)
// Closes a file descriptor.
// - Fetches the file descriptor `fd` and its `struct file *f`.
// - Removes the fd from the current process's open file table (`myproc()->ofile[fd] = 0`).
// - Calls `fileclose` (from file.c) to decrement the `struct file`'s reference count
//   and potentially close the underlying file/pipe if the ref count reaches zero.
// Returns 0 on success, -1 on error.
int
sys_close(void)
{
  int fd;
  struct file *f;

  // Fetch fd and corresponding struct file*.
  if(argfd(0, &fd, &f) < 0)
    return -1;
  myproc()->ofile[fd] = 0; // Clear entry in process's fd table.
  fileclose(f);            // Close the struct file.
  return 0;
}

// System call: fstat(int fd, struct stat *st)
// Gets status information (metadata) about an open file.
// - Fetches the file descriptor `fd` and its `struct file *f`.
// - Fetches a user pointer `st` to a `struct stat` where results will be stored.
// - Calls `filestat` (from file.c) to populate the `struct stat`.
// Returns 0 on success, -1 on error.
int
sys_fstat(void)
{
  struct file *f;
  struct stat *st;

  // Fetch fd, struct file*, and user pointer to struct stat.
  if(argfd(0, 0, &f) < 0 || argptr(1, (void*)&st, sizeof(*st)) < 0)
    return -1;
  return filestat(f, st);
}

// System call: link(char *oldname, char *newname)
// Creates a new hard link `newname` that points to the same inode as `oldname`.
// - Fetches string arguments `old` and `new` paths.
// - Begins a file system transaction (`begin_op`).
// - Looks up the inode for `oldname` using `namei`.
// - Checks that `oldname` is not a directory (cannot hard link directories).
// - Increments `ip->nlink` for the inode and updates it on disk (`iupdate`).
// - Looks up the parent directory inode for `newname` using `nameiparent`.
// - Calls `dirlink` to create the new directory entry for `name` pointing to `ip->inum`.
// - Releases inodes and ends the transaction.
// - Handles error cases by decrementing `nlink` if steps fail.
// Returns 0 on success, -1 on error.
int
sys_link(void)
{
  char name[DIRSIZ], *new, *old;
  struct inode *dp, *ip;

  if(argstr(0, &old) < 0 || argstr(1, &new) < 0)
    return -1;

  begin_op();
  if((ip = namei(old)) == 0){
    end_op();
    return -1;
  }

  ilock(ip);
  if(ip->type == T_DIR){
    iunlockput(ip);
    end_op();
    return -1;
  }

  ip->nlink++;
  iupdate(ip);
  iunlock(ip);

  if((dp = nameiparent(new, name)) == 0)
    goto bad;
  ilock(dp);
  if(dp->dev != ip->dev || dirlink(dp, name, ip->inum) < 0){
    iunlockput(dp);
    goto bad;
  }
  iunlockput(dp);
  iput(ip);

  end_op();

  return 0;

bad:
  ilock(ip);
  ip->nlink--;
  iupdate(ip);
  iunlockput(ip);
  end_op();
  return -1;
}

// Is the directory dp empty except for "." and ".." ?
static int
isdirempty(struct inode *dp)
{
  int off;
  struct dirent de;

  for(off=2*sizeof(de); off<dp->size; off+=sizeof(de)){
    if(readi(dp, (char*)&de, off, sizeof(de)) != sizeof(de))
      panic("isdirempty: readi");
    if(de.inum != 0)
      return 0;
  }
  return 1;
}

//PAGEBREAK!
// System call: unlink(char *pathname)
// Removes a name (link) from the file system.
// If this is the last link to an inode, the inode and its data are freed.
// - Fetches the `path` argument.
// - Begins a file system transaction.
// - Looks up the parent directory `dp` of `path` and the final name component.
// - Checks that `name` is not "." or "..".
// - Looks up the inode `ip` for `name` within `dp`.
// - If `ip` is a directory, checks if it's empty (except for "." and "..").
// - Clears the directory entry for `name` in `dp` by writing zeros.
// - If `ip` was a directory, decrements `dp->nlink` (for the ".." entry in child).
// - Decrements `ip->nlink`.
// - Updates inodes on disk and releases them. `iput` will handle freeing if `nlink` is 0.
// - Ends the transaction.
// Returns 0 on success, -1 on error.
int
sys_unlink(void)
{
  struct inode *ip, *dp;
  struct dirent de;
  char name[DIRSIZ], *path;
  uint off;

  if(argstr(0, &path) < 0)
    return -1;

  begin_op();
  if((dp = nameiparent(path, name)) == 0){
    end_op();
    return -1;
  }

  ilock(dp);

  // Cannot unlink "." or "..".
  if(namecmp(name, ".") == 0 || namecmp(name, "..") == 0)
    goto bad;

  if((ip = dirlookup(dp, name, &off)) == 0)
    goto bad;
  ilock(ip);

  if(ip->nlink < 1)
    panic("unlink: nlink < 1");
  if(ip->type == T_DIR && !isdirempty(ip)){
    iunlockput(ip);
    goto bad;
  }

  memset(&de, 0, sizeof(de));
  if(writei(dp, (char*)&de, off, sizeof(de)) != sizeof(de))
    panic("unlink: writei");
  if(ip->type == T_DIR){
    dp->nlink--;
    iupdate(dp);
  }
  iunlockput(dp);

  ip->nlink--;
  iupdate(ip);
  iunlockput(ip);

  end_op();

  return 0;

bad:
  iunlockput(dp);
  end_op();
  return -1;
}

// Helper function: create a new file or directory or device file.
// - `path`: The full path to the new file/directory.
// - `type`: T_FILE, T_DIR, or T_DEV.
// - `major`, `minor`: Device numbers if type is T_DEV.
//
// Operation:
// 1. Find parent directory `dp` of `path` and the final name component.
// 2. Lock `dp`.
// 3. Check if `name` already exists:
//    - If it exists and is a file, and `type` is T_FILE, return existing inode (for O_CREATE).
//    - Otherwise (exists but wrong type, or creating a dir/dev that exists), fail.
// 4. If it doesn't exist, allocate a new inode `ip` of the given `type` using `ialloc`.
// 5. Lock `ip`, set its major/minor (if T_DEV), nlink=1, and update it on disk.
// 6. If `type` is T_DIR:
//    - Increment `dp->nlink` (for the ".." entry in the new directory).
//    - Update `dp` on disk.
//    - Create "." and ".." entries within the new directory `ip`.
// 7. Link the new inode `ip` into the parent directory `dp` under `name`.
// 8. Unlock and release `dp`.
// Returns the locked inode `ip` on success, 0 on failure.
// The caller is responsible for unlocking `ip` (e.g., via `iunlockput` if it's an intermediate step
// or just `iunlock` if returning the inode to be used further by `sys_open`).
static struct inode*
create(char *path, short type, short major, short minor)
{
  struct inode *ip, *dp;
  char name[DIRSIZ];

  // Find parent directory.
  if((dp = nameiparent(path, name)) == 0)
    return 0;
  ilock(dp); // Lock parent directory.

  // Check if name already exists.
  if((ip = dirlookup(dp, name, 0)) != 0){
    iunlockput(dp); // Release parent.
    ilock(ip);      // Lock existing inode.
    // If trying to create a file and a file of the same name already exists,
    // open will succeed with this existing file.
    if(type == T_FILE && ip->type == T_FILE)
      return ip;
    // Otherwise (e.g. creating a dir that exists, or type mismatch), it's an error.
    iunlockput(ip);
    return 0;
  }

  // Allocate new inode.
  if((ip = ialloc(dp->dev, type)) == 0)
    panic("create: ialloc failed"); // Should not happen if disk isn't full.

  ilock(ip); // Lock the newly allocated inode.
  ip->major = major;
  ip->minor = minor;
  ip->nlink = 1; // New files/dirs/devs start with one link.
  iupdate(ip);   // Write new inode to disk.

  if(type == T_DIR){  // Special setup for directories.
    dp->nlink++;      // Parent's link count increases because ".." in new dir points to parent.
    iupdate(dp);
    // Create "." and ".." entries in the new directory.
    // "." points to itself. ".." points to parent dp.
    // No ip->nlink++ for ".": avoid cyclic ref count issues with ".".
    if(dirlink(ip, ".", ip->inum) < 0 || dirlink(ip, "..", dp->inum) < 0)
      panic("create dots failed");
  }

  // Link the new inode into the parent directory.
  if(dirlink(dp, name, ip->inum) < 0)
    panic("create: dirlink failed");

  iunlockput(dp); // Unlock and release parent directory.

  return ip; // Return the locked inode for the new file/dir.
}

// System call: open(char *path, int omode)
// Opens a file specified by `path` with given open mode `omode` (e.g., O_RDONLY, O_CREATE).
// - Fetches `path` string and `omode` integer arguments.
// - Begins a file system transaction.
// - If `O_CREATE` is specified in `omode`:
//   - Calls `create()` to create the file (as T_FILE) if it doesn't exist.
// - Else (not creating):
//   - Looks up the inode for `path` using `namei`.
//   - If it's a directory, checks if `omode` is O_RDONLY (directories can only be opened read-only).
// - Allocates a `struct file` (`f`) and a file descriptor (`fd`).
// - Initializes `f` (type, inode, offset, readable/writable flags based on `omode`).
// - Unlocks the inode (it was locked by `create` or `namei`/`ilock`).
// - Ends the transaction.
// Returns the file descriptor `fd` on success, -1 on error.
int
sys_open(void)
{
  char *path;
  int fd, omode;
  struct file *f;
  struct inode *ip;

  if(argstr(0, &path) < 0 || argint(1, &omode) < 0)
    return -1;

  begin_op();

  if(omode & O_CREATE){
    ip = create(path, T_FILE, 0, 0);
    if(ip == 0){
      end_op();
      return -1;
    }
  } else {
    if((ip = namei(path)) == 0){
      end_op();
      return -1;
    }
    ilock(ip);
    if(ip->type == T_DIR && omode != O_RDONLY){
      iunlockput(ip);
      end_op();
      return -1;
    }
  }

  if((f = filealloc()) == 0 || (fd = fdalloc(f)) < 0){
    if(f)
      fileclose(f);
    iunlockput(ip);
    end_op();
    return -1;
  }
  iunlock(ip);
  end_op();

  f->type = FD_INODE;
  f->ip = ip;
  f->off = 0;
  f->readable = !(omode & O_WRONLY);
  f->writable = (omode & O_WRONLY) || (omode & O_RDWR);
  return fd;
}

// System call: mkdir(char *path)
// Creates a new directory specified by `path`.
// - Fetches `path` string argument.
// - Begins a transaction.
// - Calls `create()` with type T_DIR to create the directory inode and its "." and ".." entries.
// - Unlocks and releases the new directory inode (returned locked by `create`).
// - Ends the transaction.
// Returns 0 on success, -1 on error.
int
sys_mkdir(void)
{
  char *path;
  struct inode *ip;

  begin_op();
  if(argstr(0, &path) < 0 || (ip = create(path, T_DIR, 0, 0)) == 0){
    end_op();
    return -1;
  }
  iunlockput(ip); // create() returns a locked inode.
  end_op();
  return 0;
}

// System call: mknod(char *path, short major, short minor)
// Creates a special device file specified by `path`.
// - Fetches `path` string, `major` device number, and `minor` device number arguments.
// - Begins a transaction.
// - Calls `create()` with type T_DEV and the given major/minor numbers.
// - Unlocks and releases the new device file inode.
// - Ends the transaction.
// Returns 0 on success, -1 on error.
int
sys_mknod(void)
{
  struct inode *ip;
  char *path;
  int major, minor;

  begin_op();
  if((argstr(0, &path)) < 0 ||
     argint(1, &major) < 0 ||
     argint(2, &minor) < 0 ||
     (ip = create(path, T_DEV, major, minor)) == 0){
    end_op();
    return -1;
  }
  iunlockput(ip); // create() returns a locked inode.
  end_op();
  return 0;
}

// System call: chdir(char *path)
// Changes the current working directory of the calling process to `path`.
// - Fetches `path` string argument.
// - Begins a transaction.
// - Looks up the inode for `path` using `namei`.
// - Locks the inode and checks if it's a directory (T_DIR).
// - Unlocks the inode.
// - Releases the old current working directory inode (`curproc->cwd`) using `iput`.
// - Ends the transaction.
// - Sets `curproc->cwd` to the new directory inode.
// Returns 0 on success, -1 on error.
int
sys_chdir(void)
{
  char *path;
  struct inode *ip;
  struct proc *curproc = myproc();
  
  begin_op();
  if(argstr(0, &path) < 0 || (ip = namei(path)) == 0){
    end_op();
    return -1;
  }
  ilock(ip);
  if(ip->type != T_DIR){
    iunlockput(ip);
    end_op();
    return -1;
  }
  iunlock(ip);
  iput(curproc->cwd);
  end_op();
  curproc->cwd = ip;
  return 0;
}

// System call: exec(char *path, char **argv)
// Replaces the current process's memory image with a new program.
// - Fetches `path` string (executable to run) and `uargv` (user pointer to array of argument strings).
// - Iteratively fetches argument strings from the user `argv` array:
//   - `fetchint` gets each `char*` pointer from `uargv`.
//   - `fetchstr` validates each argument string.
// - Calls `exec()` (from exec.c) to perform the actual replacement.
// Returns -1 on error (exec itself does not return on success).
int
sys_exec(void)
{
  char *path, *argv[MAXARG]; // Kernel array to hold pointers to user argument strings.
  int i;
  uint uargv, uarg; // User virtual addresses for argv array and individual argument strings.

  // Fetch path to executable and user pointer to argv array.
  if(argstr(0, &path) < 0 || argint(1, (int*)&uargv) < 0){
    return -1;
  }
  memset(argv, 0, sizeof(argv)); // Zero out kernel argv array.
  // Loop to fetch argument string pointers from user space.
  for(i=0;; i++){
    if(i >= NELEM(argv)) // Too many arguments.
      return -1;
    // Fetch the i-th pointer (uarg) from the user's argv array (at uargv+4*i).
    if(fetchint(uargv+4*i, (int*)&uarg) < 0)
      return -1;
    if(uarg == 0){ // Null pointer marks end of argv array.
      argv[i] = 0;
      break;
    }
    // Fetch the actual string content for argv[i] from user address uarg.
    if(fetchstr(uarg, &argv[i]) < 0)
      return -1;
  }
  return exec(path, argv); // Call the exec implementation.
}

// System call: pipe(int pipefd[2])
// Creates a pipe, a unidirectional data channel.
// - Fetches user pointer `fd` to an array of two integers where the new file descriptors will be stored.
// - Calls `pipealloc()` (from pipe.c) to allocate a pipe structure and get two `struct file`
//   pointers (`rf` for read end, `wf` for write end).
// - Allocates two file descriptors (`fd0` for read, `fd1` for write) using `fdalloc`.
// - If allocation fails at any step, cleans up previously allocated resources.
// - Stores `fd0` and `fd1` into the user-provided array `fd`.
// Returns 0 on success, -1 on error.
int
sys_pipe(void)
{
  int *fd;
  struct file *rf, *wf;
  int fd0, fd1;

  if(argptr(0, (void*)&fd, 2*sizeof(fd[0])) < 0)
    return -1;
  if(pipealloc(&rf, &wf) < 0)
    return -1;
  fd0 = -1;
  if((fd0 = fdalloc(rf)) < 0 || (fd1 = fdalloc(wf)) < 0){
    if(fd0 >= 0)
      myproc()->ofile[fd0] = 0;
    fileclose(rf);
    fileclose(wf);
    return -1;
  }
  fd[0] = fd0;
  fd[1] = fd1;
  return 0;
}
