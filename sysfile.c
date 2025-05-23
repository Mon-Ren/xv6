// 本文件实现了与文件和文件系统操作相关的系统调用的内核端。
//
// 这些函数通常执行以下步骤：
// 1. 参数获取和验证：
//    - 使用像 `argint`、`argstr`、`argptr`（来自 syscall.c）这样的辅助函数
//      从用户进程的栈和内存中检索参数（例如，文件描述符、路径、指针、大小）。
//    - 验证这些参数（例如，检查文件描述符是否有效，
//      指针是否在用户空间内，路径是否合理）。
// 2. 调用底层文件系统函数：
//    - 调用 `file.c` 中的函数（用于文件描述符和打开文件表操作，
//      如 `filealloc`、`fileclose`、`fileread`、`filewrite`）
//      和 `fs.c` 中的函数（用于 inode 和块级操作，如 `namei`、
//      `dirlink`、`create`、`readi`、`writei`）。
// 3. 事务管理：
//    - 许多修改磁盘结构的文件系统操作（例如，创建文件、链接、取消链接）
//      都包装在 `begin_op()` 和 `end_op()` 中，
//      以通过日志系统（log.c）确保原子性和崩溃恢复。
// 4. 返回值：
//    -向用户返回适当的值（例如，读取/写入的字节数、新的文件描述符、
//      成功时返回0，或错误时返回-1）。

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

// 获取第n个字大小的系统调用参数，该参数应为一个文件描述符(fd)。
// - `n`: 参数编号（从0开始）。
// - `pfd`: 如果不为null，则fd的整数值存储在此处。
// - `pf`: 如果不为null，则指向当前进程打开文件表 (`myproc()->ofile[fd]`) 中相应 `struct file` 的指针存储在此处。
// 成功返回0。
// 如果出现以下情况，则返回-1：
//   - `argint` 获取fd值失败。
//   - fd 超出范围（小于0或大于等于NOFILE）。
//   - 进程没有为该fd打开文件 (`myproc()->ofile[fd] == 0`)。
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

// 在当前进程的打开文件表 (`myproc()->ofile`) 中分配一个未使用的文件描述符，
// 并将给定的 `struct file *f` 分配给它。
// 此函数有效地为进程提供了一个已打开文件结构 `f` 的句柄 (fd)。
// 如果是对 `struct file *f` 的新引用（例如，在 `sys_open` 中），或者它正在被复制 (`sys_dup`)，
// 则调用者应已增加其引用计数。
// `fdalloc` 本身不修改 `f->ref`。
// 成功时返回分配的文件描述符（一个整数）。
// 如果进程表中没有可用的空闲文件描述符，则返回-1。
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

// 系统调用：dup(int fd)
// 复制一个现有的文件描述符。
// - 使用 `argfd` 获取文件描述符参数。
// - 使用 `fdalloc` 分配一个新的文件描述符。
// - 使用 `filedup` 增加底层 `struct file` 的引用计数。
// 成功时返回新的文件描述符，错误时返回-1。
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

// 系统调用：read(int fd, char *buf, int n)
// 从文件描述符 `fd` 读取最多 `n` 字节到缓冲区 `buf`。
// - 获取参数：fd、用户缓冲区指针 `p` 和计数 `n`。
// - 调用 `fileread` (来自 file.c) 执行读取操作。
// 返回读取的字节数，错误时返回-1。
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

// 系统调用：write(int fd, char *buf, int n)
// 将缓冲区 `buf` 中的 `n` 字节写入文件描述符 `fd`。
// - 获取参数：fd、用户缓冲区指针 `p` 和计数 `n`。
// - 调用 `filewrite` (来自 file.c) 执行写入操作。
// 返回写入的字节数，错误时返回-1。
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

// 系统调用：close(int fd)
// 关闭一个文件描述符。
// - 获取文件描述符 `fd` 及其 `struct file *f`。
// - 从当前进程的打开文件表中移除 fd (`myproc()->ofile[fd] = 0`)。
// - 调用 `fileclose` (来自 file.c) 以减少 `struct file` 的引用计数，
//   如果引用计数达到零，则可能关闭底层文件/管道。
// 成功返回0，错误返回-1。
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

// 系统调用：fstat(int fd, struct stat *st)
// 获取有关打开文件的状态信息（元数据）。
// - 获取文件描述符 `fd` 及其 `struct file *f`。
// - 获取一个用户指针 `st`，指向用于存储结果的 `struct stat`。
// - 调用 `filestat` (来自 file.c) 来填充 `struct stat`。
// 成功返回0，错误返回-1。
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

// 系统调用：link(char *oldname, char *newname)
// 创建一个新的硬链接 `newname`，指向与 `oldname` 相同的 inode。
// - 获取字符串参数 `old` 和 `new` 路径。
// - 开始文件系统事务 (`begin_op`)。
// - 使用 `namei` 查找 `oldname` 的 inode。
// - 检查 `oldname` 不是目录（不能硬链接目录）。
// - 增加 inode 的 `ip->nlink` 并在磁盘上更新它 (`iupdate`)。
// - 使用 `nameiparent` 查找 `newname` 的父目录 inode。
// - 调用 `dirlink` 为指向 `ip->inum` 的 `name` 创建新的目录条目。
// - 释放 inode 并结束事务。
// - 如果步骤失败，通过减少 `nlink` 来处理错误情况。
// 成功返回0，错误返回-1。
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

// 目录 dp 是否为空（除了 "." 和 ".."）？
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
// 系统调用：unlink(char *pathname)
// 从文件系统中移除一个名称（链接）。
// 如果这是指向 inode 的最后一个链接，则释放该 inode 及其数据。
// - 获取 `path` 参数。
// - 开始文件系统事务。
// - 查找 `path` 的父目录 `dp` 和最终名称组件。
// - 检查 `name` 不是 "." 或 ".."。
// - 在 `dp` 中查找 `name` 的 inode `ip`。
// - 如果 `ip` 是目录，则检查它是否为空（除了 "." 和 ".."）。
// - 通过写入零来清除 `dp` 中 `name` 的目录条目。
// - 如果 `ip` 是目录，则减少 `dp->nlink`（用于子目录中的 ".." 条目）。
// - 减少 `ip->nlink`。
// - 更新磁盘上的 inode 并释放它们。如果 `nlink` 为0，`iput` 将处理释放。
// - 结束事务。
// 成功返回0，错误返回-1。
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

// 辅助函数：创建一个新的文件、目录或设备文件。
// - `path`: 新文件/目录的完整路径。
// - `type`: T_FILE, T_DIR, 或 T_DEV。
// - `major`, `minor`: 如果类型是 T_DEV，则为设备号。
//
// 操作：
// 1. 找到 `path` 的父目录 `dp` 和最终名称组件。
// 2. 锁定 `dp`。
// 3. 检查 `name` 是否已存在：
//    - 如果它存在并且是一个文件，并且 `type` 是 T_FILE，则返回现有 inode (用于 O_CREATE)。
//    - 否则（存在但类型错误，或正在创建的目录/设备已存在），失败。
// 4. 如果它不存在，则使用 `ialloc` 分配一个给定 `type` 的新 inode `ip`。
// 5. 锁定 `ip`，设置其主/次设备号（如果为 T_DEV），nlink=1，并在磁盘上更新它。
// 6. 如果 `type` 是 T_DIR：
//    - 增加 `dp->nlink`（用于新目录中的 ".." 条目）。
//    - 更新磁盘上的 `dp`。
//    - 在新目录 `ip` 中创建 "." 和 ".." 条目。
// 7. 将新的 inode `ip` 链接到父目录 `dp` 下的 `name`。
// 8. 解锁并释放 `dp`。
// 成功时返回锁定的 inode `ip`，失败时返回0。
// 调用者负责解锁 `ip`（例如，如果是中间步骤，则通过 `iunlockput`；
// 或者如果返回的 inode 将由 `sys_open` 进一步使用，则只需 `iunlock`）。
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

// 系统调用：open(char *path, int omode)
// 使用给定的打开模式 `omode`（例如，O_RDONLY, O_CREATE）打开由 `path` 指定的文件。
// - 获取 `path` 字符串和 `omode` 整数参数。
// - 开始文件系统事务。
// - 如果在 `omode` 中指定了 `O_CREATE`：
//   - 如果文件不存在，则调用 `create()` 创建文件（类型为 T_FILE）。
// - 否则（不创建）：
//   - 使用 `namei` 查找 `path` 的 inode。
//   - 如果是目录，则检查 `omode` 是否为 O_RDONLY（目录只能以只读方式打开）。
// - 分配一个 `struct file` (`f`) 和一个文件描述符 (`fd`)。
// - 初始化 `f`（类型、inode、偏移量、基于 `omode` 的可读/可写标志）。
// - 解锁 inode（它被 `create` 或 `namei`/`ilock` 锁定）。
// - 结束事务。
// 成功时返回文件描述符 `fd`，错误时返回-1。
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

// 系统调用：mkdir(char *path)
// 创建由 `path` 指定的新目录。
// - 获取 `path` 字符串参数。
// - 开始一个事务。
// - 调用 `create()` 并指定类型 T_DIR 来创建目录 inode 及其 "." 和 ".." 条目。
// - 解锁并释放新的目录 inode（由 `create` 返回时是锁定的）。
// - 结束事务。
// 成功返回0，错误返回-1。
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

// 系统调用：mknod(char *path, short major, short minor)
// 创建由 `path` 指定的特殊设备文件。
// - 获取 `path` 字符串、主设备号 `major` 和次设备号 `minor` 参数。
// - 开始一个事务。
// - 调用 `create()` 并指定类型 T_DEV 以及给定的主/次设备号。
// - 解锁并释放新的设备文件 inode。
// - 结束事务。
// 成功返回0，错误返回-1。
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

// 系统调用：chdir(char *path)
// 将调用进程的当前工作目录更改为 `path`。
// - 获取 `path` 字符串参数。
// - 开始一个事务。
// - 使用 `namei` 查找 `path` 的 inode。
// - 锁定 inode 并检查它是否是目录 (T_DIR)。
// - 解锁 inode。
// - 使用 `iput` 释放旧的当前工作目录 inode (`curproc->cwd`)。
// - 结束事务。
// - 将 `curproc->cwd` 设置为新的目录 inode。
// 成功返回0，错误返回-1。
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

// 系统调用：exec(char *path, char **argv)
// 用新程序替换当前进程的内存映像。
// - 获取 `path` 字符串（要运行的可执行文件）和 `uargv`（指向参数字符串数组的用户指针）。
// - 从用户 `argv` 数组中迭代获取参数字符串：
//   - `fetchint` 从 `uargv` 获取每个 `char*` 指针。
//   - `fetchstr` 验证每个参数字符串。
// - 调用 `exec()` (来自 exec.c) 来执行实际的替换。
// 错误时返回-1（exec 本身成功时不返回）。
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

// 系统调用：pipe(int pipefd[2])
// 创建一个管道，一个单向数据通道。
// - 获取用户指针 `fd`，它指向一个包含两个整数的数组，新的文件描述符将存储在此处。
// - 调用 `pipealloc()` (来自 pipe.c) 来分配一个管道结构并获取两个 `struct file` 指针
//   （`rf` 用于读取端，`wf` 用于写入端）。
// - 使用 `fdalloc` 分配两个文件描述符（`fd0` 用于读取，`fd1` 用于写入）。
// - 如果在任何步骤分配失败，则清理先前分配的资源。
// - 将 `fd0` 和 `fd1` 存储到用户提供的数组 `fd` 中。
// 成功返回0，错误返回-1。
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
