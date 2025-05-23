#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "x86.h"
#include "syscall.h"

// 本文件实现了 xv6 的系统调用分发机制。
//
// 系统调用过程概述：
// 1. 用户级代码（例如，像 `read()` 这样的C库函数）设置系统调用的参数，
//    将系统调用号（在 syscall.h 中定义，例如 SYS_read）放入 %eax 寄存器，
//    并执行 `INT T_SYSCALL`（中断 0x40）指令。
// 2. 此中断导致陷入内核模式。CPU 将用户寄存器（包括 %esp, %eip）
//    保存到内核栈上（`struct trapframe` 的一部分）。
//    陷阱处理程序（trap.c 中的 `trap()`）将其识别为系统调用。
// 3. `trap()` 调用 `syscall()`（本文件）来处理系统调用。
// 4. `syscall()` 从 `curproc->tf->eax` 检索系统调用号。
// 5. 此编号用作 `syscalls[]` 数组的索引，该数组是一个函数指针数组，
//    每个指针指向一个特定的系统调用实现（例如 `sys_read`, `sys_fork`）。
// 6. 调用相应的 `sys_xxx` 函数。这些函数通常使用辅助函数
//    （`argint`, `argptr`, `argstr`）从用户进程的栈和地址空间中
//    安全地获取参数。
// 7. `sys_xxx` 函数执行系统调用逻辑。
// 8. `sys_xxx` 函数的返回值存储回 `curproc->tf->eax`，
//    在返回用户模式时，该值将恢复到 %eax。
// 9. 控制权返回给 `trap()`，然后安排返回用户模式。

// 从当前进程的用户虚拟地址 `addr` 获取32位整数。
// 将获取的整数存储到 `*ip` 中。
// 成功返回0，失败返回-1（例如，如果 `addr` 无效或
// 跨越进程用户内存的末尾）。
// 它在验证后直接解引用地址，因为当进程的页表激活时，
// 内核对用户内存共享相同的地址空间视图。
int
fetchint(uint addr, int *ip)
{
  struct proc *curproc = myproc();

  if(addr >= curproc->sz || addr+4 > curproc->sz)
    return -1;
  *ip = *(int*)(addr);
  return 0;
}

// 从当前进程的用户虚拟地址 `addr` 获取以nul结尾的字符串。
// 此函数不复制字符串数据本身。相反，它验证字符串是否存在于
// 用户进程的地址空间内，并将 `*pp` 设置为直接指向用户内存中的字符串。
// 成功则返回字符串的长度（不包括nul终止符）。
// 失败则返回-1（例如，如果 `addr` 无效，字符串超出边界，
// 或在进程内存中未找到nul终止符）。
int
fetchstr(uint addr, char **pp)
{
  char *s, *ep;
  struct proc *curproc = myproc();

  if(addr >= curproc->sz)
    return -1;
  *pp = (char*)addr;
  ep = (char*)curproc->sz;
  for(s = *pp; s < ep; s++){
    if(*s == 0)
      return s - *pp;
  }
  return -1;
}

// 从用户栈获取第n个32位整型系统调用参数。
// 系统调用参数在 `INT T_SYSCALL` 指令之前由用户级库桩函数压入用户栈。
// `myproc()->tf->esp` 指向保存的用户栈指针，该指针位于保存的用户EIP之上。
// 第一个参数位于 `tf->esp + 4`，第二个位于 `tf->esp + 8`，依此类推。
// 将获取的整数存储到 `*ip` 中。
// 成功返回0，失败返回-1（委托给 `fetchint`）。
int
argint(int n, int *ip)
{
  return fetchint((myproc()->tf->esp) + 4 + 4*n, ip);
}

// 获取第n个字大小的系统调用参数，该参数应为用户指针。
// 此函数验证指针 `*pp` 以及从 `*pp` 开始的 `size` 字节内存区域
// 完全位于当前进程的用户地址空间内。
// 将 `*pp` 设置为已验证的用户地址（作为char指针）。
// 成功返回0，失败返回-1（例如，如果无法获取参数，
// 指针无效，或内存区域超出边界）。
int
argptr(int n, char **pp, int size)
{
  int i;
  struct proc *curproc = myproc();
 
  if(argint(n, &i) < 0)
    return -1;
  if(size < 0 || (uint)i >= curproc->sz || (uint)i+size > curproc->sz)
    return -1;
  *pp = (char*)i;
  return 0;
}

// 获取第n个字大小的系统调用参数，该参数应为指向用户内存中以nul结尾的字符串的指针。
// 它首先使用 `argint` 获取指针值，然后使用 `fetchstr` 验证字符串并获取指向它的指针。
// 将 `*pp` 设置为指向用户内存中的字符串。
// 成功则返回字符串的长度，失败则返回-1。
// 关于没有共享可写内存的注释意味着内核不需要担心在系统调用期间
// 由于另一个线程/进程导致字符串内容发生更改，因为xv6进程具有私有地址空间。
int
argstr(int n, char **pp)
{
  int addr;
  if(argint(n, &addr) < 0)
    return -1;
  return fetchstr(addr, pp);
}

extern int sys_chdir(void);
extern int sys_close(void);
extern int sys_dup(void);
extern int sys_exec(void);
extern int sys_exit(void);
extern int sys_fork(void);
extern int sys_fstat(void);
extern int sys_getpid(void);
extern int sys_kill(void);
extern int sys_link(void);
extern int sys_mkdir(void);
extern int sys_mknod(void);
extern int sys_open(void);
extern int sys_pipe(void);
extern int sys_read(void);
extern int sys_sbrk(void);
extern int sys_sleep(void);
extern int sys_unlink(void);
extern int sys_wait(void);
extern int sys_write(void);
extern int sys_uptime(void);

static int (*syscalls[])(void) = {
[SYS_fork]    sys_fork,
[SYS_exit]    sys_exit,
[SYS_wait]    sys_wait,
[SYS_pipe]    sys_pipe,
[SYS_read]    sys_read,
[SYS_kill]    sys_kill,
[SYS_exec]    sys_exec,
[SYS_fstat]   sys_fstat,
[SYS_chdir]   sys_chdir,
[SYS_dup]     sys_dup,
[SYS_getpid]  sys_getpid,
[SYS_sbrk]    sys_sbrk,
[SYS_sleep]   sys_sleep,
[SYS_uptime]  sys_uptime,
[SYS_open]    sys_open,
[SYS_write]   sys_write,
[SYS_mknod]   sys_mknod,
[SYS_unlink]  sys_unlink,
[SYS_link]    sys_link,
[SYS_mkdir]   sys_mkdir,
[SYS_close]   sys_close,
};
// 此数组将系统调用号（例如 SYS_fork）映射到其对应的内核实现函数（例如 sys_fork）。
// 从用户空间在 %eax 中传递的系统调用号用作此数组的索引。

// 主系统调用处理程序。
// 当发生 INT T_SYSCALL 时，`trap()` 会调用此函数。
// 1. 它从当前进程的陷阱帧 (`curproc->tf->eax`) 中检索系统调用号。
// 2. 它检查编号是否有效以及 `syscalls` 数组中是否存在处理程序。
// 3. 如果有效，它调用处理函数 (`syscalls[num]()`)。
// 4. 处理程序的返回值存储回 `curproc->tf->eax`，
//    这将是用户程序看到的返回值。
// 5. 如果系统调用号无效或不存在处理程序，则打印错误
//    并将返回值设置为-1。
void
syscall(void)
{
  int num;
  struct proc *curproc = myproc();

  num = curproc->tf->eax;
  if(num > 0 && num < NELEM(syscalls) && syscalls[num]) {
    curproc->tf->eax = syscalls[num]();
  } else {
    cprintf("%d %s: unknown sys call %d\n",
            curproc->pid, curproc->name, num);
    curproc->tf->eax = -1;
  }
}
