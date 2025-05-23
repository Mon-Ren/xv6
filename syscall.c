#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "x86.h"
#include "syscall.h"

// This file implements the system call dispatch mechanism for xv6.
//
// System Call Process Overview:
// 1. User-level code (e.g., a C library function like `read()`) sets up
//    arguments for the system call, places the system call number (defined in
//    syscall.h, e.g., SYS_read) into the %eax register, and executes an
//    `INT T_SYSCALL` (interrupt 0x40) instruction.
// 2. This interrupt causes a trap into kernel mode. The CPU saves user registers
//    (including %esp, %eip) onto the kernel stack (part of `struct trapframe`).
//    The trap handler (`trap()` in trap.c) identifies it as a system call.
// 3. `trap()` calls `syscall()` (this file) to handle the system call.
// 4. `syscall()` retrieves the system call number from `curproc->tf->eax`.
// 5. This number is used as an index into the `syscalls[]` array, which is an
//    array of function pointers, each pointing to a specific system call
//    implementation (e.g., `sys_read`, `sys_fork`).
// 6. The corresponding `sys_xxx` function is called. These functions typically
//    use helper functions (`argint`, `argptr`, `argstr`) to fetch arguments
//    safely from the user process's stack and address space.
// 7. The `sys_xxx` function executes the system call logic.
// 8. The return value of the `sys_xxx` function is stored back into
//    `curproc->tf->eax`, which will be restored to %eax when returning to user mode.
// 9. Control returns to `trap()`, which then arranges to return to user mode.

// Fetch the 32-bit integer at user virtual address `addr` from the current process.
// Stores the fetched integer into `*ip`.
// Returns 0 on success, -1 on failure (e.g., if `addr` is invalid or
// spans across the end of the process's user memory).
// It directly dereferences the address after validation because the kernel
// shares the same address space view for user memory when the process's
// page table is active.
int
fetchint(uint addr, int *ip)
{
  struct proc *curproc = myproc();

  if(addr >= curproc->sz || addr+4 > curproc->sz)
    return -1;
  *ip = *(int*)(addr);
  return 0;
}

// Fetch the nul-terminated string at user virtual address `addr` from the current process.
// This function does not copy the string data itself. Instead, it validates
// that the string exists within the user process's address space and sets `*pp`
// to point directly to the string in user memory.
// Returns the length of the string (excluding the nul terminator) on success.
// Returns -1 on failure (e.g., if `addr` is invalid, string goes out of bounds,
// or no nul terminator is found within the process's memory).
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

// Fetch the n-th 32-bit integer system call argument from the user stack.
// System call arguments are pushed onto the user stack by the user-level
// library stub before the `INT T_SYSCALL` instruction.
// `myproc()->tf->esp` points to the saved user stack pointer, which is
// just above the saved user EIP. The first argument is at `tf->esp + 4`,
// the second at `tf->esp + 8`, and so on.
// Stores the fetched integer into `*ip`.
// Returns 0 on success, -1 on failure (delegated to `fetchint`).
int
argint(int n, int *ip)
{
  return fetchint((myproc()->tf->esp) + 4 + 4*n, ip);
}

// Fetch the n-th word-sized system call argument, which is expected to be a user pointer.
// This function validates that the pointer `*pp` and the memory region of `size` bytes
// starting at `*pp` lie entirely within the current process's user address space.
// Sets `*pp` to the validated user address (as a char pointer).
// Returns 0 on success, -1 on failure (e.g., if the argument cannot be fetched,
// the pointer is invalid, or the memory region is out of bounds).
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

// Fetch the n-th word-sized system call argument, which is expected to be a pointer
// to a nul-terminated string in user memory.
// It first fetches the pointer value using `argint`, then uses `fetchstr`
// to validate the string and get a pointer to it.
// Sets `*pp` to point to the string in user memory.
// Returns the length of the string on success, -1 on failure.
// The comment about no shared writable memory implies that the kernel doesn't
// need to worry about the string content changing due to another thread/process
// during the system call, as xv6 processes have private address spaces.
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
// This array maps system call numbers (e.g., SYS_fork) to their
// corresponding kernel implementation functions (e.g., sys_fork).
// The system call number passed in %eax from user space is used as an
// index into this array.

// The main system call handler.
// This function is called from `trap()` when an INT T_SYSCALL occurs.
// 1. It retrieves the system call number from the current process's trap frame (`curproc->tf->eax`).
// 2. It checks if the number is valid and if a handler exists in the `syscalls` array.
// 3. If valid, it calls the handler function (`syscalls[num]()`).
// 4. The return value of the handler is stored back in `curproc->tf->eax`,
//    which will be the return value seen by the user program.
// 5. If the system call number is invalid or no handler exists, it prints an error
//    and sets the return value to -1.
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
