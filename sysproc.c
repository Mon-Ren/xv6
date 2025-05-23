// This file contains the kernel-side implementations of system calls
// that are primarily related to process management and control.
// These functions are invoked via the system call dispatch mechanism
// in `syscall.c` when a user program makes a system call.
// They often act as wrappers that call lower-level kernel functions
// (e.g., from `proc.c`) to perform the requested operations.

#include "types.h"
#include "x86.h"
#include "defs.h"
#include "date.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"

// System call implementation for fork().
// Calls the `fork()` function (defined in proc.c) which creates a new process
// by duplicating the calling process.
// Returns the child's PID to the parent, and 0 to the child.
// Returns -1 on failure.
int
sys_fork(void)
{
  return fork();
}

// System call implementation for exit().
// Calls the `exit()` function (defined in proc.c) which terminates the
// current process. This function does not return to the user program.
// The `return 0;` is never reached.
int
sys_exit(void)
{
  exit();
  return 0;  // not reached
}

// System call implementation for wait().
// Calls the `wait()` function (defined in proc.c) which allows a parent
// process to wait for one of its child processes to exit and retrieve its PID.
// Returns the PID of the exited child, or -1 if the caller has no children
// or if other error conditions occur.
int
sys_wait(void)
{
  return wait();
}

// System call implementation for kill().
// Fetches the PID argument from the user stack using `argint`.
// Calls the `kill()` function (defined in proc.c) to send a signal
// (effectively, mark for termination) to the process with the specified PID.
// Returns 0 on success, -1 on failure (e.g., PID not found).
int
sys_kill(void)
{
  int pid;

  if(argint(0, &pid) < 0) // Fetch the first argument (PID).
    return -1;
  return kill(pid);
}

// System call implementation for getpid().
// Returns the process ID (PID) of the current process.
// `myproc()` (defined in proc.c) returns a pointer to the current process's
// `struct proc`, from which `pid` is accessed.
int
sys_getpid(void)
{
  return myproc()->pid;
}

// System call implementation for sbrk().
// Used by user programs to change their data segment size (heap).
// Fetches the integer argument `n` (number of bytes to grow/shrink by)
// from the user stack.
// Calls `growproc()` (defined in proc.c) to adjust the process's memory size.
// Returns the old size of the process's memory (the address of the previous
// "break") on success, or -1 on failure.
int
sys_sbrk(void)
{
  int addr;
  int n;

  if(argint(0, &n) < 0) // Fetch the first argument (size increment).
    return -1;
  addr = myproc()->sz;   // Current size of the process.
  if(growproc(n) < 0)    // Attempt to grow/shrink memory.
    return -1;
  return addr;           // Return old size.
}

// System call implementation for sleep().
// Pauses the current process for a specified number of system clock ticks.
// Fetches the integer argument `n` (number of ticks to sleep) from user stack.
// It acquires `tickslock` to safely read `ticks` and call `sleep()` on the
// `&ticks` channel. The process will be awakened by clock interrupts.
// If the process is killed while sleeping, it returns -1.
// Otherwise, returns 0 after sleeping for at least `n` ticks.
int
sys_sleep(void)
{
  int n;
  uint ticks0;

  if(argint(0, &n) < 0) // Fetch the first argument (number of ticks).
    return -1;
  acquire(&tickslock);   // Protect access to global `ticks` and sleep channel.
  ticks0 = ticks;        // Record current ticks.
  while(ticks - ticks0 < n){ // Loop until desired ticks have passed.
    if(myproc()->killed){  // Check if process was killed during sleep.
      release(&tickslock);
      return -1;
    }
    sleep(&ticks, &tickslock); // Sleep on the `ticks` channel, releasing `tickslock` atomically.
  }
  release(&tickslock);   // Release lock after waking up.
  return 0;
}

// return how many clock tick interrupts have occurred
// since system startup.
int
sys_uptime(void)
{
  uint xticks;

  acquire(&tickslock);
  xticks = ticks;
  release(&tickslock);
  return xticks;
}
