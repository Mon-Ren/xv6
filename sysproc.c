// 本文件包含主要与进程管理和控制相关的系统调用的内核端实现。
// 当用户程序进行系统调用时，这些函数通过 `syscall.c` 中的系统调用分发机制被调用。
// 它们通常充当包装器，调用更低级的内核函数（例如，来自 `proc.c`）以执行所请求的操作。

#include "types.h"
#include "x86.h"
#include "defs.h"
#include "date.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"

// fork() 的系统调用实现。
// 调用 `fork()` 函数（在 proc.c 中定义），该函数通过复制调用进程来创建一个新进程。
// 向父进程返回子进程的PID，向子进程返回0。
// 失败时返回-1。
int
sys_fork(void)
{
  return fork();
}

// exit() 的系统调用实现。
// 调用 `exit()` 函数（在 proc.c 中定义），该函数终止当前进程。
// 此函数不会返回到用户程序。
// `return 0;` 永远不会执行到。
int
sys_exit(void)
{
  exit();
  return 0;  // not reached
}

// wait() 的系统调用实现。
// 调用 `wait()` 函数（在 proc.c 中定义），该函数允许父进程等待其某个子进程退出并检索其PID。
// 返回已退出子进程的PID，如果调用者没有子进程或发生其他错误情况，则返回-1。
int
sys_wait(void)
{
  return wait();
}

// kill() 的系统调用实现。
// 使用 `argint` 从用户栈获取PID参数。
// 调用 `kill()` 函数（在 proc.c 中定义）向具有指定PID的进程发送信号（实际上是标记为终止）。
// 成功返回0，失败返回-1（例如，未找到PID）。
int
sys_kill(void)
{
  int pid;

  if(argint(0, &pid) < 0) // Fetch the first argument (PID).
    return -1;
  return kill(pid);
}

// getpid() 的系统调用实现。
// 返回当前进程的进程ID (PID)。
// `myproc()`（在 proc.c 中定义）返回一个指向当前进程 `struct proc` 的指针，
// 从中可以访问 `pid`。
int
sys_getpid(void)
{
  return myproc()->pid;
}

// sbrk() 的系统调用实现。
// 用户程序使用此调用来更改其数据段大小（堆）。
// 从用户栈获取整数参数 `n`（增加/减少的字节数）。
// 调用 `growproc()`（在 proc.c 中定义）来调整进程的内存大小。
// 成功时返回进程内存的旧大小（前一个“中断点”的地址），失败时返回-1。
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

// sleep() 的系统调用实现。
// 将当前进程暂停指定的系统时钟滴答数。
// 从用户栈获取整数参数 `n`（要睡眠的滴答数）。
// 它获取 `tickslock` 以安全地读取 `ticks` 并在 `&ticks` 通道上调用 `sleep()`。
// 进程将由时钟中断唤醒。
// 如果进程在睡眠时被杀死，则返回-1。
// 否则，在睡眠至少 `n` 个滴答后返回0。
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

// 返回自系统启动以来已发生的时钟滴答中断次数。
int
sys_uptime(void)
{
  uint xticks;

  acquire(&tickslock);
  xticks = ticks;
  release(&tickslock);
  return xticks;
}
