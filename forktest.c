// 测试 fork 是否能优雅地失败以及 wait 是否正常工作。
// 这是一个微小的可执行文件，如果N足够大，测试可能会填满进程表，
// 从而测试资源限制。

#include "types.h"
#include "stat.h"
#include "user.h"

#define N  1000 // 定义尝试创建的子进程数量。

// 一个简单的用户级printf实现，用于向给定的文件描述符写入。
// 此处用于向控制台(fd 1)打印消息。
void
printf(int fd, const char *s, ...)
{
  write(fd, s, strlen(s));
}

// fork测试的主要逻辑。
// 此函数测试进程创建和终止的几个方面：
// 1. 进程创建 (`fork`): 它重复调用 `fork()` 来创建子进程。
//    - 它检查 `fork()` 是否返回负值，这表示失败（例如，进程表已满）。
//    - 在子进程中 (`pid == 0`)，它立即调用 `exit()`。
// 2. 进程限制: 通过尝试创建 `N` 个进程，它可以测试系统如何处理达到最大进程数 (`NPROC` 内核限制) 的情况。
// 3. 进程终止 (`exit` 和 `wait`):
//    - 在尝试创建子进程后，父进程调用 `wait()` 来清理每个成功创建的子进程。
//    - 它检查 `wait()` 是否按预期工作（例如，当没有子进程时调用是否返回错误）。
void
forktest(void)
{
  int n, pid;

  printf(1, "fork test\n");

  // 尝试创建N个子进程的循环。
  for(n=0; n<N; n++){
    pid = fork(); // 尝试创建一个子进程。
    if(pid < 0)   // 如果fork()失败（例如，进程表已满）...
      break;      // ...停止尝试创建更多子进程。
    if(pid == 0)  // 如果这是子进程...
      exit();     // ...子进程立即退出。
  }

  // 循环之后，'n' 保存成功创建的子进程数量。
  // 如果 'n' 等于 'N'，则意味着fork()从未返回错误。
  // 如果N非常大（例如 > NPROC），这可能出乎意料，因为fork应该失败。
  // 原始注释 "fork claimed to work N times!" 表明这是一个检查
  // fork 是否在应该失败时正确报告失败。
  if(n == N){
    printf(1, "fork claimed to work N times!\n", N); // 如果 N > NPROC，这可能表明存在问题。
    exit(); // 父进程退出。
  }

  // 父进程现在等待所有 'n' 个成功创建的子进程退出。
  for(; n > 0; n--){
    if(wait() < 0){ // wait() 应该返回已退出子进程的PID。负值表示错误。
      printf(1, "wait stopped early\n"); // 错误：wait() 在所有子进程被回收之前失败。
      exit();
    }
  }

  // 在等待循环中创建的所有子进程之后，
  // 再次调用wait()应该返回-1（或其他错误代码），
  // 表示没有更多子进程可等待。
  if(wait() != -1){
    printf(1, "wait got too many\n"); // 错误：wait() 在应该失败时成功了。
    exit();
  }

  printf(1, "fork test OK\n"); // 如果所有测试都通过。
}

// forktest 用户程序的主入口点。
int
main(void)
{
  forktest(); // 运行fork测试。
  exit();     // 确保程序在测试后干净地退出。
}
