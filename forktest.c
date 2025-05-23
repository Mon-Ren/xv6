// Test that fork fails gracefully and that wait works correctly.
// This is a tiny executable, allowing the test to potentially fill the process table
// if N is large enough, thus testing resource limits.

#include "types.h"
#include "stat.h"
#include "user.h"

#define N  1000 // Defines the number of child processes to attempt to create.

// A simple user-level printf implementation that writes to a given file descriptor.
// Used here to print messages to the console (fd 1).
void
printf(int fd, const char *s, ...)
{
  write(fd, s, strlen(s));
}

// The main fork test logic.
// This function tests several aspects of process creation and termination:
// 1. Process Creation (`fork`): It repeatedly calls `fork()` to create child processes.
//    - It checks if `fork()` returns a negative value, which indicates failure (e.g., process table full).
//    - In the child process (`pid == 0`), it calls `exit()` immediately.
// 2. Process Limits: By attempting to create `N` processes, it can test how the system
//    handles reaching the maximum number of processes (`NPROC` in kernel).
// 3. Process Termination (`exit` and `wait`):
//    - After attempting to create children, the parent process calls `wait()` to clean up
//      each child that was successfully created.
//    - It checks if `wait()` behaves as expected (e.g., returns an error if called when no
//      children are left).
void
forktest(void)
{
  int n, pid;

  printf(1, "fork test\n");

  // Loop to attempt to create N child processes.
  for(n=0; n<N; n++){
    pid = fork(); // Attempt to create a child.
    if(pid < 0)   // If fork() fails (e.g., process table is full)...
      break;      // ...stop trying to create more children.
    if(pid == 0)  // If this is the child process...
      exit();     // ...the child exits immediately.
  }

  // After the loop, 'n' holds the number of children successfully created.
  // If 'n' is equal to 'N', it means fork() never returned an error.
  // This might be unexpected if N is very large (e.g., > NPROC), as fork should fail.
  // The original comment "fork claimed to work N times!" suggests this is a check
  // for whether fork correctly reports failure when it should.
  if(n == N){
    printf(1, "fork claimed to work N times!\n", N); // This could indicate an issue if N > NPROC.
    exit(); // Parent exits.
  }

  // Parent process now waits for all 'n' successfully created children to exit.
  for(; n > 0; n--){
    if(wait() < 0){ // wait() should return the PID of an exited child. Negative means error.
      printf(1, "wait stopped early\n"); // Error: wait() failed before all children were collected.
      exit();
    }
  }

  // After all children created in the loop have been waited for,
  // another call to wait() should return -1 (or another error code),
  // indicating that there are no more children to wait for.
  if(wait() != -1){
    printf(1, "wait got too many\n"); // Error: wait() succeeded when it should have failed.
    exit();
  }

  printf(1, "fork test OK\n"); // If all tests pass.
}

// Main entry point for the forktest user program.
int
main(void)
{
  forktest(); // Run the fork test.
  exit();     // Ensure the program exits cleanly after the test.
}
