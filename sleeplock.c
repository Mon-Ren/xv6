// This file implements sleeping locks (also known as mutexes or blocking locks).
//
// Purpose of Sleep-Locks:
// Sleep-locks provide mutual exclusion for longer critical sections, where it would
// be inefficient for a CPU to spin (busy-wait) if the lock is unavailable.
// Instead of spinning, a process attempting to acquire a held sleep-lock will
// block (go to sleep), allowing the CPU to schedule and run other processes.
// When the lock is released, one of the waiting processes is awakened.
//
// Comparison with Spinlocks:
// - Spinlocks: Best for very short critical sections where the lock is expected
//   to be held briefly. The overhead of context switching a process might be
//   greater than the time spent spinning. Interrupts must be disabled on the
//   CPU holding the spinlock.
// - Sleep-locks: Suitable for longer critical sections, or when the lock holder
//   might itself need to sleep (e.g., waiting for I/O). Processes block instead
//   of spinning, which is more CPU-efficient if waits are potentially long.
//   Interrupts do *not* need to be disabled by the caller around a sleep-lock's
//   critical section (though the sleep-lock implementation itself uses a spinlock
//   internally for short critical moments, during which interrupts are disabled).
//
// `struct sleeplock` fields:
// - `locked`: An unsigned integer, 0 if the lock is free, 1 if held.
// - `lk`: A `struct spinlock`. This internal spinlock is crucial. It protects
//         the fields of the `sleeplock` itself (like `locked` and `pid`) during
//         the acquire and release operations. This is necessary because multiple
//         processes might try to acquire or release the same sleep-lock concurrently,
//         and these operations need to be atomic with respect to the sleeplock's state.
// - `name`: A character string name for debugging purposes.
// - `pid`: The process ID (PID) of the process currently holding the sleep-lock.
//          Used for debugging and to check if the current process holds the lock.

#include "types.h"
#include "defs.h"
#include "param.h"
#include "x86.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "spinlock.h"
#include "sleeplock.h"

// Initialize a sleep-lock.
// - `lk`: Pointer to the `struct sleeplock` to initialize.
// - `name`: A descriptive name for the lock (for debugging).
// Initializes the internal spinlock `lk->lk` (which protects the sleeplock's state)
// and sets the sleeplock itself to be initially unlocked (`locked = 0`), with no owning PID.
void
initsleeplock(struct sleeplock *lk, char *name)
{
  initlock(&lk->lk, "sleep lock"); // Initialize the internal spinlock.
  lk->name = name;
  lk->locked = 0; // Sleeplock is initially free.
  lk->pid = 0;    // No process owns it initially.
}

// Acquire the sleep-lock `lk`.
// If the lock is currently held by another process, the calling process will sleep
// (block) until the lock is released.
//
// Steps:
// 1. Acquire Internal Spinlock: `acquire(&lk->lk)` is called to acquire the
//    spinlock that protects the `sleeplock`'s fields (`locked`, `pid`). This ensures
//    that checking `lk->locked` and potentially going to sleep is an atomic operation
//    with respect to other processes trying to acquire or release this sleep-lock.
// 2. Check if Locked: `while (lk->locked)` loop:
//    - If `lk->locked` is true (the sleep-lock is held by another process):
//      - `sleep(lk, &lk->lk)`: The current process is put to sleep.
//        - `sleep()` (from proc.c) is called with `lk` (address of the sleeplock)
//          as the sleep channel and `&lk->lk` (address of the internal spinlock)
//          as the lock to be released atomically during sleep.
//        - `sleep()` will atomically release `lk->lk` and put the current process
//          into the SLEEPING state on the channel `lk`.
//        - When another process calls `releasesleep()`, it will call `wakeup(lk)`,
//          which will make this sleeping process RUNNABLE.
//        - Upon returning from `sleep()`, `lk->lk` will have been reacquired by `sleep()`.
//          The loop condition `lk->locked` is then re-evaluated.
// 3. Acquire Sleep-Lock: Once `lk->locked` is false (the sleep-lock is free),
//    the loop terminates. The current process now "claims" the sleep-lock:
//    - `lk->locked = 1;`
//    - `lk->pid = myproc()->pid;` (records the PID of the new owner).
// 4. Release Internal Spinlock: `release(&lk->lk)` releases the spinlock, allowing
//    other processes to attempt to acquire/release this or other sleep-locks.
//
// Note: Interrupts are managed by the internal spinlock `lk->lk` (via its
// `acquire`/`release` which use `pushcli`/`popcli`). The caller of `acquiresleep`
// does not need to disable interrupts separately for the sleep-lock itself,
// as sleeping inherently allows other interrupts and processes to run.
void
acquiresleep(struct sleeplock *lk)
{
  acquire(&lk->lk); // Acquire internal spinlock to protect sleeplock state.
  while (lk->locked) { // Loop while the sleeplock is held by someone else.
    // Atomically release lk->lk and sleep on channel 'lk'.
    // sleep() will reacquire lk->lk before returning.
    sleep(lk, &lk->lk);
  }
  // Lock is now free. Acquire it.
  lk->locked = 1;
  lk->pid = myproc()->pid; // Record current process PID as owner.
  release(&lk->lk); // Release internal spinlock.
}

// Release the sleep-lock `lk`.
// If any processes are sleeping waiting for this lock, one of them will be awakened.
//
// Steps:
// 1. Acquire Internal Spinlock: `acquire(&lk->lk)` protects access to `lk->locked`, `lk->pid`.
//    It's important to ensure that only the holder of the sleep-lock (or kernel code
//    under specific circumstances) releases it, though this function doesn't explicitly
//    check `lk->pid == myproc()->pid` (caller should ensure correctness, or `holdingsleep`
//    can be used for checks).
// 2. Release Sleep-Lock:
//    - `lk->locked = 0;`
//    - `lk->pid = 0;` (clear owner PID).
// 3. Wake Up Waiters: `wakeup(lk)` is called. This function (from proc.c) will
//    find any processes sleeping on the channel `lk` and transition one or more
//    (typically one for mutex-like behavior) of them to the RUNNABLE state.
// 4. Release Internal Spinlock: `release(&lk->lk)`.
void
releasesleep(struct sleeplock *lk)
{
  acquire(&lk->lk); // Acquire internal spinlock.
  lk->locked = 0;   // Mark sleeplock as free.
  lk->pid = 0;      // Clear owner PID.
  wakeup(lk);       // Wake up any process sleeping on this lock.
  release(&lk->lk); // Release internal spinlock.
}

// Check if the current process holds the sleep-lock `lk`.
// Returns 1 if the current process holds the lock, 0 otherwise.
// Acquires and releases the internal spinlock `lk->lk` to safely check
// `lk->locked` and `lk->pid`.
int
holdingsleep(struct sleeplock *lk)
{
  int r;
  
  acquire(&lk->lk); // Acquire internal spinlock for safe check.
  r = lk->locked && (lk->pid == myproc()->pid); // Check if locked and PID matches.
  release(&lk->lk); // Release internal spinlock.
  return r;
}



