// This file implements spinlocks, a low-level synchronization primitive
// used for mutual exclusion in a multiprocessor environment.
//
// Purpose of Spinlocks:
// Spinlocks are used to protect critical sections of code where shared data
// is accessed. They ensure that only one CPU can execute such a critical
// section at any given time. If a CPU tries to acquire a spinlock that is
// already held by another CPU, it will "spin" in a tight loop (busy-wait)
// until the lock becomes available.
//
// Characteristics:
// - Suitable for short critical sections because spinning can be wasteful if
//   locks are held for long periods.
// - Require interrupts to be disabled on the CPU holding the lock to prevent
//   deadlocks (e.g., if an interrupt handler on the same CPU tries to acquire
//   the same lock). `pushcli()` and `popcli()` are used for this.
// - Rely on atomic hardware instructions (like `xchg`) for lock acquisition.
//
// `struct spinlock` fields:
// - `locked`: An unsigned integer, 0 if the lock is free, 1 if held.
//             Accessed atomically using `xchg`.
// - `name`: A character string name for debugging purposes.
// - `cpu`: A pointer to the `struct cpu` that currently holds the lock.
//          Used for debugging and to check if the current CPU already holds the lock (reentrancy check).
// - `pcs[]`: An array to store program counter (PC) values from the call stack
//            when the lock was acquired. Used for debugging lock contention issues.

#include "types.h"
#include "defs.h"
#include "param.h"
#include "x86.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "spinlock.h"

// Initialize a spinlock.
// - `lk`: Pointer to the `struct spinlock` to initialize.
// - `name`: A descriptive name for the lock (for debugging).
// Sets the lock's name, marks it as initially unlocked (`locked = 0`),
// and clears the CPU holding the lock (`cpu = 0`).
void
initlock(struct spinlock *lk, char *name)
{
  lk->name = name;
  lk->locked = 0; // Initially, the lock is free.
  lk->cpu = 0;    // No CPU is holding the lock.
}

// Acquire the spinlock `lk`.
// This function will loop (spin) until the lock is successfully acquired.
//
// Steps:
// 1. `pushcli()`: Disables interrupts on the current CPU. This is crucial:
//    - To prevent deadlocks: If a CPU holding a lock is interrupted, and the
//      interrupt handler tries to acquire the same lock, it will deadlock.
//    - To prevent the current CPU from being rescheduled by a timer interrupt
//      while it's in the process of acquiring the lock or within the critical section.
// 2. Panic on Re-acquisition: If the current CPU already holds this lock (`holding(lk)`),
//    it's a bug (potential deadlock or incorrect lock usage), so panic. Spinlocks
//    in xv6 are not designed to be recursive.
// 3. Atomic Test-and-Set: The `while(xchg(&lk->locked, 1) != 0)` loop attempts
//    to acquire the lock. `xchg(&lk->locked, 1)` atomically exchanges the value
//    of `lk->locked` with 1.
//    - If `lk->locked` was 0 (free), `xchg` sets it to 1 and returns the old value 0.
//      The loop condition `(0 != 0)` becomes false, and the loop terminates (lock acquired).
//    - If `lk->locked` was 1 (held), `xchg` sets it to 1 (no change) and returns 1.
//      The loop condition `(1 != 0)` is true, and the CPU continues to spin.
// 4. Memory Barrier: `__sync_synchronize()` ensures that no memory loads or stores
//    are reordered by the compiler or CPU across this point. This guarantees that
//    all memory operations within the critical section happen *after* the lock
//    is acquired.
// 5. Debug Information: Records which CPU acquired the lock and the call stack
//    (Program Counters) leading to the acquisition for debugging purposes.
void
acquire(struct spinlock *lk)
{
  pushcli(); // Disable interrupts to prevent deadlock and preemption issues.
  if(holding(lk)) // Check if current CPU already holds this lock.
    panic("acquire");

  // The `xchg` (exchange) instruction is atomic.
  // It writes 1 to lk->locked and returns the old value of lk->locked.
  // The loop continues as long as xchg returns non-zero (meaning lock was already set).
  while(xchg(&lk->locked, 1) != 0)
    ; // Spin.

  // Tell the C compiler and the processor to not move loads or stores
  // past this point, to ensure that the critical section's memory
  // references happen *after* the lock is acquired. This is a full memory barrier.
  __sync_synchronize();

  // Record information about lock acquisition for debugging.
  lk->cpu = mycpu();         // Store pointer to current CPU structure.
  getcallerpcs(&lk, lk->pcs); // Store call stack.
}

// Release the spinlock `lk`.
//
// Steps:
// 1. Panic if Not Holding: If the current CPU does not hold this lock (`!holding(lk)`),
//    it's a bug (releasing a lock not held or held by another CPU), so panic.
// 2. Clear Debug Information: Resets `lk->pcs[0]` and `lk->cpu`.
// 3. Memory Barrier: `__sync_synchronize()` ensures that all memory stores
//    within the critical section are completed and visible to other CPUs *before*
//    the lock is released.
// 4. Atomic Release: `asm volatile("movl $0, %0" : "+m" (lk->locked) : );`
//    atomically sets `lk->locked` to 0. This is done using inline assembly
//    because a simple C assignment `lk->locked = 0;` might not be atomic on
//    all architectures or with all compiler optimizations.
// 5. `popcli()`: Re-enables interrupts if they were enabled before the corresponding
//    `pushcli` call and if this is the outermost critical section (ncli becomes 0).
void
release(struct spinlock *lk)
{
  if(!holding(lk)) // Check if current CPU actually holds the lock.
    panic("release");

  // Clear debugging information.
  lk->pcs[0] = 0;
  lk->cpu = 0;

  // Tell the C compiler and the processor to not move loads or stores
  // past this point, to ensure that all the stores in the critical
  // section are visible to other cores *before* the lock is released.
  // This is a full memory barrier.
  __sync_synchronize();

  // Release the lock. This must be atomic.
  // `asm volatile` ensures the compiler doesn't optimize this away or reorder it.
  // `movl $0, %0` moves 0 into the memory location of `lk->locked`.
  // `"+m"` means the operand is memory that is both read and written.
  asm volatile("movl $0, %0" : "+m" (lk->locked) : );

  popcli(); // Re-enable interrupts if appropriate.
}

// Record the current call stack in `pcs[]` by following the frame pointer (`%ebp`) chain.
// This function is used for debugging purposes to see who acquired a lock.
// It walks up the stack, saving the return addresses (saved %eip values) at each level.
// - `v`: A pointer from which to start finding the EBP chain (usually `&lk` in `acquire`).
// - `pcs[]`: An array to store the program counter values.
void
getcallerpcs(void *v, uint pcs[])
{
  uint *ebp;
  int i;

  ebp = (uint*)v - 2;
  for(i = 0; i < 10; i++){
    if(ebp == 0 || ebp < (uint*)KERNBASE || ebp == (uint*)0xffffffff)
      break;
    pcs[i] = ebp[1];     // saved %eip
    ebp = (uint*)ebp[0]; // saved %ebp
  }
  for(; i < 10; i++)
    pcs[i] = 0;
}

// Check whether this cpu is holding the lock.
int
holding(struct spinlock *lock)
{
  int r;
  pushcli();
  r = lock->locked && lock->cpu == mycpu();
  popcli();
  return r;
}


// `pushcli()` and `popcli()` are xv6's mechanisms for managing interrupt enable/disable state.
// They are designed to be nestable: multiple `pushcli()` calls require a corresponding
// number of `popcli()` calls to actually re-enable interrupts (if they were originally on).
// This allows functions that need interrupts disabled to call other functions that
// also need interrupts disabled, without prematurely re-enabling them.
//
// `pushcli()`: Disable interrupts on the current CPU.
// - Reads the EFLAGS register to check the current interrupt flag (FL_IF).
// - Issues a `cli` instruction to disable interrupts.
// - If this is the first `pushcli` call in a nested sequence (`mycpu()->ncli == 0`),
//   it records the original interrupt enable state (`mycpu()->intena`).
// - Increments `mycpu()->ncli`, the nesting count for cli/sti calls on this CPU.
void
pushcli(void)
{
  int eflags;

  eflags = readeflags();
  cli();
  if(mycpu()->ncli == 0)
    mycpu()->intena = eflags & FL_IF;
  mycpu()->ncli += 1;
}

// `popcli()`: Re-enable interrupts on the current CPU if `pushcli` was the last one.
// - Panics if interrupts are somehow already enabled when `popcli` is called,
//   as this indicates a bug in lock management or interrupt handling.
// - Decrements `mycpu()->ncli`. Panics if `ncli` goes below zero (too many `popcli` calls).
// - If `mycpu()->ncli` becomes 0 (outermost `popcli`) AND interrupts were
//   originally enabled before the first `pushcli` (`mycpu()->intena` is true),
//   then it issues an `sti` instruction to re-enable interrupts.
void
popcli(void)
{
  if(readeflags()&FL_IF) // Interrupts should be off when popcli is called.
    panic("popcli - interruptible");
  if(--mycpu()->ncli < 0) // Decrement nesting count; panic if it goes negative.
    panic("popcli");
  // If nesting count is zero and interrupts were originally enabled, re-enable them.
  if(mycpu()->ncli == 0 && mycpu()->intena)
    sti();
}

