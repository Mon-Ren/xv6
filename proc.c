#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "x86.h"
#include "proc.h"
#include "spinlock.h"

// The process table (ptable) holds all process control blocks (PCBs).
// It is protected by a spinlock to ensure thread-safe access and modification
// of process states and information. `NPROC` is the maximum number of processes.
// Each `struct proc` entry in the `proc` array represents a single process.
// The `lock` field is a spinlock that must be acquired before accessing ptable
// to prevent race conditions.
struct {
  struct spinlock lock; // Lock to protect access to the ptable.
  struct proc proc[NPROC]; // Array of NPROC process control blocks.
} ptable;

static struct proc *initproc; // Pointer to the initial process (init), the first user process.

int nextpid = 1; // Counter for assigning unique process IDs (PIDs).
extern void forkret(void); // Assembly function called when a new process starts executing after fork.
extern void trapret(void); // Assembly function for returning from a trap (system call or interrupt).

static void wakeup1(void *chan); // Internal helper function for wakeup, assumes ptable.lock is held.

// Initialize the process table lock.
// This function is called once during kernel initialization (in main.c).
void
pinit(void)
{
  initlock(&ptable.lock, "ptable"); // Initialize the spinlock for ptable.
}

// Must be called with interrupts disabled
int
cpuid() {
  return mycpu()-cpus;
}

// Must be called with interrupts disabled to avoid the caller being
// rescheduled between reading lapicid and running through the loop.
struct cpu*
mycpu(void)
{
  int apicid, i;
  
  if(readeflags()&FL_IF)
    panic("mycpu called with interrupts enabled\n");
  
  apicid = lapicid();
  // APIC IDs are not guaranteed to be contiguous. Maybe we should have
  // a reverse map, or reserve a register to store &cpus[i].
  for (i = 0; i < ncpu; ++i) {
    if (cpus[i].apicid == apicid)
      return &cpus[i];
  }
  panic("unknown apicid\n");
}

// Disable interrupts so that we are not rescheduled
// while reading proc from the cpu structure
struct proc*
myproc(void) {
  struct cpu *c;
  struct proc *p;
  pushcli();
  c = mycpu();
  p = c->proc;
  popcli();
  return p;
}

//PAGEBREAK: 32
// Look in the process table for an UNUSED proc.
// If found, change state to EMBRYO and initialize
// state required to run in the kernel.
// Otherwise return 0.
//
// Process states (`p->state` in `struct proc`):
//   UNUSED:   The process table entry is free.
//   EMBRYO:   Process is being created; initial state before it's ready to run.
//             Kernel stack allocated, initial trap frame and context set up.
//   SLEEPING: Process is waiting for an event (e.g., I/O completion, child exit, pipe data).
//             `p->chan` stores the channel it's waiting on.
//   RUNNABLE: Process is ready to run but waiting for a CPU to become available.
//   RUNNING:  Process is currently executing on a CPU.
//   ZOMBIE:   Process has exited but is waiting for its parent to collect its status
//             and resources via `wait()`. Its resources (except PID and exit status) are freed.
//
// Allocates a new process control block (PCB) from the ptable.
// If found, it initializes the PCB to the EMBRYO state, assigns a PID,
// and sets up the kernel stack and initial kernel context for the new process.
// The initial context is set up to start execution at `forkret`, which
// then returns to `trapret`, eventually leading to user-space execution if it's a user process.
// Returns a pointer to the new `struct proc` on success, or 0 if no free PCB is found
// or if kernel stack allocation fails.
static struct proc*
allocproc(void)
{
  struct proc *p;
  char *sp;

  acquire(&ptable.lock);

  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++)
    if(p->state == UNUSED)
      goto found;

  release(&ptable.lock);
  return 0;

found:
  p->state = EMBRYO;
  p->pid = nextpid++;

  release(&ptable.lock);

  // Allocate kernel stack.
  if((p->kstack = kalloc()) == 0){
    p->state = UNUSED;
    return 0;
  }
  sp = p->kstack + KSTACKSIZE;

  // Leave room for trap frame.
  sp -= sizeof *p->tf;
  p->tf = (struct trapframe*)sp;

  // Set up new context to start executing at forkret,
  // which returns to trapret.
  sp -= 4;
  *(uint*)sp = (uint)trapret;

  sp -= sizeof *p->context;
  p->context = (struct context*)sp;
  memset(p->context, 0, sizeof *p->context);
  p->context->eip = (uint)forkret;

  return p;
}

//PAGEBREAK: 32
// Set up first user process.
// This function is called by main() during kernel initialization to create
// the very first user-mode process, known as "init" (or "initcode").
// It allocates a PCB, sets up its page directory with kernel mappings,
// copies the small initcode program into its memory (at virtual address 0),
// and prepares its trap frame to start execution in user mode at address 0.
void
userinit(void)
{
  struct proc *p;
  extern char _binary_initcode_start[], _binary_initcode_size[];

  p = allocproc();
  
  initproc = p;
  if((p->pgdir = setupkvm()) == 0)
    panic("userinit: out of memory?");
  inituvm(p->pgdir, _binary_initcode_start, (int)_binary_initcode_size);
  p->sz = PGSIZE;
  memset(p->tf, 0, sizeof(*p->tf));
  p->tf->cs = (SEG_UCODE << 3) | DPL_USER;
  p->tf->ds = (SEG_UDATA << 3) | DPL_USER;
  p->tf->es = p->tf->ds;
  p->tf->ss = p->tf->ds;
  p->tf->eflags = FL_IF;
  p->tf->esp = PGSIZE;
  p->tf->eip = 0;  // beginning of initcode.S

  safestrcpy(p->name, "initcode", sizeof(p->name));
  p->cwd = namei("/");

  // this assignment to p->state lets other cores
  // run this process. the acquire forces the above
  // writes to be visible, and the lock is also needed
  // because the assignment might not be atomic.
  acquire(&ptable.lock);

  p->state = RUNNABLE;

  release(&ptable.lock);
}

// Grow current process's memory by n bytes.
// Return 0 on success, -1 on failure.
int
growproc(int n)
{
  uint sz;
  struct proc *curproc = myproc();

  sz = curproc->sz;
  if(n > 0){
    if((sz = allocuvm(curproc->pgdir, sz, sz + n)) == 0)
      return -1;
  } else if(n < 0){
    if((sz = deallocuvm(curproc->pgdir, sz, sz + n)) == 0)
      return -1;
  }
  curproc->sz = sz;
  switchuvm(curproc);
  return 0;
}

// Create a new process by duplicating the state of the calling process (parent).
// This is the implementation of the fork() system call.
// - Allocates a new PCB for the child process using `allocproc`.
// - Copies the parent's user memory (address space) to the child using `copyuvm`.
// - Copies the parent's trap frame, so the child starts as if it also called fork.
// - Modifies the child's trap frame so that `fork()` returns 0 in the child.
// - Duplicates open files and the current working directory.
// - Sets the child's state to RUNNABLE.
// Returns the child's PID to the parent, and 0 to the child.
// Returns -1 on failure (e.g., cannot allocate PCB or copy memory).
int
fork(void)
{
  int i, pid;
  struct proc *np;
  struct proc *curproc = myproc();

  // Allocate process.
  if((np = allocproc()) == 0){
    return -1;
  }

  // Copy process state from proc.
  if((np->pgdir = copyuvm(curproc->pgdir, curproc->sz)) == 0){
    kfree(np->kstack);
    np->kstack = 0;
    np->state = UNUSED;
    return -1;
  }
  np->sz = curproc->sz;
  np->parent = curproc;
  *np->tf = *curproc->tf;

  // Clear %eax so that fork returns 0 in the child.
  np->tf->eax = 0;

  for(i = 0; i < NOFILE; i++)
    if(curproc->ofile[i])
      np->ofile[i] = filedup(curproc->ofile[i]);
  np->cwd = idup(curproc->cwd);

  safestrcpy(np->name, curproc->name, sizeof(curproc->name));

  pid = np->pid;

  acquire(&ptable.lock);

  np->state = RUNNABLE;

  release(&ptable.lock);

  return pid;
}

// Exit the current process. Does not return.
// This is the implementation of the exit() system call.
// - Closes all open files.
// - Releases the current working directory.
// - Wakes up the parent process if it's waiting in `wait()`.
// - Reparents any children of the exiting process to the `init` process.
// - Changes the process state to ZOMBIE.
// - Calls the scheduler (`sched()`) to give up the CPU permanently.
// The process's resources (like kstack and page directory) are cleaned up by the
// parent process when it calls `wait()`.
void
exit(void)
{
  struct proc *curproc = myproc();
  struct proc *p;
  int fd;

  if(curproc == initproc)
    panic("init exiting");

  // Close all open files.
  for(fd = 0; fd < NOFILE; fd++){
    if(curproc->ofile[fd]){
      fileclose(curproc->ofile[fd]);
      curproc->ofile[fd] = 0;
    }
  }

  begin_op();
  iput(curproc->cwd);
  end_op();
  curproc->cwd = 0;

  acquire(&ptable.lock);

  // Parent might be sleeping in wait().
  wakeup1(curproc->parent);

  // Pass abandoned children to init.
  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
    if(p->parent == curproc){
      p->parent = initproc;
      if(p->state == ZOMBIE)
        wakeup1(initproc);
    }
  }

  // Jump into the scheduler, never to return.
  curproc->state = ZOMBIE;
  sched();
  panic("zombie exit");
}

// Wait for a child process to exit, collect its PID, and free its resources.
// This is the implementation of the wait() system call.
// - Scans the process table for child processes of the current process.
// - If a ZOMBIE child is found, its resources (kernel stack, page directory) are freed,
//   its PCB is marked UNUSED, and its PID is returned.
// - If no ZOMBIE children are found but children exist, the current process sleeps
//   (waiting for a child to exit). `wakeup1()` in `exit()` or `kill()` will wake it.
// - If the current process has no children or is killed, it returns -1.
int
wait(void)
{
  struct proc *p;
  int havekids, pid;
  struct proc *curproc = myproc();
  
  acquire(&ptable.lock);
  for(;;){
    // Scan through table looking for exited children.
    havekids = 0;
    for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
      if(p->parent != curproc)
        continue;
      havekids = 1;
      if(p->state == ZOMBIE){
        // Found one.
        pid = p->pid;
        kfree(p->kstack);
        p->kstack = 0;
        freevm(p->pgdir);
        p->pid = 0;
        p->parent = 0;
        p->name[0] = 0;
        p->killed = 0;
        p->state = UNUSED;
        release(&ptable.lock);
        return pid;
      }
    }

    // No point waiting if we don't have any children.
    if(!havekids || curproc->killed){
      release(&ptable.lock);
      return -1;
    }

    // Wait for children to exit.  (See wakeup1 call in proc_exit.)
    sleep(curproc, &ptable.lock);  //DOC: wait-sleep
  }
}

//PAGEBREAK: 42
// Per-CPU process scheduler. Each CPU runs its own instance of scheduler().
// This function is the heart of the xv6 scheduling mechanism.
// It continuously loops, looking for a RUNNABLE process in the ptable.
// When a RUNNABLE process is found:
//  1. It sets up the CPU's context to run the chosen process (`c->proc = p`).
//  2. Switches to the process's page table (`switchuvm(p)`).
//  3. Changes the process's state to RUNNING.
//  4. Performs a context switch (`swtch`) from the scheduler's context to the process's context.
// When the process yields or its time slice ends (implicitly via timer interrupt and trap handling),
// it switches back to the scheduler's context. The scheduler then continues its loop.
// The scheduler enables interrupts (`sti()`) at the beginning of each loop iteration
// to allow for timer interrupts and other hardware events.
// It acquires `ptable.lock` before scanning `ptable` and releases it if no runnable
// process is found in one pass, or just before switching to a process (the process
// itself is responsible for releasing it via `forkret` or reacquiring/releasing it
// around `sched` calls).
void
scheduler(void)
{
  struct proc *p;
  struct cpu *c = mycpu();
  c->proc = 0;
  
  for(;;){
    // Enable interrupts on this processor.
    sti();

    // Loop over process table looking for process to run.
    acquire(&ptable.lock);
    for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
      if(p->state != RUNNABLE)
        continue;

      // Switch to chosen process.  It is the process's job
      // to release ptable.lock and then reacquire it
      // before jumping back to us.
      c->proc = p;
      switchuvm(p);
      p->state = RUNNING;

      swtch(&(c->scheduler), p->context);
      switchkvm();

      // Process is done running for now.
      // It should have changed its p->state before coming back.
      c->proc = 0;
    }
    release(&ptable.lock);

  }
}

// Relinquish the CPU and enter the scheduler.
// This function is called when a process wants to voluntarily give up the CPU
// (e.g., in `yield()`) or when it needs to sleep (`sleep()`) or exit (`exit()`).
// The caller must hold `ptable.lock` and must have already set the
// current process's state (e.g., to RUNNABLE or SLEEPING).
// `sched()` saves the current process's context (registers, EIP) and switches
// to the scheduler's context on the current CPU (`mycpu()->scheduler`).
// Interrupts must be disabled by the caller before calling `sched`, and `sched`
// itself performs checks to ensure this and other invariants.
void
sched(void)
{
  int intena;
  struct proc *p = myproc();

  if(!holding(&ptable.lock))
    panic("sched ptable.lock");
  if(mycpu()->ncli != 1)
    panic("sched locks");
  if(p->state == RUNNING)
    panic("sched running");
  if(readeflags()&FL_IF)
    panic("sched interruptible");
  intena = mycpu()->intena;
  swtch(&p->context, mycpu()->scheduler);
  mycpu()->intena = intena;
}

// Give up the CPU for one scheduling round.
void
yield(void)
{
  acquire(&ptable.lock);  //DOC: yieldlock
  myproc()->state = RUNNABLE;
  sched();
  release(&ptable.lock);
}

// This function is the entry point for a new process after it's first scheduled.
// When `allocproc` creates a new process, it sets the `eip` (instruction pointer)
// in the new process's context to `forkret`. So, when the scheduler first
// `swtch`es to this new process, `forkret` is the first C code it executes.
// Its primary responsibilities are:
//  1. Release the `ptable.lock` that was acquired by the scheduler before switching
//     to this new process. This is crucial because the new process now runs independently.
//  2. If this is the very first user process (`initproc`), it performs some one-time
//     system initializations (like `iinit` for file system and `initlog`). These
//     initializations need to run in a process context (e.g., they might call `sleep`).
// After these steps, `forkret` effectively "returns". Its "return address" was set up
// in `allocproc` to be `trapret`. `trapret` will then restore the user registers
// from the process's trap frame and switch to user mode, starting the execution
// of the user program (e.g., `initcode.S` for the first process, or the instruction
// after `fork()` for a normally forked child).
void
forkret(void)
{
  static int first = 1;
  // Still holding ptable.lock from scheduler.
  release(&ptable.lock);

  if (first) {
    // Some initialization functions must be run in the context
    // of a regular process (e.g., they call sleep), and thus cannot
    // be run from main().
    first = 0;
    iinit(ROOTDEV);
    initlog(ROOTDEV);
  }

  // Return to "caller", actually trapret (see allocproc).
}

// Put the current process to sleep on an arbitrary `chan` (channel/condition).
// This function is called when a process needs to wait for a specific event.
// - `chan`: An arbitrary pointer used as a "channel" or "wait queue". Processes
//           sleeping on the same `chan` will be woken up together by `wakeup(chan)`.
// - `lk`: A spinlock that protects the condition being waited for. This lock *must*
//         be held by the caller.
// Operation:
//  1. The current process (`p = myproc()`) is identified.
//  2. Critical section: If `lk` is not `&ptable.lock`, `ptable.lock` is acquired,
//     then `lk` is released. This order is crucial to prevent lost wakeups.
//     If `lk` is `&ptable.lock`, it's already held.
//  3. The process's state is set to `SLEEPING`, and `p->chan` is set to `chan`.
//  4. `sched()` is called to relinquish the CPU and switch to the scheduler.
//  5. When the process is awakened by `wakeup(chan)` (which sets its state to RUNNABLE),
//     it will eventually be scheduled again, and `sched()` will return here.
//  6. `p->chan` is cleared.
//  7. The original lock `lk` is reacquired (if it wasn't `&ptable.lock`). If `lk` was
//     `&ptable.lock`, it's released by the caller of `sleep` or by `sched`'s caller.
// The atomicity of releasing `lk` and going to sleep (relative to `wakeup`) is
// ensured by holding `ptable.lock` during the state change and `sched()` call.
void
sleep(void *chan, struct spinlock *lk)
{
  struct proc *p = myproc();
  
  if(p == 0)
    panic("sleep");

  if(lk == 0)
    panic("sleep without lk");

  // Must acquire ptable.lock in order to
  // change p->state and then call sched.
  // Once we hold ptable.lock, we can be
  // guaranteed that we won't miss any wakeup
  // (wakeup runs with ptable.lock locked),
  // so it's okay to release lk.
  if(lk != &ptable.lock){  //DOC: sleeplock0
    acquire(&ptable.lock);  //DOC: sleeplock1
    release(lk);
  }
  // Go to sleep.
  p->chan = chan;
  p->state = SLEEPING;

  sched();

  // Tidy up.
  p->chan = 0;

  // Reacquire original lock.
  if(lk != &ptable.lock){  //DOC: sleeplock2
    release(&ptable.lock);
    acquire(lk);
  }
}

//PAGEBREAK!
// Wake up all processes sleeping on the given `chan` (channel/condition).
// This is an internal helper function. The caller *must* hold `ptable.lock`.
// It iterates through the `ptable`. If a process is `SLEEPING` and its
// `p->chan` matches the given `chan`, its state is changed to `RUNNABLE`.
// These newly RUNNABLE processes will then be eligible to be picked by the scheduler.
static void
wakeup1(void *chan)
{
  struct proc *p;

  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++)
    if(p->state == SLEEPING && p->chan == chan)
      p->state = RUNNABLE;
}

// Wake up all processes sleeping on chan.
void
wakeup(void *chan)
{
  acquire(&ptable.lock);
  wakeup1(chan);
  release(&ptable.lock);
}

// Kill the process with the given `pid`.
// This is the implementation of the `kill()` system call.
// - It searches the `ptable` for a process with the matching `pid`.
// - If found, it sets the `p->killed` flag of that process to 1.
// - If the target process is currently `SLEEPING`, its state is changed to `RUNNABLE`.
//   This ensures that a sleeping process (e.g., waiting for I/O) gets a chance
//   to run, notice its `killed` flag (in `trap()`), and exit.
// A process doesn't die immediately when `kill()` is called on it.
// The `p->killed` flag is checked when the process traps into the kernel (e.g.,
// on a system call or timer interrupt). If the flag is set, the process then calls `exit()`.
// Returns 0 on success (process found and marked), -1 if no process with `pid` exists.
int
kill(int pid)
{
  struct proc *p;

  acquire(&ptable.lock);
  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
    if(p->pid == pid){
      p->killed = 1;
      // Wake process from sleep if necessary.
      if(p->state == SLEEPING)
        p->state = RUNNABLE;
      release(&ptable.lock);
      return 0;
    }
  }
  release(&ptable.lock);
  return -1;
}

//PAGEBREAK: 36
// Print a process listing to console.  For debugging.
// Runs when user types ^P on console.
// No lock to avoid wedging a stuck machine further.
void
procdump(void)
{
  static char *states[] = {
  [UNUSED]    "unused",
  [EMBRYO]    "embryo",
  [SLEEPING]  "sleep ",
  [RUNNABLE]  "runble",
  [RUNNING]   "run   ",
  [ZOMBIE]    "zombie"
  };
  int i;
  struct proc *p;
  char *state;
  uint pc[10];

  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
    if(p->state == UNUSED)
      continue;
    if(p->state >= 0 && p->state < NELEM(states) && states[p->state])
      state = states[p->state];
    else
      state = "???";
    cprintf("%d %s %s", p->pid, state, p->name);
    if(p->state == SLEEPING){
      getcallerpcs((uint*)p->context->ebp+2, pc);
      for(i=0; i<10 && pc[i] != 0; i++)
        cprintf(" %p", pc[i]);
    }
    cprintf("\n");
  }
}
