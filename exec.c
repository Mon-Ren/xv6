#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "defs.h"
#include "x86.h"
#include "elf.h" // For ELF file format structures (elfhdr, proghdr).

// The `exec` system call replaces the current process's memory image
// with a new program loaded from an ELF (Executable and Linkable Format) file.
// - `path`: A string representing the path to the executable file.
// - `argv`: An array of strings representing the arguments to be passed to the new program's `main` function.
//
// Key steps involved:
// 1. File System Interaction: Opens the executable file by its `path` using `namei`.
// 2. ELF Parsing: Reads and validates the ELF header (`struct elfhdr`) to ensure it's a valid executable.
// 3. Memory Allocation:
//    - A new page directory (`pgdir`) is created for the new program image using `setupkvm`.
//    - It iterates through the program headers (`struct proghdr`) in the ELF file. For each loadable segment:
//      - Memory is allocated for the segment using `allocuvm`.
//      - The segment's content is loaded from the file into the allocated memory using `loaduvm`.
// 4. User Stack Setup:
//    - Two pages are allocated for the user stack: one actual stack page and one guard page (made inaccessible by `clearpteu` to catch stack overflows).
//    - Argument strings (`argv`) are copied from the old address space to the new user stack.
//    - An array of pointers to these argument strings (the `argv` array for the new program) is created on the stack.
//    - `argc` (argument count) and a fake return address are also pushed onto the stack, conforming to the C calling convention for `main(int argc, char *argv[])`.
// 5. Committing to New Image:
//    - The process's name (`curproc->name`) is updated to the new program's name.
//    - The process's page directory (`curproc->pgdir`) is switched to the new `pgdir`.
//    - The process's size (`curproc->sz`) is updated.
//    - The instruction pointer (`curproc->tf->eip`) in the process's trap frame is set to the ELF entry point (`elf.entry`).
//    - The stack pointer (`curproc->tf->esp`) in the trap frame is set to the newly prepared stack pointer `sp`.
// 6. Context Switch and Cleanup:
//    - `switchuvm(curproc)` activates the new address space and TSS.
//    - The old page directory (`oldpgdir`) and its associated memory are freed using `freevm`.
//
// If `exec` succeeds, it does not return to the caller in the old program. Instead,
// the process begins execution at the entry point of the new program.
// If any step fails (e.g., file not found, invalid ELF format, out of memory),
// `exec` cleans up any allocated resources (like the new `pgdir`) and returns -1 to the caller.
int
exec(char *path, char **argv)
{
  char *s, *last;
  int i, off;
  uint argc, sz, sp, ustack[3+MAXARG+1]; // ustack is for preparing arguments and pointers for the new user stack.
  struct elfhdr elf; // To store the ELF header of the executable.
  struct inode *ip;  // Inode pointer for the executable file.
  struct proghdr ph; // To store program headers from the ELF file.
  pde_t *pgdir = 0, *oldpgdir; // Page directories: 'pgdir' for the new image, 'oldpgdir' for the old one.
  struct proc *curproc = myproc(); // Pointer to the current process control block.

  begin_op(); // Start a file system operation block.

  // Find and lock the inode for the executable file path.
  if((ip = namei(path)) == 0){
    end_op(); // End file system operation block.
    cprintf("exec: file not found %s\n", path);
    return -1;
  }
  ilock(ip); // Lock the inode to prevent concurrent modification.
  // pgdir is initialized to 0; it will hold the new page directory if allocation succeeds.

  // Check ELF header.
  if(readi(ip, (char*)&elf, 0, sizeof(elf)) != sizeof(elf)) // Read ELF header from inode.
    goto bad; // Go to cleanup if read fails.
  if(elf.magic != ELF_MAGIC) // Validate ELF magic number.
    goto bad; // Not a valid ELF file.

  // Allocate a new page directory and set up kernel mappings.
  if((pgdir = setupkvm()) == 0)
    goto bad; // Out of memory for page directory.

  // Load program segments into memory.
  sz = 0; // 'sz' will track the size of the new process's user memory.
  for(i=0, off=elf.phoff; i<elf.phnum; i++, off+=sizeof(ph)){ // Iterate through program headers.
    if(readi(ip, (char*)&ph, off, sizeof(ph)) != sizeof(ph)) // Read program header.
      goto bad;
    if(ph.type != ELF_PROG_LOAD) // Skip non-loadable segments.
      continue;
    if(ph.memsz < ph.filesz) // Sanity check: memory size should be at least file size.
      goto bad;
    if(ph.vaddr + ph.memsz < ph.vaddr) // Sanity check: prevent address overflow.
      goto bad;
    // Allocate user memory for this segment.
    if((sz = allocuvm(pgdir, sz, ph.vaddr + ph.memsz)) == 0)
      goto bad;
    // xv6 requires program segments to be page-aligned (older ELF versions might not).
    if(ph.vaddr % PGSIZE != 0)
      goto bad;
    // Load segment data from file into allocated memory.
    if(loaduvm(pgdir, (char*)ph.vaddr, ip, ph.off, ph.filesz) < 0)
      goto bad;
  }
  iunlockput(ip); // Unlock and release the inode for the executable file.
  end_op();       // End file system operation block.
  ip = 0;         // Mark inode as processed/released.

  // Allocate user stack (two pages: one guard page, one actual stack page).
  sz = PGROUNDUP(sz); // Align current size 'sz' to a page boundary.
  // Allocate two more pages: sz becomes the top of the stack page.
  if((sz = allocuvm(pgdir, sz, sz + 2*PGSIZE)) == 0)
    goto bad;
  clearpteu(pgdir, (char*)(sz - 2*PGSIZE)); // Make the lower page (guard page) user-inaccessible.
  sp = sz; // Stack pointer initially points to the top of the stack region (highest address).

  // Push argument strings onto the stack and prepare argv array.
  for(argc = 0; argv[argc]; argc++) { // Loop through arguments from the caller.
    if(argc >= MAXARG) // Check against maximum allowed arguments.
      goto bad;
    // Allocate space on stack for the argument string (strlen + null terminator), aligned.
    sp = (sp - (strlen(argv[argc]) + 1)) & ~3; // ~3 ensures 4-byte alignment.
    // Copy the argument string from kernel (old address space) to the new user stack.
    if(copyout(pgdir, sp, argv[argc], strlen(argv[argc]) + 1) < 0)
      goto bad;
    ustack[3+argc] = sp; // Store the stack pointer (address of string) in ustack.
                         // ustack[3] onwards will become the argv array for the new program.
  }
  ustack[3+argc] = 0; // Null-terminate the argv array on the stack.

  // Prepare the final stack frame elements for main(argc, argv).
  ustack[0] = 0xffffffff;  // Fake return PC (program should call exit(), not return from main).
  ustack[1] = argc;        // argc for the new program.
  // ustack[2] will be the pointer to the argv array. Its value is the address `sp` will have
  // after the ustack array itself is pushed.
  ustack[2] = sp - (argc+1)*4;

  // Push ustack (fake PC, argc, argv pointer, and the argv string pointers) onto the stack.
  sp -= (3+argc+1) * 4; // Adjust stack pointer for these elements.
  if(copyout(pgdir, sp, ustack, (3+argc+1)*4) < 0)
    goto bad;

  // Save program name for debugging (e.g., for procdump).
  for(last=s=path; *s; s++) // Extract filename from path.
    if(*s == '/')
      last = s+1;
  safestrcpy(curproc->name, last, sizeof(curproc->name));

  // Commit to the new user image. These changes are irreversible for this process.
  oldpgdir = curproc->pgdir;   // Save current page directory to free it later.
  curproc->pgdir = pgdir;      // Switch to the new page directory.
  curproc->sz = sz;            // Update process size.
  curproc->tf->eip = elf.entry; // Set instruction pointer to the ELF entry point.
  curproc->tf->esp = sp;       // Set stack pointer to the prepared user stack.
  switchuvm(curproc);          // Activate the new address space and TSS for the current process.
  freevm(oldpgdir);            // Free the old page directory and associated user memory.
  return 0;                    // exec() does not return to the caller on success.
                               // The process starts executing the new program.

bad: // Error handling: if any step above fails, jump to 'bad'.
  if(pgdir)
    freevm(pgdir); // Free the new page directory if it was allocated.
  if(ip){
    iunlockput(ip); // Unlock and release inode if it was locked.
    end_op();       // End file system operation block.
  }
  return -1; // Return -1 to indicate failure.
}
