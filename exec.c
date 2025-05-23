#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "defs.h"
#include "x86.h"
#include "elf.h" // 对于ELF文件格式结构 (elfhdr, proghdr)。

// `exec` 系统调用用从ELF（可执行和可链接格式）文件加载的新程序替换当前进程的内存映像。
// - `path`: 指向可执行文件的路径字符串。
// - `argv`: 要传递给新程序 `main` 函数的参数字符串数组。
//
// 关键步骤包括：
// 1. 文件系统交互：使用 `namei` 通过其 `path` 打开可执行文件。
// 2. ELF解析：读取并验证ELF头部 (`struct elfhdr`) 以确保它是有效的可执行文件。
// 3. 内存分配：
//    - 使用 `setupkvm` 为新程序映像创建一个新的页目录 (`pgdir`)。
//    - 遍历ELF文件中的程序头部 (`struct proghdr`)。对于每个可加载段：
//      - 使用 `allocuvm` 为段分配内存。
//      - 使用 `loaduvm` 将段的内容从文件加载到分配的内存中。
// 4. 用户栈设置：
//    - 为用户栈分配两个页：一个实际的栈页和一个保护页（通过 `clearpteu` 设置为不可访问以捕获栈溢出）。
//    - 将参数字符串 (`argv`) 从旧地址空间复制到新的用户栈。
//    - 在栈上创建一个指向这些参数字符串的指针数组（新程序的 `argv` 数组）。
//    - `argc`（参数计数）和一个伪返回地址也被压入栈中，以符合 `main(int argc, char *argv[])` 的C调用约定。
// 5. 提交到新映像：
//    - 进程的名称 (`curproc->name`) 更新为新程序的名称。
//    - 进程的页目录 (`curproc->pgdir`) 切换到新的 `pgdir`。
//    - 更新进程的大小 (`curproc->sz`)。
//    - 进程陷阱帧中的指令指针 (`curproc->tf->eip`) 设置为ELF入口点 (`elf.entry`)。
//    - 陷阱帧中的栈指针 (`curproc->tf->esp`) 设置为新准备的栈指针 `sp`。
// 6. 上下文切换和清理：
//    - `switchuvm(curproc)` 激活新地址空间和TSS。
//    - 使用 `freevm` 释放旧页目录 (`oldpgdir`) 及其关联内存。
//
// 如果 `exec` 成功，它不会返回到旧程序中的调用者。相反，进程在新程序的入口点开始执行。
// 如果任何步骤失败（例如，文件未找到、无效的ELF格式、内存不足），
// `exec` 会清理任何已分配的资源（如新的 `pgdir`）并向调用者返回-1。
int
exec(char *path, char **argv)
{
  char *s, *last;
  int i, off;
  uint argc, sz, sp, ustack[3+MAXARG+1]; // ustack 用于为新的用户栈准备参数和指针。
  struct elfhdr elf; // 用于存储可执行文件的ELF头部。
  struct inode *ip;  // 可执行文件的inode指针。
  struct proghdr ph; // 用于存储ELF文件中的程序头部。
  pde_t *pgdir = 0, *oldpgdir; // 页目录：'pgdir' 用于新映像，'oldpgdir' 用于旧映像。
  struct proc *curproc = myproc(); // 指向当前进程控制块的指针。

  begin_op(); // 开始文件系统操作块。

  // 查找并锁定可执行文件路径的inode。
  if((ip = namei(path)) == 0){
    end_op(); // 结束文件系统操作块。
    cprintf("exec: file not found %s\n", path);
    return -1;
  }
  ilock(ip); // 锁定inode以防止并发修改。
  // pgdir 初始化为0；如果分配成功，它将持有新的页目录。

  // 检查ELF头部。
  if(readi(ip, (char*)&elf, 0, sizeof(elf)) != sizeof(elf)) // 从inode读取ELF头部。
    goto bad; // 如果读取失败，则转到清理。
  if(elf.magic != ELF_MAGIC) // 验证ELF魔数。
    goto bad; // 不是有效的ELF文件。

  // 分配新的页目录并设置内核映射。
  if((pgdir = setupkvm()) == 0)
    goto bad; // 页目录内存不足。

  // 将程序段加载到内存中。
  sz = 0; // 'sz' 将跟踪新进程用户内存的大小。
  for(i=0, off=elf.phoff; i<elf.phnum; i++, off+=sizeof(ph)){ // 遍历程序头部。
    if(readi(ip, (char*)&ph, off, sizeof(ph)) != sizeof(ph)) // 读取程序头部。
      goto bad;
    if(ph.type != ELF_PROG_LOAD) // 跳过不可加载的段。
      continue;
    if(ph.memsz < ph.filesz) // 完整性检查：内存大小应至少为文件大小。
      goto bad;
    if(ph.vaddr + ph.memsz < ph.vaddr) // 完整性检查：防止地址溢出。
      goto bad;
    // 为此段分配用户内存。
    if((sz = allocuvm(pgdir, sz, ph.vaddr + ph.memsz)) == 0)
      goto bad;
    // xv6 要求程序段页对齐（旧版ELF可能不要求）。
    if(ph.vaddr % PGSIZE != 0)
      goto bad;
    // 将段数据从文件加载到分配的内存中。
    if(loaduvm(pgdir, (char*)ph.vaddr, ip, ph.off, ph.filesz) < 0)
      goto bad;
  }
  iunlockput(ip); // 解锁并释放可执行文件的inode。
  end_op();       // 结束文件系统操作块。
  ip = 0;         // 将inode标记为已处理/已释放。

  // 为用户栈分配空间（两个页：一个保护页，一个实际的栈页）。
  sz = PGROUNDUP(sz); // 将当前大小 'sz' 对齐到页边界。
  // 再分配两个页：sz 成为栈页的顶部。
  if((sz = allocuvm(pgdir, sz, sz + 2*PGSIZE)) == 0)
    goto bad;
  clearpteu(pgdir, (char*)(sz - 2*PGSIZE)); // 使较低的页（保护页）用户不可访问。
  sp = sz; // 栈指针最初指向栈区域的顶部（最高地址）。

  // 将参数字符串压入栈并准备argv数组。
  for(argc = 0; argv[argc]; argc++) { // 遍历调用者传入的参数。
    if(argc >= MAXARG) // 检查是否超出允许的最大参数数量。
      goto bad;
    // 在栈上为参数字符串（strlen + 空终止符）分配空间，并对齐。
    sp = (sp - (strlen(argv[argc]) + 1)) & ~3; // ~3 确保4字节对齐。
    // 将参数字符串从内核（旧地址空间）复制到新的用户栈。
    if(copyout(pgdir, sp, argv[argc], strlen(argv[argc]) + 1) < 0)
      goto bad;
    ustack[3+argc] = sp; // 将栈指针（字符串地址）存储在ustack中。
                         // ustack[3] 开始将成为新程序的argv数组。
  }
  ustack[3+argc] = 0; // 在栈上以null终止argv数组。

  // 准备 main(argc, argv) 的最终栈帧元素。
  ustack[0] = 0xffffffff;  // 伪返回PC（程序应调用exit()，而不是从main返回）。
  ustack[1] = argc;        // 新程序的argc。
  // ustack[2] 将是指向argv数组的指针。其值是 `sp` 在ustack数组本身被压入栈后将具有的地址。
  ustack[2] = sp - (argc+1)*4;

  // 将ustack（包含伪PC、argc、argv指针和argv字符串指针）复制到用户栈。
  sp -= (3+argc+1) * 4; // 为这些元素调整栈指针。
  if(copyout(pgdir, sp, ustack, (3+argc+1)*4) < 0)
    goto bad;

  // 保存程序名称以用于调试（例如，用于procdump）。
  for(last=s=path; *s; s++) // 从路径中提取文件名。
    if(*s == '/')
      last = s+1;
  safestrcpy(curproc->name, last, sizeof(curproc->name));

  // 提交到新的用户映像。这些更改对此进程而言是不可逆的。
  oldpgdir = curproc->pgdir;   // 保存当前页目录以便稍后释放。
  curproc->pgdir = pgdir;      // 切换到新的页目录。
  curproc->sz = sz;            // 更新进程大小。
  curproc->tf->eip = elf.entry; // 将指令指针设置为ELF入口点。
  curproc->tf->esp = sp;       // 将栈指针设置为准备好的用户栈。
  switchuvm(curproc);          // 为当前进程激活新的地址空间和TSS。
  freevm(oldpgdir);            // 释放旧的页目录和关联的用户内存。
  return 0;                    // exec() 成功时不返回给调用者。
                               // 进程开始执行新程序。

bad: // 错误处理：如果上述任何步骤失败，则跳转到 'bad'。
  if(pgdir)
    freevm(pgdir); // 如果已分配，则释放新的页目录。
  if(ip){
    iunlockput(ip); // 如果已锁定，则解锁并释放inode。
    end_op();       // 结束文件系统操作块。
  }
  return -1; // 返回-1表示失败。
}
