解决问题：
  1. 子进程创建时出现的错误，需要互相回调
采用在控制父进程中设置标记flag,若子进程中出现异常并结束则在父进程中重新开启		
  2. 共享内存问题
      a. 创建共享内存失败 
      b. 内存区域太小
解决方法：共享内存不会随着程序结束而自动消除，要么调用shmctl删除，要么自己用手敲命令去删除，否则永远留在系统中。检查内存 命令:ipcs -a 删除内存 ipcrm -m ID由于之前内存区域未 删除 所以无法分配，按照1修改后可以操作
  3. 往结果分区中填写时出现问题，主要是段错误
内存区域 清空操作：memset(shared->result,0,sizeof(shared->result));
内存存储结果 追加操作：strcat(shared->result,tmp.c_str());
内存添加用力数据操作：strncpy(shared->text, this->showTestCaseList().c_str(),this->showTestCaseList().size())	
  4. 子进程自杀时 出现错误 并未杀死
kill信号指令发出后 需要终止进程 
usleep(500);
exit(EXIT_SUCCESS);	
  5. 程序编译后第一次执行不会完成所有的		
