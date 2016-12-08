// Definition of the MainSem class

#ifndef MainSem_class
#define MainSem_class

#include <stdio.h>
#include <semaphore.h>  
#include <iostream>
using namespace std;

class MainSem
{
 public:
 	// MainSem();
 	sem_t sem_socket;                        //信号量  ,驱动子线程
	sem_t sem_socket_return;                   //增加的信号量 
	sem_t sem_test;
	sem_t sem_test_return;
  	void initSem();				 //初始化信号量
  	void ShowError(int res);		 //错误信息
  	void deleteSem(); 			 //清理信号量 
  	int getSem(sem_t s);
};
#endif
