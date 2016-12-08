/*
* 辅助参数类
*用于重写 输入输出重定向 以及两个缓冲区参数
*/
#ifndef SupportClass_class
#define SupportClass_class

#include "TestCase.h"
#include "tinyxml2.h"
#include <unistd.h>
#include <signal.h>  
#include <sys/types.h>  
#include <sys/stat.h>
#include <sys/shm.h>
#include <cstdio>  
#include <cstdlib>
#include <cstring>
#include <list>
#include <iostream>

typedef list<TestCase *> TestCaseList;

#define TEXT_SZ 10000
//如果size为1至4096，则实际申请到的共享内存大小为4K(一页)；4097到8192，则实际申请到的共享内存大小为8K(两页)，依此类推。

struct shared_use_st
{
    int currentIndex;               //当前测试用例ID
    char text[TEXT_SZ];             //记录写入和读取的文本
    char result[TEXT_SZ];
};

class SupportClass{

    private:
        int POLLINGNUM = 0;
        int index =1;                   //测试用例ID生成时计数
        TestCaseList testCaseList;      //测试用例链表
        string projectPath;             //项目路径

        int shmid;						//共享内存标识符
        shared_use_st *shared;          //共享内存区
        void *shm = NULL;				//分配的共享内存的原始首地址
    public:
        bool flag = false;              //判断是否出错
        SupportClass();
        ~SupportClass();
        int getCurrentIndex();
        int getTestCaseCount();
        /* 获取 项目路径 */
        string getProjectPath();
        /* 轮巡检查 */
        void polling(pid_t pid);
        /* 创建 进程 并做轮巡处理*/
        void createPidAndPolling();
        /* XML文件解析 */
        bool ReadXmlFile(string szFileName);
        /* 展示 测试用例 */
        string showTestCaseList();
        TestCase* getTestCaseAtIndex(int ID);
        /* 共享内存 创建并准备 */
        void createMem();
        /* 将 测试用例实体集 放入 共享内存中 */
        void putTestCasesInMem();
        /* 从 共享内存中 读写测试用例实体集 */
        void getTestCasesInMem();
        /* 分离 共享内存 */
        void pullMem();

        void showResult();
};
#endif
