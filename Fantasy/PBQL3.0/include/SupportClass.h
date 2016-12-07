/*
* 辅助参数类
*用于重写 输入输出重定向 以及两个缓冲区参数
*/
#ifndef SupportClass_class
#define SupportClass_class

#include "TestCase.h"
#include "tinyxml2.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <list>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <unistd.h> 

typedef list<string> StringList;
typedef list<TestCase *> TestCaseList;

#define TEST_TIME 10
#define TEXT_SZ 2048

struct shared_use_st
{
    int currentIndex;               //当前测试用例ID
    char text[TEXT_SZ];             //记录写入和读取的文本
    char result[TEXT_SZ];
};

class SupportClass{

    private:
        StringList testExecPath;            //插桩路径保存
        int currentIndex;                   //当前测试用例序号
        int count;                          //所有测试用例的数目
    public:
        SupportClass();
        /* 测试用例操作 */
        void showTestCaseList();
        TestCase* getCurrentTestCase();
        void setCurrentTestCase();

        int getParamValueWithNameAndKey(string processName,string key);
        /* 用例执行完成后 设置测试结果 */
        void setResultAfterExe(string exeSituation);
        void setTestCaseExecStatus(string exeSituation);
        void setTestCaseResultStatus(string exeSituation);

        /* 根据激励名称以及对应状态 修改 */
        void setProcessStatusWithNameAndStatus(string processName,string status);
        /* 设置激励执行状态 */
        void setProcessExecStatus(string processName);

        /* 测试用例个数 */
        void setCurrentIndex();

        /* 插桩 */
        void deal(string processName,long start,int flag);
        /* 路径 */
        void showPath();
        void cleanPath();
        void testExecPathPush(string flag);

        /* 类型转换 */
        string ltos(long l);
        /* 字符串 按某 字符 分割成list数组 */
        list<string> stringSplit(string s,const char *str);
        /* 信号处理以及设置定时器 */
        void setHandler();
        // void signal_handler(int signum);
        void setAlarm(int seconds);

        /* 共享内存 创建并准备 */
        void createMem();
        /* 将 测试用例实体集 放入 共享内存中 */
        void putTestCasesInMem();
        /* 从 共享内存中 读写测试用例实体集 */
        void getTestCasesInMem();
        /* 分离 共享内存 */
        void pullMem();
};
#endif
