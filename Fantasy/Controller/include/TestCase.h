/*
 *   测试用例链表类
 */

#ifndef TestCase_class
#define TestCase_class

#include "ProcessList.h"
#include "stdio.h"
#include <iostream>
#include <cstring>

//测试用例实体类
class TestCase
{
    private:
        //用例ID号
        int testCaseID;
        //激励链表
        ProcessList* processList;
        //测试执行状态(用来链接测试执行中激励链)
        string execStatus;
        //结果状态
        string resultStatus;

    public:
        /* 初始化构造函数 */
        TestCase(){};
        TestCase(int testCaseID);

        /* 打印测试用例 */
        string showTestCase();
        /* 获取测试用例ID */
        int getTestCaseID();
        /* 设置激励链表 */
        void setProcessList(string processName,string processParameter,string processStatus);

};
#endif
