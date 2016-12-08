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
        ~TestCase();
        TestCase(int testCaseID);
        int getTestCaseID();
        int getProcessNum();
        string getExecStatus();
        string getResultStatus();

        /* 设置当前激励执行状态 */
        void setcurrentProcessExecStatus(string processName);
        /* 打印测试用例 */
        string showTestCase();

        /* 设置激励链表 */
        void setProcessList(string processName,string processParameter,string processStatus);

        /* 根据激励名称以及参数名 获取参数值 */
        string getParamValueWithNameAndKey(string processName,string key);

        /* 根据激励名称以及对应状态 修改 */
        void setProcessStatusWithNameAndStatus(string processName,string status);

        /* 根据激励名称 获取激励节点 */
        LinkList findProcessWithName(string processName);

        /* 根据激励ID 获取激励节点 */
        LinkList findProcessWithID(int ID);

        /* 设置当前测试用例的执行状态*/
        void setCurrenetTestCaseExecStatus(string execStatus);

        /* 设置结果状态 */
        void setCurrenetTestCaseResultStatus(string exeSituation);

};
#endif
