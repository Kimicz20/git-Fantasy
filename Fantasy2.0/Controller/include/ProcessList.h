#ifndef ProcessList_class
#define ProcessList_class

#include "stdio.h"
#include "stdlib.h"
#include <iostream>
#include <string>
using namespace std;

/* 定义 */
#define OK 1
#define ERROR 0
#define TRUE 1
#define FALSE 0

/* 别名定义  */
typedef bool Status;

//激励节点
typedef struct Process
{
    /* 数据域 */
    int processID;              //激励ID
    bool isExecOK;              //激励是否执行OK
    string processName;         //激励名称
    string processParameter;    //激励参数
    string processStatus;       //激励状态
    /* 指针域 */
    struct Process *next;
}Process;

typedef struct Process *LinkList;      /* 定义LinkList */

class ProcessList{
    private:
        LinkList processList;
    public:

        ProcessList();

        /* 返回当前链表 */
        LinkList getProcessList();

        /* 创建激励节点 */
        LinkList createProcessNode(string processName,string processParameter,string processStatus);

        /* 操作结果：在L中插入新节点s，L的长度加1 */
        Status ListInsert(LinkList s);

        /* 单链表初始化 */
        Status InitList(LinkList *L);

        /* 初始条件：单链表已存在。操作结果：返回L中数据元素个数 */
        int ListLength();

        /* 遍历链表时显示函数 */
        string visit(LinkList p);

        /* 初始条件：单链表L已存在 */
        /* 操作结果：依次对L的每个数据元素输出 */
        string ListTraverse();

        /* 初始条件：单链表L已存在 */
        /* 操作结果：若L为空表，则返回TRUE,否则返回FALSE */
        Status ListIsEmpty();

        /* 初始条件：单链表L已存在 */
        /* 操作结果：将L重置为空表 */
        Status ClearList();

        /* 循环查询链表中 每个激励的状态是否 为OK*/
        string findIsSuccessTest();

};
#endif
