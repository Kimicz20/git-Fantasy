#include "ProcessList.h"
using namespace std;

ProcessList::ProcessList(){
    this->InitList(&(this->processList));
}

/* 单链表初始化 */
Status ProcessList::InitList(LinkList *L)
{
    /* 产生头结点，并使L指向次头结点 */
    *L = new Process{0,FALSE,"","","",NULL};
    if (!(*L))      /* 存储分配失败 */
    {
        return ERROR;
    }
    (*L)->next=NULL;     /* 指针域为空 */

    return OK;
}

/* 返回当前链表 */
LinkList ProcessList::getProcessList(){
    return this->processList;
}

/* 创建激励节点 */
LinkList ProcessList::createProcessNode(string processName,string processParameter,string processStatus)
{
    //创建激励结构体,并赋值结构体数据,指针域为空
    int processID = this->ListLength() + 1;
    LinkList p = new Process{processID,FALSE,processName,processParameter,processStatus,NULL};
    if (!p) {
        cout<< "创建激励节点失败";
    }
    return p;
}

/* 操作结果：在L中插入新节点s，L的长度加1 */
Status ProcessList::ListInsert(LinkList s)
{
    LinkList *L = &(this->processList);
    LinkList p;
    p = *L;
    while (p->next)           /* 寻找最后一个结点 */
    {
        p = p->next;
    }
    s->next=p->next;          /* 将p的后继结点赋值给s的后继结点 */
    p->next=s;                /* 将s赋值给p的后继结点 */
    return OK;
}

/* 初始条件：单链表已存在。操作结果：返回L中数据元素个数 */
int ProcessList::ListLength()
{
    int i = 0;
    LinkList p = this->processList->next;        /* p指向第一个结点 */
    while (p)
    {
        i++;
        p=p->next;
    }
    return i;
}

/* 遍历链表时显示函数 */
string ProcessList::visit(LinkList p)
{
    string str = "";
    string processID = to_string(p->processID);
    str = str +"\n\t" + (p->isExecOK?"√":"x")+" 激励ID : "+processID+" ";
    str = str +"   激励名称 : "+p->processName+" ( ";
    str = str +"   激励参数 : "+((p->processParameter == "")?"空":p->processParameter);
    str = str +"   激励状态 : "+((p->processStatus == "")?"空":p->processStatus)+" ) ";
    return str;
}

/* 操作结果：依次对L的每个数据元素输出 */
string ProcessList::ListTraverse()
{
    string str = "";
    LinkList p = this->processList->next;
    while (p)
    {
        str += visit(p);
        p=p->next;
    }
    return str;
}

/* 操作结果：若L为空表，则返回TRUE,否则返回FALSE */
Status ProcessList::ListIsEmpty()
{
    if (this->processList->next)
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

/* 操作结果：将L重置为空表 */
Status ProcessList::ClearList()
{
    LinkList *L = &(this->processList);
    LinkList p, q;
    p = (*L)->next;      /* p指向第一个结点 */
    while (p)           /* 没到表尾 */
    {
        q = p->next;
        delete(p);
        p = q;
    }
    (*L)->next = NULL;   /* 头结点指针域置为空 */

    return OK;
}
