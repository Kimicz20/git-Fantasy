#include "../../include/TestCase.h"
using namespace std;

/* 初始化操作 */
TestCase::TestCase(int testCaseID)
{
    this->testCaseID = testCaseID;
    this->processList = new ProcessList();
    this->execStatus = "";
    this->resultStatus = "";
}

/* 获取测试用例ID */
int TestCase::getTestCaseID(){
    return this->testCaseID;
}

/* 打印测试用例 */
string TestCase::showTestCase(){
    string str = "";
    string testCaseID = to_string(this->testCaseID);
    str = str + "测试用例ID: "+testCaseID+"\n  -->激励链表: [ ";
    if(this->processList->ListIsEmpty()){
        str += "空";
    }else {
        str += this->processList->ListTraverse();
    }
    str = str + " ]\n  -->测试执行状态: [ ";
    if(this->execStatus == ""){
        str +="空";
    }else{
        str += this->execStatus;
    }
    str +=" ]\n  -->结果状态: [ ";
    if(this->resultStatus == ""){
        str +="空";
    }else{
        str += this->resultStatus;
     }
    str +=" ]";
    return str;
}

/* 设置测试执行状态 */
void TestCase::setProcessList(string processName,string processParameter,string processStatus){
    //1.创建一个激励节点
    LinkList s = this->processList->createProcessNode(processName,processParameter,processStatus);
    //2.添加激励节点在激励链表中
    this->processList->ListInsert(s);
}
