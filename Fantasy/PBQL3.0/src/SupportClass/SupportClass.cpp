// Implementation of the SupportClass class
#include "SupportClass.h"
using namespace std;
extern TestCase* currentTestCase = new TestCase();          //当前测试用例
shared_use_st *shared;             						    //共享内存区
void *shm = NULL;				    					    //分配的共享内存的原始首地址

SupportClass::SupportClass(){
	this->createMem();
	this->currentIndex = shared->currentIndex;
}

/*  前测试用例操作  */
TestCase* SupportClass::getCurrentTestCase(){
	return currentTestCase;
}

/* 设置当前 测试用例序号 同步到内存区域中*/
void SupportClass::setCurrentIndex(){
	shared->currentIndex = this->currentIndex;
}

/* 设置当前 测试用例实体类 从内存中获取*/
void SupportClass::setCurrentTestCase(){
	this->getTestCasesInMem();
}

/* 设置激励回写状态：包括 测试用例执行状态 以及测试用例结果状态
* 如果执行中没有 出现异常情况：
*	1. 正常执行
*	2. 按照用例执行不会出现死循环以及异常错误
*/
void SupportClass::setResultAfterExe(string exeSituation){
	//1. 设置当前测试用例执行状态
	this->setTestCaseExecStatus(exeSituation);
	//2. 设置当前测试用例结果状态
	currentTestCase->setCurrenetTestCaseResultStatus(exeSituation);
}

void SupportClass::setTestCaseExecStatus(string exeSituation){
	if(exeSituation == "OK"){
		string str;
		//获取 测试队列中 第一条和最后一条测试激励的时间差
		list<string>::iterator iter;
		iter = testExecPath.begin();
		str = (*iter).substr(((*iter).find(":")+1),((*iter).size()-1));
		int begin = atoi(str.c_str());
		iter = --testExecPath.end();
		str = (*iter).substr(((*iter).find(":")+1),((*iter).size()-1));
		long end = atoi(str.c_str());

		currentTestCase->setCurrenetTestCaseExecStatus(" 测试耗时 :"+this->ltos(end-begin));
	}else{
		currentTestCase->setCurrenetTestCaseExecStatus(exeSituation);
	}

}

void SupportClass::setTestCaseResultStatus(string exeSituation){

}

/* 根据激励名称以及对应状态 修改 */
void SupportClass::setProcessStatusWithNameAndStatus(string processName,string status){
	currentTestCase->setProcessStatusWithNameAndStatus(processName,status);
}

//在当前 测试用例 中根据激励名称以及参数名 获取参数值
int SupportClass::getParamValueWithNameAndKey(string processName,string key){

	string value = currentTestCase->getParamValueWithNameAndKey(processName,key);
	if(value == "true"){
		return 1;
	}else if (value == "true") {
		return 0;
	}else{
		return atoi(value.c_str());
	}
}

/* 设置执行状态 */
void SupportClass::setProcessExecStatus(string processName){
	currentTestCase->setcurrentProcessExecStatus(processName);
}

/* 插桩操作 */
void SupportClass::deal(string processName,long mtime,int flag){
	/* 插桩路径记录 */
	string str = "";
	if(flag == 1){
		//激励前插桩
		 str = processName+"[ begin time :"+this->ltos(mtime)+"]";
		 this->testExecPathPush(str);
	}else if(flag == 2 ){
		//激励后插桩
		 str = processName+"[ end time :"+this->ltos(mtime)+"]";
		 this->testExecPathPush(str);
	}else{
		//激励执行状态 以及 执行时间
		this->setProcessExecStatus(processName);
		this->setProcessStatusWithNameAndStatus(processName,this->ltos(mtime));
	}
}

/*	路径操作 */
void SupportClass::showPath(){
	int index = 0;
	string str = "";
	//定义list的迭代器
    list<string>::iterator iter;
	//进行迭代遍历
    for(iter = testExecPath.begin(); iter != testExecPath.end(); iter++)
    {
		cout<<(*iter)<<endl;
    }
}

void SupportClass::testExecPathPush(string flag){
	this->testExecPath.push_back(flag);
}
void SupportClass::cleanPath(){
	this->testExecPath.clear();
}

/*	类型转换 */
string SupportClass::ltos(long l)
{
    ostringstream os;
    os<<l;
    string result;
    istringstream is(os.str());
    is>>result;
    return result;
}

/*  信号处理以及设置定时器 */
void signal_handler(int signum)
{
    static int flag = 0;

    switch(signum)
    {
        //用alarm函数设置的timer超时或setitimer函数设置的interval timer超时
        case SIGALRM:
            if(flag == 0)
            {
                cout << "程序异常或出现死循环，3秒后退出！\n";
                //10秒倒计时
                alarm(3);
            }else{
                //传送信号给指定的进程 ,SIGKILL:中止某个进程
				//对出错的用例进行操作处理
				string str = shared->result;
				string exeSituation = "程序出现死循环或者抛出异常！";
				currentTestCase->setCurrenetTestCaseExecStatus(exeSituation);
				currentTestCase->setCurrenetTestCaseResultStatus(exeSituation);
				string tmp = currentTestCase->showTestCase();
				shared->currentIndex++;
				cout <<"result:"<< str << endl <<"tmp:"<<tmp<<endl;
				if(shmdt(shm) == -1)
				{
					cout << "共享内存从当前进程中分离失败!" << endl;
					exit(EXIT_FAILURE);
				}
				strcpy(shared->result+str.size(),tmp.c_str());
				raise(SIGKILL);
                kill(getppid(),SIGKILL);
            }

            flag = 1;
            break;
        //由终端按键产生，通常是CTRL+C或者DELETE
        //此处用作取消 关闭
        case SIGINT:
            cout << "The alarm is cancel.\n";
            //取消以前的闹钟时钟
            alarm(0);
            break;
    }
}

void SupportClass::setHandler(){
	signal(SIGALRM,signal_handler);
}

void SupportClass::setAlarm(int seconds){
	alarm(seconds);
}

/* 共享内存 创建并准备 */
void SupportClass::createMem(){
	int shmid;						//共享内存标识符

    //1.创建共享内存
    shmid = shmget((key_t)1234, sizeof(struct shared_use_st), 0666|IPC_CREAT);

	if(shmid == -1)
    {
        cout << "创建共享内存失败!" << endl;
        exit(EXIT_FAILURE);
    }

	//2.将共享内存连接到当前进程的地址空间
    shm = shmat(shmid, 0, 0);
    if(shm == (void*)-1)
    {
        cout << "共享内存连接到当前进程失败!" << endl;
        exit(EXIT_FAILURE);
    }

	//3.设置共享内存
	shared = (struct shared_use_st *)shm;

	//4.统计所有用例的数目
	string str = shared->text,tmp="测试用例ID: ";
	this->count = std::atoi(str.substr(str.rfind(tmp)+tmp.size(),1).c_str());
}

/* 将 测试用例实体集 放入 共享内存中 */
void SupportClass::putTestCasesInMem(){
	//更新 共享内存区中 当前测试用例号
	this->setCurrentIndex();
	//更新 数据区域
	string str = shared->result,tmp = currentTestCase->showTestCase();
	if(this->currentIndex < this->count)
		tmp += "\n";
	strcpy(shared->result+str.size(),tmp.c_str());
	if(this->currentIndex < this->count)
		this->currentIndex ++;
}

/* 字符串 按某 字符 分割成list数组 */
list<string> SupportClass::stringSplit(string s,const char *str){
	int l = 0;
	int r = 0;
	list<string> arr;
	string tmp(str);
	while(r != std::string::npos)
	{
		r = s.find_first_of(tmp,l);
		if(r != std::string::npos)
			arr.push_back(s.substr(l,r - l));
		else
			arr.push_back(s.substr(l,s.length() - l));
		l = r + tmp.length();
	}
	return arr;
}

/* 从 共享内存中 读写测试用例实体集 */
void SupportClass::getTestCasesInMem(){

	string testcase = shared->text,tmp = "测试用例ID: ";

 	//获取 当前测试用例的字符串
	int begin = testcase.find(tmp+to_string(this->currentIndex));
	int len = testcase.find(tmp+to_string(this->currentIndex+1)) - begin;
	string tcStr = testcase.substr(begin,len-1);

	//根据ID 构造测试用例实体
	currentTestCase = new TestCase(this->currentIndex);

	//构造激励链表
	tmp = "  -->激励链表: [";
	begin = tcStr.find(tmp) + tmp.size()+2;
	tmp = "  -->测试执行状态";
	len = tcStr.find(tmp) - begin - 3;
	string pStr = tcStr.substr(begin,len);

	list<string> plist = this->stringSplit(pStr,"\n");

	//定义list的迭代器
    list<string>::iterator iter;
	//进行迭代遍历
    for(iter = plist.begin(); iter != plist.end(); iter++)
    {
		tmp = "激励名称 : ";
		begin = (*iter).find(tmp) + tmp.size();
		tmp = "(";
		len = (*iter).find(tmp) - begin;
		string processName = (*iter).substr(begin,len-1);
		tmp = "激励参数 : ";
		begin = (*iter).find(tmp) + tmp.size();
		tmp = "   激励状态 : ";
		len = (*iter).find(tmp) - begin;
		string processParameter = (*iter).substr(begin,len);
		currentTestCase->setProcessList(processName,processParameter,"");
    }
	// cout << currentTestCase->showTestCase()<<endl;
}

/* 分离 共享内存 */
void SupportClass::pullMem(){
	//把共享内存从当前进程中分离
	if(shmdt(shm) == -1)
	{
		cout << "共享内存从当前进程中分离失败!" << endl;
		exit(EXIT_FAILURE);
	}
}
