// Implementation of the SupportClass class
#include "SupportClass.h"

using namespace std;
using namespace tinyxml2;

SupportClass::SupportClass(){
	this->projectPath = this->getProjectPath();
	this->createMem();
}
SupportClass::~SupportClass(){
	delete this->shared;
}

/* 获取项目路径 */
string SupportClass::getProjectPath(){
	//获取程序路径
	char buf[ 1024 ];
	getcwd(buf, 1024);
	string fulldir(buf);
	string tmp = "/Controller";
	fulldir = fulldir.substr(0,fulldir.length()-tmp.length());
	return fulldir;
}

/* 轮训检查 */
void SupportClass::polling(pid_t pid){
	int s,ifExited,status = 0;
	pid_t w;
	/* 轮询检测 ：非阻塞方式 WNOHANG */
	// w = waitpid(pid, &status, WNOHANG);
	do {
		w = waitpid(pid, &status, WUNTRACED | WCONTINUED);
		//进程是否为正常退出的,如果是,会返回一个非零值
		ifExited = WIFEXITED(status);
		if (ifExited) {
			//提取子进程的返回值
			s = WEXITSTATUS(status);
			if (s == 0) {
				cout << "正常结束！" <<endl;
				break;
			}else{
				cout<<"++++++程序异常退出++++++\n++++++重启被测程序++++++\n";
				//程序异常
				this->flag = true ;
				break;
			}
		}else{
			break;
		}
	} while (true);
}

/* 创建 进程 并做轮巡处理*/
void SupportClass::createPidAndPolling(){
	//2.创建子程序
	pid_t pid = fork();

	//2.1 控制程序轮询 操作进程 执行状况，并做判断
	switch (pid) {
		case -1:  
            cout << "创建子进程失败!" <<endl;   
            break;  
		case 0:  
			//开启子进程
			execlp("make","make","execp", 0);   
            break;
		default:
			//控制进程 轮询检测
			this->polling(pid);
			break;  
	}
}

/* XML文件处理 */
bool SupportClass::ReadXmlFile(string szFileName)
{//读取Xml文件，并遍历
    try
    {
		//1.创建一个XML的文档对象。
		XMLDocument doc;
		//2.获取文件
		szFileName = this->projectPath+"/"+ szFileName;
	    doc.LoadFile(szFileName.c_str());
		//3.获得根元素
	    XMLElement *scene = doc.RootElement();

		//4.创建测试用例实体
		TestCase *tCase;

		//5.循环遍历所有子节点
		if(scene){
			XMLElement *testcase=scene->FirstChildElement("testcase");
			while (testcase)
		    {
				//5.1读取测试用例 ID编号
				int testCaseID = this->index;
				tCase = new TestCase(testCaseID);
				//5.2读取激励函数 以及 参数，格式 激励名称_参数表
				XMLElement *process=testcase->FirstChildElement("process");
				while(process){
					//5.3读取激励函数 以及 参数
					string processName(process->FirstChildElement("operation")->GetText());
					string processParameter(process->FirstChildElement("input")->GetText());
					//5.4创建激励实体类
					tCase->setProcessList(processName,processParameter,"");
					process=process->NextSiblingElement();
				}
				//5.5向测试用例表中添加测试用例
				this->testCaseList.push_back(tCase);
				testcase = testcase->NextSiblingElement();
				if(testcase)
					this->index++;
		    }
		}
    }
    catch (string& e)
    {
        return false;
    }
    return true;
}

/* 展示 测试用例 */
string SupportClass::showTestCaseList(){
	string str = "";
	//定义list的迭代器
    list<TestCase *>::iterator iter;
	//进行迭代遍历
    for(iter = testCaseList.begin(); iter != testCaseList.end(); iter++)
    {
        str += (*iter)->showTestCase()+"\n";
    }
	return str;
}

/* 获取 测试用例实体集中编号为ID的测试用例 */
TestCase* SupportClass::getTestCaseAtIndex(int ID){
	//定义list的迭代器
    list<TestCase* >::iterator iter;
	//进行迭代遍历
    for(iter = testCaseList.begin(); iter != testCaseList.end(); iter++)
    {
		if((*iter)->getTestCaseID() == ID){
			return (*iter);
		}
    }
	return NULL;
}
/* 共享内存 创建并准备 */
void SupportClass::createMem(){

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
}

/* 将 测试用例实体集 放入 共享内存中 */
void SupportClass::putTestCasesInMem(){
	//1.获取测试用例文件 并封装成实体
	this->ReadXmlFile("test.xml");
	shared->currentIndex = 1;
	//2.保存在共享内存中
	strncpy(shared->text, this->showTestCaseList().c_str(),this->showTestCaseList().size());
}

/* 从 共享内存中 读写测试用例实体集 */
void SupportClass::getTestCasesInMem(){
	std::cout << "currentIndex :" << shared->currentIndex <<std::endl;
}

void SupportClass::showResult(){

	cout<< "---------------------------" << endl <<shared->text<<endl;
}

int SupportClass::getCurrentIndex(){
	return shared->currentIndex;
}
int SupportClass::getTestCaseCount(){
	return testCaseList.size();
}

/* 分离 共享内存 */
void SupportClass::pullMem(){

	cout<< "---------------------------" << endl <<shared->result<<endl;
	//把共享内存从当前进程中分离
	if(shmdt(shm) == -1)
	{
		cout << "共享内存从当前进程中分离失败!" << endl;
		exit(EXIT_FAILURE);
	}
	//删除共享内存
	if(shmctl(shmid, IPC_RMID, 0) == -1)
    {
        fprintf(stderr, "shmctl(IPC_RMID) failed\n");
        exit(EXIT_FAILURE);
    }else{
		std::cout << "内存区域删除成功！" << std::endl;
	}
}
