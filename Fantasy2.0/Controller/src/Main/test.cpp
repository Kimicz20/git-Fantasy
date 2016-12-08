#include "ServerSocket.h"
#include "SupportClass.h"

using namespace std;

#define POST 5555
extern ServerSocket *serverSocket = NULL;
extern SupportClass *supt = new SupportClass();

int main()
{
	//1.创建连接Socket
	// serverSocket = new ServerSocket(POST);

	//1.1文件接收
	// serverSocket->serverReceive();

	//1.测试用例放入共享内存中
	supt->putTestCasesInMem();
	//2.创建子程序
	do {
		//出现异常时 flag 为true
		supt->flag = false;
		supt->createPidAndPolling();

		std::cout << "当前测试用例ID:"<< supt->getCurrentIndex() <<"	所有测试用例数目:"<< supt->getTestCaseCount() << std::endl;
	} while(supt->flag && (supt->getCurrentIndex()< supt->getTestCaseCount()));
	supt->pullMem();
	return 0;
}
