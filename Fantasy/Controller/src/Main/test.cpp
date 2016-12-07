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
	supt->createPidAndPolling();

	supt->pullMem();
	return 0;
}
