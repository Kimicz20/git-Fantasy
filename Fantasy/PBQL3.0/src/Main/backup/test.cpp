#include "Copter.h"
#include "SupportClass.h"
#include <pthread.h>

using namespace std;
pthread_t socket_thread;

void read_radio_test();

Copter copter;

int main()
{
	/* 被测程序预处理过程 */
    while(true){
       //1.创建所需的对象
		copter.supt = new SupportClass();

	    string xmlName = copter.supt->serverSocket.serverReceive();

		copter.supt->ReadXmlFile(xmlName);
		// copter.supt->showTestCaseList();

		//4.被测程序运行
		// while (clock() - 2000 < 0);
		read_radio_test();
    }

	return 0;
}


void read_radio_test()
{
	/* 初始化过程 ，无需赋值 在radio.cpp内*/
	copter.init_rc_in();

	int index = 1;
	while(index <= copter.supt->getIndex()){
		copter.supt->setCurrentTestCase(index);
		index++;
		/* 初始化过程 ，需赋值 在system.cpp内*/
		copter.load_parameters();
	//while (clock()%10==0)
	//{
		/* 插桩路径记录 */
		long start = clock();
		copter.supt->deal("read_radio",start,1);
		copter.read_radio();  //radio.cpp
		/* 插桩路径记录 */
		long end = clock();
		copter.supt->deal("read_radio",end,2);
		copter.supt->deal("read_radio",(end-start),3);
		copter.supt->setTestCaseExecStatus();
		copter.supt->setTestCaseResultStatus();

		std::cout << (int)copter.has_new_input << endl
			<< (int)copter.channel_roll->control_in << endl
			<< (int)copter.channel_throttle->control_in << endl
			<< (int)copter.failsafe.radio << endl;

		std::cout<<"测试路径 :"<<endl;
			copter.supt->showPath();
		std::cout<<endl<<"测试执行完后 :"<<endl<<copter.supt->getCurrentTestCase()->showTestCase()<<endl;
		std::cout<<endl;
		// copter.supt->serverSocket.sendResult(copter.supt->getCurrentTestCase()->showTestCase());
		copter.supt->cleanPath();
	}
}
