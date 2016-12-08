#include "Copter.h"

extern Copter copter = Copter();
using namespace std;

int main()
{
	/* 被测程序预处理过程 */
	//1.创建所需的对象
	copter.supt = new SupportClass();

	//2.被测程序运行
	copter.setup();
	int starttime = clock();
	int i= 0 ;
	while (clock()-starttime<=3000)
	{
		if (clock()%3==0)//每3ms执行一次
		{
			// cout <<"starttime :"<<starttime<<" time :"<<clock()<<endl;
			std::cout << copter.supt->getCurrentIndex()<< " < "<<copter.supt->getTestCaseCount() << std::endl;
			if((copter.supt->getCurrentIndex() == copter.supt->getTestCaseCount())){
				i++;
				if(i == 2)
					break;
			}
			copter.loop();
		}
	}
	copter.supt->pullMem();
	return 0;
}
