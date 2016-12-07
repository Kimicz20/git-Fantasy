// Definition of the controllMain class

#ifndef Parameter_class
#define Parameter_class

#include <iostream>
#include <string>
#include <map>
#include "MainSem.h"

class Parameter
{
 public:
 	map<string, string> testCaseMap;
 	map<string, string> parameterMap;
 	MainSem totalSem;

 	//查找在表中是否有对应的属性
 	string getValueWithKey(string key);
 	bool isInMap(string key);
};
#endif
