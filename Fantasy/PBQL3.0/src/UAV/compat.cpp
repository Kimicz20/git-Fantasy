#include "Copter.h"

void Copter::delay(uint32_t ms)
{
    //hal.scheduler->delay(ms);
	/*-------------—” ±—≠ª∑ gc-----------*/
	int time_point = 0;
	time_point = clock();
	while (clock() - time_point < ms)
	{
		//do nothing
	}
	/*-------------------------------*/
}

uint32_t Copter::millis()
{
    //return hal.scheduler->millis();
	return 0;
}

uint32_t Copter::micros()
{
    //return hal.scheduler->micros();
	return 0;
}
