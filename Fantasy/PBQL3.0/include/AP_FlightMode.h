#ifndef AP_FlightMode_H_
#define AP_FlightMode_H_

#include "AP_Common.h"

class AP_FlightMode
{
public:
	AP_FlightMode(){
		control_mode = STABILIZE;
		desired_mode = STABILIZE;
	}
	

	//set flight mode
	bool set_mode(uint8_t mode);
	uint8_t get_control_mode();
	void update_flight_mode();
	void exit_mode(uint8_t old_control_mode, uint8_t new_control_mode);
	bool mode_requires_GPS(uint8_t mode);
	bool mode_has_manual_throttle(uint8_t mode);
	bool mode_allows_arming(uint8_t mode, bool arming_from_gcs);
	void notify_flight_mode(uint8_t mode);

	~AP_FlightMode(){

	}

private:
	uint8_t control_mode;
	uint8_t desired_mode;
};

#endif