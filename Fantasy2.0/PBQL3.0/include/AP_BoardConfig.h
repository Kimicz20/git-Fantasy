/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BOARDCONFIG_H__
#define __AP_BOARDCONFIG_H__

#include "AP_HAL.h"
#include "AP_Common.h"
#include "AP_Param.h"

class AP_BoardConfig
{
public:
    // constructor
    AP_BoardConfig(void)
    {
		//AP_Param::setup_object_defaults(this, var_info);
		_pwm_count = 4;
		_ser1_rtscts = 2;
		_ser2_rtscts = 2;
		_safety_enable = 1;
		_sbus_out_enable = 0;
		vehicleSerialNumber=0;
    };

    void init(void);

    //static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Int16 vehicleSerialNumber;


    AP_Int8 _pwm_count;
    AP_Int8 _ser1_rtscts;
    AP_Int8 _ser2_rtscts;
    AP_Int8 _safety_enable;
    AP_Int8 _sbus_out_enable;
};

#endif // __AP_BOARDCONFIG_H__


