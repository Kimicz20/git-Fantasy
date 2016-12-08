// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

void Copter::load_parameters(){

	//输出故障保护的参数设定
	std::cout << "Throttle Failsafe Enable" << std::endl << "0:Disabled,1:Enabled." << std::endl;
	int8_t throttle_failsafe;
	std::cin >> throttle_failsafe;
	g.failsafe_throttle = throttle_failsafe;

	std::cout << "Throttle Failsafe Value" << std::endl << "Range: 925 1100" << std::endl;
	int16_t failsafe_throttle_value;
	std::cin >> failsafe_throttle_value;
	g.failsafe_throttle_value = failsafe_throttle_value;
	//------------------------------------

	//设置switch position与mode的对应关系
	flightmode = new int8_t[6]{0};
	flightmode[0] = STABILIZE;
	flightmode[1] = LAND;
	flightmode[2] = LAND;
	flightmode[3] = LAND;
	flightmode[4] = LAND;
	flightmode[5] = LAND;
	//---------------------------------



}

void Copter::init_ardupilot()
{
	using std::cout;

	//报告无人机版本
	cout << THISFIRMWARE;

    //将源代码所列出的参数列表在构造方法内赋值,这个地方需要写的是用户设置参数的部分（主要是g对象）。
    //load_parameters();
	ahrs.set_correct_centrifugal(false);   //未在模型中描述
    // initialise battery monitor  初始化任务完成
    battery.init();
    // Init RSSI
    //rssi.init();
	//初始化任务完成
    barometer.init();
    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.
    // ap.usb_connected = true;  永远置为false
    //check_usb_mux();

	//信号输入部分，以及马达输出部分初始化完成。
    init_rc_in();               // sets up rc channels from radio  100行
    init_rc_out();              // sets up motors and output to escs   100行

    // initialise which outputs Servo and Relay events can use
    //ServoRelayEvents.set_channel_mask(~motors.get_motor_mask());

    //relay.init();

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    //hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // Do GPS init  初始化完成。
    gps.init();


	//完成  未在模型里描述
    if(g.compass_enabled)
        init_compass();

    // initialise attitude and position controllers  未在模型里描述
    attitude_control.set_dt(MAIN_LOOP_SECONDS);
	//位置控制器的初始化暂时不用----------------------------------位置控制器的初始化暂时不用
    //pos_control.set_dt(MAIN_LOOP_SECONDS);

    // init the optical flow sensor
    //init_optflow();
//#if CONFIG_SONAR == ENABLED
//	init_sonar();---------------------------------------对声呐的初始化，不知道是不是必不可少的装置。
//#endif
    // read Baro pressure at ground
    //-----------------------------未在模型里描述
    init_barometer(true);

    // initialise AP_RPM library
    //rpm_sensor.init();

    // initialise mission library
    //mission.init();

    // initialise the flight mode and aux switch
    // ---------------------------
    reset_control_switch();
    //init_aux_switches();


	//未在模型中描述
    startup_INS_ground();

    // set landed flags
    set_land_complete(true);
    set_land_complete_maybe(true);

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    //serial_manager.set_blocking_writes_all(false);

    // enable CPU failsafe  未添加到模型中
    failsafe_enable();

	//开始记录到日志文件，可以输出到控制台
    //ins.set_raw_logging(should_log(MASK_LOG_IMU_RAW));
    //ins.set_dataflash(&DataFlash);

    // init vehicle capabilties
    //init_capabilities();

	//此处可以输出到控制台
    //cliSerial->print_P(PSTR("\nReady to FLY "));

    // flag that initialisation has completed
    ap.initialised = true;

	cout << "\nReady to FLY \n";

	//用户设置的参数部分（针对每个对象的具体设置。这个函数要放到各个组件初始化结束后执行，否则各个组件又把用户设定的参数给初始化掉了。）
	//load_parameters();
}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
void Copter::startup_INS_ground()
{
    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();
    ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);

    // Warm up and calibrate gyro offsets
    ins.init(ins_sample_rate);

    // reset ahrs including gyro bias
    ahrs.reset();
}

// calibrate gyros - returns true if succesfully calibrated
bool Copter::calibrate_gyros()
{
    // gyro offset calibration
    ins.init_gyro();

    // reset ahrs gyro bias
    if (ins.gyro_calibrated_ok_all()) {
        ahrs.reset_gyro_drift();
        return true;
    }

    return false;
}

// position_ok - returns true if the horizontal absolute position is ok and home position is set
bool Copter::position_ok()
{
    // return false if ekf failsafe has triggered
    if (failsafe.ekf) {
        return false;
    }

    // check ekf position estimate
    return (ekf_position_ok() || optflow_position_ok());
}

// ekf_position_ok - returns true if the ekf claims it's horizontal absolute position estimate is ok and home position is set
bool Copter::ekf_position_ok()
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // if disarmed we accept a predicted horizontal position
    if (!motors.armed()) {
        return ((filt_status.flags.horiz_pos_abs || filt_status.flags.pred_horiz_pos_abs));
    } else {
        // once armed we require a good absolute position and EKF must not be in const_pos_mode
        return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);
    }
}

// optflow_position_ok - returns true if optical flow based position estimate is ok
bool Copter::optflow_position_ok()
{
    return false;
}

// update_auto_armed - update status of auto_armed flag
void Copter::update_auto_armed()
{
    // disarm checks
    if(ap.auto_armed){
        // if motors are disarmed, auto_armed should also be false
        if(!motors.armed()) {
            set_auto_armed(false);
            return;
        }
        // if in stabilize or acro flight mode and throttle is zero, auto-armed should become false
        if(mode_has_manual_throttle(control_mode) && ap.throttle_zero && !failsafe.radio) {
            set_auto_armed(false);
        }
#if FRAME_CONFIG == HELI_FRAME
        // if helicopters are on the ground, and the motor is switched off, auto-armed should be false
        // so that rotor runup is checked again before attempting to take-off
        if(ap.land_complete && !motors.rotor_runup_complete()) {
            set_auto_armed(false);
        }
#endif // HELI_FRAME
    }else{
        // arm checks

#if FRAME_CONFIG == HELI_FRAME
        // for tradheli if motors are armed and throttle is above zero and the motor is started, auto_armed should be true
        if(motors.armed() && !ap.throttle_zero && motors.rotor_runup_complete()) {
            set_auto_armed(true);
        }
#else
        // if motors are armed and throttle is above zero auto_armed should be true
        if(motors.armed() && !ap.throttle_zero) {
            set_auto_armed(true);
        }
#endif // HELI_FRAME
    }
}

void Copter::check_usb_mux(void)
{
    //bool usb_check = hal.gpio->usb_connected();
    //if (usb_check == ap.usb_connected) {
    //    return;
    //}

    //// the user has switched to/from the telemetry port
    ap.usb_connected = false;
}
