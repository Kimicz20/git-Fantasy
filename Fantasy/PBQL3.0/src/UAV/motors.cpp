/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define LOST_VEHICLE_DELAY      10  // called at 10hz so 1 second

static uint8_t auto_disarming_counter;

// arm_motors_check - checks for pilot input to arm or disarm the copter
// called at 10hz  
//来检测用户通过摇杆来进行加解锁操作。  调用周期为10hz
//可作为一个单独的用例。
void Copter::arm_motors_check()
{
    //static int16_t arming_counter;
	arm_check_throttle = channel_throttle->get_the_control_in();
	arm_check_yaw = channel_yaw->get_the_control_in();
	arm_check_motorstate = motors.armed();
	mode_manual_throttle = mode_has_manual_throttle(control_mode);
    // ensure throttle is down
	if (arm_check_throttle > 0) {
		reset_arming_counter();        
	}
	else
	{
		// full right
		if (arm_check_yaw > 4000) {    //我要给电机解锁

			// increase the arming counter to a maximum of 1 beyond the auto trim counter  会经过10s
			if (arming_counter <= AUTO_TRIM_DELAY) {
				increase_arming_counter();
			}

			// arm the motors and configure for flight  会经过2s
			if (arming_counter == ARM_DELAY && arm_check_motorstate==0) {    //经过了2s，而且电机目前是锁着的状态
				// reset arming counter if arming fail
				init_arm_done = init_arm_motors(false);
				if (init_arm_done==0) {
					reset_arming_counter();
				}
			}

			// arm the motors and configure for flight
			if (arming_counter == AUTO_TRIM_DELAY && motors.armed() && control_mode == STABILIZE) {
				auto_trim_counter = 250;
				// ensure auto-disarm doesn't trigger immediately
				auto_disarming_counter = 0;
			}

			// full left
		}
		else if (arm_check_yaw < -4000) {     //我要给电机加锁

			//在空中的时候直接返回，加锁失败  并且   在其他非自稳模式下无法加锁操作。
			if (mode_manual_throttle==0 && ap.land_complete==0) {
				reset_arming_counter();
				//return;
			}
			else
			{
				// increase the counter to a maximum of 1 beyond the disarm delay
				if (arming_counter <= DISARM_DELAY) {
					increase_arming_counter();
				}

				// disarm the motors
				if (arming_counter == DISARM_DELAY && arm_check_motorstate == 1) {
					init_disarm_motors();
				}
			}
			// Yaw is centered so reset arming counter
		}
		else{
			reset_arming_counter();
		}
	}    
}

// auto_disarm_check - disarms the copter if it has been sitting on the ground in manual mode with throttle low for at least 15 seconds
// called at 1hz
//当无人机落地时候，并且在手动模式下，此时油门信号控制值小于临界值15ms是，自动加锁。
//可作为一个单独的用例。该用例最终的操作只是用来进行加锁操作的。不涉及解锁操作的。
void Copter::auto_disarm_check()
{
    uint8_t disarm_delay = constrain_int16(g.disarm_delay, 0, 127);

    // exit immediately if we are already disarmed, or if auto
    // disarming is disabled
    if (!motors.armed() || disarm_delay == 0) {
        auto_disarming_counter = 0;
        return;
    }

    // always allow auto disarm if using interlock switch or motors are Emergency Stopped
    if ((ap.using_interlock && !motors.get_interlock()) || ap.motor_emergency_stop) {
        auto_disarming_counter++;
    } else {
        bool sprung_throttle_stick = (g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0;
        bool thr_low;
        if (mode_has_manual_throttle(control_mode) || !sprung_throttle_stick) {
            thr_low = ap.throttle_zero;
        } else {
            float deadband_top = g.rc_3.get_control_mid() + g.throttle_deadzone;
            thr_low = g.rc_3.control_in <= deadband_top;
        }

        if (thr_low && ap.land_complete) {   //落地状态标志为0，且油门信号输入值为0时，我们认为可以进行加锁操作
            // increment counter
            auto_disarming_counter++;
        } else {
            // reset counter
            auto_disarming_counter = 0;
        }
    }

    // disarm once counter expires
    if (auto_disarming_counter >= disarm_delay) {
        init_disarm_motors();
        auto_disarming_counter = 0;
    }
}

// init_arm_motors - performs arming process including initialisation of barometer and gyros
//  returns false if arming failed because of pre-arm checks, arming checks or a gyro calibration failure
//这是一个解锁操作，可以让无人机起飞。既然是解锁操作。那么许多检查工作是必不可少的，远比加锁操作复杂。
//首先进行的pre_arm_checks预解锁检查，关于预解锁操作，代码位置在本页244行。详细可查看
//还要找到家的位置，没有的话需要设定。
bool Copter::init_arm_motors(bool arming_from_gcs)
{
    static bool in_arm_motors = false;

    // exit immediately if already in this function  这就是考虑的高并发的情况。所以才有这种情况。
    if (in_arm_motors) {
        return false;
    }
    in_arm_motors = true;

    // run pre-arm-checks and display failures
	//if判断语句里面的arming_from_gcs解释:当对应操作员操作摇杆来进行解锁操作时，此时调用init_arm_motors时，arming_from_gcs传入的自然是false。
    if(!pre_arm_checks(true) || !arm_checks(true, arming_from_gcs)) {
        //AP_Notify::events.arming_failed = true;
        in_arm_motors = false;
        return false;
    }

    // disable cpu failsafe because initialising everything takes a while  用处不大吧。
    failsafe_disable();

    // reset battery failsafe 
    set_failsafe_battery(false);

    // notify that arming will occur (we do this early to give plenty of warning)
    //AP_Notify::flags.armed = true;
    // call update_notify a few times to ensure the message gets out
    /*for (uint8_t i=0; i<=10; i++) {
        update_notify();
    }*/
    // Remember Orientation
    // --------------------
    init_simple_bearing();

    initial_armed_bearing = ahrs.yaw_sensor;

    if (ap.home_state == HOME_UNSET) {
        // Reset EKF altitude if home hasn't been set yet (we use EKF altitude as substitute for alt above home)
        ahrs.resetHeightDatum();
        //Log_Write_Event(DATA_EKF_ALT_RESET);
    } else if (ap.home_state == HOME_SET_NOT_LOCKED) {
        // Reset home position if it has already been set before (but not locked)
        set_home_to_current_location();
    }
    calc_distance_and_bearing();


    // check if we are using motor interlock control on an aux switch
    //set_using_interlock(check_if_auxsw_mode_used(AUXSW_MOTOR_INTERLOCK));


    // if we are using motor interlock switch and it's enabled, fail to arm
    //if (ap.using_interlock && motors.get_interlock()){
    //    //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("Arm: Motor Interlock Enabled"));
    //    //AP_Notify::flags.armed = false;
    //    in_arm_motors = false;
    //    return false;
    //}

    // if we are not using Emergency Stop switch option, force Estop false to ensure motors
    // can run normally
    //if (!check_if_auxsw_mode_used(AUXSW_MOTOR_ESTOP)){
    set_motor_emergency_stop(false);
    // if we are using motor Estop switch, it must not be in Estop position
    //} else if (check_if_auxsw_mode_used(AUXSW_MOTOR_ESTOP) && ap.motor_emergency_stop){
        //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("Arm: Motor Emergency Stopped"));
        //AP_Notify::flags.armed = false;
        /*in_arm_motors = false;
        return false;*/
    //}

    // enable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(true);
    //hal.util->set_soft_armed(true);

    // short delay to allow reading of rc inputs
    delay(30);         //延迟等待收到遥控信号

    // enable output to motors
    enable_motor_output();   

    // finally actually arm the motors
    motors.armed(true);

    // log arming to dataflash
    //Log_Write_Event(DATA_ARMED);   //不考虑

    // log flight mode in case it was changed while vehicle was disarmed
    //DataFlash.Log_Write_Mode(control_mode);    //不考虑

    // reenable failsafe
    failsafe_enable();   

    // perf monitor ignores delay due to arming
    perf_ignore_this_loop();

    // flag exiting this function
    in_arm_motors = false;

    // return success
    return true;
}

// perform pre-arm checks and set ap.pre_arm_check flag
//  return true if the checks pass successfully
//预解锁检查，属于解锁的一部分。
//在本操作中需要完成的有：
//大部分都不要考虑
//气压计
//IMU
//磁力计
//电池监控器。
//各种输入是否进入了故障
bool Copter::pre_arm_checks(bool display_failure)
{
    // exit immediately if already armed
    if (motors.armed()) {
        return true;
    }

    // exit immediately if we've already successfully performed the pre-arm check
	//该情况需要考虑，关于这个代码在615行。
    if (ap.pre_arm_check) {
        // run gps checks because results may change and affect LED colour
        // no need to display failures because arm_checks will do that if the pilot tries to arm
        pre_arm_gps_checks(false);
        return true;
    }

    // succeed if pre arm checks are disabled
    if(g.arming_check == ARMING_CHECK_NONE) {
        set_pre_arm_check(true);
        set_pre_arm_rc_check(true);
        return true;
    }

    // pre-arm rc checks a prerequisite
	//这个代码在570行，可以不考虑这个摇杆的检查了。
    pre_arm_rc_checks();
    if(!ap.pre_arm_rc_check) {
        if (display_failure) {
            //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: RC not calibrated"));
        }
        return false;
    }
    // check Baro
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_BARO)) {
        // barometer health check
        if(!barometer.all_healthy()) {
            if (display_failure) {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Barometer not healthy"));
            }
            return false;
        }
        // Check baro & inav alt are within 1m if EKF is operating in an absolute position mode.
        // Do not check if intending to operate in a ground relative height mode as EKF will output a ground relative height
        // that may differ from the baro height due to baro drift.
        nav_filter_status filt_status = inertial_nav.get_filter_status();
        bool using_baro_ref = (!filt_status.flags.pred_horiz_pos_rel && filt_status.flags.pred_horiz_pos_abs);
        if (using_baro_ref) {
            if (fabsf(inertial_nav.get_altitude() - baro_alt) > PREARM_MAX_ALT_DISPARITY_CM) {
                if (display_failure) {
                    //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Altitude disparity"));
                }
                return false;
            }
        }
    }

    // check Compass
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_COMPASS)) {
        // check the primary compass is healthy
        if(!compass.healthy()) {
            if (display_failure) {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Compass not healthy"));
            }
            return false;
        }

        // check compass learning is on or offsets have been set
        if(!compass.configured()) {
            if (display_failure) {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Compass not calibrated"));
            }
            return false;
        }

        // check for unreasonable compass offsets
        Vector3f offsets = compass.get_offsets();
        if(offsets.length() > COMPASS_OFFSETS_MAX) {
            if (display_failure) {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Compass offsets too high"));
            }
            return false;
        }

        // check for unreasonable mag field length
        float mag_field = compass.get_field().length();
        if (mag_field > COMPASS_MAGFIELD_EXPECTED*1.65f || mag_field < COMPASS_MAGFIELD_EXPECTED*0.35f) {
            if (display_failure) {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Check mag field"));
            }
            return false;
        }

        // check all compasses point in roughly same direction
        if (!compass.consistent()) {
            if (display_failure) {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: inconsistent compasses"));
            }
            return false;
        }

    }

    // check GPS
    if (!pre_arm_gps_checks(display_failure)) {
        return false;
    }



    // check INS
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) {
        // check accelerometers have been calibrated
        if(!ins.accel_calibrated_ok_all()) {
            if (display_failure) {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Accels not calibrated"));
            }
            return false;
        }

        // check accels are healthy
        if(!ins.get_accel_health_all()) {
            if (display_failure) {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Accelerometers not healthy"));
            }
            return false;
        }

        // check all accelerometers point in roughly same direction
        if (ins.get_accel_count() > 1) {
            const Vector3f &prime_accel_vec = ins.get_accel();
            for(uint8_t i=0; i<ins.get_accel_count(); i++) {
                // get next accel vector
                const Vector3f &accel_vec = ins.get_accel(i);
                Vector3f vec_diff = accel_vec - prime_accel_vec;
                float threshold = PREARM_MAX_ACCEL_VECTOR_DIFF;
                if (i >= 2) {
                    /*
                      for boards with 3 IMUs we only use the first two
                      in the EKF. Allow for larger accel discrepancy
                      for IMU3 as it may be running at a different temperature
                     */
                    threshold *= 2;
                }
                if (vec_diff.length() > threshold) {
                    if (display_failure) {
                        //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: inconsistent Accelerometers"));
                    }
                    return false;
                }
            }
        }

        // check gyros are healthy
        if(!ins.get_gyro_health_all()) {
            if (display_failure) {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Gyros not healthy"));
            }
            return false;
        }

        // check all gyros are consistent
        if (ins.get_gyro_count() > 1) {
            for(uint8_t i=0; i<ins.get_gyro_count(); i++) {
                // get rotation rate difference between gyro #i and primary gyro
                Vector3f vec_diff = ins.get_gyro(i) - ins.get_gyro();
                if (vec_diff.length() > PREARM_MAX_GYRO_VECTOR_DIFF) {
                    if (display_failure) {
                        //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: inconsistent Gyros"));
                    }
                    return false;
                }
            }
        }

        // get ekf attitude (if bad, it's usually the gyro biases)
        if (!pre_arm_ekf_attitude_check()) {
            if (display_failure) {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: gyros still settling"));
            }
            return false;
        }
    }

    // check battery voltage
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_VOLTAGE)) {
        if (failsafe.battery || (!ap.usb_connected && battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah))) {
            if (display_failure) {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Check Battery"));
            }
            return false;
        }
    }

    // check various parameter values 不考虑了。
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_PARAMETERS)) {

        // ensure ch7 and ch8 have different functions
        if (check_duplicate_auxsw()) {
            if (display_failure) {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Duplicate Aux Switch Options"));
            }
            return false;
        }

        // failsafe parameter checks
        if (g.failsafe_throttle) {
            // check throttle min is above throttle failsafe trigger and that the trigger is above ppm encoder's loss-of-signal value of 900
            if (channel_throttle->radio_min <= g.failsafe_throttle_value+10 || g.failsafe_throttle_value < 910) {
                if (display_failure) {
                    //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Check FS_THR_VALUE"));
                }
                return false;
            }
        }

        // lean angle parameter check
        if (aparm.angle_max < 1000 || aparm.angle_max > 8000) {
            if (display_failure) {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Check ANGLE_MAX"));
            }
            return false;
        }

        // acro balance parameter check  //不考虑了。
        if ((g.acro_balance_roll > g.p_stabilize_roll.kP()) || (g.acro_balance_pitch > g.p_stabilize_pitch.kP())) {
            if (display_failure) {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: ACRO_BAL_ROLL/PITCH"));
            }
            return false;
        }
    }

    // check throttle is above failsafe throttle
    // this is near the bottom to allow other failures to be displayed before checking pilot throttle
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_RC)) {
        if (g.failsafe_throttle != FS_THR_DISABLED && channel_throttle->radio_in < g.failsafe_throttle_value) {
            if (display_failure) {
    #if FRAME_CONFIG == HELI_FRAME
                gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Collective below Failsafe"));
    #else
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Throttle below Failsafe"));
    #endif
            }
            return false;
        }
    }

    // if we've gotten this far then pre arm checks have completed
    set_pre_arm_check(true);
    return true;
}

// perform pre_arm_rc_checks checks and set ap.pre_arm_rc_check flag
//
void Copter::pre_arm_rc_checks()
{
    // exit immediately if we've already successfully performed the pre-arm rc check
    if( ap.pre_arm_rc_check ) {
        return;
    }

    // set rc-checks to success if RC checks are disabled
    if ((g.arming_check != ARMING_CHECK_ALL) && !(g.arming_check & ARMING_CHECK_RC)) {
        set_pre_arm_rc_check(true);
        return;
    }

    // check if radio has been calibrated
    if(!channel_throttle->radio_min.load() && !channel_throttle->radio_max.load()) {
        return;
    }

    // check channels 1 & 2 have min <= 1300 and max >= 1700
    if (channel_roll->radio_min > 1300 || channel_roll->radio_max < 1700 || channel_pitch->radio_min > 1300 || channel_pitch->radio_max < 1700) {
        return;
    }

    // check channels 3 & 4 have min <= 1300 and max >= 1700
    if (channel_throttle->radio_min > 1300 || channel_throttle->radio_max < 1700 || channel_yaw->radio_min > 1300 || channel_yaw->radio_max < 1700) {
        return;
    }

    // check channels 1 & 2 have trim >= 1300 and <= 1700
    if (channel_roll->radio_trim < 1300 || channel_roll->radio_trim > 1700 || channel_pitch->radio_trim < 1300 || channel_pitch->radio_trim > 1700) {
        return;
    }

    // check channel 4 has trim >= 1300 and <= 1700
    if (channel_yaw->radio_trim < 1300 || channel_yaw->radio_trim > 1700) {
        return;
    }

    // if we've gotten this far rc is ok
    set_pre_arm_rc_check(true);
}

// performs pre_arm gps related checks and returns true if passed
//这个是在解锁过程中，关于当前模式是否用到GPS信息。
//解锁的是否用户是否可以启用GPS,看来设定参数也需要一个单独的用例了。
bool Copter::pre_arm_gps_checks(bool display_failure)
{
    // always check if inertial nav has started and is ready
	//在航姿系统里面如果当前系统时间与上次出现错误时间超过了5s,就认为是healthy。
	//目前不考虑这种情况。
    if(!ahrs.healthy()) {
        if (display_failure) {
            //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Waiting for Nav Checks"));
        }
        return false;
    }

    // check if flight mode requires GPS
    bool gps_required = mode_requires_GPS(control_mode);


	//下面的预处理代表了围栏模式，暂不考虑
//#if AC_FENCE == ENABLED   
//     if circular fence is enabled we need GPS
//    if ((fence.get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) != 0) {
//        gps_required = true;
//    }   
//#endif

    // return true if GPS is not required
    if (!gps_required) {
        //AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // ensure GPS is ok
	//总之就是确定了GPS是否好使，包括家的位置是否确定了。
    if (!position_ok()) {
        if (display_failure) {
            const char *reason = ahrs.prearm_failure_reason();
            if (reason) {
                //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, PSTR("PreArm: %s"), reason);
            } else {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Need 3D Fix"));
            }
        }
        //AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // check EKF compass variance is below failsafe threshold
	//检查磁力计是否可以正常使用
    float vel_variance, pos_variance, hgt_variance, tas_variance;
    Vector3f mag_variance;
    Vector2f offset;
    ahrs.get_variances(vel_variance, pos_variance, hgt_variance, mag_variance, tas_variance, offset);
    if (mag_variance.length() >= g.fs_ekf_thresh) {
        if (display_failure) {
            //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: EKF compass variance"));
        }
        return false;
    }

    // check home and EKF origin are not too far
    if (far_from_EKF_origin(ahrs.get_home())) {
        if (display_failure) {
            //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: EKF-home variance"));
        }
        //AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // return true immediately if gps check is disabled
	//用户输入的是否启用GPS，不用的话肯定就可以跳过去了。
    if (!(g.arming_check == ARMING_CHECK_ALL || g.arming_check & ARMING_CHECK_GPS)) {
        //AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // warn about hdop separately - to prevent user confusion with no gps lock
	//水平精度因子，精度因子包括几何精度因子(GDOP)、位置精度因子(PDOP)、水平精度因子(HDOP)、和时间精度因子(TDOP)
    if (gps.get_hdop() > g.gps_hdop_good) {
        if (display_failure) {
            //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("PreArm: High GPS HDOP"));
        }
        //AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // if we got here all must be ok
    //AP_Notify::flags.pre_arm_gps_check = true;
    return true;
}

// check ekf attitude is acceptable
bool Copter::pre_arm_ekf_attitude_check()
{
    // get ekf filter status
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    return filt_status.flags.attitude;
}

// arm_checks - perform final checks before arming
//  always called just before arming.  Return true if ok to arm
//  has side-effect that logging is started
//	与pre_arm_check差不多的功能。
bool Copter::arm_checks(bool display_failure, bool arming_from_gcs)
{
    // check accels and gyro are healthy
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) {
        if(!ins.get_accel_health_all()) {
            if (display_failure) {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("Arm: Accelerometers not healthy"));
            }
            return false;
        }
        if(!ins.get_gyro_health_all()) {
            if (display_failure) {
               // gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("Arm: Gyros not healthy"));
            }
            return false;
        }
        // get ekf attitude (if bad, it's usually the gyro biases)
        if (!pre_arm_ekf_attitude_check()) {
            if (display_failure) {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("Arm: gyros still settling"));
            }
            return false;
        }
    }

    // always check if inertial nav has started and is ready
    if(!ahrs.healthy()) {
        if (display_failure) {
            //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("Arm: Waiting for Nav Checks"));
        }
        return false;
    }

    if(compass.is_calibrating()) {
        if (display_failure) {
            //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("Arm: Compass calibration running"));
        }
        return false;
    }

    // always check if the current mode allows arming
    if (!mode_allows_arming(control_mode, arming_from_gcs)) {
        if (display_failure) {
            //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("Arm: Mode not armable"));
        }
        return false;
    }

    // always check gps
    if (!pre_arm_gps_checks(display_failure)) {
        return false;
    }

    // succeed if arming checks are disabled
    if (g.arming_check == ARMING_CHECK_NONE) {
        return true;
    }

    // baro checks
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_BARO)) {
        // baro health check
        if (!barometer.all_healthy()) {
            if (display_failure) {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("Arm: Barometer not healthy"));
            }
            return false;
        }
        // Check baro & inav alt are within 1m if EKF is operating in an absolute position mode.
        // Do not check if intending to operate in a ground relative height mode as EKF will output a ground relative height
        // that may differ from the baro height due to baro drift.
        nav_filter_status filt_status = inertial_nav.get_filter_status();
        bool using_baro_ref = (!filt_status.flags.pred_horiz_pos_rel && filt_status.flags.pred_horiz_pos_abs);
        if (using_baro_ref && (fabsf(inertial_nav.get_altitude() - baro_alt) > PREARM_MAX_ALT_DISPARITY_CM)) {
            if (display_failure) {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("Arm: Altitude disparity"));
            }
            return false;
        }
    }

    // check lean angle
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) {
        if (degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()))*100.0f > aparm.angle_max) {
            if (display_failure) {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("Arm: Leaning"));
            }
            return false;
        }
    }

    // check battery voltage
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_VOLTAGE)) {
        if (failsafe.battery || (!ap.usb_connected && battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah))) {
            if (display_failure) {
                //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("Arm: Check Battery"));
            }
            return false;
        }
    }

    // check throttle
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_RC)) {
        // check throttle is not too low - must be above failsafe throttle
        if (g.failsafe_throttle != FS_THR_DISABLED && channel_throttle->radio_in < g.failsafe_throttle_value) {
            if (display_failure) {

            }
            return false;
        }

        // check throttle is not too high - skips checks if arming from GCS in Guided
        if (!(arming_from_gcs && control_mode == GUIDED)) {
            // above top of deadband is too always high
            if (channel_throttle->control_in > get_takeoff_trigger_throttle()) {
                if (display_failure) {

                }
                return false;
            }
            // in manual modes throttle must be at zero
            if ((mode_has_manual_throttle(control_mode) || control_mode == DRIFT) && channel_throttle->control_in > 0) {
                return false;
            }
        }
    }

    // check if safety switch has been pushed
    //if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
    //    if (display_failure) {
    //        //gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("Arm: Safety Switch"));
    //    }
    //    return false;
    //}

    // if we've gotten this far all is ok
    return true;
}

// init_disarm_motors - disarm motors
// 初始化加锁操作，不在让飞了，比较简单，作为复用代码块。
void Copter::init_disarm_motors()
{
    // return immediately if we are already disarmed
    if (!motors.armed()) {
        return;
    }

    // save compass offsets learned by the EKF
    Vector3f magOffsets;
    if (ahrs.use_compass() && ahrs.getMagOffsets(magOffsets)) {
        compass.set_and_save_offsets(compass.get_primary(), magOffsets);
    }

    // we are not in the air
    set_land_complete(true);
    set_land_complete_maybe(true);

    // log disarm to the dataflash
    //Log_Write_Event(DATA_DISARMED);

    // send disarm command to motors
    motors.armed(false);

    // reset the mission
    //mission.reset();

    // suspend logging
    //if (!(g.log_bitmask & MASK_LOG_WHEN_DISARMED)) {
    //    //DataFlash.EnableWrites(false);
    //}

    // disable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(false);
    //hal.util->set_soft_armed(false);
}

// motors_output - send output to motors library which will adjust and send to ESCs and servos
void Copter::motors_output()
{
    // check if we are performing the motor test
    if (ap.motor_test) {
        motor_test_output();
    } else {
        //if (!ap.using_interlock){
        //    // if not using interlock switch, set according to Emergency Stop status
        //    // where Emergency Stop is forced false during arming if Emergency Stop switch
        //    // is not used. Interlock enabled means motors run, so we must
        //    // invert motor_emergency_stop status for motors to run.
        //    motors.set_interlock(!ap.motor_emergency_stop);
        //}
        motors.output();
    }
}

// check for pilot stick input to trigger lost vehicle alarm
void Copter::lost_vehicle_check()
{
    static uint8_t soundalarm_counter;

    // disable if aux switch is setup to vehicle alarm as the two could interfere
    if (check_if_auxsw_mode_used(AUXSW_LOST_COPTER_SOUND)) {
        return;
    }

    // ensure throttle is down, motors not armed, pitch and roll rc at max. Note: rc1=roll rc2=pitch
    if (ap.throttle_zero && !motors.armed() && (channel_roll->control_in > 4000) && (channel_pitch->control_in > 4000)) {
        if (soundalarm_counter >= LOST_VEHICLE_DELAY) {
            /*if (AP_Notify::flags.vehicle_lost == false) {
                AP_Notify::flags.vehicle_lost = true;
                gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("Locate Copter Alarm!"));
            }*/
        } else {
            soundalarm_counter++;
        }
    } else {
        soundalarm_counter = 0;
       /* if (AP_Notify::flags.vehicle_lost == true) {
            AP_Notify::flags.vehicle_lost = false;
        }*/
    }
}
