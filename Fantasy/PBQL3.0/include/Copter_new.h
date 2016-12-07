//重要头文件
#ifndef _COPTER_H
#define _COPTER_H

#define THISFIRMWARE "APM:Copter V3.4-dev"
#define FIRMWARE_VERSION 3,4,0,FIRMWARE_VERSION_TYPE_DEV

#include <time.h>
#include <iostream>
#include <cmath>
#include <cstdio>
#include <cstdarg>
#include "AP_HAL.h"
// Common dependencies
#include "AP_Common.h"
#include "AP_Progmem.h"
#include "AP_Menu.h"
#include "AP_Param.h"
#include "StorageManager.h"
// Application dependencies
#include "GCS.h"
#include "GCS_MAVLink.h"        // MAVLink GCS definitions
#include "AP_SerialManager.h"   // Serial manager library
#include "AP_GPS.h"             // ArduPilot GPS library
#include "AP_Baro.h"
#include "AP_Compass.h"         // ArduPilot Mega Magnetometer Library
#include "AP_Math.h"            // ArduPilot Mega Vector/Matrix math Library
#include "AP_Curve.h"           // Curve used to linearlise throttle pwm to thrust
#include "AP_InertialSensor.h"  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include "AP_AHRS.h"
#include "AP_NavEKF.h"
#include "AP_NavEKF2.h"
#include "AC_PID.h"             // PID library
#include "AC_PI_2D.h"           // PID library (2-axis)
#include "AC_P.h"               // P library
#include "AC_AttitudeControl_Multi.h" // Attitude control library
#include "AC_PosControl.h"      // Position control library
#include "RC_Channel.h"         // RC Channel Library
#include "AP_Motors.h"          // AP Motors library
#include "AP_RSSI.h"                   // RSSI Library
#include "Filter.h"             // Filter library
#include "AP_Buffer.h"          // APM FIFO Buffer
#include "AP_Airspeed.h"        // needed for AHRS build
#include "AP_Vehicle.h"         // needed for AHRS build
#include "AP_InertialNav.h"     // ArduPilot Mega inertial navigation library
#include "AP_Scheduler.h"       // main loop scheduler
#include "AP_RCMapper.h"        // RC input mapping library
#include "AP_BattMonitor.h"     // Battery monitor library
#include "AP_BoardConfig.h"     // board configuration library
#include "AP_LandingGear.h"     // Landing Gear library
#include "AP_RPM.h"
#include "AP_MotorsQuad.h"
#include "AP_FlightMode.h"
// AP_HAL to Arduino compatibility layer
// Configuration
#include "defines.h"
#include "config.h"
#include "config_channels.h"
// Local modules
#include "Parameters.h"

/*--------------- 辅助类 -----------------*/
#include "SupportClass.h"

class Copter {
public:
	friend class Parameters;

	Copter(void);
public:

	/*-------时间标识变量-----------*/
	uint32_t last_update_radio_ms;  // 关于radio input用例
	uint32_t last_update_radio_elapsed;

	uint32_t last_nonzero_throttle_ms;
	/*-----------------------------*/


	/*--------判断标志位------------*/
	bool has_new_input;

	/*-----------------------------*/


	/*--------存放信号-------------*/
	struct
	{
		int16_t roll_radio_in;
		int16_t pitch_radio_in;
		int16_t throttle_radio_in;
		int16_t yaw_radio_in;
		int16_t roll_control_in;
		int16_t pitch_control_in;
		int16_t throttle_control_in;
		int16_t yaw_control_in;
	}receive_input;
	/*-----------------------------*/

	Parameters g;

	AP_FlightMode flightmode;

	// main loop scheduler
	AP_Scheduler scheduler;

	// used to detect MAVLink acks from GCS to stop compassmot
	uint8_t command_ack_counter;

	// primary input control channels
	RC_Channel  * channel_roll;
	RC_Channel  * channel_pitch;
	RC_Channel  * channel_throttle;
	RC_Channel  * channel_yaw;

	// the rate we run the main loop at
	const AP_InertialSensor::Sample_rate ins_sample_rate;

	AP_GPS gps;
	AP_Baro barometer;
	Compass compass;
	AP_InertialSensor ins;
	AP_RPM rpm_sensor;

	// Inertial Navigation EKF
	NavEKF EKF{ &ahrs, barometer};
	NavEKF2 EKF2{ &ahrs, barometer};
	AP_AHRS_NavEKF ahrs{ ins, barometer, gps, EKF, EKF2, AP_AHRS_NavEKF::FLAG_ALWAYS_USE_EKF };

	// gnd speed limit required to observe optical flow sensor limits
	float ekfGndSpdLimit;

	// scale factor applied to velocity controller gain to prevent optical flow noise causing excessive angle demand noise
	float ekfNavVelGainScaler;

	// system time in milliseconds of last recorded yaw reset from ekf
	uint32_t ekfYawReset_ms = 0;

	// GCS selection
	//AP_SerialManager serial_manager;
	static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS;

	//GCS_MAVLINK gcs[MAVLINK_COMM_NUM_BUFFERS];

	// Documentation of GLobals:
	union {
		struct {
			uint8_t unused1; // 0
			uint8_t simple_mode; // 1,2     // This is the state of simple mode : 0 = disabled ; 1 = SIMPLE ; 2 = SUPERSIMPLE
			uint8_t pre_arm_rc_check; // 3       // true if rc input pre-arm checks have been completed successfully
			uint8_t pre_arm_check; // 4       // true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
			uint8_t auto_armed; // 5       // stops auto missions from beginning until throttle is raised
			uint8_t logging_started; // 6       // true if dataflash logging has started
			uint8_t land_complete; // 7       // true if we have detected a landing
			uint8_t new_radio_frame; // 8       // Set true if we have new PWM data to act on from the Radio
			uint8_t usb_connected; // 9       // true if APM is powered from USB connection
			uint8_t rc_receiver_present; // 10      // true if we have an rc receiver present (i.e. if we've ever received an update
			uint8_t compass_mot; // 11      // true if we are currently performing compassmot calibration
			uint8_t motor_test; // 12      // true if we are currently performing the motors test
			uint8_t initialised; // 13      // true once the init_ardupilot function has completed.  Extended status to GCS is not sent until this completes
			uint8_t land_complete_maybe; // 14      // true if we may have landed (less strict version of land_complete)
			uint8_t throttle_zero; // 15      // true if the throttle stick is at zero, debounced, determines if pilot intends shut-down when not using motor interlock
			uint8_t system_time_set; // 16      // true if the system time has been set from the GPS
			uint8_t gps_base_pos_set; // 17      // true when the gps base position has been set (used for RTK gps only)
			enum HomeState home_state; // 18,19   // home status (unset, set, locked)
			uint8_t using_interlock; // 20      // aux switch motor interlock function is in use
			uint8_t motor_emergency_stop; // 21      // motor estop switch, shuts off motors when enabled
			uint8_t land_repo_active; // 22      // true if the pilot is overriding the landing position
		};
		uint32_t value;
	} ap;

	// This is the state of the flight control system
	// There are multiple states defined such as STABILIZE, ACRO,
	int8_t control_mode;

	// Structure used to detect changes in the flight mode control switch
	struct {
		int8_t debounced_switch_position;   // currently used switch position
		int8_t last_switch_position;        // switch position in previous iteration
		uint32_t last_edge_time_ms;         // system time that switch position was last changed
	} control_switch_state;

	struct {
		bool running;
		float speed;
		uint32_t start_ms;
		uint32_t time_ms;
	} takeoff_state;

	RCMapper rcmap;

	// board specific config
	AP_BoardConfig BoardConfig;

	// receiver RSSI
	uint8_t receiver_rssi;

	// Failsafe
	struct {
		uint8_t rc_override_active; // 0   // true if rc control are overwritten by ground station
		uint8_t radio; // 1   // A status flag for the radio failsafe
		uint8_t battery; // 2   // A status flag for the battery failsafe
		uint8_t gcs; // 4   // A status flag for the ground station failsafe
		uint8_t ekf; // 5   // true if ekf failsafe has occurred

		int8_t radio_counter;            // number of iterations with throttle below throttle_fs_value

		uint32_t last_heartbeat_ms;      // the time when the last HEARTBEAT message arrived from a GCS - used for triggering gcs failsafe
	} failsafe;

	struct
	{
		uint8_t radio;
		uint8_t battery;
		AP_Int16  failsafe_throttle_value;
	} failsafe_config;

	// sensor health for logging
	struct {
		uint8_t baro;    // true if baro is healthy
		uint8_t compass;    // true if compass is healthy
	} sensor_health;

	AP_MotorsQuad motors;

	// key aircraft parameters passed to multiple libraries
	AP_Vehicle::MultiCopter aparm;

	// GPS variables
	// Sometimes we need to remove the scaling for distance calcs
	float scaleLongDown;

	// Location & eigation
	int32_t wp_bearing;
	// The location of home in relation to the copter in centi-degrees
	int32_t home_bearing;
	// distance between plane and home in cm
	int32_t home_distance;
	// distance between plane and next waypoint in cm.
	uint32_t wp_distance;
	uint8_t land_state;              // records state of land (flying to location, descending)

	// Auto
	AutoMode auto_mode;   // controls which auto controller is run

	// Guided
	GuidedMode guided_mode;  // controls which controller is run (pos or vel)

	// RTL
	RTLState rtl_state;  // records state of rtl (initial climb, returning home, etc)
	bool rtl_state_complete; // set to true if the current state is completed
	float rtl_alt;     // altitude the vehicle is returning at

	// Circle
	bool circle_pilot_yaw_override; // true if pilot is overriding yaw

	// SIMPLE Mode
	// Used to track the orientation of the copter for Simple mode. This value is reset at each arming
	// or in SuperSimple mode when the copter leaves a 20m radius from home.
	float simple_cos_yaw;
	float simple_sin_yaw;
	int32_t super_simple_last_bearing;
	float super_simple_cos_yaw;
	float super_simple_sin_yaw;

	// Stores initial bearing when armed - initial simple bearing is modified in super simple mode so not suitable
	int32_t initial_armed_bearing;

	// Throttle variables
	float throttle_average;              // estimated throttle required to hover
	int16_t desired_climb_rate;          // pilot desired climb rate - for logging purposes only

	// Loiter control
	uint16_t loiter_time_max;                // How long we should stay in Loiter Mode for mission scripting (time in seconds)
	uint32_t loiter_time;                    // How long have we been loitering - The start time in millis

	// Flip
	Vector3f flip_orig_attitude;         // original copter attitude before flip

	// Battery Sensors
	AP_BattMonitor battery;

	// Altitude
	// The cm/s we are moving up or down based on filtered data - Positive = UP
	int16_t climb_rate;
	// The altitude as reported by Sonar in cm - Values are 20 to 700 generally.
	bool sonar_enabled; // enable user switch for sonar
	int16_t sonar_alt;
	uint8_t sonar_alt_health;    // true if we can trust the altitude from the sonar
	float target_sonar_alt;      // desired altitude in cm above the ground
	int32_t baro_alt;            // barometer altitude in cm above home
	float baro_climbrate;        // barometer climbrate in cm/s
	LowPassFilterVector3f land_accel_ef_filter; // accelerations for land and crash detector tests

	// 3D Location vectors
	// Current location of the copter (altitude is relative to home)
	struct Location current_loc;

	// Navigation Yaw control
	// auto flight mode's yaw mode
	uint8_t auto_yaw_mode;

	// Yaw will point at this location if auto_yaw_mode is set to AUTO_YAW_ROI
	Vector3f roi_WP;

	// bearing from current location to the yaw_look_at_WP
	float yaw_look_at_WP_bearing;

	// yaw used for YAW_LOOK_AT_HEADING yaw_mode
	int32_t yaw_look_at_heading;

	// Deg/s we should turn
	int16_t yaw_look_at_heading_slew;

	// heading when in yaw_look_ahead_bearing
	float yaw_look_ahead_bearing;

	// Delay Mission Scripting Command
	int32_t condition_value;  // used in condition commands (eg delay, change alt, etc.)
	uint32_t condition_start;

	// IMU variables
	// Integration time (in seconds) for the gyros (DCM algorithm)
	// Updated with the fast loop
	float G_Dt;

	// Inertial Navigation
	AP_InertialNav_NavEKF inertial_nav;

	// Attitude, Position and Waypoint navigation objects
	// To-Do: move inertial nav up or other navigation variables down here
	AC_AttitudeControl_Multi attitude_control;
	AC_PosControl pos_control;

	// Performance monitoring
	int16_t pmTest1;

	// System Timers
	// --------------
	// Time in microseconds of main control loop
	uint32_t fast_loopTimer;
	// Counter of main loop executions.  Used for performance monitoring and failsafe processing
	uint16_t mainLoop_count;
	// Loiter timer - Records how long we have been in loiter
	uint32_t rtl_loiter_start_time;

	// Used to exit the roll and pitch auto trim function
	uint8_t auto_trim_counter;

	// RSSI
	//AP_RSSI rssi;

	// use this to prevent recursion during sensor init
	bool in_mavlink_delay;

	// true if we are out of time in our event timeslice
	bool gcs_out_of_time;

	// Top-level logic
	// setup the var_info table
	AP_Param param_loader;

	static const AP_Scheduler::Task scheduler_tasks[];
	//static const AP_Param::Info var_info[];
	static const struct LogStructure log_structure[];


	/*-----------add methods here------------------*/
	void set_has_new_input(bool);
	void set_last_update_radio_ms(long);
	uint32_t get_last_update_radio_elapsed(long);
	void increase_radio_counter(void);
	void decrease_radio_counter(void);
	void update_throttle_zero(int8_t);
	void load_parameters(void);
	/*-------------------------------------*/



	void load_component(void);
	void compass_accumulate(void);
	void compass_cal_update(void);
	void barometer_accumulate(void);
	void perf_update(void);
	void fast_loop();
	void rc_loop();
	void throttle_loop();
	void update_mount();
	void update_batt_compass(void);
	void ten_hz_logging_loop();
	void fifty_hz_logging_loop();
	void full_rate_logging_loop();
	void three_hz_loop();
	void one_hz_loop();
	void update_GPS(void);
	void init_simple_bearing();
	void update_simple_mode(void);
	void update_super_simple_bearing(bool force_update);
	void read_AHRS(void);
	void update_altitude();
	void set_home_state(enum HomeState new_home_state);
	bool home_is_set();
	void set_auto_armed(bool b);
	void set_simple_mode(uint8_t b);
	void set_failsafe_radio(bool b);
	void set_failsafe_battery(bool b);
	void set_failsafe_gcs(bool b);
	void set_land_complete(bool b);
	void set_land_complete_maybe(bool b);
	void set_pre_arm_check(bool b);
	void set_pre_arm_rc_check(bool b);
	void set_using_interlock(bool b);
	void set_motor_emergency_stop(bool b);
	float get_smoothing_gain();
	void get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max);
	float get_pilot_desired_yaw_rate(int16_t stick_angle);
	void check_ekf_yaw_reset();
	float get_roi_yaw();
	float get_look_ahead_yaw();
	void update_thr_average();
	void set_throttle_takeoff();
	int16_t get_pilot_desired_throttle(int16_t throttle_control);
	float get_pilot_desired_climb_rate(float throttle_control);
	float get_non_takeoff_throttle();
	float get_takeoff_trigger_throttle();
	float get_throttle_pre_takeoff(float input_thr);
	float get_surface_tracking_climb_rate(int16_t target_rate, float current_alt_target, float dt);
	void set_accel_throttle_I_from_pilot_throttle(int16_t pilot_throttle);
	void update_poscon_alt_max();
	void rotate_body_frame_to_NE(float &x, float &y);
	void gcs_send_heartbeat(void);
	void gcs_send_deferred(void);
	void gcs_send_message(enum ap_message id);
	void gcs_send_mission_item_reached_message(uint16_t mission_index);
	void gcs_data_stream_send(void);
	void gcs_check_input(void);

	//void load_parameters(void);
	void userhook_init();
	void userhook_FastLoop();
	void userhook_50Hz();
	void userhook_MediumLoop();
	void userhook_SlowLoop();
	void userhook_SuperSlowLoop();
	void update_home_from_EKF();
	void set_home_to_current_location_inflight();
	bool set_home_to_current_location();
	bool set_home_to_current_location_and_lock();
	bool set_home_and_lock(const Location& loc);
	bool set_home(const Location& loc);
	bool far_from_EKF_origin(const Location& loc);
	void set_system_time_from_GPS();
	void exit_mission();
	void do_RTL(void);
	bool verify_takeoff();
	bool verify_land();
	bool verify_loiter_unlimited();
	bool verify_loiter_time();
	bool verify_RTL();
	bool verify_wait_delay();
	bool verify_change_alt();
	bool verify_within_distance();
	bool verify_yaw();
	void do_take_picture();
	void log_picture();
	void delay(uint32_t ms);
	uint32_t millis();
	uint32_t micros();
	bool acro_init(bool ignore_checks);
	void acro_run();
	void get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out);
	bool althold_init(bool ignore_checks);
	void althold_run();
	bool auto_init(bool ignore_checks);
	void auto_run();
	void auto_takeoff_start(float final_alt_above_home);
	void auto_takeoff_run();
	void auto_wp_start(const Vector3f& destination);
	void auto_wp_run();
	void auto_spline_run();
	void auto_land_start();
	void auto_land_start(const Vector3f& destination);
	void auto_land_run();
	void auto_rtl_start();
	void auto_rtl_run();
	void auto_circle_movetoedge_start();
	void auto_circle_start();
	void auto_circle_run();
	void auto_nav_guided_start();
	void auto_nav_guided_run();
	bool auto_loiter_start();
	void auto_loiter_run();
	uint8_t get_default_auto_yaw_mode(bool rtl);
	void set_auto_yaw_mode(uint8_t yaw_mode);
	void set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, uint8_t relative_angle);
	void set_auto_yaw_roi(const Location &roi_location);
	float get_auto_heading(void);
	bool autotune_init(bool ignore_checks);
	void autotune_stop();
	bool autotune_start(bool ignore_checks);
	void autotune_run();
	void autotune_attitude_control();
	void autotune_backup_gains_and_initialise();
	void autotune_load_orig_gains();
	void autotune_load_tuned_gains();
	void autotune_load_intra_test_gains();
	void autotune_load_twitch_gains();
	void autotune_save_tuning_gains();
	void autotune_update_gcs(uint8_t message_id);
	bool autotune_roll_enabled();
	bool autotune_pitch_enabled();
	bool autotune_yaw_enabled();
	void autotune_twitching_test(float measurement, float target, float &measurement_min, float &measurement_max);
	void autotune_updating_d_up(float &tune_d, float tune_d_min, float tune_d_max, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max);
	void autotune_updating_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max);
	void autotune_updating_p_down(float &tune_p, float tune_p_min, float tune_p_step_ratio, float target, float measurement_max);
	void autotune_updating_p_up(float &tune_p, float tune_p_max, float tune_p_step_ratio, float target, float measurement_max);
	void autotune_updating_p_up_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max);
	void autotune_twitching_measure_acceleration(float &rate_of_change, float rate_measurement, float &rate_measurement_max);
	bool brake_init(bool ignore_checks);
	void brake_run();
	bool circle_init(bool ignore_checks);
	void circle_run();
	bool drift_init(bool ignore_checks);
	void drift_run();
	int16_t get_throttle_assist(float velz, int16_t pilot_throttle_scaled);
	bool flip_init(bool ignore_checks);
	void flip_run();
	bool guided_init(bool ignore_checks);
	void guided_takeoff_start(float final_alt_above_home);
	void guided_pos_control_start();
	void guided_vel_control_start();
	void guided_posvel_control_start();
	void guided_angle_control_start();
	void guided_set_destination(const Vector3f& destination);
	void guided_set_velocity(const Vector3f& velocity);
	void guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity);
	void guided_set_angle(const Quaternion &q, float climb_rate_cms);
	void guided_run();
	void guided_takeoff_run();
	void guided_pos_control_run();
	void guided_vel_control_run();
	void guided_posvel_control_run();
	void guided_angle_control_run();
	void guided_limit_clear();
	void guided_limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm);
	void guided_limit_init_time_and_pos();
	bool guided_limit_check();
	bool land_init(bool ignore_checks);
	void land_run();
	void land_gps_run();
	void land_nogps_run();
	float get_land_descent_speed();
	void land_do_not_use_GPS();
	void set_mode_land_with_pause();
	bool landing_with_GPS();
	bool loiter_init(bool ignore_checks);
	void loiter_run();
	bool poshold_init(bool ignore_checks);
	void poshold_run();
	void poshold_update_pilot_lean_angle(float &lean_angle_filtered, float &lean_angle_raw);
	int16_t poshold_mix_controls(float mix_ratio, int16_t first_control, int16_t second_control);
	void poshold_update_brake_angle_from_velocity(int16_t &brake_angle, float velocity);
	void poshold_update_wind_comp_estimate();
	void poshold_get_wind_comp_lean_angles(int16_t &roll_angle, int16_t &pitch_angle);
	void poshold_roll_controller_to_pilot_override();
	void poshold_pitch_controller_to_pilot_override();

	bool rtl_init(bool ignore_checks);
	void rtl_run();
	void rtl_climb_start();
	void rtl_return_start();
	void rtl_climb_return_run();
	void rtl_loiterathome_start();
	void rtl_loiterathome_run();
	void rtl_descent_start();
	void rtl_descent_run();
	void rtl_land_start();
	void rtl_land_run();
	float get_RTL_alt();
	bool sport_init(bool ignore_checks);
	void sport_run();
	bool stabilize_init(bool ignore_checks);
	void stabilize_run();
	void crash_check();
	void parachute_check();
	void parachute_release();
	void parachute_manual_release();
	void ekf_check();
	bool ekf_over_threshold();
	void failsafe_ekf_event();
	void failsafe_ekf_off_event(void);
	void esc_calibration_startup_check();
	void esc_calibration_passthrough();
	void esc_calibration_auto();
	void failsafe_radio_on_event();
	void failsafe_radio_off_event();
	void failsafe_battery_event(void);
	void failsafe_gcs_check();
	void failsafe_gcs_off_event(void);
	void set_mode_RTL_or_land_with_pause();
	void update_events();
	void failsafe_enable();
	void failsafe_disable();
	void fence_check();
	bool set_mode(uint8_t mode);
	void update_flight_mode();
	void exit_mode(uint8_t old_control_mode, uint8_t new_control_mode);
	bool mode_requires_GPS(uint8_t mode);
	bool mode_has_manual_throttle(uint8_t mode);
	bool mode_allows_arming(uint8_t mode, bool arming_from_gcs);
	void notify_flight_mode(uint8_t mode);
	void heli_init();
	int16_t get_pilot_desired_collective(int16_t control_in);
	void check_dynamic_flight(void);
	void update_heli_control_dynamics(void);
	void heli_update_landing_swash();
	void heli_update_rotor_speed_targets();
	void heli_radio_passthrough();
	bool heli_acro_init(bool ignore_checks);
	void heli_acro_run();
	bool heli_stabilize_init(bool ignore_checks);
	void heli_stabilize_run();
	void read_inertia();
	void read_inertial_altitude();
	bool land_complete_maybe();
	void update_land_and_crash_detectors();
	void update_land_detector();
	void update_throttle_thr_mix();
	void landinggear_update();
	void update_notify();
	void motor_test_output();
	void motor_test_stop();
	void arm_motors_check();
	void auto_disarm_check();
	bool init_arm_motors(bool arming_from_gcs);
	bool pre_arm_checks(bool display_failure);
	void pre_arm_rc_checks();
	bool pre_arm_gps_checks(bool display_failure);
	bool pre_arm_ekf_attitude_check();
	bool arm_checks(bool display_failure, bool arming_from_gcs);
	void init_disarm_motors();
	void motors_output();
	void lost_vehicle_check();
	void run_nav_updates(void);
	void calc_position();
	void calc_distance_and_bearing();
	void calc_wp_distance();
	void calc_wp_bearing();
	void calc_home_distance_and_bearing();
	void run_autopilot();
	void perf_info_reset();
	void perf_ignore_this_loop();
	void perf_info_check_loop_time(uint32_t time_in_micros);
	uint16_t perf_info_get_num_loops();
	uint32_t perf_info_get_max_time();
	uint32_t perf_info_get_min_time();
	uint16_t perf_info_get_num_long_running();
	Vector3f pv_location_to_vector(const Location& loc);
	Vector3f pv_location_to_vector_with_default(const Location& loc, const Vector3f& default_posvec);
	float pv_alt_above_origin(float alt_above_home_cm);
	float pv_alt_above_home(float alt_above_origin_cm);
	float pv_get_bearing_cd(const Vector3f &origin, const Vector3f &destination);
	float pv_get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination);
	void default_dead_zones();
	void init_rc_in();
	void init_rc_out();
	void enable_motor_output();
	void read_radio();
	void set_throttle_and_failsafe();
	void set_throttle_zero_flag(uint32_t tnow_ms);
	void init_barometer(bool full_calibration);
	void read_barometer(void);
	void init_sonar(void);
	int16_t read_sonar(void);
	void init_compass();
	void init_optflow();
	void update_optical_flow(void);
	void init_precland();
	void update_precland();
	void read_battery(void);
	void read_receiver_rssi(void);
	void epm_update();
	void report_batt_monitor();
	void report_frame();
	void report_radio();
	void report_ins();
	void report_flight_modes();
	void report_optflow();
	void print_radio_values();
	void print_switch(uint8_t p, uint8_t m, bool b);
	void print_accel_offsets_and_scaling(void);
	void print_gyro_offsets(void);
	void report_compass();
	void print_blanks(int16_t num);
	void print_divider(void);
	void print_enabled(bool b);
	void report_version();
	void read_control_switch();
	bool check_if_auxsw_mode_used(uint8_t auxsw_mode_check);
	bool check_duplicate_auxsw(void);
	void reset_control_switch();
	uint8_t read_3pos_switch(int16_t radio_in);
	void read_aux_switches();
	void init_aux_switches();
	void init_aux_switch_function(int8_t ch_option, uint8_t ch_flag);
	void do_aux_switch_function(int8_t ch_function, uint8_t ch_flag);
	void save_trim();
	void auto_trim();
	void init_ardupilot();
	void startup_INS_ground();
	bool calibrate_gyros();
	bool position_ok();
	bool ekf_position_ok();
	bool optflow_position_ok();
	void update_auto_armed();
	void check_usb_mux(void);
	void frsky_telemetry_send(void);
	bool should_log(uint32_t mask);
	bool current_mode_has_user_takeoff(bool must_navigate);
	bool do_user_takeoff(float takeoff_alt_cm, bool must_navigate);
	void takeoff_timer_start(float alt_cm);
	void takeoff_stop();
	void takeoff_get_climb_rates(float& pilot_climb_rate, float& takeoff_climb_rate);
	void print_hit_enter();
	void tuning();
	void gcs_send_text_fmt(const prog_char_t *fmt, ...);


	void log_init(void);

	void init_capabilities(void);
	void dataflash_periodic(void);

public:
	void mavlink_delay_cb();
	void failsafe_check();
	/*int8_t dump_log(uint8_t argc, const Menu::arg *argv);
	int8_t erase_logs(uint8_t argc, const Menu::arg *argv);
	int8_t select_logs(uint8_t argc, const Menu::arg *argv);
	bool print_log_menu(void);

	int8_t process_logs(uint8_t argc, const Menu::arg *argv);
	int8_t main_menu_help(uint8_t, const Menu::arg*);
	int8_t setup_mode(uint8_t argc, const Menu::arg *argv);
	int8_t setup_factory(uint8_t argc, const Menu::arg *argv);
	int8_t setup_set(uint8_t argc, const Menu::arg *argv);
	int8_t setup_show(uint8_t argc, const Menu::arg *argv);
	int8_t esc_calib(uint8_t argc, const Menu::arg *argv);*/
};

#define MENU_FUNC(func) FUNCTOR_BIND(&copter, &Copter::func, int8_t, uint8_t, const Menu::arg *)
#endif // _COPTER_H_
