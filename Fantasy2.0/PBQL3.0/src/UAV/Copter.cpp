/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  constructor for main Copter class
 */


Copter::Copter(void) :
    ins_sample_rate(AP_InertialSensor::RATE_400HZ),
    //flight_modes(&g.flight_mode1),  这个需要自己重写
    sonar_enabled(0),       //手动设置为false
    control_mode(STABILIZE),
    motors(MAIN_LOOP_RATE),
    scaleLongDown(1),
    wp_bearing(0),
    home_bearing(0),
    home_distance(0),
    wp_distance(0),
    auto_mode(Auto_TakeOff),
    guided_mode(Guided_TakeOff),
    rtl_state(RTL_InitialClimb),
    rtl_state_complete(false),
    rtl_alt(0.0f),
    circle_pilot_yaw_override(false),
    simple_cos_yaw(1.0f),
    simple_sin_yaw(0.0f),
    super_simple_last_bearing(0),
    super_simple_cos_yaw(1.0),
    super_simple_sin_yaw(0.0f),
    initial_armed_bearing(0),
    throttle_average(0.0f),
    desired_climb_rate(0),
    loiter_time_max(0),
    loiter_time(0),
    climb_rate(0),
    sonar_alt(0),
    sonar_alt_health(0),
    target_sonar_alt(0.0f),
    baro_alt(0),
    baro_climbrate(0.0f),
    land_accel_ef_filter(LAND_DETECTOR_ACCEL_LPF_CUTOFF),
    auto_yaw_mode(AUTO_YAW_LOOK_AT_NEXT_WP),
    yaw_look_at_WP_bearing(0.0f),
    yaw_look_at_heading(0),
    yaw_look_at_heading_slew(0),
    yaw_look_ahead_bearing(0.0f),
    condition_value(0),
    condition_start(0),
    G_Dt(0.0025f),
    inertial_nav(ahrs),
    attitude_control(ahrs, aparm, motors, g.p_stabilize_roll, g.p_stabilize_pitch, g.p_stabilize_yaw,
                     g.pid_rate_roll, g.pid_rate_pitch, g.pid_rate_yaw),
    pos_control(ahrs, inertial_nav, motors, attitude_control,
                g.p_alt_hold, g.p_vel_z, g.pid_accel_z,
                g.p_pos_xy, g.pi_vel_xy),
    pmTest1(0),
    fast_loopTimer(0),
    mainLoop_count(0),
    rtl_loiter_start_time(0),
    auto_trim_counter(0),
    in_mavlink_delay(0),
    gcs_out_of_time(0),
	last_update_radio_elapsed(0),
	last_update_radio_ms(0),
	last_nonzero_throttle_ms(0),
	has_new_input(0)
    //param_loader(var_info)
{
    memset(&current_loc, 0, sizeof(current_loc));

    // init sensor error logging flags
    sensor_health.baro = 1;
    sensor_health.compass = 1;


	failsafe.rc_override_active = 0;
	failsafe.radio = 0;

	ap.using_interlock = 0;
	ap.motor_emergency_stop = 0;

	arming_counter = 0;
}
