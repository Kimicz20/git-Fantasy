#include"Copter.h"

void Copter::set_has_new_input(bool b){
	has_new_input = b;
}

void Copter::set_last_update_radio_ms(long t){
	last_update_radio_ms = t;
}

uint32_t Copter::get_last_update_radio_elapsed(long t){
	return t - last_update_radio_ms;
}

int8_t Copter::get_desired_mode(){
	return flightmode[control_switch_state.switch_position];
}

void Copter::set_control_switch_last_edge_time()
{
	control_switch_state.last_edge_time_ms = control_switch_state.now_time;
}


void Copter::reset_arming_counter(){
	arming_counter = 0;
}

void Copter::increase_arming_counter(){
	arming_counter++;
}