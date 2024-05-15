/*
 * pid.c
 *
 *  Created on: Apr 26, 2024
 *      Author: naker
 */
#include "pid.h"

void PID_init(PID* pid, float32_t _kp,  float32_t _ki, float32_t _kd, float32_t _sampt){
	pid -> kp = _kp;
	pid -> ki = _ki;
	pid -> kd = _kd;
	pid -> sampt = _sampt;
	pid -> y_n = 0.0;
}
int32_t Update_pid(PID *pid, float32_t error, float32_t pid_sat, float32_t plant_sat) {
//	static float32_t y_n = 0; // Output[n]
	static float32_t y_n_1 = 0; // Output[n-1]
	float32_t e_n = error; // error[n]
	static float32_t e_n_1 = 0; // error[n-1]

	float32_t p_term = e_n * pid -> kp;
	float32_t d_term = ((e_n - e_n_1) * pid -> kd) / pid -> sampt;
	float32_t i_term = ((pid -> ki * pid -> sampt / 2.0)*(e_n + e_n_1)) + y_n_1;

	if(pid -> ki == 0){
		i_term = 0;
	}
	if(pid -> kd == 0){
		d_term = 0;
	}
	pid -> y_n = p_term + d_term + i_term; // pid output
	uint8_t is_sat = 0;
	// check is pid output is saturating
	if(pid -> y_n > pid_sat){
		is_sat = 1;
	}
	else if(pid -> y_n < -(pid_sat)){
		is_sat = 1;
	}
	// check is error sign and output sign is equal
	if(e_n * pid -> y_n == fabs(e_n * pid -> y_n)){
		// if pid output is saturating and error sign and output sign is  i_term = 0;
		if(is_sat == 1){
			pid -> y_n = p_term + d_term;
		}
	}
	// Plant saturation
	if(pid -> y_n > plant_sat){
		pid -> y_n = plant_sat;
	}
	else if(pid -> y_n < -(plant_sat)){
		pid -> y_n = (-(plant_sat));
	}
	// Update value
	y_n_1 = pid -> y_n;
	e_n_1 = e_n;
	return pid -> y_n;
}
void Reset_pid(PID* pid){
	pid -> kp = 0;
	pid -> ki = 0;
	pid -> kd = 0;
	pid -> y_n = 0;
}
