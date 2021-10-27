/*
 * pid.c
 *
 *  Created on: 11 lis 2020
 *      Author: Łukasz Zelek
 */

#include <pid.h>
#include <math.h>
#include <lowlvl.h>
#include <PWM_access.h>

#define VELOCITY_MAX_IN_MM_PER_SEC (1100)
#define OMEGA_MAX_IN_RAD_PER_SEC (10*M_PI)
#define ANGULAR_ERROR_COMPENSATION_TIME (0.2)
#define SPEED_MEASURMENTS_SIZE 32
#define EXP_CONST 0.2f
// pomiar prędkości
/* pomiar prędkości liniowej robota z enkoderów */
int32_t left_cnt = 0;
int32_t right_cnt = 0;
float __velocity = 0;
float __omega = 0;
float left_speed = 0;
float right_speed = 0;
float time_scale_factor = ENCODER_TICKS_PER_FULL_WHEEL_ROTATION / (29 * M_PI);
float t_period = (0.01f);
float left_speed_measurnemts[SPEED_MEASURMENTS_SIZE];
float right_speed_measurnemts[SPEED_MEASURMENTS_SIZE];
float left_speed_av = 0;
float right_speed_av = 0;
float last_left_speed_av_exp = 0;
float last_right_speed_av_exp = 0;
float left_speed_av_exp = 0;
float right_speed_av_exp = 0;

int debug_turn_sat_l;
int debug_turn_sat_r;

void performEncodersMeasurements() {
	uint32_t m = clock_timer_end_point();
	left_cnt = getLeftTickCount();
	right_cnt = getRightTickCount();
	resetLeftTickCount();
	resetRightTickCount();
	clock_timer_start_point();
//	// for SDKF usages
//	float std_dev = 10.045f;
//	// state-space model
//	float A = 1.f;
//	float C = 1.f;
//
//	// covariance matrices
//	float V = (50 * std_dev * t_period);
//	float W = (std_dev * std_dev);
//
//	// 1st kalman filter for left_encoder measurement
//	static float xpri = 0;
//	static float Ppri = 10000;
//	static float xpost = 0;
//	static float Ppost = 10000;
//
//	xpri = A * xpost;
//	Ppri = A * Ppost * A + V;
//
//	float eps = (float) left_cnt - C * xpri;
//	float S = C * Ppri * C + W;
//	float K = Ppri * C / S;
//	xpost = xpri + K * eps;
//	Ppost = Ppri - K * S * K;
//
//	//printf("%d %.1f ", left_cnt, xpost);
//	float left_cntr = xpost;
//	// END
//	static float xpri2 = 0;
//	static float Ppri2 = 10000;
//	static float xpost2 = 0;
//	static float Ppost2 = 10000;
//
//	xpri2 = A * xpost2;
//	Ppri2 = A * Ppost2 * A + V;
//
//	float eps2 = (float) right_cnt - C * xpri2;
//	float S2 = C * Ppri2 * C + W;
//	float K2 = Ppri2 * C / S2;
//	xpost2 = xpri2 + K2 * eps2;
//	Ppost2 = Ppri2 - K2 * S2 * K2;
//
//	//printf("%ld %.1f\n", right_cnt, xpost2);
//	float right_cntr = xpost2;
//	//END
	float time_factor = (1000000 / (m * time_scale_factor));
	left_speed = time_factor * (left_cnt);
	right_speed = time_factor * (right_cnt);
	__velocity = (left_speed + right_speed) / 2;
	__omega = (right_speed - left_speed) / DISTANCE_BETWEEN_WHEELS;
}

extern float __x, __y; // real position in the world
extern float __alpha;

float reference_velocity = 0;
float reference_omega = 0;
float left_velocity = 0;
float right_velocity = 0;
float reference_x = 0;
float reference_y = 0;
float reference_alpha = 0;

float vP = 1, vI = 0, vD = 0;
float wP = 1, wI = 0, wD = 0;
int pid_print_flag = 0;

/* FIXME powyższy model ustalania offset-u jest bez sensu */
/* rozważyć wyliczanie odległości od ścieżki */

float pid_omega(float error) {
	static float error_sum, last_error;
	error_sum += ((error+last_error)/2) * t_period;
	float derivative = (error - last_error) / t_period;
//	float Kp = 306.f;
//	float Ki = 2417.f;
//	float Kd = 0.f;
	float Kp = wP;//300.f;
	float Ki = wI;//2000.f;
	float Kd = wD;
	float Turn = Kp * error + error_sum * Ki + Kd * derivative;
	last_error = error;
	return Turn;
}

float pid_velocity(float error) {
	static float error_sum, last_error;
	error_sum += ((error+last_error)/2) * t_period;
	float derivative = (error - last_error) / t_period;
//	float Kp = 6.57f;
//	float Ki = 50.738923f;
//	float Kd = 0.03422;
	float Kp = vP;//6.3f;
	float Ki = vI;
	float Kd = vD;
	float Turn = Kp * error + error_sum * Ki + Kd * derivative;
	last_error = error;
	return Turn;
}

//void pid_global() {
//	static int print_counter = 0;
//	//print_counter = (print_counter + 1) % 100;
//	//if(print_counter == 1)
//	performEncodersMeasurements(); // instead of this, we will use kalman estimated values
//	// we can use ukf only to proper estimate a position with freq 100Hz
//	// but here, we need simple v and omega, so we use single dimensional kf for those variables with pid global freq 1kHz
//
//	/* profiler start*/
//	// we can set that in main loop and use algorithm to making a decision
//	reference_x = 0;
//	reference_y = 0; // FIXME: here can be 2 or more values vector
//	reference_alpha = 0;
//	/* profiler end*/
//	// float delta_x = reference_x - __x;
//	// float delta_y = reference_y - __y;
//	// float delta_alpha = reference_alpha - __alpha;
//	// if(fabsf(delta_alpha) > 0.1)
//	// {
//	// 	reference_velocity = 0; // stop a robot
//	// 	reference_omega = OMEGA_MAX_IN_RAD_PER_SEC/10; // allow robot to rotate
//	// }else{
//	// 	reference_velocity = VELOCITY_MAX_IN_MM_PER_SEC/10;
//	// 	float delta_correction_from_bad_position;
//	// 	if(delta_x > 0.5 || delta_y > 0.5)
//	// 	{
//	// 		delta_correction_from_bad_position = atan2f(delta_y,delta_x);
//	// 	}
//	// 	else delta_correction_from_bad_position = 0;
//	// 	reference_omega = (delta_alpha+delta_correction_from_bad_position)/ANGULAR_ERROR_COMPENSATION_TIME;
//	// }
//	float angularTurn = pid_omega(reference_omega - __omega);
//	float linearTurn = pid_velocity(reference_velocity - __velocity);
//
//	float left_motor_turn = linearTurn - angularTurn;
//	float right_motor_turn = linearTurn + angularTurn;
//
//	//saturacja
//	if (right_motor_turn > 1000)
//		right_motor_turn = 1000;
//	if (right_motor_turn < -1000)
//		right_motor_turn = -1000;
//	if (left_motor_turn > 1000)
//		left_motor_turn = 1000;
//	if (left_motor_turn < -1000)
//		left_motor_turn = -1000;
//
//	//set_velocity(1, (int) (left_motor_turn));
//	//set_velocity(2, (int) (right_motor_turn));
//	//printf("%.1f %.1f\n", left_speed, right_speed);
//	//printf("%.1f %.1f %.1f\n", __velocity, reference_velocity, linearTurn);
//
//	//if(print_counter == 0)if(pid_print_flag == 3)printf("%.1f %.1f %.1f %.1f %.1f %.1f\n", reference_velocity, __velocity, linearTurn, reference_omega, __omega, angularTurn);
//	//if(print_counter == 0)if(pid_print_flag == 1)printf("%.1f ", __omega);
//	if(print_counter == 0)if(pid_print_flag == 1)printf("%.1f %.1f\n", left_speed, right_speed);
//	if(print_counter == 0)if(pid_print_flag == 2)printf("%.1f ", __velocity);
//	if(print_counter == 0)if(pid_print_flag)printf("\n");
//	//printf("%.1f %.1f %.1f\n", __omega, reference_omega, angularTurn);
//}

float get_moving_average(float* values, int size){
	float average = .0f;
	for(int i = 0; i < size; i++){
		average += values[i];
	}
	average /= size;
	return average;
}

void move_values(float* values, int size){

	for(int i = 1; i < (size-1); i++){
		values[i-1] = values[i];
	}
}

void pid_left(float error){
	const float Iconst = 0.17f;
	const float Pconst = 7.1f;
	static float integral = 0;
	integral+= error;
	int turn = (int)(Pconst*error + Iconst*integral);
	int turn_sat;
	if(turn > 1000)turn_sat = 1000;
	else if(turn < -1000)turn_sat = -1000;
	else turn_sat = turn;
	set_velocity(1, turn_sat);

	debug_turn_sat_l = turn_sat;
}

void pid_right(float error){
	static float integral = 0;
	const float Iconst = 0.07f;
	const float Pconst = 3.1f;
	integral+= error;
	int turn = (int)(Pconst*error + Iconst*integral);
	int turn_sat;
	if(turn > 1000)turn_sat = 1000;
	else if(turn < -1000)turn_sat = -1000;
	else turn_sat = turn;
	set_velocity(2,   turn_sat);
	debug_turn_sat_r = turn_sat;
}

void pid_global() {
	performEncodersMeasurements();
	left_speed_measurnemts[SPEED_MEASURMENTS_SIZE-1] = left_speed;
	right_speed_measurnemts[SPEED_MEASURMENTS_SIZE-1] = right_speed;
	right_speed_av = get_moving_average(right_speed_measurnemts, SPEED_MEASURMENTS_SIZE);
	left_speed_av = get_moving_average(left_speed_measurnemts, SPEED_MEASURMENTS_SIZE);
	left_speed_av_exp = left_speed_av * EXP_CONST + last_left_speed_av_exp*(1- EXP_CONST);
	right_speed_av_exp = right_speed_av * EXP_CONST + last_right_speed_av_exp*(1- EXP_CONST);
	move_values(left_speed_measurnemts, SPEED_MEASURMENTS_SIZE);
	move_values(right_speed_measurnemts, SPEED_MEASURMENTS_SIZE);

	pid_left(left_velocity-left_speed_av);
	pid_right(right_velocity-right_speed_av);

	last_left_speed_av_exp = left_speed_av_exp;
	last_right_speed_av_exp = right_speed_av_exp;
	static int print_counter = 0;
	print_counter = (print_counter + 1) % 150;
	if(print_counter == 0)if(pid_print_flag == 1)printf("%.1f %.1f\n", left_speed, right_speed);
	if(print_counter == 0)if(pid_print_flag == 2)printf("%.1f ", __velocity);
	if(print_counter == 0)if(pid_print_flag)printf("\n");
}
