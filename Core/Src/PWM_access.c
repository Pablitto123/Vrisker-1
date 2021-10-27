/*
 * PWM_access.c
 *
 *  Created on: 26 paź 2020
 *      Author: Łukasz Zelek
 */

#include <PWM_access.h>
#include <lowlvl.h>

void set_velocity(int motor_id, int velocity){
	switch(motor_id){
		case 1:
			motorLeftSetPWM(velocity);
			break;
		case 2:
			motorRightSetPWM(velocity);
			break;
	}
}
