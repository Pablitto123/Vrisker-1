/*
 * drive_driver.c
 *
 *  Created on: 27 Nov 2021
 *      Author: macie
 */

#include "drive_driver.h"
#include "I2C_access.h"

extern float left_velocity;
extern float right_velocity;
extern uint32_t stopTime;
extern uint8_t PIDstopFlag;


void turnByDuration(int direction, int speed, uint32_t duration){
	left_velocity = (float)(speed*direction);
	right_velocity = (float)(-speed*direction);
	PIDstopFlag = 1;
	stopTime = HAL_GetTick() + duration;
}

//void turnByAngle(int direction, int angle){
//
//}
