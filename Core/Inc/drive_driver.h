/*
 * drive_driver.h
 *
 *  Created on: 27 Nov 2021
 *      Author: macie
 */

#ifndef INC_DRIVE_DRIVER_H_
#define INC_DRIVE_DRIVER_H_

#include "stm32f4xx.h"

#define TURN_LEFT -1
#define TURN_COUNTERCLOCKWISE -1
#define TURN_RIGHT 1
#define TURN_CLOCKWISE 1

void driveStraightByDuration(int direction, int speed, int duration);
void turnByDuration(int direction, int speed, uint32_t duration);
//void turnByAngle(int direction, int angle);
void moveOneUnit();

#endif /* INC_DRIVE_DRIVER_H_ */
