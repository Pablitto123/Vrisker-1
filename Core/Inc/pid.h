/*
 * pid.h
 *
 *  Created on: 11 lis 2020
 *      Author: Łukasz Zelek
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <main.h>
#include <lowlvl.h>

void performEncodersMeasurements();
void pid_global();

#endif /* INC_PID_H_ */
