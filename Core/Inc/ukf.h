/*
 * ukf.h
 *
 *  Created on: Nov 1, 2020
 *      Author: Łukasz Zelek
 */

#ifndef INC_UKF_H_
#define INC_UKF_H_

#include "main.h"

//biegun – 9,83332
//normalne – 9,80665
//równik – 9,78030
//Gdańsk – 9,8145
//Warszawa – 9,8123
//Kraków – 9,8105
//Katowice – 9,8101
//Wrocław – 9,8115

/*
switch(Range)
	{
		case MPU6050_ACCEL_FS_2:
			Acc_Scale = 0.000061;
			break;
		case MPU6050_ACCEL_FS_4:
			Acc_Scale = 0.000122;
			break;
		case MPU6050_ACCEL_FS_8:
			Acc_Scale = 0.000244;
			break;
		case MPU6050_ACCEL_FS_16:
			Acc_Scale = 0.0004882;
			break;
		default:
			break;
	}
	switch(Range)
	{
		case MPU6050_GYRO_FS_250:
			Gyr_Scale = 0.007633;
			break;
		case MPU6050_GYRO_FS_500:
			Gyr_Scale = 0.015267;
			break;
		case MPU6050_GYRO_FS_1000:
			Gyr_Scale = 0.030487;
			break;
		case MPU6050_GYRO_FS_2000:
			Gyr_Scale = 0.060975;
			break;
		default:
			break;
	}
*/

#define GRAVITY_ACCELERATION (9810.5f)
#define ACCELEROMETER_SCALE (0.000061f) // according to MPU6050_ACCEL_FS_2
#define ACC_SCALAR (0.5984405f) // according to (ACCELEROMETER_SCALE*GRAVITY_ACCELERATION) result mm/s^2
#define GYRO_SCALAR (0.060975f) //according to MPU6050_GYRO_FS_2000 result deg/s

void kalman_init();
void kalman_filter();

#endif /* INC_UKF_H_ */
