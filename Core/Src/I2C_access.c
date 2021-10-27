/*
 * I2C_access.c
 *
 *  Created on: 26 paź 2020
 *      Author: Łukasz Zelek
 */

#include <I2C_access.h>
#include <lowlvl.h>
#define IMU (receivedMPUData)

int get_proximity(int sensor_id){
	return DIST[sensor_id-1];
}

int get_angular_velocity(char axis){
	switch(axis){
		case 'x':
			return IMU[4];
			break;
		case 'y':
			return IMU[5];
			break;
		case 'z':
			return IMU[6];
			break;
	}
	return 0;
}

int get_acceleration(char axis){
	switch(axis){
		case 'x':
			return IMU[0];
			break;
		case 'y':
			return IMU[1];
			break;
		case 'z':
			return IMU[2];
			break;
	}
	return 0;
}

int get_temperature(void){
	return IMU[3];
}

int write_bytes(uint8_t data[], uint16_t address, int size){
	return 0;
}

int read_bytes(uint8_t *data, uint16_t address, int size){
	return 0;
}
