/*
 * I2C_access.h
 *
 *  Created on: 26 paź 2020
 *      Author: Łukasz Zelek
 */

#ifndef INC_I2C_ACCESS_H_
#define INC_I2C_ACCESS_H_
#include <main.h>

int get_proximity(int sensor_id);

int get_angular_velocity(char axis);
int get_acceleration(char axis);
int get_temperature(void);

int write_bytes(uint8_t data[], uint16_t address, int size);//t
int read_bytes(uint8_t *data, uint16_t address, int size);//t

#endif /* INC_I2C_ACCESS_H_ */
