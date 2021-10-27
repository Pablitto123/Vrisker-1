/*
 * ADC_access.c
 *
 *  Created on: 26 paź 2020
 *      Author: Łukasz Zelek
 */
#include <ADC_access.h>
extern float battery_voltage;

float get_voltage(void){
	return battery_voltage;
}
