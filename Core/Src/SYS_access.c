/*
 * SYS_access.c
 *
 *  Created on: 26 paź 2020
 *      Author: Łukasz Zelek
 */

#include <SYS_access.h>
#include <lowlvl.h>

void set_diode(int diode_id, boolean val){
	if(val)
		ledEnable(diode_id);
	else
		ledDisable(diode_id);
}

void toggle_diode(int diode_id){
	ledToggle(diode_id);
}
