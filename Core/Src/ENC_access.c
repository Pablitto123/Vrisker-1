/*
 * ENC_access.c
 *
 *  Created on: 26 paź 2020
 *      Author: Łukasz Zelek
 */

#include <ENC_access.h>
#include <lowlvl.h>

void set_position(int encoder_id, int encoder_value){
	setEncoderCount(encoder_id, encoder_value);
}

int read_position(int encoder_id){
	switch(encoder_id){
		case 1:
			return getLeftTickCount();
			break;
		case 2:
			return getRightTickCount();
			break;
	}
	return 0;
}
