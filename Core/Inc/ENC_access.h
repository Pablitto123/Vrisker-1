/*
 * ENC_access.h
 *
 *  Created on: 26 paź 2020
 *      Author: Łukasz Zelek
 */

#ifndef INC_ENC_ACCESS_H_
#define INC_ENC_ACCESS_H_

void set_position(int encoder_id, int encoder_value);
int read_position(int encoder_id);

#endif /* INC_ENC_ACCESS_H_ */
