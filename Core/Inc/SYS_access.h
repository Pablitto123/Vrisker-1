/*
 * SYS_access.h
 *
 *  Created on: 26 paź 2020
 *      Author: Łukasz Zelek
 */

#ifndef INC_SYS_ACCESS_H_
#define INC_SYS_ACCESS_H_
#include <main.h>

int read_input(int button_id);//t

void set_diode(int diode_id, boolean val);
void toggle_diode(int diode_id);

void send_string(char buffer[], int size);//t
int read_string(char buffer[]);//t

#endif /* INC_SYS_ACCESS_H_ */
