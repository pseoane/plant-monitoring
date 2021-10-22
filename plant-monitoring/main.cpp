/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "./MMA8451Q.h"
Ticker ticker;
bool tick_event;
void ticker_isr(void){tick_event =true;}

int main(void){
	MMA8451Q acc(PB_9,PB_8,0x1d<<1);
	float x,y,z;
	ticker.attach(ticker_isr,2000ms);
	printf("WHO AM I: 0x%2X\r\n", acc.getWhoAmI());
	while(true){
		if(tick_event){
			x=acc.getAccX();
			y=acc.getAccY();
			z=acc.getAccZ();
			printf("x =%f \t y=%f\t z=%f\n",x,y,z);
			tick_event = false;
		}
	}		
}	