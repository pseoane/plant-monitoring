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
	float accValues[3];
	ticker.attach(ticker_isr,2000ms);
	printf("WHO AM I: 0x%2X\r\n", acc.getWhoAmI());
	while(true){
		if(tick_event){
			acc.getAllAxis(accValues);
			printf("ACCELEROMETER: X_AXIS=%f \t Y_AXIS=%f\t Z_AXIS=%f\n",accValues[0], accValues[1], accValues[2]);
			tick_event = false;
		}
	}		
}	