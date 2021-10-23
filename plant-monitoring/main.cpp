/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "./MMA8451Q.h"
#include "./TCS3472_I2C.h"

Ticker ticker;
bool tick_event;
void ticker_isr(void){tick_event = true;}

int main(void){
	MMA8451Q acc(PB_9,PB_8,0x1d<<1);
	TCS3472_I2C rgbSensor(PB_9, PB_8);
	rgbSensor.enablePowerAndRGBC();
	float accValues[3];
	int rgbValues[4];
	ticker.attach(ticker_isr,2000ms);
	printf("WHO AM I: 0x%2X\r\n", acc.getWhoAmI());
	while(true){
		if(tick_event){
			acc.getAllAxis(accValues);
			rgbSensor.getAllColors(rgbValues);
			printf("ACCELEROMETER: X_AXIS=%f \t Y_AXIS=%f\t Z_AXIS=%f\n",accValues[0], accValues[1], accValues[2]);
			printf("COLOR SENSOR: CLEAR=%d, RED=%d, GREEN=%d, BLUE=%d\n", rgbValues[0], rgbValues[1], rgbValues[2], rgbValues[3]);
			tick_event = false;
		}
	}		
}	