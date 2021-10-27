/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "./MMA8451Q.h"
#include "./TCS3472_I2C.h"
#include "./HW5P1_2015.h"
#include "./RGBLED.h"
#include "./Si7021.h"

Ticker ticker;
MMA8451Q acc(PB_9,PB_8,0x1d<<1);
TCS3472_I2C rgbSensor(PB_9, PB_8);
HW5P1_2015 lightSensor(A0);
RGBLED rgbLed(PH_0, PH_1, PB_13);
Si7021 humtempsensor(PB_9,PB_8);
bool tick_event;
void ticker_isr(void){tick_event = true;}
char const* colorNames[3] = {"RED", "GREEN", "BLUE"};
int main(void){
	rgbSensor.enablePowerAndRGBC();
	float accValues[3];
	uint16_t rgbValues[4];
	
	ticker.attach(ticker_isr, 2000ms);
	while(true){
		if(tick_event){
			acc.getAllAxis(accValues);
  		humtempsensor.measure();
			int dominantColor = rgbSensor.getAllColors(rgbValues); // 0 = RED, 1 = GREEN, 2 = BLUE
			char const* dominantColorName = colorNames[dominantColor];
			rgbLed.setColor(dominantColor);
			printf("ACCELEROMETER: X_AXIS=%f \t Y_AXIS=%f\t Z_AXIS=%f\n", accValues[0], accValues[1], accValues[2]);
			printf("COLOR SENSOR: CLEAR=%d, RED=%d, GREEN=%d, BLUE=%d -- DOMINANT COLOR=%s\n", rgbValues[0], rgbValues[1], rgbValues[2], rgbValues[3], dominantColorName);
			printf("LIGHT: %3.1f%%", lightSensor.readLight());
			printf("TEMPERATURE: %2.2f C \n ", humtempsensor.get_temperature());
			printf("HUMIDITY: %2.2f%%  \n ", humtempsensor.get_humidity());
			printf("\n\n");
			tick_event = false;
		}
	}		
}	