/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */



Ticker ticker;
bool tick_event;
void ticker_isr(void){tick_event = true;}

int main(void){
	MMA8451Q acc(PB_9,PB_8,0x1d<<1);
	TCS3472_I2C rgbSensor(PB_9, PB_8);
	HW5P1_2015 lightSensor(A0);
	rgbSensor.enablePowerAndRGBC();
	float accValues[3];
	uint16_t rgbValues[4];
	char const* colors[3] = {"RED", "GREEN", "BLUE"};
	ticker.attach(ticker_isr, 2000ms);
	while(true){
		if(tick_event){
			acc.getAllAxis(accValues);
			char const* dominantColor = colors[rgbSensor.getAllColors(rgbValues)];
			printf("ACCELEROMETER: X_AXIS=%f \t Y_AXIS=%f\t Z_AXIS=%f\n", accValues[0], accValues[1], accValues[2]);
			printf("COLOR SENSOR: CLEAR=%d, RED=%d, GREEN=%d, BLUE=%d -- DOMINANT COLOR=%s\n", rgbValues[0], rgbValues[1], rgbValues[2], rgbValues[3], dominantColor);
			printf("LIGHT: %3.1f%%", lightSensor.readLight());
			printf("\n\n");
			tick_event = false;
		}
	}		
}	