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
#include "./MBed_Adafruit_GPS.h"
#include "./SEN_13322.h"

enum Mode { TEST, NORMAL };

Mode currentMode;
MMA8451Q acc(PB_9,PB_8,0x1d<<1);
TCS3472_I2C rgbSensor(PB_9, PB_8);
HW5P1_2015 lightSensor(A0);
RGBLED rgbLed(PH_0, PH_1, PB_13);
Si7021 humtempsensor(PB_9,PB_8);
SEN_13322 soilMoistureSensor(PA_0);

Ticker ticker;
bool tick_event;
void ticker_isr(void){tick_event = true;}
char const* colorNames[3] = {"RED", "GREEN", "BLUE"};
using namespace std::chrono;
UnbufferedSerial * gps_Serial;
bool buttonPressed = false;
float accValues[3];
uint16_t rgbValues[4];
Thread workerThread;

InterruptIn userButton(PB_2);

void buttonPressedIsr() {
	buttonPressed = true;
}

void normalMode() {
}

void testMode() {
	ticker.detach();
	ticker.attach(ticker_isr, 2000ms);
	
	while(true) {
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
			printf("SOIL MOISTURE: %2.2f%% \n ", soilMoistureSensor.getMoistureValue());
			printf("\n\n");
			tick_event = false;
		}
	}
}

int main(void){
	rgbSensor.enablePowerAndRGBC();

	//GPS
	gps_Serial = new UnbufferedSerial(PA_9, PA_10,9600); //serial object for use w/ GPS
	Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
	char c; //when read via Adafruit_GPS::read(), the class returns single character stored here
	Timer refresh_Timer; //sets up a timer for use in loop; how often do we print GPS info?
	const int refresh_Time = 2000; //refresh time in ms

  refresh_Timer.start();  //starts the clock on the timer
	
	userButton.fall(buttonPressedIsr);
	currentMode = TEST;
	workerThread.start(testMode);
	
	while(true){

		if (buttonPressed) {
			workerThread.terminate();
			switch (currentMode) {
				case(TEST):
					printf("Current mode set to NORMAL\n");
					currentMode = NORMAL;
				  workerThread.start(testMode);
					break;
				case(NORMAL):
					printf("Current mode set to TEST\n");
					currentMode = TEST;
					workerThread.start(normalMode);
					break;
			}
		}
	}		
}	

//void readAndPrintGps() {
//	while(true) {
//				c = myGPS.read();   //queries the GPS
////        if (c) { printf("%c", c); } //this line will echo the GPS data if not paused

//        //check if we recieved a new message from GPS, if so, attempt to parse it,
//		if ( myGPS.newNMEAreceived() ) {
//				if ( !myGPS.parse(myGPS.lastNMEA()) ) {
//						continue;
//				}
//		}

//		//check if enough time has passed to warrant printing GPS info to screen
//		//note if refresh_Time is too low or pc.baud is too low, GPS data may be lost during printing
//		if (duration_cast<milliseconds>(refresh_Timer.elapsed_time()).count() >= refresh_Time) {
//		//if (refresh_Timer.read_ms() >= refresh_Time) {
//				refresh_Timer.reset();
//				printf("Time: %d:%d:%d.%u\r\n", myGPS.hour, myGPS.minute, myGPS.seconds, myGPS.milliseconds);
//				printf("Date: %d/%d/20%d\r\n", myGPS.day, myGPS.month, myGPS.year);
//				printf("Quality: %d\r\n", (int) myGPS.fixquality);
//				if ((int)myGPS.fixquality > 0) {
//					printf("Location: %5.2f %c, %5.2f %c\r\n", myGPS.latitude, myGPS.lat, myGPS.longitude, myGPS.lon);
//					printf("Speed: %5.2f knots\r\n", myGPS.speed);
//					printf("Angle: %5.2f\r\n", myGPS.angle);
//					printf("Altitude: %5.2f\r\n", myGPS.altitude);
//					printf("Satellites: %d\r\n", myGPS.satellites);
//				}
//		}
//	}
//}