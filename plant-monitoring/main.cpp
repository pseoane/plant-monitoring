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

#define TEMP_LIMIT_MIN  0
#define TEMP_LIMIT_MAX	45

#define HUM_LIMIT_MIN   10
#define HUM_LIMIT_MAX		95

#define SOILM_LIMIT_MIN 10
#define SOILM_LIMIT_MAX 90

#define LIGHT_LIMIT_MIN 2
#define LIGHT_LIMIT_MAX 95

#define GREEN_LIMIT_MIN 1 //450
#define STAND_LIMIT			30

#define CLEAR_MIN_LIMIT 1 //if below, clear led = ON

#define NORMAL_MODE_CADENCE 2500ms //Seconds of normal mode cadence monitoring without scaling

#define N_MEASURES 			10   //Number of measures made in NORMAL_MODE after which mean,min,max values are displayed without scaling

#define SCALE_FACTOR    12  // With this factor we can scale to fullfill th requirements (avg,min,.max,... each 1h) 


enum Mode { TEST, NORMAL };

Mode currentMode = TEST;
MMA8451Q acc(PB_9,PB_8,0x1d<<1);
TCS3472_I2C rgbSensor(PB_9, PB_8);
HW5P1_2015 lightSensor(A0);
RGBLED rgbLed(PH_0, PB_13, PH_1);
Si7021 humtempsensor(PB_9,PB_8);
SEN_13322 soilMoistureSensor(PA_0);
UnbufferedSerial* gps_Serial = new UnbufferedSerial(PA_9, PA_10,9600); //serial object for use w/ GPS
Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
Ticker ticker;
bool tick_event;
void ticker_isr(void){tick_event = true;}
char const* colorNames[3] = {"RED", "GREEN", "BLUE"};
using namespace std::chrono;
bool buttonPressed = false;
float accValues[3];
uint16_t rgbValues[4];
InterruptIn userButton(PB_2);
Thread gps_thread(osPriorityNormal,2048);;
bool gpsInfoAvailable = false;
uint8_t hour, minute, seconds_gps, year, month, day;
uint16_t milliseconds_gps;
float latitude, longitude, geoidheight, altitude;
float speed, angle;
char lat, lon, mag;
bool fix;
uint8_t fixquality, satellites;
uint8_t nmeasuresdone= 0;
Mutex mutex;


void buttonPressedIsr() {
	buttonPressed = true;
}

void readGps(void) {
	char c; //when read via Adafruit_GPS::read(), the class returns single character stored here
	//Timer refresh_Timer; //sets up a timer for use in loop; how often do we print GPS info?
	//const int refresh_Time = 2000; //refresh time in ms
	while(true) {
			//refresh_Timer.start(); 
			c = myGPS.read();   //queries the GPS
			//if (c) { printf("%c", c); } //this line will echo the GPS data if not paused
			//check if we recieved a new message from GPS, if so, attempt to parse it,
			if ( myGPS.newNMEAreceived() ) {
				if ( !myGPS.parse(myGPS.lastNMEA()) ) {
					continue;
				}
			}

			//check if enough time has passed to warrant printing GPS info to screen
			//note if refresh_Time is too low or pc.baud is too low, GPS data may be lost during printing
			//if (duration_cast<milliseconds>(refresh_Timer.elapsed_time()).count() >= refresh_Time) {
			//if (refresh_Timer.read_ms() >= refresh_Time) {
					//refresh_Timer.reset();
			mutex.lock();
			hour = myGPS.hour;
			minute = myGPS.minute;
			seconds_gps = myGPS.seconds;
			milliseconds_gps = myGPS.milliseconds;
			day = myGPS.day;
			month = myGPS.month;
			year = myGPS.year;
			fixquality = myGPS.fixquality;
			lat = myGPS.lat;
			lon = myGPS.lon;
			latitude = myGPS.latitude;
			longitude = myGPS.longitude;
			speed = myGPS.speed;
			angle = myGPS.angle;
			altitude = myGPS.altitude;
			satellites = myGPS.satellites;
			mutex.unlock();
			gpsInfoAvailable = true;
					
					
					//}
			//}
			}
}

void normalMode() {
	printf("Norrrmal\n");
	ticker.attach(ticker_isr, NORMAL_MODE_CADENCE );
	while (!buttonPressed){
		if(tick_event){
			nmeasuresdone ++;
			if(nmeasuresdone == N_MEASURES){
				//print media,min,max,...
				nmeasuresdone = 0;
			}
			
			
			
			
			
			if (gpsInfoAvailable){
				mutex.lock();
				printf("Time: %d:%d:%d.%u\r\n", hour, minute, seconds_gps, milliseconds_gps);
				printf("Date: %d/%d/20%d\r\n",day, month, year);
				printf("Quality: %d\r\n", (int) fixquality);
				//if ((int)myGPS.fixquality > 0) {
				printf("Location: %5.2f %c, %5.2f %c\r\n", latitude, lat, longitude, lon);
				printf("Speed: %5.2f knots\r\n", speed);
				printf("Angle: %5.2f\r\n", angle);
				printf("Altitude: %5.2f\r\n", altitude);
				printf("Satellites: %d\r\n", satellites);
				mutex.unlock();
				gpsInfoAvailable = false;
			}
			tick_event = false;
	
		}
	}
}
void testMode(){
	printf("Testing\n");
	ticker.attach(ticker_isr, 2000ms);
	while(!buttonPressed) {
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
			if (gpsInfoAvailable){
				printf("Time: %d:%d:%d.%u\r\n", hour, minute, seconds_gps, milliseconds_gps);
				printf("Date: %d/%d/20%d\r\n", day, month, year);
				printf("Quality: %d\r\n", (int) fixquality);
				//if ((int)myGPS.fixquality > 0) {
				printf("Location: %5.2f %c, %5.2f %c\r\n", latitude, lat, longitude, lon);
				printf("Speed: %5.2f knots\r\n",speed);
				printf("Angle: %5.2f\r\n", angle);
				printf("Altitude: %5.2f\r\n", altitude);
				printf("Satellites: %d\r\n", satellites);
				gpsInfoAvailable = false;
			
			}
			tick_event = false;
		}
	}
	

}


int main(void){
	rgbSensor.enablePowerAndRGBC();

	//GPS
	gps_thread.start(readGps);
	userButton.fall(buttonPressedIsr);
	currentMode = TEST;
	testMode();
	while(true){
		if (buttonPressed) {
			switch (currentMode) {
				case(TEST):
					currentMode = NORMAL;
					printf("Current mode set to NORMAL\n");
					buttonPressed = false;
					normalMode();
					break;
				case(NORMAL):
					currentMode = TEST;
					printf("Current mode set to TEST\n");
					buttonPressed = false;
					testMode();
					break;
			}
			
		}
		
	}
}	


