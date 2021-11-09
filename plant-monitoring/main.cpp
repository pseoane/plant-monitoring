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

// Magnitude : Degrees
#define TEMP_LIMIT_MIN  0
#define TEMP_LIMIT_MAX	45
#define TEMP_ALERT_COLOR "RED"
// Maginutude : %
#define HUM_LIMIT_MIN   10
#define HUM_LIMIT_MAX		95
#define HUM_ALERT_COLOR "GREEN"
// Magnitude : %
#define SOILM_LIMIT_MIN 10
#define SOILM_LIMIT_MAX 90
#define SOILM_ALERT_COLOR "BLUE"
// Magnitude : %
#define LIGHT_LIMIT_MIN 2
#define LIGHT_LIMIT_MAX 95
#define LIGHT_ALERT_COLOR "REDGREEN"
// Magnitude : 
#define GREEN_LIMIT_MIN 450
#define NOT_GREEN_ALERT_COLOR "REDBLUE"
// Magnitude : g
#define STAND_LIMIT			-0.5
#define NOT_STAND_ALERT_COLOR "REDGREENBLUE"

// Magnitude :
#define CLEAR_MIN_LIMIT 1 //if below, clear led = ON

#define NORMAL_MODE_CADENCE 3000ms
#define TEST_MODE_CADENCE 2000ms
#define COMPUTE_METRICS_CADENCE 15000ms //1 hour in miliseconds

#define N_MEASURES 			10   //Number of measures made in NORMAL_MODE after which mean,min,max values are displayed without scaling

#define SCALE_FACTOR    12  // With this factor we can scale to fullfill th requirements (avg,min,.max,... each 1h) 

using namespace std::chrono;

enum Mode { TEST, NORMAL };

Mode currentMode;

MMA8451Q acc(PB_9,PB_8,0x1d<<1);
TCS3472_I2C rgbSensor(PB_9, PB_8);
HW5P1_2015 lightSensor(A0);
RGBLED rgbLed(PH_0, PB_13, PH_1);
Si7021 humtempsensor(PB_9,PB_8);
SEN_13322 soilMoistureSensor(PA_0);
UnbufferedSerial* gps_Serial = new UnbufferedSerial(PA_9, PA_10,9600); //serial object for use w/ GPS
Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
InterruptIn userButton(PB_2);
Thread gps_thread(osPriorityNormal,2048);
DigitalOut clear_led(PB_7);

uint8_t hour, minute, seconds_gps, year, month, day;
uint16_t milliseconds_gps;
uint8_t fixquality, satellites;
float latitude, longitude, geoidheight, altitude;
float speed, angle;
char lat, lon, mag;
char const* colorNames[3] = {"RED", "GREEN", "BLUE"};
bool tick_event;
bool gpsInfoAvailable = false;
bool buttonPressed = false;
bool shouldComputeMetrics = false;

Mutex mutex;
Ticker ticker;

void ticker_isr(void){
	tick_event = true;
}

void buttonPressedIsr() {
	buttonPressed = true;
}

void readGps() {
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
		}
}

void printValues(float acc[], uint16_t rgb[], char const* dominantColorName, float light, float humidity, float temp, float soilMoisture) {
	printf("ACCELEROMETER: X_AXIS=%f \t Y_AXIS=%f\t Z_AXIS=%f\n", acc[0], acc[1], acc[2]);
	printf("COLOR SENSOR: CLEAR=%d, RED=%d, GREEN=%d, BLUE=%d -- DOMINANT COLOR=%s\n", rgb[0], rgb[1], rgb[2], rgb[3], dominantColorName);
	printf("LIGHT: %3.1f%%", light);
	printf("TEMPERATURE: %2.2f C \n", temp);
	printf("HUMIDITY: %2.2f%%  \n", humidity);
	printf("SOIL MOISTURE: %2.2f%% \n", soilMoisture);
	printf("\n\n");
}

void printGpsInfo() {
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

void computeMetricsTickerIsr() {
	shouldComputeMetrics = true;
}

void printMetrics() {
	printf("METRICS FOR LIGHT IN THE LAST HOUR\n");
	printf("AVG %3.1f\n", lightSensor.metricsManager.computeAverage());
	printf("MAX %3.1f\n", lightSensor.metricsManager.computeMax());
	printf("MIN %3.1f\n", lightSensor.metricsManager.computeMin());
	
	printf("METRICS FOR TEMPERATURE IN THE LAST HOUR\n");
	printf("AVG %3.1f\n", humtempsensor.tempMetricsManager.computeAverage());
	printf("MAX %3.1f\n", humtempsensor.tempMetricsManager.computeMax());
	printf("MIN %3.1f\n", humtempsensor.tempMetricsManager.computeMin());
	
	printf("METRICS FOR HUMIDITY IN THE LAST HOUR\n");
	printf("AVG %3.1f\n", humtempsensor.humMetricsManager.computeAverage());
	printf("MAX %3.1f\n", humtempsensor.humMetricsManager.computeMax());
	printf("MIN %3.1f\n", humtempsensor.humMetricsManager.computeMin());	
	
	printf("METRICS FOR SOIL MOISTURE IN THE LAST HOUR\n");
	printf("AVG %3.1f\n", soilMoistureSensor.metricsManager.computeAverage());
	printf("MAX %3.1f\n", soilMoistureSensor.metricsManager.computeMax());
	printf("MIN %3.1f\n", soilMoistureSensor.metricsManager.computeMin());	
	
	printf("METRICS FOR ACCELEROMETER IN THE LAST HOUR\n");
	printf("X : MAX %3.1f\n", acc.xAxMetricsManager.computeMax());
	printf("X : MIN %3.1f\n", acc.xAxMetricsManager.computeMin());
	
	printf("Y : MAX %3.1f\n", acc.yAxMetricsManager.computeMax());
	printf("Y : MIN %3.1f\n", acc.yAxMetricsManager.computeMin());
	
	printf("Z : MAX %3.1f\n", acc.zAxMetricsManager.computeMax());
	printf("Z : MIN %3.1f\n", acc.zAxMetricsManager.computeMin());
	
	printf("\n\n");
	// Do this for all the other sensors
}

void normalMode() {
	ticker.attach(ticker_isr, NORMAL_MODE_CADENCE);
	Ticker measuresTicker;
	measuresTicker.attach(computeMetricsTickerIsr, COMPUTE_METRICS_CADENCE);
	
	rgbLed.setColor(0, 0, 0);
	while (!buttonPressed){
		if(tick_event){			
			float accValues[3];
			uint16_t rgbValues[4];
			acc.getAllAxis(accValues);
			humtempsensor.measure();
			int dominantColor = rgbSensor.getAllColors(rgbValues); // 0 = RED, 1 = GREEN, 2 = BLUE
			char const* dominantColorName = colorNames[dominantColor];
			float humidity = humtempsensor.get_humidity();
			float temp = humtempsensor.get_temperature();
			float soilMoisture = soilMoistureSensor.getMoistureValue();
			float light  = lightSensor.readLight();
			printValues(accValues, rgbValues, dominantColorName, light, humidity, temp, soilMoisture);
			
			if (gpsInfoAvailable){
				// printGpsInfo();
			}
			
			if (shouldComputeMetrics) {
				printMetrics();
				// Also update a global variable for the predominant color
				lightSensor.metricsManager.reset();
				shouldComputeMetrics = false;
			}
			
			// Check alerts here
			
			if (rgbValues[0] < CLEAR_MIN_LIMIT){ clear_led = 1;} 
			else{clear_led = 0;}
			
			//If the measures are out of range the corresponding RGB color is set
			if(accValues[2] > STAND_LIMIT ){	//Under this value the plant has fallen 			
				rgbLed.setColor(1, 1, 1);
			}else if(temp < TEMP_LIMIT_MIN  || temp > TEMP_LIMIT_MAX ){
				rgbLed.setColor(1, 0, 0);
			}else if (humidity < HUM_LIMIT_MIN  || humidity > HUM_LIMIT_MAX ){
			  rgbLed.setColor(0, 0, 1);
			}else if (light < LIGHT_LIMIT_MIN  || light > LIGHT_LIMIT_MAX ){
				rgbLed.setColor(0, 0, 1);
			}else if (soilMoisture < SOILM_LIMIT_MIN  || soilMoisture > SOILM_LIMIT_MAX ){
				rgbLed.setColor(0, 0, 1);
			}else if (rgbValues[2] < GREEN_LIMIT_MIN){
				rgbLed.setColor(0, 0, 1);
			}else{ //No error
				rgbLed.setColor(0, 0, 0);
			}		
			
			tick_event = false;
		}
	}
}

void testMode() {
	ticker.attach(ticker_isr, TEST_MODE_CADENCE);
	while(!buttonPressed) {
		if(tick_event){
			float accValues[3];
			uint16_t rgbValues[4];
			acc.getAllAxis(accValues);
			humtempsensor.measure();
			int dominantColor = rgbSensor.getAllColors(rgbValues); // 0 = RED, 1 = GREEN, 2 = BLUE
			char const* dominantColorName = colorNames[dominantColor];
			float humidity = humtempsensor.get_humidity();
			float temp = humtempsensor.get_temperature();
			float soilMoisture = soilMoistureSensor.getMoistureValue();
			float light  = lightSensor.readLight();
			rgbLed.setColor(dominantColor);
			printValues(accValues, rgbValues, dominantColorName, light, humidity, temp, soilMoisture);
			if (gpsInfoAvailable){
				//printGpsInfo();
			}
			tick_event = false;
		}
	}
}


int main(void) {
	rgbSensor.enablePowerAndRGBC();

	//GPS
	gps_thread.start(readGps);
	userButton.fall(buttonPressedIsr);
	currentMode = TEST;
	testMode();
	while(true) {
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