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
#define TEMP_ALERT_COLOR 0 //"RED"
// Maginutude : %
#define HUM_LIMIT_MIN   10
#define HUM_LIMIT_MAX		95
#define HUM_ALERT_COLOR 1 //"GREEN"
// Magnitude : %
#define SOILM_LIMIT_MIN 10
#define SOILM_LIMIT_MAX 90
#define SOILM_ALERT_COLOR 2 //"BLUE"
// Magnitude : %
#define LIGHT_LIMIT_MIN 2
#define LIGHT_LIMIT_MAX 95
#define LIGHT_ALERT_COLOR 3 //"REDGREEN"
// Magnitude : 
#define GREEN_LIMIT_MIN 450
#define NOT_GREEN_ALERT_COLOR 4 //"REDBLUE"
// Magnitude : g
#define STAND_LIMIT			-0.5
#define NOT_STAND_ALERT_COLOR 5 //"REDGREENBLUE"

#define TURN_OFF_RGBLED 6 

// Magnitude :
#define CLEAR_MIN_LIMIT 2000 //if below, clear led = ON

#define NORMAL_MODE_CADENCE 1000ms
#define TEST_MODE_CADENCE 2000ms
#define COMPUTE_METRICS_CADENCE 30000ms //1 hour in miliseconds

#define N_MEASURES 			10   //Number of measures made in NORMAL_MODE after which mean,min,max values are displayed without scaling

#define SCALE_FACTOR    12  // With this factor we can scale to fullfill th requirements (avg,min,.max,... each 1h) 

using namespace std::chrono;

enum Mode { TEST, NORMAL };

Mode currentMode;

MMA8451Q acc(PB_9,PB_8,0x1d<<1);
TCS3472_I2C rgbSensor(PB_9, PB_8);
HW5P1_2015 lightSensor(PA_4);
RGBLED rgbLed(PH_0, PB_13, PH_1);
Si7021 humtempsensor(PB_9,PB_8);
SEN_13322 soilMoistureSensor(PA_0);
BufferedSerial* gps_Serial = new BufferedSerial(PA_9, PA_10,9600); //serial object for use w/ GPS
Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
InterruptIn userButton(PB_2);
InterruptIn accelerometerInt(PB_12);
InterruptIn accelerometerFreefallInt(PB_14);
Thread gps_thread(osPriorityNormal,2048);
DigitalOut clear_led(PB_7);
DigitalOut led1(LED1);
DigitalOut led2(LED2);

uint8_t hour, minute, seconds_gps, year, month, day;
uint16_t milliseconds_gps;
uint8_t fixquality, satellites;
float latitude, longitude, geoidheight, altitude;
float speed, angle;
char lat, lon, mag;
char const* colorNames[3] = {"RED", "GREEN", "BLUE"};
bool tick_event;
bool buttonPressed = false;
bool accInterrupted = false;
bool accFreeFallInterrupted = false;
bool shouldComputeMetrics = false;
bool accDoubleInterrupted = false;

Mutex mutex;
Ticker ticker;
Timeout doubleTapDetections;

void ticker_isr(void){
	tick_event = true;
}

void buttonPressedIsr() {
	buttonPressed = true;
}

void accIsr() {
	accInterrupted = true;
}

void accFreeFallIsr() {
	accFreeFallInterrupted = true;
}

void doubleTapIsr() {
	accDoubleInterrupted = true;
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
				if (!myGPS.parse(myGPS.lastNMEA())) {
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
		}
}

void printGpsInfo() {
	mutex.lock();
	printf("TIME: %d:%d:%d.%u", hour, minute, seconds_gps, milliseconds_gps);
	printf(" DATE: %d/%d/20%d",day, month, year);
	printf(" LOCATION: %5.2f %c, %5.2f %c", latitude, lat, longitude, lon);
	printf(" SPEED: %5.2f knots", speed);
	printf(" ALTITUDE: %5.2fz", altitude);
	printf(" SATELLITES: %d", satellites);
	printf("\n\n");
	mutex.unlock();
}

void printValues(float acc[], uint16_t rgb[], char const* dominantColorName, float light, float humidity, float temp, float soilMoisture) {
	printf("ACCELEROMETER: X_AXIS=%f \t Y_AXIS=%f\t Z_AXIS=%f\n", acc[0], acc[1], acc[2]);
	printf("COLOR SENSOR: CLEAR=%d, RED=%d, GREEN=%d, BLUE=%d -- DOMINANT COLOR=%s\n", rgb[0], rgb[1], rgb[2], rgb[3], dominantColorName);
	printf("LIGHT: %3.1f%% \n", light);
	printf("TEMPERATURE: %2.2f C \n", temp);
	printf("HUMIDITY: %2.2f%%  \n", humidity);
	printf("SOIL MOISTURE: %2.2f%% \n", soilMoisture);
	printGpsInfo();
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
	
	printf("DOMINANT COLOR IN THE LAST HOUR: %s\n", colorNames[rgbSensor.getPredominantColor()]);
	
	printf("\n\n");
	// Do this for all the other sensors
}
void checkAlerts(float acc[], uint16_t rgb[], char const* dominantColorName, float light, float humidity, float temp, float soilMoisture){
	if (rgb[0] < CLEAR_MIN_LIMIT) {
			clear_led = 1;
	} else {
		clear_led = 0;
	}
	
	//If the measures are out of range the corresponding RGB color is set
	if(acc[2] > STAND_LIMIT ){	//Under this value the plant has fallen 			
		rgbLed.setColor(NOT_STAND_ALERT_COLOR);
	} else if(temp < TEMP_LIMIT_MIN  || temp > TEMP_LIMIT_MAX ) {
			rgbLed.setColor(TEMP_ALERT_COLOR);
	} else if (humidity < HUM_LIMIT_MIN  || humidity > HUM_LIMIT_MAX ) {
			rgbLed.setColor(HUM_ALERT_COLOR);
	} else if (light < LIGHT_LIMIT_MIN  || light > LIGHT_LIMIT_MAX ) {
			rgbLed.setColor(LIGHT_ALERT_COLOR);
	} else if (soilMoisture < SOILM_LIMIT_MIN  || soilMoisture > SOILM_LIMIT_MAX ) {
			rgbLed.setColor(SOILM_ALERT_COLOR);
	} else if (rgb[2] < GREEN_LIMIT_MIN) {
			rgbLed.setColor(NOT_GREEN_ALERT_COLOR);
	} else { //No alerts
			rgbLed.setColor(TURN_OFF_RGBLED);
	}
}

void normalMode() {
	ticker.attach(ticker_isr, NORMAL_MODE_CADENCE);
	Ticker measuresTicker;
	measuresTicker.attach(computeMetricsTickerIsr, COMPUTE_METRICS_CADENCE);
	clear_led = 0;
	led1 = 0;
	led2 = 1;
	rgbLed.setColor(TURN_OFF_RGBLED);
	while (!buttonPressed) {
		if(tick_event) {			
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
			
			if (shouldComputeMetrics) {
				printMetrics();
				// Also update a global variable for the predominant color
				lightSensor.metricsManager.reset();
				humtempsensor.humMetricsManager.reset();
				humtempsensor.tempMetricsManager.reset();
				soilMoistureSensor.metricsManager.reset();
				acc.xAxMetricsManager.reset();
				acc.yAxMetricsManager.reset();
				acc.zAxMetricsManager.reset();
				
				shouldComputeMetrics = false;
			}
			// Check alerts here
			checkAlerts(accValues, rgbValues, dominantColorName, light, humidity, temp, soilMoisture);
			tick_event = false;
		}
		if (accInterrupted) {
				uint8_t c = 0;
				acc.readRegs(0x22, &c, 1);
			  if ((c&0x08)==0x08) {
					printf("DOUBLE TAP DETECTED\n");
				}else{
					printf("TAP DETECTED\n");
				}
				accInterrupted = false;
		}
		if (accFreeFallInterrupted) {
				printf("FREEFALL DETECTED\n");
				accFreeFallInterrupted = false;
		}
	}
}

void testMode() {
	ticker.attach(ticker_isr, TEST_MODE_CADENCE);
	clear_led = 1;
		led1 = 1;
		led2 = 0;
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
			tick_event = false;
		}
		if (accInterrupted) {
			uint8_t c = 0;
			acc.readRegs(0x0C, &c, 1);
			wait_us(800000);
			acc.readRegs(0x0C, &c, 1);
			if ((c&0x08)==0x08) {
				printf("DOUBLE TAP DETECTED\n");
			}else{
				printf("TAP DETECTED\n");
			}
			accInterrupted = false;
		}
		if (accFreeFallInterrupted) {
				printf("FREEFALL DETECTED\n");
				accFreeFallInterrupted = false;
		}
	}
}


int main(void) {
	rgbSensor.enablePowerAndRGBC();
	accelerometerInt.rise(accIsr);
	accelerometerFreefallInt.rise(accFreeFallIsr);
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