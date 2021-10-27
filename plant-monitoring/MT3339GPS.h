#ifndef  MT3339GPS_H
#define  MT3339GPS_H


#include "mbed.h"

class HW5P1_2015 {
	public:
	 Serial gps;
	 Si7021(PinName tx, PinName rx);
	 ~Si7021();

	void getLine(char *data);
	uint8_t getHourGPS(char *data);
	uint8_t getMinutesGPS(char *data);
	uint8_t getSecondsGPS(char *data);

	float getLatGPS(char *data);
	float getLongGPS(char *data);
	float getAltitude(char *data);
	uint8_t getNumSatellites(char * data);
};

#endif