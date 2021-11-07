#ifndef GPSinfo_H
#define GPSinfo_H
 
#include "mbed.h"

class GPSinfo
{
public:
    GPSinfo(uint8_t hour,uint8_t minute,uint8_t seconds_gps,uint8_t year,uint8_t month,uint8_t day,uint8_t fixquality,uint8_t satellites, );
    ~GPSinfo();
		uint8_t hour, minute, seconds_gps, year, month, day;
		uint16_t milliseconds_gps;
		float latitude, longitude, geoidheight, altitude;
		float speed, angle;
		char lat, lon, mag;
		bool fix;
		uint8_t fixquality, satellites;
	
	
    
};
 #endif