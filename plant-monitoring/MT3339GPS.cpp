#include "MT3339GPS.h"

//String -> >$GPGGA,064951.000,2307.1256,N,12016.4438,E,1,8,0.95,39.9,M,17.8,M,,*65


//Get GPGGA frame
void getLine(char *data){

	do{
		while(gps.getc() != '$'); //Wait for the start of a line
		
		//Store the header (GPGGA, GPGSA, GPRMC, ....)
		for(uint16_t i = 0; i<5; i++){
				data[i] = gps.getc();
		}	
			
		//Check that the header equals "GPGAA"
		if (data[0]=='G' && data[1]=='P' && data[2]=='G' && data[3]=='G' && data[4]=='A'){
			//if true, store next values
			for(uint16_t i = 5; i<90; i++){ //81
				data[i] = gps.getc();
				if(data[i] == '\r'){
						data[i] = 0;
				}
			}	
			
		}
	}while(data[0]!='G' || data[1]!='P' || data[2]!='G' || data[3]!='G' || data[4]!='A');

}
	
			
			
uint8_t getHourGPS(char *data){

  int time = 0;

  sscanf(data, "GPGGA,%d,%*f,%*c,%*f,%*c,%*d,%*d,%*f,%*f,%*c,%*f", &time);

  return (uint8_t)(time/10000);
}

uint8_t getMinutesGPS(char *data){

  int time = 0;

  sscanf(data, "GPGGA,%d,%*f,%*c,%*f,%*c,%*d,%*d,%*f,%*f,%*c,%*f", &time);

  return (uint8_t)((time/100)-((time/10000)*100));
}

uint8_t getSecondsGPS(char *data){

  int time = 0;

  sscanf(data, "GPGGA,%d,%*f,%*c,%*f,%*c,%*d,%*d,%*f,%*f,%*c,%*f", &time);

  return (uint8_t)((time)-((int)(time/100)*100));
}
  
float getLatGPS(char *data){

  float latitude = 0; 
  char ns;  

  sscanf(data, "GPGGA,%*f,%f,%c,%*f,%*c,%*d,%*d,%*f,%*f,%*c,%*f", &latitude,&ns);
  latitude = (int)(latitude/100) +  ((latitude - ((int)((latitude/100))*100))/60);
  if(ns == 'S')latitude *= -1.0;
  
  return latitude;
  
}

float getLongGPS(char *data){

  float longitude = 0; 
  char ew;  

  sscanf(data, "GPGGA,%*f,%*f,%*c,%f,%c,%*d,%*d,%*f,%*f,%*c,%*f", &longitude,&ew);
  longitude = (int)(longitude/100) +  (longitude - ((int)((longitude/100))*100))/60;
  if(ew == 'W')longitude *= -1.0;

  return longitude;
}

uint8_t getNumSatellites(char *data){
  
  int satellites = 0;
  sscanf(data, "GPGGA,%*f,%*f,%*c,%*f,%*c,%*d,%d,%*f,%*f,%*c,%*f",&satellites);
  return ((uint8_t) satellites);

}

float getAltitude(char *data){
  
  float altitude = 0;

  sscanf(data, "GPGGA,%*f,%*f,%*c,%*f,%*c,%*d,%*d,%*f,%f,%*c,%*f",&altitude);
  
  return altitude;
}



