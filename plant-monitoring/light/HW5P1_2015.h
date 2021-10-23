#ifndef HW5P1_2015_H
#define HW5P1_2015_H
#include "mbed.h"

class HW5P1_2015 {
	
	public:
		AnalogIn input;
		/**
	  * Constructor
	  * @param ain analog input to which the sensor is connected
	  */
		HW5P1_2015(AnalogIn ain);
	
		/**
	  * Destructor 
	  */
		~HW5P1_2015();
	
	  /** Read light value
	  * @returns 
	  * Current light value from 0.0 to 100.0
	  */
		float readLight();
};

#endif
