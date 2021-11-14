#ifndef RGBLED_H
#define RGBLED_H
#include "mbed.h"

enum Color { RED = 0, GREEN = 1, BLUE = 2};	

class RGBLED {
	public:
		DigitalOut redPin;
		DigitalOut greenPin;
	  DigitalOut bluePin;
		/**
		* Constructor
		* @param redPin PWM pin for red
		* @param grenPin PWM pin for green
		* @param bluePin PWM pin for blue
		*/
		RGBLED(DigitalOut redPin, DigitalOut greenPin, DigitalOut bluePin); 
		
		/** 
		* Destructor
		*/
		~RGBLED();
	
	/**
	* Sets the led to the desired color, expressed in RGB format (0.0 - 1.0)
	* @param red component for red (0.0 - 1.0)
	* @param blue component for blue (0.0 - 1.0)
	* @param green component for green (0.0 - 1.0)
	*/
		void setColor(uint8_t red, uint8_t green, uint8_t blue);
		
		void setColor(int color);
		
};

#endif