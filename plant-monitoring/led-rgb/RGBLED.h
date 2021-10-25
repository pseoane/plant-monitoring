#ifndef RGBLED_H
#define RGBLED_H
#include "mbed.h"
class RGBLED {
	public:
		PwmOut redPin;
		PwmOut greenPin;
	  PwmOut bluePin;
		/**
		* Constructor
		* @param redPin PWM pin for red
		* @param grenPin PWM pin for green
		* @param bluePin PWM pin for blue
		*/
		RGBLED(PwmOut redPin, PwmOut greenPin, PwmOut bluePin); 
		
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
		void setColor(float red, float green, float blue);
};

#endif