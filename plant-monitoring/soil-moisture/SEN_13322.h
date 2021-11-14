#ifndef SEN_13322_H
#define SEN_13322_H
#include "mbed.h"
#include "./MetricsManager.h"

class SEN_13322 {
	public:
		AnalogIn pin;
		
		/**
		* Constructor
		*/
		SEN_13322(PinName input);
	
		/**
		* Destructor
		*/
		~SEN_13322();
	
		/** Returns the current moisture percentage
		* @returns current humidity value from 0.0 to 100.0
		*/
		float getMoistureValue();
	
		MetricsManager metricsManager;
			
};

#endif