#ifndef MetricsManager_H
#define MetricsManager_H
#include "mbed.h"

class MetricsManager {
	public:
		/**
		* Constructor
		*/
		MetricsManager();
		
	
		/** 
		* Destructor
		*/
		~MetricsManager();
	
		/**
		* Adds a value to the average
		* @param value value to be added 
		*/
		void addValue(float value);
		
		/**
		* Removes all metrics information
		*/
		void reset();
		
		/**
		* @returns the average of all values added
		*/
		float computeAverage();
		
		/**
		* @returns the max of all values added
		*/
		float computeMax();
		
		/**
		* @returns the min of all values added
		*/
		float computeMin();
	private:
		float currentAvg;
		float currentMin;
		float currentMax;
		bool firstRead;
		uint16_t numMeasures;
};
#endif