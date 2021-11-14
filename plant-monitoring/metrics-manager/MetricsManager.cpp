#include "./MetricsManager.h"

MetricsManager::MetricsManager() {
	currentAvg = 0;
	currentMax = 0;
	currentMin = 0;
	numMeasures = 0;
}


MetricsManager::~MetricsManager() {};

void MetricsManager::addValue(float value) {
	if (!numMeasures || numMeasures == UINT16_MAX) {
		currentAvg = value;
		currentMax = value;
		currentMin = value;
		numMeasures = 1;
	} else {
		currentAvg = (currentAvg * numMeasures + value) / (numMeasures + 1);
		currentMax = value > currentMax ? value : currentMax;
		currentMin = value < currentMin ? value : currentMin;
		numMeasures += 1;
	}
}

float MetricsManager::computeAverage() {
	return currentAvg;
}

float MetricsManager::computeMax() {
	return currentMax;
}

float MetricsManager::computeMin() {
	return currentMin;
}

void MetricsManager::reset() {
	currentAvg = 0;
	currentMax = 0;
	currentMin = 0;
	numMeasures = 0;
}
