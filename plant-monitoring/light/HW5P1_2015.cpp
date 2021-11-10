#include "./HW5P1_2015.h"
#include "./MetricsManager.h"

HW5P1_2015::HW5P1_2015(AnalogIn ain) : input(ain) {
	metricsManager = MetricsManager();
};

HW5P1_2015::~HW5P1_2015() {};

float HW5P1_2015::readLight() {
	float value = float(input.read_u16()) / float(UINT16_MAX) * 300.0;
	value = value > 100 ? 100 : value;
	metricsManager.addValue(value);
	return value;
}	
