#include "./HW5P1_2015.h"

HW5P1_2015::HW5P1_2015(AnalogIn ain) : input(ain) {};
HW5P1_2015::~HW5P1_2015() {};

float HW5P1_2015::readLight() {
	return float(input.read_u16()) / float(UINT16_MAX) * 100.0;
}	
