#include "./RGBLED.h"
#include "mbed.h"

RGBLED::RGBLED(PwmOut redPin, PwmOut greenPin, PwmOut bluePin) : redPin(redPin), greenPin(greenPin), bluePin(bluePin) {};
	
void RGBLED::setColor(float red, float green, float blue) {
	redPin = red;
	redPin.period_us(1000);
	greenPin = green;
	greenPin.period_us(1000);
	bluePin = blue;
	bluePin.period_us(1000);
}