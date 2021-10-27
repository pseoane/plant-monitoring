#include "./RGBLED.h"
#include "mbed.h"

RGBLED::RGBLED(DigitalOut redPin, DigitalOut greenPin, DigitalOut bluePin) : redPin(redPin), greenPin(greenPin), bluePin(bluePin) {};

void RGBLED::setColor(uint8_t red, uint8_t green, uint8_t blue) {
	redPin = red;
	greenPin = green;
	bluePin = blue;
}

void RGBLED::setColor(int color) {
	redPin = color == RED;
	bluePin = color == BLUE;
	greenPin = color == GREEN;
}