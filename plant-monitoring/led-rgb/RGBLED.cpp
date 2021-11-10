#include "./RGBLED.h"
#include "mbed.h"

RGBLED::RGBLED(DigitalOut redPin, DigitalOut greenPin, DigitalOut bluePin) : redPin(redPin), greenPin(greenPin), bluePin(bluePin) {};

void RGBLED::setColor(uint8_t red, uint8_t green, uint8_t blue) {
	redPin = red;
	greenPin = green;
	bluePin = blue;
}

void RGBLED::setColor(int color) {
	switch(color){
		case 0: 
			setColor(1,0,0);
      break;
		case 1: 
			setColor(0,1,0);
      break;
		case 2: 
			setColor(0,0,1);
      break;
		case 3: 
			setColor(1,1,0);
      break;
		case 4: 
			setColor(1,0,1);
      break;
		case 5: 
			setColor(1,1,1);
      break;
		case 6:
			setColor(0,0,0);
	default : break;
}

}