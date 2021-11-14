#include "MMA8451Q.h"
#include "./MetricsManager.h"
 
#define REG_WHO_AM_I      0x0D
#define REG_CTRL_REG_1    0x2A
#define REG_OUT_X_MSB     0x01
#define REG_OUT_Y_MSB     0x03
#define REG_OUT_Z_MSB     0x05
 
#define UINT14_MAX        16383
 
MMA8451Q::MMA8451Q(PinName sda, PinName scl, int addr) : m_i2c(sda, scl), m_addr(addr) {
    // activate the peripheral
    uint8_t data[2] = {REG_CTRL_REG_1, 0x01};
    writeRegs(data, 2);
		xAxMetricsManager = MetricsManager();
		yAxMetricsManager = MetricsManager();
		zAxMetricsManager = MetricsManager();
}
 
MMA8451Q::~MMA8451Q() { }
 
uint8_t MMA8451Q::getWhoAmI() {
    uint8_t who_am_i = 0;
    readRegs(REG_WHO_AM_I, &who_am_i, 1);
    return who_am_i;
}

int16_t MMA8451Q::concatValues(int16_t reg0, int16_t reg1) {
	int16_t value = (reg0 << 6) | (reg1 >> 2);
	if (value > UINT14_MAX/2)
		value -= UINT14_MAX;
	return value;
}

void MMA8451Q::getAllAxis(float * returnValue) {
	uint8_t res[6];
	
	readRegs(REG_OUT_X_MSB, res, 6);
	returnValue[0] = (-1)*float(concatValues(res[0], res[1])) / 4096.0;
	xAxMetricsManager.addValue(returnValue[0]);
	returnValue[1] = (-1)*float(concatValues(res[2], res[3])) / 4096.0;
	yAxMetricsManager.addValue(returnValue[1]);
	returnValue[2] = (-1)*float(concatValues(res[4], res[5])) / 4069.0;
	zAxMetricsManager.addValue(returnValue[2]);
	
}
 
void MMA8451Q::readRegs(int addr, uint8_t * data, int len) {
    char t[1] = {(char)addr};
    m_i2c.write(m_addr, t, 1, true);
    m_i2c.read(m_addr, (char *)data, len);
}
 
void MMA8451Q::writeRegs(uint8_t * data, int len) {
    m_i2c.write(m_addr, (char *)data, len);
}