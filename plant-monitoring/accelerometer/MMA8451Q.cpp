#include "MMA8451Q.h"
#include "./MetricsManager.h"
 
#define REG_WHO_AM_I      		0x0D
#define REG_CTRL_REG_1    		0x2A
#define REG_OUT_X_MSB     		0x01
#define REG_OUT_Y_MSB     		0x03
#define REG_OUT_Z_MSB     		0x05
#define REG_INTERRUPT_CFG 		0x2E //Bit 2 = Free fall, Bit 3 = Tap
#define REG_PULSE_CFG     		0x21
#define REG_PULSE_SRC					0x22
#define CTRL_REG4         		0x2D
#define REG_PULSE_THRESHOLD_Z 0x25
#define PULSE_TMLT  					0x26
#define PULSE_LTCY  					0x27
#define REG_FF_MT_CFG					0x15
#define REG_FF_MT_SRC					0x16
#define REG_FF_MT_THS 				0x17

#define UINT14_MAX        16383
 
MMA8451Q::MMA8451Q(PinName sda, PinName scl, int addr) : m_i2c(sda, scl), m_addr(addr) {
    
		//Stand by mode for write registers
		uint8_t standbyactive[2] = {REG_CTRL_REG_1,0x08};
		writeRegs(standbyactive, 1);
		// enable single pulse in z 
		uint8_t tapCfgData[2] = {REG_PULSE_CFG, 0x10};
		writeRegs(tapCfgData, 2);
		// set threshold
		uint8_t tapTHZ[2] = {REG_PULSE_THRESHOLD_Z, 0x02};
		writeRegs(tapTHZ, 2);
		// set limit tap detection
		uint8_t window[2] = {PULSE_TMLT, 0x30};
		writeRegs(window, 2);
		// Set latency
		uint8_t latency[2] = {PULSE_LTCY, 0xF0};
		writeRegs(latency, 2);
		// Route INT1 to system interrupt
		uint8_t pulseEnableInt[2] = {CTRL_REG4, 0x08};
		writeRegs(pulseEnableInt, 2);
		uint8_t interruptCfgData[2] = {REG_INTERRUPT_CFG, 0x08};
		writeRegs(interruptCfgData, 2);
		
		// Enable freefall detection on X, Y, Z
		uint8_t freeFallEnableData[2] = {REG_FF_MT_CFG, 0x1D};
		writeRegs(freeFallEnableData, 2);
		
		// Set freefall threshold
		uint8_t freeFallThresholdData[2] = {REG_FF_MT_THS, 0x14};
		writeRegs(freeFallThresholdData, 2);
		
		// Set acc to active mode
		uint8_t c = 0;
		readRegs(REG_CTRL_REG_1, &c, 1);
		uint8_t active = c | 0x01;
		uint8_t setActivemode[2] = {REG_CTRL_REG_1, active};
		writeRegs(setActivemode, 2); 
		
		
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