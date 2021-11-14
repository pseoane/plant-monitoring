#include "./SEN_13322.h"
#include "./MetricsManager.h"

SEN_13322::SEN_13322(PinName pinName) : pin(pinName) {
	metricsManager = MetricsManager();
}
SEN_13322::~SEN_13322() {}
	
float SEN_13322::getMoistureValue() {
	float value = (float(pin.read_u16()) / 65536.0) * 100.0;
	metricsManager.addValue(value);
	return value;
}