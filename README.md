# Plant monitoring system
Plant monitoring system for B-L072Z-LRWAN1 ARM platform

# System requirements
- The system should measure:
  - Temperature
  - Soil moisture
  - Humidity
  - Light
  - Dominant color
  - Location
  - Current date and time
  - Acceleration in X, Y and Z axis
- The system should have 2 operational modes:
  - Normal
    - All values should be printed every 2 seconds through the COM serial interface
    - The RGB LED should be colored in the dominant color detected by the colour sensor
  - Test
    - All values should be printed every 30 seconds
    - The system should calculate the mean, maximum and minimum values of temperature, relative humidity,
  ambient light and soil moisture every hour. These values are sent to the computer when calculated.
    - The system should calculate the dominant colour of the leave every hour. This means to calculate which colour
  has appeared as dominant more times during the last hour. This value is sent to the computer when
  calculated.
    - The system should calculate the maximum and minimum values of the three axes (X, y and Z) of the
  accelerometer every hour. These values are sent to the computer when calculated.
    - The global location of the plant (coordinates) should be sent to the computer every 30 seconds. This should
  include the GPS time converted to local time.
    - Limits for every measured variable (temperature, humidity, ambient light, soil moisture, colour and
  acceleration) should be fixed. If current values of the measured parameters are outside the limits, the
  RGB LED should indicate this situation using different colour for every parameter

In addition, the accelerometer sensor should be configured such that both freefall and tap events are detected and notified using an interruption.

# Sensors used
- **GPS**: Mbed adafruit TCS34725
- **Temperature and humidity sensor**: Adafruit Si7021 
- **Soil moisture sensor**
- **HW5P-1 Photo Transistor Light sensor**
- **RGB color sensor**: Adafruit TCS34725
- **Accelerometer**: MMA8451Q
