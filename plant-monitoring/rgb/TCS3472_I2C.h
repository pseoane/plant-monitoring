#ifndef TCS3472_I2C_H
#define TCS3472_I2C_H
#include "mbed.h"
 
//Defines 
#define SLAVE_ADDRESS           0x29
 
#define ENABLE                  0x00
#define ATIME                   0x01
#define WTIME                   0x03
#define AILTL                   0x04
#define AIHTL                   0x06
#define PERS                    0x0C
#define CONFIG                  0x0D
#define CONTROL                 0x0F
#define ID                      0x12
#define STATUS                  0x13
#define CDATA                   0x14
#define RDATA                   0x16
#define GDATA                   0x18
#define BDATA                   0x1A
 
/** TCS3472_I2C class.
 *  Used to read to and write from TCS3472 color sensor.
 */
class TCS3472_I2C {
public:
    /** Create TCS3472_I2C instance
     *
     * @param sda sda pin for I2C
     * @param scl scl pin for I2C
     */
    TCS3472_I2C( PinName sda, PinName scl );
    
    /** Destructor
     */
    ~TCS3472_I2C();
    
    /** Read red, green, blue and clear values into array
     *
     * @param readings Array of four integers to store the read data
		 * @returns
		 *   Predominant color (0 = RED, 1 = GREEN, 2 = BLUE)
     */
    int getAllColors(uint16_t* readings );
    
    /** Power ON. Activates the internal oscillator to permit the timers and ADC channels to operate.
     *
     * @returns
     *     1 if successful
     *     0 if otherwise
     */
    int enablePower();
    
    /** Power OFF. Disables the internal oscillator.
     *
     * @returns
     *     1 if successful
     *     0 if otherwise
     */
    int disablePower();
    
    /** Checks if power is on (i.e. internal oscillator enabled)
     *
     * @returns
     *     1 if power ON.
     *     0 if power OFF.
     */
    bool isPowerEnabled();
    
    /** Activates the two-channel ADC.
     *
     * @returns
     *     1 if successful
     *     0 if otherwise
     */
    int enableRGBC();
    
    /** Disables the two-channel ADC.
     *
     * @returns
     *     1 if successful
     *     0 if otherwise
     */
    int disableRGBC();
    
    /** Checks if the two-channel ADC is enabled or not.
     *
     * @returns
     *     1 if ADC enabled.
     *     0 if ADC not enabled.
     */
    bool isRGBCEnabled();
    
    /** Activates internal oscillator and two-channel ADC simultaneously (both necessary for standard operation).
     *
     * @returns
     *     1 if successful
     *     0 if otherwise
     */
    int enablePowerAndRGBC();
    
    /** Disables internal oscillator and two-channel ADC simultaneously.
     *
     * @returns
     *     1 if successful
     *     0 if otherwise
     */
    int disablePowerAndRGBC();
    
    /** Activates the wait feature.
     *
     * @returns
     *     1 if successful
     *     0 if otherwise
     */
    int enableWait();
    
    /** Disables the wait feature.
     *
     * @returns
     *     1 if successful
     *     0 if otherwise
     */
    int disableWait();
    
    /** Checks if wait is enabled
     *
     * @returns
     *     1 if enabled.
     *     0 if not enabled.
     */
    bool isWaitEnabled();
    
    /** Permits RGBC interrupts to be generated.
     *
     * @returns
     *     1 if successful
     *     0 if otherwise
     */
    int enableInterrupt();
    
    /** Forbids RGBC interrupts to be generated.
     *
     * @returns
     *     1 if successful
     *     0 if otherwise
     */
    int disableInterrupt();
    
    /** Checks if RGBC interrupt is enabled.
     *
     * @returns
     *     1 if enabled.
     *     0 if not enabled.
     */
    bool isInterruptEnabled();
    
    /** Sets wait time.
     *
     * @param wtime Wait time to set in milliseconds. Should be in the range 2.4 - 7400ms.
     *
     * @returns
     *     1 if successful
     *     0 if otherwise
     */
    int setWaitTime( const float wtime );
    
    /** Reads the current wait time.
     *
     * @returns
     *     Wait time in milliseconds
     */
    float readWaitTime();
    
    /** Sets integration time.
     *
     * @param itime Integration time to set in milliseconds. Should be in the range 2.4 - 614.4ms.
     *
     * @returns
     *     1 if successful
     *     0 if otherwise
     */
    int setIntegrationTime( const float itime );
    
    /** Reads the current integration time.
     *
     * @returns
     *     Integration time in milliseconds
     */
    float readIntegrationTime();
    
    /** Reads the enable register byte as a char.
     *
     * @returns
     *     Enable register byte
     */
    char readEnableRegister();
    
    /** Reads the low trigger point for the comparison function for interrupt generation.
     *
     * @returns
     *     Low threshold value
     */
    int readLowInterruptThreshold();
    
    /** Reads the high trigger point for the comparison function for interrupt generation.
     *
     * @returns
     *     High threshold value
     */
    int readHighInterruptThreshold();
    
    /** Sets the low trigger point for the comparison function for interrupt generation.
     *
     * @param threshold Low threshold value
     *
     * @returns
     *     1 if successful
     *     0 if otherwise
     */
    int setLowInterruptThreshold( const int threshold );
    
    /** Sets the high trigger point for the comparison function for interrupt generation.
     *
     * @param threshold High threshold value
     *
     * @returns
     *     1 if successful
     *     0 if otherwise
     */
    int setHighInterruptThreshold( const int threshold );
    
    /** Returns the number of consecutive values out of range (set by the low and high thresholds) required to trigger an interrupt
     *
     * @returns
     *     interrput persistence
     */
    int readInterruptPersistence();
    
    /** Sets the number of consecutive values out of range (set by the low and high thresholds) required to trigger an interrupt
     *
     * @param persistence Value to set. Must be 0, 1, 2, 3, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55 or 60.
     *
     * @returns
     *     1 if successful
     *     0 if otherwise
     */
    int setInterruptPersistence( const int persistence );
    
    /** Clears the interrupt bit, allowing normal operation to resume.
     *  (writes 0b11100110 to the command register to clear interrupt)
     *
     * @returns
     *     1 if successful
     *     0 if otherwise
     */
    int clearInterrupt();
    
    /** Read RGBC gain.
     *
     * @returns
     *     RGBC gain
     */
    int readRGBCGain();
    
    /** Sets RGBC gain
     *
     * @param gain Gain to set. Must be 1, 4, 16 or 60.
     *
     * @returns
     *     1 if successful
     *     0 if otherwise
     */
    int setRGBCGain( const int gain );
    
    /** Returns Device ID.
     *
     * @returns
     *     0x44 = TCS34721 and TCS34725
     *     0x4D = TCS34723 and TCS34727
     */
    char getDeviceID();
    
    /** Reads the status register byte as a char.
     *
     * @returns
     *     Status register byte
     */
    char readStatusRegister();
		
		int getPredominantColor();
 
private:
    I2C i2c;
    
    int writeSingleRegister( char address, char data );
    int writeMultipleRegisters( char address, char* data, int quantity );
    char readSingleRegister( char address );
    int readMultipleRegisters( char address, char* output, int quantity );
    int getPredominantColor(uint16_t* readings);
    float roundTowardsZero( const float value );
		int dominantColorFrecuencies[3];
};
#endif