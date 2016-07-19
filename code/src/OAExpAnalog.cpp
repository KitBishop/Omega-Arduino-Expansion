#include "OAExpAnalog.h"
#include <unistd.h>

/*=========================================================================
CONVERSION DELAY (in mS)
-----------------------------------------------------------------------*/
#define ADS1015_CONVERSIONDELAY         (1)
#define ADS1115_CONVERSIONDELAY         (8)
/*=========================================================================*/

/*=========================================================================
POINTER REGISTER
-----------------------------------------------------------------------*/
#define ADS1X15_REG_POINTER_MASK        (0x03)
#define ADS1X15_REG_POINTER_CONVERT     (0x00)
#define ADS1X15_REG_POINTER_CONFIG      (0x01)
#define ADS1X15_REG_POINTER_LOWTHRESH   (0x02)
#define ADS1X15_REG_POINTER_HITHRESH    (0x03)
/*=========================================================================*/

/*=========================================================================
CONFIG REGISTER
-----------------------------------------------------------------------*/
#define ADS1X15_REG_CONFIG_OS_MASK      (0x8000)
#define ADS1X15_REG_CONFIG_OS_SINGLE    (0x8000)  // Write: Set to start a single-conversion
#define ADS1X15_REG_CONFIG_OS_BUSY      (0x0000)  // Read: Bit = 0 when conversion is in progress
#define ADS1X15_REG_CONFIG_OS_NOTBUSY   (0x8000)  // Read: Bit = 1 when device is not performing a conversion

#define ADS1X15_REG_CONFIG_MUX_MASK     (0x7000)
#define ADS1X15_REG_CONFIG_MUX_DIFF_0_1 (0x0000)  // Differential P = AIN0, N = AIN1 (default)
#define ADS1X15_REG_CONFIG_MUX_DIFF_0_3 (0x1000)  // Differential P = AIN0, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_DIFF_1_3 (0x2000)  // Differential P = AIN1, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_DIFF_2_3 (0x3000)  // Differential P = AIN2, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // Single-ended AIN0
#define ADS1X15_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // Single-ended AIN1
#define ADS1X15_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // Single-ended AIN2
#define ADS1X15_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // Single-ended AIN3

#define ADS1X15_REG_CONFIG_PGA_MASK     (0x0E00)
#define ADS1X15_REG_CONFIG_PGA_6_144V   (0x0000)  // +/-6.144V range = Gain 2/3
#define ADS1X15_REG_CONFIG_PGA_4_096V   (0x0200)  // +/-4.096V range = Gain 1
#define ADS1X15_REG_CONFIG_PGA_2_048V   (0x0400)  // +/-2.048V range = Gain 2 (default)
#define ADS1X15_REG_CONFIG_PGA_1_024V   (0x0600)  // +/-1.024V range = Gain 4
#define ADS1X15_REG_CONFIG_PGA_0_512V   (0x0800)  // +/-0.512V range = Gain 8
#define ADS1X15_REG_CONFIG_PGA_0_256V   (0x0A00)  // +/-0.256V range = Gain 16

#define ADS1X15_REG_CONFIG_MODE_MASK    (0x0100)
#define ADS1X15_REG_CONFIG_MODE_CONTIN  (0x0000)  // Continuous conversion mode
#define ADS1X15_REG_CONFIG_MODE_SINGLE  (0x0100)  // Power-down single-shot mode (default)

#define ADS1X15_REG_CONFIG_DR_MASK      (0x00E0)  
#define ADS1X15_REG_CONFIG_DR_128SPS    (0x0000)  // 128 samples per second
#define ADS1X15_REG_CONFIG_DR_250SPS    (0x0020)  // 250 samples per second
#define ADS1X15_REG_CONFIG_DR_490SPS    (0x0040)  // 490 samples per second
#define ADS1X15_REG_CONFIG_DR_920SPS    (0x0060)  // 920 samples per second
#define ADS1X15_REG_CONFIG_DR_1600SPS   (0x0080)  // 1600 samples per second (default)
#define ADS1X15_REG_CONFIG_DR_2400SPS   (0x00A0)  // 2400 samples per second
#define ADS1X15_REG_CONFIG_DR_3300SPS   (0x00C0)  // 3300 samples per second

#define ADS1X15_REG_CONFIG_CMODE_MASK   (0x0010)
#define ADS1X15_REG_CONFIG_CMODE_TRAD   (0x0000)  // Traditional comparator with hysteresis (default)
#define ADS1X15_REG_CONFIG_CMODE_WINDOW (0x0010)  // Window comparator

#define ADS1X15_REG_CONFIG_CPOL_MASK    (0x0008)
#define ADS1X15_REG_CONFIG_CPOL_ACTVLOW (0x0000)  // ALERT/RDY pin is low when active (default)
#define ADS1X15_REG_CONFIG_CPOL_ACTVHI  (0x0008)  // ALERT/RDY pin is high when active

#define ADS1X15_REG_CONFIG_CLAT_MASK    (0x0004)  // Determines if ALERT/RDY pin latches once asserted
#define ADS1X15_REG_CONFIG_CLAT_NONLAT  (0x0000)  // Non-latching comparator (default)
#define ADS1X15_REG_CONFIG_CLAT_LATCH   (0x0004)  // Latching comparator

#define ADS1X15_REG_CONFIG_CQUE_MASK    (0x0003)
#define ADS1X15_REG_CONFIG_CQUE_1CONV   (0x0000)  // Assert ALERT/RDY after one conversions
#define ADS1X15_REG_CONFIG_CQUE_2CONV   (0x0001)  // Assert ALERT/RDY after two conversions
#define ADS1X15_REG_CONFIG_CQUE_4CONV   (0x0002)  // Assert ALERT/RDY after four conversions
#define ADS1X15_REG_CONFIG_CQUE_NONE    (0x0003)  // Disable the comparator and put ALERT/RDY in high state (default)

/*=========================================================================*/

OAExpAnalog::OAExpAnalog(OAExpAnalog_DeviceType type) {
    setup(new I2CDevice(DEFAULT_OAEXPANALOG_DEV_ADDR), type);
}

OAExpAnalog::OAExpAnalog(byte devAddr, OAExpAnalog_DeviceType type) {
    setup(new I2CDevice(devAddr), type);
}

OAExpAnalog::OAExpAnalog(I2CDevice * i2cDev, OAExpAnalog_DeviceType type) {
    setup(i2cDev, type);
}

void OAExpAnalog::setup(I2CDevice * i2cDev, OAExpAnalog_DeviceType type) {
    theI2CDev = i2cDev;
    if (type == OAEXPANALOG_DEVICE_TYPE_1015) {
        conversionDelay = ADS1015_CONVERSIONDELAY;
    } else {
        conversionDelay = ADS1115_CONVERSIONDELAY;
    }
}

bool OAExpAnalog::begin() {
    unsigned int val;
    I2C_Result r = theI2CDev->read16(ADS1X15_REG_POINTER_CONFIG, val);

    if (r != I2C_OK) {
        return false;
    }
    
    return true;
}

bool OAExpAnalog::read(float & val, unsigned char channel, OAExpAnalog_Range range) {
    val = 0;
    if (channel > 3) {
        return false;
    }

    // Set single-ended input channel
    uint16_t channelInf;
    switch (channel) {
        case 0:
            channelInf = ADS1X15_REG_CONFIG_MUX_SINGLE_0;
            break;
        case 1:
            channelInf = ADS1X15_REG_CONFIG_MUX_SINGLE_1;
            break;
        case 2:
            channelInf = ADS1X15_REG_CONFIG_MUX_SINGLE_2;
            break;
        case 3:
            channelInf = ADS1X15_REG_CONFIG_MUX_SINGLE_3;
            break;
    }

    return doRead(val, channelInf, range);
}

bool OAExpAnalog::readDifferential(float & val, OAExpAnalog_DifferentialChanel difChan, OAExpAnalog_Range range) {
    val = 0;
    // Set channels
    uint16_t channelInf;
    
    if (difChan == OAEXPANALOG_DIFFERENTIAL_CHANNEL_0_1) {
        channelInf = ADS1X15_REG_CONFIG_MUX_DIFF_0_1; // AIN0 = P, AIN1 = N
    } else {
        channelInf = ADS1X15_REG_CONFIG_MUX_DIFF_2_3; // AIN2 = P, AIN3 = N
    }

    return doRead(val, channelInf, range);
}

bool OAExpAnalog::doRead(float & val, uint16_t channelInf, OAExpAnalog_Range range) {
    I2C_Result r;
    
    val = 0;
    
    // Start with default values
    uint16_t config = ADS1X15_REG_CONFIG_CQUE_NONE | // Disable the comparator (default val)
            ADS1X15_REG_CONFIG_CLAT_NONLAT | // Non-latching (default val)
            ADS1X15_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
            ADS1X15_REG_CONFIG_CMODE_TRAD | // Traditional comparator (default val)
            ADS1X15_REG_CONFIG_DR_1600SPS | // 1600 samples per second (default)
            ADS1X15_REG_CONFIG_MODE_SINGLE; // Single-shot mode (default)

    // Set PGA/voltage range
    uint16_t gain;
    switch (range) {
        case OAEXPANALOG_RANGE_6_144V:
            gain = ADS1X15_REG_CONFIG_PGA_6_144V;
            break;

        case OAEXPANALOG_RANGE_4_096V:
            gain = ADS1X15_REG_CONFIG_PGA_4_096V;
            break;

        case OAEXPANALOG_RANGE_2_048V:
            gain = ADS1X15_REG_CONFIG_PGA_2_048V;
            break;

        case OAEXPANALOG_RANGE_1_024V:
            gain = ADS1X15_REG_CONFIG_PGA_1_024V;
            break;

        case OAEXPANALOG_RANGE_0_512V:
            gain = ADS1X15_REG_CONFIG_PGA_0_512V;
            break;

        case OAEXPANALOG_RANGE_0_256V:
            gain = ADS1X15_REG_CONFIG_PGA_0_256V;
            break;
    }
    config |= gain;

    // Set channels
    config |= channelInf;

    // Set 'start single-conversion' bit
    config |= ADS1X15_REG_CONFIG_OS_SINGLE;

    // Write config register to the ADC
    config = swap16(config);
    r = theI2CDev->write16(ADS1X15_REG_POINTER_CONFIG, config);
    if (r != I2C_OK) {
        return false;
    }

    // Wait for the conversion to complete
    usleep(conversionDelay * 1000L);
    
    // Read the conversion results
    unsigned int uval;
    r = theI2CDev->read16(ADS1X15_REG_POINTER_CONVERT, uval);
    if (r != I2C_OK) {
        return false;
    }

    uval = swap16(uval);
    int16_t ival = (int16_t) (uval & 0xffff);

    // Scale to float value according to gain
    float fval = 0.0f;
    switch (range) {
        case OAEXPANALOG_RANGE_6_144V:
            fval = (1.0f * ival) * 0.1875f;
            break;

        case OAEXPANALOG_RANGE_4_096V:
            fval = (1.0f * ival) * 0.125f;
            break;

        case OAEXPANALOG_RANGE_2_048V:
            fval = (1.0f * ival) * 0.0625f;
            break;

        case OAEXPANALOG_RANGE_1_024V:
            fval = (1.0f * ival) * 0.03125f;
            break;

        case OAEXPANALOG_RANGE_0_512V:
            fval = (1.0f * ival) * 0.015625f;
            break;

        case OAEXPANALOG_RANGE_0_256V:
            fval = (1.0f * ival) * 0.0078125f;
            break;
    }

    val = fval / 1000.0f;

    return true;
}
