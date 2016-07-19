#ifndef OAExpAnalog_H
#define OAExpAnalog_H

#include "I2CDevice.h"

typedef enum
{
    OAEXPANALOG_DEVICE_TYPE_1015,
    OAEXPANALOG_DEVICE_TYPE_1115
} OAExpAnalog_DeviceType;

typedef enum
{
    OAEXPANALOG_DIFFERENTIAL_CHANNEL_0_1,
    OAEXPANALOG_DIFFERENTIAL_CHANNEL_2_3
} OAExpAnalog_DifferentialChanel;

typedef enum
{
    OAEXPANALOG_RANGE_6_144V,   // Max of 6.144 volts
    OAEXPANALOG_RANGE_4_096V,   // Max of 4.096 volts
    OAEXPANALOG_RANGE_2_048V,   // Max of 2.048 volts
    OAEXPANALOG_RANGE_1_024V,   // Max of 1.024 volts
    OAEXPANALOG_RANGE_0_512V,   // Max of 0.512 volts
    OAEXPANALOG_RANGE_0_256V    // Max of 0.256 volts
} OAExpAnalog_Range;

#define DEFAULT_OAEXPANALOG_DEV_ADDR    0x48

typedef unsigned char byte;

class OAExpAnalog {
public:
    OAExpAnalog(OAExpAnalog_DeviceType type = OAEXPANALOG_DEVICE_TYPE_1015);
    OAExpAnalog(byte devAddr, OAExpAnalog_DeviceType type = OAEXPANALOG_DEVICE_TYPE_1015);
    OAExpAnalog(I2CDevice * i2cDev, OAExpAnalog_DeviceType type = OAEXPANALOG_DEVICE_TYPE_1015);

    bool begin();
    
    bool read(float & val, unsigned char channel, OAExpAnalog_Range range = OAEXPANALOG_RANGE_6_144V);
    bool readDifferential(float & val, OAExpAnalog_DifferentialChanel difChan, OAExpAnalog_Range range = OAEXPANALOG_RANGE_6_144V);

private:
    I2CDevice * theI2CDev;
    int conversionDelay;
    

    void setup(I2CDevice * i2cDev, OAExpAnalog_DeviceType type);

    bool doRead(float & val, uint16_t channelInf, OAExpAnalog_Range range);
};

#endif

