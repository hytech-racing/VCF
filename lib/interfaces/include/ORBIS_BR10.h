#ifndef ORBIS_BR10_H
#define ORBIS_BR10_H

/* Library Includes */
#include <Arduino.h>
#include "SteeringEncoderInterface.h"
#include <ORBIS_BR10.h>
#include <etl/singleton.h>


/* --- Basic Commands From Datasheet --- */ 
//  Position offset setting: 'Z' (0x5A)
//  Multiturn counter setting: 'M' (0x4D)
//  Baud rate setting: 'B' (0x42)
//  Continuous-response setting: 'T' (0x54)
//  Continuous-response start: 'S' (0x53)
//  Continuous-response stop: 'P' (0x50)        
//  Configuration parameters save: 'c' (0x63)
//  Configuration parameters reset: 'r' (0x72)


/* --- Basic Definitions --- */ 
namespace OrbisDefaultParams {
    const int ORBIS_BR_DEFAULT_BAUD_RATE    = 115200;
    const int POS_DATA_MASK1                = 8;
    const int POS_DATA_MASK2                = 2;
    const float ENCODER_RESOLUTION          = 16384.0f;
    const float DEGREES_PER_REVOLUTION      = 360.0f;
}

/* --- Error Definitions --- */ 
// Based on encoder packet structure (page 16 of datasheet)
// General errors are included in the first byte of the detailed position request response. While the detailed errors are from the fourth byte.
namespace OrbisGeneralErrorMasks {
    const uint16_t ORBIS_BR_BITMASK_GENERAL_WARNING           = (0b1 << 0);     // 0b00000001, error if low, position data is valid, but some operating conditions are close to limits
    const uint16_t ORBIS_BR_BITMASK_GENERAL_ERROR             = (0b1 << 1);     // 0b00000010, error if low, position data is not valid 
}
namespace OrbisDetailedErrorMasks {
    const uint16_t ORBIS_BR_BITMASK_DETAILED_COUNTER_ERROR    = (0b1 << 3);     // 0b00001000, errors if high
    const uint16_t ORBIS_BR_BITMASK_DETAILED_SPEED_HIGH       = (0b1 << 4);     // 0b00010000, errors if high
    const uint16_t ORBIS_BR_BITMASK_DETAILED_TEMP_RANGE       = (0b1 << 5);     // 0b00100000, errors if high
    const uint16_t ORBIS_BR_BITMASK_DETAILED_DIST_FAR         = (0b1 << 6);     // 0b01000000, errors if high
    const uint16_t ORBIS_BR_BITMASK_DETAILED_DIST_NEAR        = (0b1 << 7);     // 0b10000000, errors if high
}


/* --- Commands --- */ 
namespace OrbisCommands {
    const byte UNLOCK_SEQUENCE[4]           = {0xCD, 0xEF, 0x89, 0xAB}; 
    const byte FACTORY_RESET                = 0x72;
    const byte SELF_CALIB_START             = 0x41; // requires unlock sequence
    const byte SELF_CALIB_STATUS            = 0x69;
    const byte POSITION_OFFSET              = 0x5A; // requires unlock sequence
    const byte SAVE_CONFIGURATION           = 0x63; // requires unlock sequence
    const byte SHORT_POS_REQUEST            = 0x33;
    const byte DETAILED_POS_REQUEST         = 0x64;
}

class OrbisBR10 : public SteeringEncoderInterface
{
public:
// Constructors
    OrbisBR10(HardwareSerial* serial);
// Functions
    void sample();    
    void init() {}
    SteeringEncoderConversion_s position();
    bool performSelfCalibration();
    void setEncoderOffset();
    void saveConfiguration();
    void factoryReset();

private:
// Data

    SteeringEncoderConversion_s convert() {}

    void decodeErrors(uint8_t general, uint8_t detailed);

    HardwareSerial* _serial;
    int _serialSpeed;

    SteeringEncoderConversion_s _lastConversion;

    bool _isCalibrated = false;
    bool _isOffsetSet = false;
};

using OrbisBRInstance = etl::singleton<OrbisBR10>;

#endif /* ORBIS_BR10_H */