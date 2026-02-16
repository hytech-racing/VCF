#ifndef ORBIS_BR_H
#define ORBIS_BR_H

/* Library Includes */
#include <Arduino.h>
#include <etl/singleton.h>
#include "SteeringEncoderInterface.h"

/* --- Constants --- */ 
#define ORBIS_BR_DEFAULT_BAUD_RATE                  115200
#define SELF_CALIBRATION_STATUS_MASK                0b00000011
#define SELF_CALIBRATION_MAX_TIME                   10000
#define SELF_CALIBRATION_NEW_COUNTER_MASK           0b00000011
#define SELF_CALIBRATION_TIMEOUT_ERROR_MASK         0b00000100
#define SELF_CALIRATION_PARAMETER_ERROR_MASK        0b00001000
#define SELF_CALIBRATION_COUNTER_CHECK_MASK         0b00000011
#define OFFSET_SHORT_POSITION_RECOMBINING_SHIFT     8
#define OFFSET_SHORT_POSITION_RECOMBINING_MASK      0xFF
#define ANGLE_CONVERSION_MARKER                     180.0f
#define ANGLE_CONVERSION                            360.0f
#define POSITION_DATA_MASK_1                        8
#define POSITION_DATA_MASK_2                        2
#define ENCODER_RESOLUTION                          16384.0f
#define DEGREES_PER_REVOLUTION                      360.0f


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

/* --- Basic Commands From Datasheet --- */ 
//  Position offset setting: 'Z' (0x5A)
//  Multiturn counter setting: 'M' (0x4D)
//  Baud rate setting: 'B' (0x42)
//  Continuous-response setting: 'T' (0x54)
//  Continuous-response start: 'S' (0x53)
//  Continuous-response stop: 'P' (0x50)        
//  Configuration parameters save: 'c' (0x63)
//  Configuration parameters reset: 'r' (0x72)


class OrbisBR : public SteeringEncoderInterface
{
public:
// Constructors
    OrbisBR(HardwareSerial* serial, int serialSpeed);
// Functions
    void sample();    
    void init() {} // not sure why I need this, but doesn't build w/o it currently
    
    SteeringEncoderConversion_s convert() override; // Override a virtual function from the base class. Will give compile error if signatures don't match. 
    

    bool performSelfCalibration();
    void setEncoderOffset();
    void saveConfiguration();
    void factoryReset();

private:
// Data

    void decodeErrors(uint8_t general, uint8_t detailed);

    HardwareSerial* _serial;
    int _serialSpeed;

    SteeringEncoderConversion_s _lastConversion; // initialize variable _lastConversion of type SteeringEncoderConversion_s. 

    bool _isCalibrated = false;
    bool _isOffsetSet = false;
};

using OrbisBRInstance = etl::singleton<OrbisBR>;

#endif /* ORBIS_BR_H */