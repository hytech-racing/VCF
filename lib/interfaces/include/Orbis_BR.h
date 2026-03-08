#ifndef ORBIS_BR_H
#define ORBIS_BR_H

/* Library Includes */
#include <Arduino.h>
#include <etl/singleton.h>
#include "SteeringEncoderInterface.h"

/* --- Constants --- */
namespace OrbisConstants {
    const uint32_t ORBIS_BR_DEFAULT_BAUD_RATE                = 115200;
    const uint8_t REQUIRED_RESET_DELAY                       = 200;
    const uint8_t REQUIRED_SAVE_CONFIGURATION_DELAY          = 200;

    const uint8_t SELF_CALIBRATION_STATUS_MASK               = 0b00000011;
    const uint16_t SELF_CALIBRATION_MAX_TIME                 = 10000;
    const uint8_t SELF_CALIBRATION_NEW_COUNTER_MASK          = 0b00000011;
    const uint8_t SELF_CALIBRATION_TIMEOUT_ERROR_MASK        = 0b00000100;
    const uint8_t SELF_CALIRATION_PARAMETER_ERROR_MASK       = 0b00001000;

    const uint8_t OFFSET_SHORT_POSITION_RECOMBINING_SHIFT    = 8;
    const uint8_t OFFSET_SHORT_POSITION_RECOMBINING_MASK     = 0xFF;

    const uint8_t POSITION_DATA_MASK_1                       = 8;
    const uint8_t POSITION_DATA_MASK_2                       = 2;

    const float ANGLE_CONVERSION_MARKER                      = 180.0f;
    const float ANGLE_CONVERSION                             = 360.0f;
    const float ENCODER_RESOLUTION                           = 16384.0f;
    const float DEGREES_PER_REVOLUTION                       = 360.0f;
}



/* --- Error Definitions --- */
// Based on encoder packet structure (page 16 of datasheet)
// General errors are included in the first byte of the detailed position request response. While the detailed errors are from the fourth byte.
namespace OrbisErrorMasks {
    const uint16_t ORBIS_BR_BITMASK_GENERAL_WARNING           = 0b00000001;     // Error if low, position data is valid, but some operating conditions are close to limits
    const uint16_t ORBIS_BR_BITMASK_GENERAL_ERROR             = 0b00000010;     // Error if low, position data is not valid

    const uint16_t ORBIS_BR_BITMASK_DETAILED_COUNTER_ERROR    = 0b00001000;     // Errors if high
    const uint16_t ORBIS_BR_BITMASK_DETAILED_SPEED_HIGH       = 0b00010000;     // Errors if high
    const uint16_t ORBIS_BR_BITMASK_DETAILED_TEMP_RANGE       = 0b00100000;     // Errors if high
    const uint16_t ORBIS_BR_BITMASK_DETAILED_DIST_FAR         = 0b01000000;     // Errors if high
    const uint16_t ORBIS_BR_BITMASK_DETAILED_DIST_NEAR        = 0b10000000;     // Errors if high
}

/* --- Commands --- */
namespace OrbisCommands {
    const byte UNLOCK_SEQUENCE[4]           = {0xCD, 0xEF, 0x89, 0xAB};
    const byte SELF_CALIB_START             = 0x41; //      requires unlock sequence
    const byte SELF_CALIB_STATUS            = 0x69;
    const byte POSITION_OFFSET              = 0x5A; // 'Z'  requires unlock sequence
    const byte SAVE_CONFIGURATION           = 0x63; // 'c'  requires unlock sequence
    const byte FACTORY_RESET                = 0x72; // 'r'  requires unlock sequence
    const byte SHORT_POS_REQUEST            = 0x33;
    const byte DETAILED_POS_REQUEST         = 0x64;
    const byte MULTITURN_COUNTER_SETTING    = 0x4D; // 'M'  requires unlock sequence
    const byte CONTINUOUS_RESPONSE_SETTING  = 0x54; // 'T'  requires unlock sequence
    const byte CONTINUOUS_RESPONSE_START    = 0x53; // 'S'  requires unlock sequence
    const byte CONTINUOUS_RESPONSE_STOP     = 0x50; // 'P'  requires unlock sequence
    const byte BAUD_RATE_SETTING            = 0x42; // 'B'  requires unlock sequence
}

struct OrbisErrorFlags_s
{
    bool calibrationTimeout       = false;  // Ring did not make complete during 10 seconds
    bool calibrationParameter     = false;  // Mechanical installation outside tolerance
    bool counterError             = false;  // Multiturn counter error (bit=1 means error)
    bool speedHigh                = false;  // Speed too high (bit=1 means error)
    bool tempRange                = false;  // Temperature out of range (bit=1 means error)
    bool distFar                  = false;  // Dist b/w readhead and ring too far (bit=1 means error)
    bool distNear                 = false;  // Dist b/w readhead and ring too close (bit=1 means error)
};

class OrbisBR : public SteeringEncoderInterface
{
public:
// Constructors
    OrbisBR(HardwareSerial* serial);
// Functions
    bool performSelfCalibration();
    void setEncoderOffset();
    void saveConfiguration();

    void sample() override;
    SteeringEncoderConversion_s convert() override; // Override a virtual function from the base class. Will give compile error if signatures don't match.

    void factoryReset();

    OrbisErrorFlags_s getOrbisErrors() const { return _orbisErrors; }

private:
// Data

    void _decodeErrors(uint8_t general, uint8_t detailed);

    HardwareSerial* _serial;
    SteeringEncoderConversion_s _lastConversion; // initialize variable _lastConversion of type SteeringEncoderConversion_s.
    OrbisErrorFlags_s _orbisErrors;

    bool _isCalibrated = false;
    bool _isOffsetSet = false;
};

using OrbisBRInstance = etl::singleton<OrbisBR>;

#endif /* ORBIS_BR_H */