#ifndef ORBIS_BR_H
#define ORBIS_BR_H

/* Library Includes */
#include <Arduino.h>
#include <etl/singleton.h>
#include "SteeringEncoderInterface.h"

/* --- Constants --- */
namespace OrbisConstants {
    const uint8_t TIMEOUT                              = 10; // ms

    const uint32_t ORBIS_BR_DEFAULT_BAUD_RATE          = 115200;
    const uint8_t FACTORY_RESET_DELAY_MS               = 200;
    const uint8_t SAVE_CONFIG_DELAY_MS                 = 200;

    const uint8_t SELF_CALIB_STATUS_MASK               = 0b00000011;
    const uint16_t SELF_CALIB_MAX_TIME_MS              = 10000;
    const uint8_t SELF_CALIB_NEW_COUNTER_MASK          = 0b00000011;
    const uint8_t SELF_CALIB_TIMEOUT_ERROR_MASK        = 0b00000100;
    const uint8_t SELF_CALIB_PARAMETER_ERROR_MASK      = 0b00001000;

    const uint8_t OFFSET_HIGH_BYTE_SHIFT               = 8;
    const uint8_t OFFSET_RECOMBINING_MASK              = 0xFF;

    const uint8_t POSITION_DATA_HIGH_BYTE_SHIFT        = 8;
    const uint8_t POSITION_DATA_RIGHT_SHIFT            = 2;

    const float ANGLE_WRAPAROUND_THRESHOLD             = 180.0f;
    const float FULL_ROTATION_DEGREES                  = 360.0f;
    const float ENCODER_RESOLUTION                     = 16384.0f;      // 14-bit resolution = 2^14 counts per revolution
    const float DEGREES_PER_REVOLUTION                 = 360.0f;
}



/* --- Error Definitions --- */
// General errors are included in the first byte of the detailed position request response. Detailed errors are from the fourth byte.
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
    const byte UNLOCK_SEQUENCE[4]               = {0xCD, 0xEF, 0x89, 0xAB};
    const byte SELF_CALIB_START                 = 0x41; //      requires unlock sequence
    const byte SELF_CALIB_STATUS                = 0x69;
    const byte POSITION_OFFSET                  = 0x5A; // 'Z'  requires unlock sequence
    const byte SAVE_CONFIG                      = 0x63; // 'c'  requires unlock sequence
    const byte FACTORY_RESET                    = 0x72; // 'r'  requires unlock sequence
    const byte SHORT_POS_REQUEST                = 0x33;
    const byte DETAILED_POS_REQUEST             = 0x64;
    const byte MULTITURN_COUNTER_SETTING        = 0x4D; // 'M'  requires unlock sequence
    const byte CONTINUOUS_RESPONSE_SETTING      = 0x54; // 'T'  requires unlock sequence
    const byte CONTINUOUS_RESPONSE_START        = 0x53; // 'S'  requires unlock sequence
    const byte CONTINUOUS_RESPONSE_STOP         = 0x50; // 'P'  requires unlock sequence
    const byte BAUD_RATE_SETTING                = 0x42; // 'B'  requires unlock sequence
}

struct OrbisErrorFlags_s
{
    bool calibration_timeout       = false;  // Ring did not make complete during 10 seconds
    bool calibration_parameter     = false;  // Mechanical installation outside tolerance
    bool counter_error             = false;  // Multiturn counter error (bit=1 means error)
    bool speed_high                = false;  // Speed too high (bit=1 means error)
    bool temp_out_of_range         = false;  // Temperature out of range (bit=1 means error)
    bool dist_far                  = false;  // Dist b/w readhead and ring too far (bit=1 means error)
    bool dist_near                 = false;  // Dist b/w readhead and ring too close (bit=1 means error)
};

class OrbisBR : public SteeringEncoderInterface
{
public:
// Constructors
    OrbisBR(HardwareSerial* serial);

//Fields
    SteeringEncoderReading_s getLastReading() override; // Override a virtual function from the base class.
    OrbisErrorFlags_s getOrbisDetailedErrors() const { return _orbisErrors; }

// Functions
    bool performSelfCalibration();
    void setEncoderOffset();
    void saveConfiguration();
    void sample() override;
    void factoryReset();



private:
// Fields
    HardwareSerial* _serial;
    SteeringEncoderReading_s _lastReading; // Most recently sampled encoder reading.
    OrbisErrorFlags_s _orbisErrors;
// Functions
    void _decodeErrors(uint8_t general, uint8_t detailed);
    void _sendUnlockSequence();
    void _flushSerialBuffer();
};

using OrbisBRInstance = etl::singleton<OrbisBR>;

#endif /* ORBIS_BR_H */