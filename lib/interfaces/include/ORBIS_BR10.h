#ifndef ORBIS_BR10_H
#define ORBIS_BR10_H

#include <Arduino.h>
#include "SteeringEncoderInterface.h"
#include <ORBIS_BR10.h>
#include <etl/singleton.h>


// Basic Commands
//  Position offset setting: 'Z' (0x5A)
//  Multiturn counter setting: 'M' (0x4D)
//  Baud rate setting: 'B' (0x42)
//  Continuous-response setting: 'T' (0x54)
//  Continuous-response start: 'S' (0x53)
//  Continuous-response stop: 'P' (0x50)        
//  Configuration parameters save: 'c' (0x63)
//  Configuration parameters reset: 'r' (0x72)

// Definitions
const int      ORBIS_BR_DEFAULT_BAUD_RATE                 = 115200;

// Error Definitions
const uint16_t ORBIS_BR_BITMASK_GENERAL_WARNING           = (0b1 << 0);     // 0b00000001, error if low, position data is valid, but some operating conditions are close to limits
const uint16_t ORBIS_BR_BITMASK_GENERAL_ERROR             = (0b1 << 1);     // 0b00000010, error if low, position data is not valid 

const uint16_t ORBIS_BR_BITMASK_DETAILED_COUNTER_ERROR    = (0b1 << 3);     // 0b00001000, errors if high
const uint16_t ORBIS_BR_BITMASK_DETAILED_SPEED_HIGH       = (0b1 << 4);     // 0b00010000, errors if high
const uint16_t ORBIS_BR_BITMASK_DETAILED_TEMP_RANGE       = (0b1 << 5);     // 0b00100000, errors if high
const uint16_t ORBIS_BR_BITMASK_DETAILED_DIST_FAR         = (0b1 << 6);     // 0b01000000, errors if high
const uint16_t ORBIS_BR_BITMASK_DETAILED_DIST_NEAR        = (0b1 << 7);     // 0b10000000, errors if high

const byte FACTORY_RESET                = 0x72;

const byte ENCODER_OFFSET               = 0x00;
// const byte CONTINUOUS_RESPONSE[5]       = {0x54, 0x01, 0x64, 0x00, 0xFA};  // autostart
const byte CONTINUOUS_RESPONSE[5]       = {0x54, 0x00, 0x64, 0x00, 0xFA};  // no autostart
const byte CONTINUOUS_RESPONSE_START    = 0x53;
const byte UNLOCK_SEQUENCE[4]           = {0xCD, 0xEF, 0x89, 0xAB}; 
const byte SELF_CALIB_START             = 0x41;
const byte SELF_CALIB_STATUS            = 0x69;
const byte START_OFFSET                 = 0x5A;
const byte SAVE_CONFIG                  = 0x63;
const byte DETAILED_POS_REQUEST         = 0x64;
const byte DECODE_GEN_ERRORS            = 0x03;
const int POS_DATA_MASK1                = 8;
const int POS_DATA_MASK2                = 2;
const float BIT_ANGLE_CONVERSION        = 8192.0f;
const float CLIP_PREVENT                = 180.0f;

class OrbisBR10 : public SteeringEncoderInterface
{
public:
// Constructors
    OrbisBR10(HardwareSerial* serial, int serialSpeed);
// Functions
    void sample();    
    void init();
    SteeringEncoderConversion_s position();
    void setOffset(float newOffset);

private:
// Data

    SteeringEncoderConversion_s convert() {}

    bool performSelfCalibration();
    void setEncoderOffset(uint16_t offsetCounts);
    void saveConfiguration();
    void decodeErrors(uint8_t general, uint8_t detailed);

    HardwareSerial* _serial;
    int _serialSpeed;

    uint16_t _position_data;
    SteeringEncoderConversion_s _lastConversion;
    float _angleOffset = 0.0f;

    bool _isCalibrated = false;
    bool _isOffsetSet = false;
};

using OrbisBRInstance = etl::singleton<OrbisBR10>;

#endif /* ORBIS_BR10_H */