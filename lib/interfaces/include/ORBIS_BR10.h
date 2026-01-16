#ifndef ORBIS_BR10_H
#define ORBIS_BR10_H

#include <Arduino.h>
#include "SteeringEncoderInterface.h"

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

#endif /* ORBIS_BR10_H */