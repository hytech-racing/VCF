#include "ORBIS_BR10.h"
#include <Arduino.h>

OrbisBR10::OrbisBR10(HardwareSerial* serial, int serialSpeed)
: _serial(serial)
, _serialSpeed(serialSpeed)
{
   _lastConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_ERROR;
}

void OrbisBR10::init() // all initialization (calibration and configuration)
{
    _serial->begin(_serialSpeed);

    // for (byte command : UNLOCK_SEQUENCE)
    // {
    //     Serial3.write(command);
    // } // may need delay(1)
    
    // for (byte command : CONTINUOUS_RESPONSE)
    // {
    //     Serial3.write(command);
    // } // may need delay(1)

    // delay(10);

    // for (byte command : UNLOCK_SEQUENCE)
    // {
    //     _serial->write(command);
    //     delay(1);
    // } // may need delay(1)

    // _serial->write('P');

    // delay(10);

    // for (byte command : UNLOCK_SEQUENCE)
    // {
    //     Serial3.write(command);
    // } // may need delay(1)

    // Serial3.write(SAVE_CONFIG);

    // for (byte command : UNLOCK_SEQUENCE)
    // {
    //     Serial3.write(command);
    // } // may need delay(1)

    // Serial3.write('r');

    // bool _isCalibrated = false;
    // while (!_isCalibrated)
    // {
    //     _isCalibrated = performSelfCalibration();
    // }
       
    // setEncoderOffset(ENCODER_OFFSET);
    // delay(10);

    //Continous-Response Setting ('T')
    // Using auto-start, and short position request, period = 1000 µs, should have delay(1)
    // _serial->write(0x54);           // 'T' command
    // _serial->write(0x01);           // Auto-start enabled after power-on
    // _serial->write(0x64);           // position request + detailed status
    // _serial->write(0x00);           // Period high byte (0x00)
    // _serial->write(0xFA);           // Period low byte (0x3E8 = 1000 µs)
    
    // for (byte command : UNLOCK_SEQUENCE)
    // {
    //     _serial->write(command);
    // } // may need delay(1)
    
    // for (byte command : CONTINUOUS_RESPONSE)
    // {
    //     _serial->write(command);
    // } // may need delay(1)

    // saveConfiguration();
    // delay(50);

    //Continous-Response Start ('S')
    //_serial->write(CONTINUOUS_RESPONSE_START); delay(1); 
    
    //encoder should auto-start on future power-ups
}

// Self-Calibration Function 
bool OrbisBR10::performSelfCalibration()
{
    Serial.println("Starting self-calibration...");

    // Unlock encoder sequence
    for (byte command : UNLOCK_SEQUENCE)
    {
        _serial->write(command);
    } // may need delay(1)


    uint8_t previousCounter = 0;
    if (_serial->available() >= 2)    // Calibration checker loop
    {
        uint8_t echo = _serial->read();     // echo byte
        uint8_t status = _serial->read();   // status byte

        previousCounter = status & 0b00000011;
        Serial.println("Previous counter: ");
        Serial.println(previousCounter);
    }

    for (byte command : UNLOCK_SEQUENCE)
    {
        _serial->write(command);
    } // may need delay(1)

    _serial->write(SELF_CALIB_START);
    unsigned long myTime = millis();

    delay(10000); // max 10 seconds for self-calibration

    _serial->write(SELF_CALIB_STATUS);      // self-calibration status request


    if (_serial->available() < 2) {
        Serial.println("ERROR: No response from encoder");
        _lastConversion.errors.noData = true;
        return false;
    }

    uint8_t echo = _serial->read();     // echo byte
    uint8_t status = _serial->read();

    Serial.print("Status byte: 0x");
    Serial.println(status, HEX);
    
    uint8_t newCounter = status & 0b00000011;
    bool timeoutError = status & 0b00000100;      // b2
    bool parameterError = status & 0b00001000;    // b3
    
    Serial.print("New counter: ");
    Serial.println(newCounter);
    Serial.print("Timeout error: ");
    Serial.println(timeoutError);
    Serial.print("Parameter error: ");
    Serial.println(parameterError);

    if (((previousCounter + 1) & 0b00000011) != newCounter) {
        Serial.println("ERROR: Calibration did not complete");
        //return false;
    }
    
    if (timeoutError || parameterError)
    {
        Serial.println("ERROR: Calibration failed");
        _lastConversion.errors.calibrationTimeout   = timeoutError;
        _lastConversion.errors.calibrationParameter = parameterError;
        return false;
    }
    // else
    // {
    //     return true;
    // }    
    // } 
    // else
    // {
    //     _lastConversion.errors.noData = true;  // no response
    //     return false;
    // }
    Serial.println("Calibration successful!");
    return true;
}

// Offset Function
void OrbisBR10::setEncoderOffset(uint16_t offsetCounts)
{
    for (byte b : UNLOCK_SEQUENCE)  // unlock sequence
    {
        _serial->write(b);
    }

    _serial->write(START_OFFSET);       // offset command
    _serial->write(offsetCounts);    // offset position, originally 0x00
    _serial->write(offsetCounts);
    _serial->write(offsetCounts); 
    _serial->write(offsetCounts);     
}

// Save Configuration in Non-Volatile Memory Function
void OrbisBR10::saveConfiguration()
{
    for (byte command : UNLOCK_SEQUENCE)
    {
        _serial->write(command);
    } // may need delay(1)

    _serial->write(SAVE_CONFIG);  // 'c' save to non-volatile
}


// check/flag for individual errors
void OrbisBR10::decodeErrors(uint8_t general, uint8_t detailed)
{
    // General bits error low (0)
    _lastConversion.errors.generalError     = !(general & ORBIS_BR_BITMASK_GENERAL_ERROR);
    _lastConversion.errors.generalWarning   = !(general & ORBIS_BR_BITMASK_GENERAL_WARNING);

    // Detailed bits are error high (1)
    _lastConversion.errors.counterError     = detailed & ORBIS_BR_BITMASK_DETAILED_COUNTER_ERROR;
    _lastConversion.errors.speedHigh        = detailed & ORBIS_BR_BITMASK_DETAILED_SPEED_HIGH;
    _lastConversion.errors.tempRange        = detailed & ORBIS_BR_BITMASK_DETAILED_TEMP_RANGE;
    _lastConversion.errors.distFar          = detailed & ORBIS_BR_BITMASK_DETAILED_DIST_FAR;
    _lastConversion.errors.distNear         = detailed & ORBIS_BR_BITMASK_DETAILED_DIST_NEAR;
}

// sample data function
void OrbisBR10::sample()
{
    for (byte command : UNLOCK_SEQUENCE)
    {
        _serial->write(command);
        delay(1);
    } // may need delay(1)

    _serial->write(DETAILED_POS_REQUEST); delay(5);   // position request + detailed status: 1 byte echo, 2 byte position, 1 byte detailed status
    
    // if (_serial->available() < 4)     // check if received all 4 bytes
    // { 
    //     _lastConversion.errors.noData = true;
    //     _lastConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_ERROR;
    //     return;
    // }   

    Serial.println("sent commands.");

    // Read reponse bytes
    uint8_t echo     = _serial->read();
    uint8_t general1 = _serial->read();
    uint8_t general2 = _serial->read();
    uint8_t detailed = _serial->read();

    Serial.println(echo, HEX);
    Serial.println(general1, HEX);
    Serial.println(general2, HEX);
    Serial.println(detailed, HEX);

    if (echo != DETAILED_POS_REQUEST)
    {
        _lastConversion.errors.noData = true;
        _lastConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_ERROR;
        return;
    }
    
    // Decode errors, general status bytes
    uint8_t general = general1 & DECODE_GEN_ERRORS;
    decodeErrors(general, detailed); 

    // Extract 14-bit position data
    uint16_t raw_position_data = ((uint16_t) general2 << POS_DATA_MASK1) | (uint16_t) general1;
    _position_data = raw_position_data >> POS_DATA_MASK2;
    
    // Convert position data to angle
    float angle = ((float)_position_data - BIT_ANGLE_CONVERSION) / BIT_ANGLE_CONVERSION * CLIP_PREVENT;
    angle += _angleOffset;  // Apply software offset
    _lastConversion.raw = _position_data;
    _lastConversion.angle = angle;
    

    // Decode errors, detailed status bytes
    bool anyError = 
     (
        _lastConversion.errors.generalError   ||
        _lastConversion.errors.counterError   ||
        _lastConversion.errors.speedHigh      ||
        _lastConversion.errors.tempRange      ||
        _lastConversion.errors.distFar        ||
        _lastConversion.errors.distNear       ||
        _lastConversion.errors.noData
     );
    
    _lastConversion.status = anyError
         ? SteeringEncoderStatus_e::STEERING_ENCODER_ERROR
         : SteeringEncoderStatus_e::STEERING_ENCODER_NOMINAL;
}

SteeringEncoderConversion_s OrbisBR10::position()
{
    return _lastConversion;
}

void OrbisBR10::setOffset(float newOffset)
{
    _angleOffset = newOffset;
}