#include "ORBIS_BR10.h"
#include <Arduino.h>

OrbisBR10::OrbisBR10(HardwareSerial* serial, int serialSpeed)
: _serial(serial)
, _serialSpeed(serialSpeed)
{
   _lastConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_ERROR;
   _serial->begin(_serialSpeed, SERIAL_8N1);
}

void OrbisBR10::init() // Function housing all initialization (calibration and configuration)
{
    
    // Serial.println("Cont. Resp. Stop...");
    //_serial->write('P');

    Serial.println("HELLO CHUD");
    bool _isCalibrated = false;   // Assume sensor not self-calibrated
    while (!_isCalibrated)
    {
        _isCalibrated = performSelfCalibration();
    }
       
    // setEncoderOffset(ENCODER_OFFSET);
    // delay(10);

    //saveConfiguration();
    // delay(10);
}

// Self-Calibration Function 
bool OrbisBR10::performSelfCalibration()
{
    Serial.println("Starting self-calibration..."); // Debug line

    // // Unlock Encoder Sequence
    // for (byte command : UNLOCK_SEQUENCE)
    // {
    //     _serial->write(command); delay(1);
    // } 

    delay(100);

    _serial->write(SELF_CALIB_STATUS);       
    // self-calibration status request, datasheet says to do status before self-calib start
    // 0x69 command (status) returns 2 bytes: First is the echo byte and the next is the status byte
    
    uint8_t currentCounter = 0;
    // if (_serial->available())          
    // {

    while (!_serial->available());

        Serial.printf("Available Bytes: %d\n", _serial->available());
        uint8_t echo1 = _serial->read();     // echo byte
        Serial.printf("Available Bytes: %d\n", _serial->available());
        uint8_t status1 = _serial->read();   // status byte

        Serial.print("Echo 1: 0x");
        Serial.println(echo1, HEX);
        Serial.print("Status byte 1: 0x");
        Serial.println(status1, HEX);

        currentCounter = status1 & 0b00000011;  // counter bits are b1 and b0 of status byte
        Serial.print("Current Counter: ");
        Serial.println(currentCounter);
    // }
    
    delay(50);

    _serial->write(SELF_CALIB_START);
    // unsigned long myTime = millis(); 

    delay(10000); // max 10 seconds for self-calibration


    // Handling self-calib status info funtionality
    _serial->write(SELF_CALIB_STATUS);      // self-calibration status request again after, expect counter to change
    delay(10);

    if (_serial->available() < 2) {
        Serial.println("ERROR: No response from sensor");
        _lastConversion.errors.noData = true;
        return false;
    }

    Serial.printf("Available Bytes: %d\n", _serial->available());
    uint8_t echo2 = _serial->read();     // echo byte
    Serial.printf("Available Bytes: %d\n", _serial->available());
    uint8_t status2 = _serial->read();   // status byte

    Serial.print("Echo 2: 0x");
    Serial.println(echo2, HEX);
    Serial.print("Status byte 2: 0x");
    Serial.println(status2, HEX);
    
    uint8_t newCounter = status2 & 0b00000011;
    bool timeoutError = status2 & 0b00000100;      // b2
    bool parameterError = status2 & 0b00001000;    // b3
    
    Serial.print("New counter: ");
    Serial.println(newCounter);
    Serial.print("Timeout error: ");
    Serial.println(timeoutError);
    Serial.print("Parameter error: ");
    Serial.println(parameterError);

    // if (((currentCounter + 1) & 0b00000011) != newCounter) {
    //     Serial.println("ERROR: Calibration did not complete");
    //     //return false;
    // }
    
    // if (timeoutError || parameterError)
    // {
    //     Serial.println("ERROR: Calibration failed");
    //     _lastConversion.errors.calibrationTimeout   = timeoutError;
    //     _lastConversion.errors.calibrationParameter = parameterError;
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
    _serial->write(DETAILED_POS_REQUEST); delay(1);   // position request + detailed status: 1 byte echo, 2 byte position, 1 byte detailed status
    
    // if (_serial->available() < 4)     // check if received all 4 bytes
    // { 
    //     _lastConversion.errors.noData = true;
    //     _lastConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_ERROR;
    //     return;
    // }   

    Serial.println("Sent position request on liner.");

    // Read reponse bytes
    Serial.printf("Available Bits: %d\n", _serial->available());
    uint8_t echo     = _serial->read();
    Serial.printf("Available Bytes: %d\n", _serial->available());
    uint8_t general1 = _serial->read();
    Serial.printf("Available Bytes: %d\n", _serial->available());
    uint8_t general2 = _serial->read();
    Serial.printf("Available Bytes: %d\n", _serial->available());
    uint8_t detailed = _serial->read();

    Serial.println(echo, HEX);
    Serial.println(general1, HEX);
    Serial.println(general2, HEX);
    Serial.println(detailed, HEX);

    // Serial.println(_serial->read(), HEX);
    // Serial.println(_serial->read(), HEX);
    // Serial.println(_serial->read(), HEX);
    

    if (echo != DETAILED_POS_REQUEST)
    {
        _lastConversion.errors.noData = true;
        _lastConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_ERROR;
        return;
    }
    
    // Decode errors, general status bytes
    uint16_t general = general1 & DECODE_GEN_ERRORS; // fix this by casting
    decodeErrors(general, detailed); 

    // Extract 14-bit position data
    uint16_t raw_position_data = (((uint16_t) general1) << POS_DATA_MASK1) | (uint16_t) general2;
    _position_data = raw_position_data >> POS_DATA_MASK2;
    
    // Convert position data to angle
    float angle = ((float)_position_data - BIT_ANGLE_CONVERSION) / BIT_ANGLE_CONVERSION * CLIP_PREVENT;
    angle += _angleOffset;  // Apply software offset
    _lastConversion.raw = _position_data;
    _lastConversion.angle = angle;
    
    Serial.print("Raw ");
    Serial.println(_lastConversion.raw);
    Serial.print("Angle ");
    Serial.println(_lastConversion.angle);

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