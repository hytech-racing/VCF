/* Library Includes */
#include "ORBIS_BR10.h"
#include <Arduino.h>

OrbisBR10::OrbisBR10(HardwareSerial* serial, int serialSpeed)
: _serial(serial)
, _serialSpeed(serialSpeed)
{
   _lastConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_ERROR;
   _serial->begin(_serialSpeed, SERIAL_8N1);
}


void OrbisBR10::init() 
{
    bool _isCalibrated = false;    // Assume sensor not self-calibrated
    while (!_isCalibrated)
    {
        _isCalibrated = performSelfCalibration();
    }
       
    uint16_t offset_position = offsetPosition(); delay(100);
    
    setEncoderOffset(offset_position); delay(100);

    //saveConfiguration(); delay(100);
}



/* -------------------- Initialization Functions -------------------- */

bool OrbisBR10::performSelfCalibration()
{
    Serial.println("Starting self-calibration..."); // Debug line



    /* ----- SELF CALIBRATION STATUS 1 ----- */
    _serial->write(SELF_CALIB_STATUS); delay(1);      
    // Datasheet says to do status before self-calib start. 
    // Returns 2 bytes: Echo byte then status byte
    
    while (!_serial->available());       // Purpose: Wait until a byte is on the line to be read
    Serial.printf("Available Bytes: %d\n", _serial->available()); // Debug line
    uint8_t echo_1 = _serial->read();     // echo byte
    
    while (!_serial->available());
    Serial.printf("Available Bytes: %d\n", _serial->available()); // Debug line
    uint8_t status_1 = _serial->read();   // status byte

    Serial.print("Echo 1: 0x");         // Debug line
    Serial.println(echo_1, HEX);         // Debug line
    Serial.print("Status byte 1: 0x");  // Debug line
    Serial.println(status_1, HEX);       // Debug line

    uint8_t current_counter = 0;             
    current_counter = status_1 & 0b00000011;  // Counter bits are b1 and b0 of status byte
    Serial.print("Current Counter: ");      // Debug line
    Serial.println(current_counter);         // Debug line
    
    // do we want some error check here, like if counter not 0 or status not 0?
    delay(50);



    /* ----- SELF CALIBRATION START ----- */
    _serial->write(SELF_CALIB_START); delay(1);
    
    while (!_serial->available());    
    uint8_t calibration_start = _serial->read();  
    
    Serial.print("Calib Start Byte: 0x");         // Debug line
    Serial.println(calibration_start, HEX);       // Debug line

    delay(10000); // Max time for a self-calibration is 10 seconds. 
                                                        
    

    /* ----- SELF CALIBRATION STATUS 2 ----- */
    // self-calibration status request again after, expect counter to change
    _serial->write(SELF_CALIB_STATUS); delay(1);

    if (_serial->available() < 2) {
        Serial.println("ERROR: No Response From Sensor"); // Debug line
        _lastConversion.errors.noData = true;
        return false;
    }

    Serial.printf("Available Bytes: %d\n", _serial->available());      // Debug line
    uint8_t echo_2 = _serial->read();     // echo byte
    Serial.printf("Available Bytes: %d\n", _serial->available());      // Debug line
    uint8_t status_2 = _serial->read();   // status byte

    Serial.print("Echo 2: 0x");              // Debug line
    Serial.println(echo_2, HEX);             // Debug line
    Serial.print("Status byte 2: 0x");       // Debug line
    Serial.println(status_2, HEX);           // Debug line
    
    uint8_t new_counter = status_2 & 0b00000011;
    bool timeout_error = status_2 & 0b00000100;      // b2
    bool parameter_error = status_2 & 0b00001000;    // b3
    
    Serial.print("New counter: ");           // Debug line
    Serial.println(new_counter);             // Debug line
    Serial.print("Timeout error: ");         // Debug line
    Serial.println(timeout_error);           // Debug line
    Serial.print("Parameter error: ");       // Debug line
    Serial.println(parameter_error);         // Debug line

    // if (((currentCounter + 1) & 0b00000011) != newCounter) {
    //     Serial.println("ERROR: Calibration did not complete");
    //     //return false;
    // }
    
    if (timeout_error || parameter_error)
    {
        Serial.println("ERROR: Calibration failed");
        _lastConversion.errors.calibrationTimeout   = timeout_error;
        _lastConversion.errors.calibrationParameter = parameter_error;
        return false;
    }
    
    Serial.println("Calibration successful!");   // Debug line
    return true;
}


uint16_t OrbisBR10::offsetPosition()
{
    while (_serial->available()) {  // Clear everything off the line. Check if this is necessary?
        _serial->read();
    }
    
    _serial->write(SHORT_POS_REQUEST); delay(1);

    while (!_serial->available());
    uint8_t position_1 = _serial->read(); 
    while (!_serial->available());
    uint8_t position_2 = _serial->read();
    
    Serial.println("Position for Offset: ");     // Debug line
    Serial.println(position_1, HEX);             // Debug line
    Serial.println(position_2, HEX);             // Debug line
    Serial.println();                            // Debug ling

    uint16_t raw_position_data = (((uint16_t) position_1) << POS_DATA_MASK1) | (uint16_t) position_2;
    _position_data = raw_position_data >> POS_DATA_MASK2;

    return _position_data;
}


void OrbisBR10::setEncoderOffset(uint16_t _position_data)
{
    while (_serial->available()) {  // Clear everything off the line. Check if this is necessary?
        _serial->read();
    }
    
    for (byte b : UNLOCK_SEQUENCE) 
    {
        _serial->write(b); delay(1);
    }

    while (!_serial->available());              
    Serial.println(_serial->read(), HEX);
    while (!_serial->available());
    Serial.println(_serial->read(), HEX);
    while (!_serial->available());
    Serial.println(_serial->read(), HEX);
    while (!_serial->available());
    Serial.println(_serial->read(), HEX);


    byte position1 = (_position_data >> 8) & 0xFF;      // b15 - b8 (See Encoder Packet Structure page 16)
    byte position2 = _position_data & 0xFF;             // b7 - b0

    _serial->write(POSITION_OFFSET); delay(1);      
    _serial->write((byte) 0x00); delay(1);      
    _serial->write((byte) 0x00); delay(1);
    _serial->write(position1); delay(1);
    _serial->write(position2); delay(1);
    delay(50);
    // After writing POSITION_OFFSET, the next four bytes are what you subtract from your initial position. We want to start at 0, so we first
    // get our initial position, and then send that position as the four bytes. Inital position - inital position = 0. 

    
    while (!_serial->available());
    Serial.println(_serial->read(), HEX);
    while (!_serial->available());
    Serial.println(_serial->read(), HEX);
    while (!_serial->available());
    Serial.println(_serial->read(), HEX);
    while (!_serial->available());
    Serial.println(_serial->read(), HEX);
    while (!_serial->available());
    Serial.println(_serial->read(), HEX);

    Serial.println("Offset command done"); // Debug line
}


void OrbisBR10::saveConfiguration()
{
    for (byte command : UNLOCK_SEQUENCE)
    {
        _serial->write(command); delay(1);
    } 

    _serial->write(SAVE_CONFIGURATION); delay(1);  

    delay(100);

    while (!_serial->available());              // This definitely needs to be cleaner, purpose is to read everything of the line. 
    Serial.println(_serial->read(), HEX);
    while (!_serial->available());
    Serial.println(_serial->read(), HEX);
    while (!_serial->available());
    Serial.println(_serial->read(), HEX);
    while (!_serial->available());
    Serial.println(_serial->read(), HEX);
    while (!_serial->available());
    Serial.println(_serial->read(), HEX);
}



/* -------------------- Error Flagging -------------------- */
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



void OrbisBR10::sample()
{
    _serial->write(DETAILED_POS_REQUEST); delay(1);   
    // Detailed Position Request:  1 byte echo, 2 byte position, 1 byte detailed status
    
    // if (_serial->available() < 4)     // check if received all 4 bytes
    // { 
    //     _lastConversion.errors.noData = true;
    //     _lastConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_ERROR;
    //     return;
    // }   

    Serial.println("Sent position request on liner.");  // Debug line

    Serial.printf("Available Bytes: %d\n", _serial->available());   // Debug line
    uint8_t echo     = _serial->read();
    Serial.printf("Available Bytes: %d\n", _serial->available());   // Debug line
    uint8_t general1 = _serial->read();
    Serial.printf("Available Bytes: %d\n", _serial->available());   // Debug line
    uint8_t general2 = _serial->read();
    Serial.printf("Available Bytes: %d\n", _serial->available());  // Debug line
    uint8_t detailed = _serial->read();

    Serial.println(echo, HEX);          // Debug line
    Serial.println(general1, HEX);      // Debug line
    Serial.println(general2, HEX);      // Debug line
    Serial.println(detailed, HEX);      // Debug line

    if (echo != DETAILED_POS_REQUEST)
    {
        _lastConversion.errors.noData = true;
        _lastConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_ERROR;
        return;
    }



    // Extract 14-bit position data
    uint16_t raw_position_data = (((uint16_t) general1) << POS_DATA_MASK1) | (uint16_t) general2;

    // Extract general status bits from the raw data
    uint8_t general_status = general2 & 0x03;  // b1:b0 are general status

    _position_data = raw_position_data >> POS_DATA_MASK2;

    // Decode errors using the extracted general status bits
    decodeErrors(general_status, detailed);


    
    // Convert position data to angle
    float angle = ((float)_position_data - BIT_ANGLE_CONVERSION) / BIT_ANGLE_CONVERSION * CLIP_PREVENTION;
    angle += _angleOffset;  // Apply software offset
    _lastConversion.raw = _position_data;
    _lastConversion.angle = angle;
    
    Serial.print("Raw ");
    Serial.println(_lastConversion.raw, HEX);
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