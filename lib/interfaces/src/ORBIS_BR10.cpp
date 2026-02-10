/* Library Includes */
#include "ORBIS_BR10.h"
#include <Arduino.h>

OrbisBR10::OrbisBR10(HardwareSerial* serial)
: _serial(serial)
, _serialSpeed(OrbisDefaultParams::ORBIS_BR_DEFAULT_BAUD_RATE)
{
   _lastConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_ERROR;
   _serial->begin(_serialSpeed, SERIAL_8N1);
}


/* -------------------- Initialization Functions -------------------- */
bool OrbisBR10::performSelfCalibration()
{
    // Serial.println("Starting self-calibration..."); // Debug line

    /* ----- SELF CALIBRATION STATUS 1 ----- */
    
    _serial->write(OrbisCommands::SELF_CALIB_STATUS); delay(1);      
    // Datasheet says to do status before self-calibration start. 
    // Returns 2 bytes: Echo byte then status byte
    
    while (!_serial->available());  
    // Purpose: Wait until a byte is on the line to be read. We had problems where we are reading too fast and didn't read the correct bytes
    // Serial.printf("Available Bytes: %d\n", _serial->available()); // Debug line
    uint8_t echo_1 = _serial->read();  
    
    while (!_serial->available());
    // Serial.printf("Available Bytes: %d\n", _serial->available()); // Debug line
    uint8_t status_1 = _serial->read(); 

    // Serial.print("Echo byte 1: 0x");     // Debug line
    // Serial.println(echo_1, HEX);         // Debug line
    // Serial.print("Status byte 1: 0x");   // Debug line
    // Serial.println(status_1, HEX);       // Debug line

    uint8_t current_counter = 0;             // global, and have it be checked so that you don't calibrate multiple times
    current_counter = status_1 & 0b00000011;  // Counter bits are b1 and b0 of status byte
    // Serial.print("Current Counter: ");        // Debug line
    // Serial.println(current_counter);          // Debug line

    /* ----- SELF CALIBRATION START ----- */
    
    for (byte b : OrbisCommands::UNLOCK_SEQUENCE) 
    {
        _serial->write(b); delay(1);
    }
    
    while (_serial->available()) {
        // Serial.print("Unlock Sequence Echo: 0x");       // Debug line
        uint8_t unlock_echo = _serial->read();
        // Serial.println(unlock_echo, HEX);               // Debug line 
    }
    
    _serial->write(OrbisCommands::SELF_CALIB_START); delay(1);
    
    while (!_serial->available());    
    uint8_t calibration_start = _serial->read();  
    // Serial.print("Calibration Start Byte: 0x");         // Debug line
    // Serial.println(calibration_start, HEX);             // Debug line
    
    Serial.println("ROTATE NOW");                       // Debug line

    delay(10000); // Max time for a self-calibration is 10 seconds. 
                                                        
    

    /* ----- SELF CALIBRATION STATUS 2 ----- */
    // self-calibration status request again after, expect counter to change
    _serial->write(OrbisCommands::SELF_CALIB_STATUS); delay(1);

    delay(50);

    if (_serial->available() < 2) { //while loop?
        Serial.println("ERROR: No Response From Sensor"); // Debug line
        _lastConversion.errors.noData = true;
        return false;
    }

    // Serial.printf("Available Bytes: %d\n", _serial->available());      // Debug line
    uint8_t echo_2 = _serial->read();     
    // Serial.printf("Available Bytes: %d\n", _serial->available());      // Debug line
    uint8_t status_2 = _serial->read();   

    // Serial.print("Echo 2: 0x");              // Debug line
    // Serial.println(echo_2, HEX);             // Debug line
    // Serial.print("Status byte 2: 0x");       // Debug line
    // Serial.println(status_2, HEX);           // Debug line
    
    uint8_t new_counter = status_2 & 0b00000011;
    bool timeout_error = status_2 & 0b00000100;      // b2
    bool parameter_error = status_2 & 0b00001000;    // b3
    
    // Serial.print("New counter: ");           // Debug line
    // Serial.println(new_counter);             // Debug line
    // Serial.print("Timeout error: ");         // Debug line
    // Serial.println(timeout_error);           // Debug line
    // Serial.print("Parameter error: ");       // Debug line
    // Serial.println(parameter_error);         // Debug line

    if (((current_counter + 1) & 0b00000011) != new_counter) {
        Serial.println("ERROR: Calibration Did Not Complete");
        return false;
    }
    
    if (timeout_error || parameter_error)
    {
        Serial.println("ERROR: Calibration Failed");
        _lastConversion.errors.calibrationTimeout   = timeout_error;
        Serial.println(_lastConversion.errors.calibrationTimeout);
        _lastConversion.errors.calibrationParameter = parameter_error>>3;
        return false;
    }
    
    Serial.println("Calibration successful!");   // Debug line
    return true;
}


void OrbisBR10::setEncoderOffset()
{
    while (_serial->available()) {  // Clear everything off the line. Check if this is necessary?
        _serial->read();
    }
    
    _serial->write(OrbisCommands::SHORT_POS_REQUEST); delay(1);

    while (!_serial->available());  //wait until bytes are available
    uint8_t position_1 = _serial->read(); 
    while (!_serial->available());  //wait until bytes are available
    uint8_t position_2 = _serial->read();
    
    // Serial.println("Position for Offset: ");     // Debug line
    // Serial.println(position_1, HEX);             // Debug line
    // Serial.println(position_2, HEX);             // Debug line
    // Serial.println();                            // Debug ling

    uint16_t position_data = ((((uint16_t) position_1) << OrbisDefaultParams::POS_DATA_MASK1) | (uint16_t) position_2) >> OrbisDefaultParams::POS_DATA_MASK2;

    byte position1 = (position_data >> OrbisDefaultParams::POS_DATA_MASK1) & 0xFF;      // b15 - b8 (See Encoder Packet Structure page 16)
    byte position2 = position_data & 0xFF;             // b7 - b0

    for (byte b : OrbisCommands::UNLOCK_SEQUENCE) 
    {
        _serial->write(b); delay(1);
        _serial->read();
    }

    // while (_serial->available()) {
    //     // Serial.print("Unlock Sequence Echo: 0x");       // Debug line
    //     uint8_t unlock_echo = _serial->read();
    //     // Serial.println(unlock_echo, HEX);               // Debug line 
    // }

    _serial->write(OrbisCommands::POSITION_OFFSET); delay(1);      
    _serial->write((byte) 0x00); delay(1);      
    _serial->write((byte) 0x00); delay(1);
    _serial->write(position1); delay(1);
    _serial->write(position2); delay(1);
    // After writing POSITION_OFFSET, the next four bytes are what you subtract from your initial position. We want to start at 0, so we first
    // get our initial position, and then send that position as the four bytes. Inital position - inital position = 0. 

    while (_serial->available()) {
        // Serial.print("Response Bytes: 0x");      // Debug line     
        uint8_t response_bytes = _serial->read();
        // Serial.println(response_bytes, HEX);     // Debug line: expect 0x5A, 0x00, 0x00, then the same bytes from the short position
    }

    Serial.println("Offset command done"); // Debug line
}


void OrbisBR10::saveConfiguration()
{
    for (byte command : OrbisCommands::UNLOCK_SEQUENCE)
    {
        _serial->write(command); delay(1);
    } 

    _serial->write(OrbisCommands::SAVE_CONFIGURATION); delay(1);  

    delay(100);

    while (_serial->available()) {
        // Serial.print("Response Bytes: 0x");      // Debug line
        uint8_t response_bytes = _serial->read();
        // Serial.println(response_bytes, HEX);     // Debug line: expect unlock then 0x63
    }

}



void OrbisBR10::sample()
{
    while (_serial->available()) {  // Clear everything off the line. Check if this is necessary?
        _serial->read();
    }
   
    _serial->write(OrbisCommands::DETAILED_POS_REQUEST); delay(1);   
    // Detailed Position Request:  1 byte echo, 2 byte position, 1 byte detailed status
    
    if (_serial->available() < 4)     // check if received all 4 bytes
    { 
        _lastConversion.errors.noData = true;
        _lastConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_ERROR;
        return;
    }   

    Serial.println("Sent position request on liner.");  // Debug line

    // Serial.printf("Available Bytes: %d\n", _serial->available());   // Debug line
    uint8_t echo     = _serial->read();

    if (echo != OrbisCommands::DETAILED_POS_REQUEST)
    {
        _lastConversion.errors.noData = true;
        _lastConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_ERROR;
        return;
    }

    // Serial.printf("Available Bytes: %d\n", _serial->available());   // Debug line
    uint8_t general1 = _serial->read();
    // Serial.println(general1, HEX);
    // Serial.printf("Available Bytes: %d\n", _serial->available());   // Debug line
    uint8_t general2 = _serial->read();
    //Serial.println(general2, HEX);
    // Serial.printf("Available Bytes: %d\n", _serial->available());  // Debug line
    uint8_t detailed = _serial->read();
    //Serial.println(detailed, HEX);

    // Serial.println(echo, HEX);          // Debug line
    // Serial.println(general1, HEX);      // Debug line
    // Serial.println(general2, HEX);      // Debug line
    // Serial.println(detailed, HEX);      // Debug line

    // Extract general status bits from the raw data
    uint8_t general_status = general2 & 0x03;  // b1:b0 are general status
    
    // Decode errors using the extracted general status bits
    decodeErrors(general_status, detailed);

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

    // Extract 14-bit position data
    uint16_t raw_position_data = (((uint16_t) general1) << OrbisDefaultParams::POS_DATA_MASK1) | (uint16_t) general2;
    uint16_t _position_data = raw_position_data >> OrbisDefaultParams::POS_DATA_MASK2;
    
    // Convert Position Data to Angle
    float angle = ((float)_position_data / OrbisDefaultParams::ENCODER_RESOLUTION) * OrbisDefaultParams::DEGREES_PER_REVOLUTION;
    _lastConversion.raw = _position_data;
    _lastConversion.angle = angle;

    if (angle > 180.0f) {   // fix this math lol
        angle -= 360.0f;
    }
    
    Serial.print("Raw ");
    Serial.println(_lastConversion.raw, HEX);
    Serial.print("Angle ");
    Serial.println(_lastConversion.angle);
 
}

SteeringEncoderConversion_s OrbisBR10::position()
{
    return _lastConversion;
}



/* -------------------- Error Flagging -------------------- */
void OrbisBR10::decodeErrors(uint8_t general, uint8_t detailed)
{
    // General bits error low (0)
    _lastConversion.errors.generalError     = !(general & OrbisGeneralErrorMasks::ORBIS_BR_BITMASK_GENERAL_ERROR);
    _lastConversion.errors.generalWarning   = !(general & OrbisGeneralErrorMasks::ORBIS_BR_BITMASK_GENERAL_WARNING);

    // Detailed bits are error high (1)
    _lastConversion.errors.counterError     = detailed & OrbisDetailedErrorMasks::ORBIS_BR_BITMASK_DETAILED_COUNTER_ERROR;
    _lastConversion.errors.speedHigh        = detailed & OrbisDetailedErrorMasks::ORBIS_BR_BITMASK_DETAILED_SPEED_HIGH;
    _lastConversion.errors.tempRange        = detailed & OrbisDetailedErrorMasks::ORBIS_BR_BITMASK_DETAILED_TEMP_RANGE;
    _lastConversion.errors.distFar          = detailed & OrbisDetailedErrorMasks::ORBIS_BR_BITMASK_DETAILED_DIST_FAR;
    _lastConversion.errors.distNear         = detailed & OrbisDetailedErrorMasks::ORBIS_BR_BITMASK_DETAILED_DIST_NEAR;
}



/* -------------------- Additional Functionalities -------------------- */
void OrbisBR10::factoryReset()
{
    for (byte command : OrbisCommands::UNLOCK_SEQUENCE)
    {
         Serial3.write(command); delay(1);
    } 

    while (_serial->available()) {
        // Serial.print("Unlock Sequence Echo: 0x"); // Debug line
        uint8_t unlock_echo = _serial->read();
        Serial.println(unlock_echo, HEX);
    }

    Serial3.write('r');
    
    while (!Serial3.available());
    uint8_t response_byte = Serial3.read();
    // Serial.println(response_byte, HEX);      // Debug line
    Serial.println("Factory Reset Done");       // Debug line
}

