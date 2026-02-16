/* Library Includes */
#include "Orbis_BR.h"
#include <Arduino.h>

OrbisBR::OrbisBR(HardwareSerial* serial, int serialSpeed)
: _serial(serial)
, _serialSpeed(serialSpeed)
{
   _lastConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_ERROR;
   _serial->begin(_serialSpeed, SERIAL_8N1);
}


/* -------------------- Initialization Methods -------------------- */
// Is there a point in having a separate method for off car set-up? The only thing in that method wold likely 

bool OrbisBR::performSelfCalibration()
{
    //Serial.println("Starting self-calibration..."); // Debug line

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

    uint8_t current_counter = 0;                    // global, and have it be checked so that you don't calibrate multiple times
    current_counter = status_1 & SELF_CALIBRATION_STATUS_MASK;   // Counter bits are b1 and b0 of status byte
    // Serial.print("Current Counter: ");           // Debug line
    // Serial.println(current_counter);             // Debug line

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

    delay(SELF_CALIBRATION_MAX_TIME); // Max time for a self-calibration is 10 seconds. 
                                                        
    

    /* ----- SELF CALIBRATION STATUS 2 ----- */
    // self-calibration status request again after, expect counter to change
    _serial->write(OrbisCommands::SELF_CALIB_STATUS); delay(1);

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
    
    uint8_t new_counter = status_2 & SELF_CALIBRATION_NEW_COUNTER_MASK;
    bool timeout_error = status_2 & SELF_CALIBRATION_TIMEOUT_ERROR_MASK;       // b2
    bool parameter_error = status_2 & SELF_CALIRATION_PARAMETER_ERROR_MASK;    // b3
    
    // Serial.print("New counter: ");           // Debug line
    // Serial.println(new_counter);             // Debug line
    // Serial.print("Timeout error: ");         // Debug line
    // Serial.println(timeout_error);           // Debug line
    // Serial.print("Parameter error: ");       // Debug line
    // Serial.println(parameter_error);         // Debug line

    if (((current_counter + 1) & SELF_CALIBRATION_COUNTER_CHECK_MASK) != new_counter) {
        Serial.println("ERROR: Calibration Did Not Complete");
        return false;
    }
    
    if (timeout_error || parameter_error)
    {
        Serial.println("ERROR: Calibration Failed");
        _lastConversion.errors.calibrationTimeout   = timeout_error;
        Serial.println(_lastConversion.errors.calibrationTimeout);
        _lastConversion.errors.calibrationParameter = parameter_error; // do I need to shift 3?? 
        return false;
    }
    
    Serial.println("Calibration successful!");   // Debug line
    return true;
}

void OrbisBR::setEncoderOffset()
{
    while (_serial->available()) {  // Clear everything off the line. Check if this is necessary?
        _serial->read();
    }


    _serial->write(OrbisCommands::SHORT_POS_REQUEST); delay(1);

    while (!_serial->available());
    uint8_t position_1 = _serial->read(); 
    while (!_serial->available());
    uint8_t position_2 = _serial->read();
    
    Serial.println("Position for Offset: ");     // Debug line
    Serial.println(position_1, HEX);             // Debug line
    Serial.println(position_2, HEX);             // Debug line
    Serial.println();                            // Debug ling

    uint16_t position_data = ((((uint16_t) position_1) << POSITION_DATA_MASK_1) | (uint16_t) position_2) >> POSITION_DATA_MASK_2;

    //The purpose of these two lines is to break the 16 bit integer into two 8 bit ones.
    byte position1 = (position_data >> OFFSET_SHORT_POSITION_RECOMBINING_SHIFT) & OFFSET_SHORT_POSITION_RECOMBINING_MASK;   // b15 - b8. 
    byte position2 = position_data & OFFSET_SHORT_POSITION_RECOMBINING_MASK;                                                // b7 - b0

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

    //saveConfiguration();
    Serial.println("Offset command done"); // Debug line
}

void OrbisBR::saveConfiguration()
{
    for (byte command : OrbisCommands::UNLOCK_SEQUENCE)
    {
        _serial->write(command); delay(1);
    } 

    _serial->write(OrbisCommands::SAVE_CONFIGURATION); delay(1);  

    while (_serial->available()) {
        // Serial.print("Response Bytes: 0x");      // Debug line
        uint8_t response_bytes = _serial->read();
        // Serial.println(response_bytes, HEX);     // Debug line: expect unlock then 0x63
    }

}



/* -------------------- Sample/Data Methods -------------------- */
void OrbisBR::sample()
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
    uint16_t raw_position_data = (((uint16_t) general1) << POSITION_DATA_MASK_1) | (uint16_t) general2;
    uint16_t _position_data = raw_position_data >> POSITION_DATA_MASK_2;
    
    // Convert Position Data to Angle
    _lastConversion.raw = _position_data;
    
    float angle = ((float)_position_data / ENCODER_RESOLUTION ) * DEGREES_PER_REVOLUTION ;
    if (angle > ANGLE_CONVERSION_MARKER) {   
        angle -= ANGLE_CONVERSION;
    }
    
    _lastConversion.angle = angle;
    
    Serial.print("Raw ");
    Serial.println(_lastConversion.raw, HEX);
    Serial.print("Angle ");
    Serial.println(_lastConversion.angle);
 
}

SteeringEncoderConversion_s OrbisBR::convert()
{
    return _lastConversion;
}

/* -------------------- Error Flagging -------------------- */
void OrbisBR::decodeErrors(uint8_t general, uint8_t detailed)
{
    general = 0b00000000;
    // General bits error low (0)
    _lastConversion.errors.generalError     = !(general & OrbisGeneralErrorMasks::ORBIS_BR_BITMASK_GENERAL_ERROR);
    if (_lastConversion.errors.generalError) {
        Serial.println("HARDCODED ERROR CHECK! BOOM!");
    }
    _lastConversion.errors.generalWarning   = !(general & OrbisGeneralErrorMasks::ORBIS_BR_BITMASK_GENERAL_WARNING);

    // Detailed bits are error high (1)
    _lastConversion.errors.counterError     = detailed & OrbisDetailedErrorMasks::ORBIS_BR_BITMASK_DETAILED_COUNTER_ERROR;
    _lastConversion.errors.speedHigh        = detailed & OrbisDetailedErrorMasks::ORBIS_BR_BITMASK_DETAILED_SPEED_HIGH;
    _lastConversion.errors.tempRange        = detailed & OrbisDetailedErrorMasks::ORBIS_BR_BITMASK_DETAILED_TEMP_RANGE;
    _lastConversion.errors.distFar          = detailed & OrbisDetailedErrorMasks::ORBIS_BR_BITMASK_DETAILED_DIST_FAR;
    _lastConversion.errors.distNear         = detailed & OrbisDetailedErrorMasks::ORBIS_BR_BITMASK_DETAILED_DIST_NEAR;
}



/* -------------------- Additional Functionalities -------------------- */
void OrbisBR::factoryReset()
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

    Serial3.write(OrbisCommands::FACTORY_RESET);
    
    while (!Serial3.available());
    uint8_t response_byte = Serial3.read();
    // Serial.println(response_byte, HEX);      // Debug line
    Serial.println("Factory Reset Done");       // Debug line
}

