/* Library Includes */
#include "Orbis_BR.h"
#include <Arduino.h>

OrbisBR::OrbisBR(HardwareSerial* serial)
: _serial(serial)
{
   _lastConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_ERROR;
   _serial->begin(OrbisConstants::ORBIS_BR_DEFAULT_BAUD_RATE, SERIAL_8N1);
}


/* -------------------- Initialization Methods -------------------- */

bool OrbisBR::performSelfCalibration()
{

    _orbisErrors.calibrationTimeout   = false;
    _orbisErrors.calibrationParameter = false;

    /* ----- SELF CALIBRATION STATUS 1 ----- */

    _serial->write(OrbisCommands::SELF_CALIB_STATUS); delay(1);
    // Datasheet says to do status before self-calibration start.
    // Returns 2 bytes: Echo byte then status byte

    uint8_t echo_1 = _serial->read();
    uint8_t status_1 = _serial->read();

    uint8_t current_counter = status_1 & OrbisConstants::SELF_CALIBRATION_STATUS_MASK;   // Counter bits are b1 and b0 of status byte


    /* ----- SELF CALIBRATION START ----- */

    for (byte b : OrbisCommands::UNLOCK_SEQUENCE)
    {
        _serial->write(b); delay(1);
        _serial->read();
    }

    _serial->write(OrbisCommands::SELF_CALIB_START); delay(1);

    while (!_serial->available());
    uint8_t calibration_start = _serial->read();

    Serial.println("ROTATE NOW");                      // Debug line

    delay(OrbisConstants::SELF_CALIBRATION_MAX_TIME); // Max time for a self-calibration is 10 seconds.


    /* ----- SELF CALIBRATION STATUS 2 ----- */
    // self-calibration status request again after, expect counter to change
    _serial->write(OrbisCommands::SELF_CALIB_STATUS); delay(1);

    if (_serial->available() < 2) {
        //Serial.println("ERROR: No Response From Sensor");
        _lastConversion.errors.noData = true;
        return false;
    }

    uint8_t echo_2 = _serial->read();
    uint8_t status_2 = _serial->read();

    uint8_t new_counter = status_2 & OrbisConstants::SELF_CALIBRATION_NEW_COUNTER_MASK;
    bool timeout_error = status_2 & OrbisConstants::SELF_CALIBRATION_TIMEOUT_ERROR_MASK;       // b2
    bool parameter_error = status_2 & OrbisConstants::SELF_CALIRATION_PARAMETER_ERROR_MASK;    // b3

    if (current_counter == new_counter) {
        Serial.println("ERROR: Calibration Did Not Complete");
        return false;
    }

    if (timeout_error || parameter_error)
    {
        //Serial.println("ERROR: Calibration Failed");
        _orbisErrors.calibrationTimeout   = timeout_error;
        _orbisErrors.calibrationParameter = parameter_error;
        return false;
    }

    return true;
}

void OrbisBR::setEncoderOffset()
{
    factoryReset();

    _serial->write(OrbisCommands::SHORT_POS_REQUEST); delay(1);

    while (!_serial->available());  //wait until bytes are available
    uint8_t position_1 = _serial->read();
    while (!_serial->available());  //wait until bytes are available
    uint8_t position_2 = _serial->read();

    uint16_t position_data = ((((uint16_t) position_1) << OrbisConstants::POSITION_DATA_MASK_1) | (uint16_t) position_2) >> OrbisConstants::POSITION_DATA_MASK_2;

    byte position1 = (position_data >> OrbisConstants::OFFSET_SHORT_POSITION_RECOMBINING_SHIFT) & OrbisConstants::OFFSET_SHORT_POSITION_RECOMBINING_MASK;   // b15 - b8.
    byte position2 = position_data & OrbisConstants::OFFSET_SHORT_POSITION_RECOMBINING_MASK;    // b7 - b0
    // For "position_data", we parse two bytes total to remove warning/error bits. End result: 16 bit integer. However, for the offset command,
    // we can only provide the position as individual bytes (8 bit integer). These two lines break down "position_data" into two bytes.

    for (byte b : OrbisCommands::UNLOCK_SEQUENCE)
    {
        _serial->write(b); delay(1);
        _serial->read();
    }

    _serial->write(OrbisCommands::POSITION_OFFSET); delay(1);
    _serial->write((byte) 0x00); delay(1);
    _serial->write((byte) 0x00); delay(1);
    _serial->write(position1); delay(1);
    _serial->write(position2); delay(1);
    // After writing POSITION_OFFSET, the next four bytes are what you subtract from your initial position. We want to start at 0, so we first
    // get our initial position, and then send that position as the four bytes. Inital position - inital position = 0.

    while (_serial->available()) {
        uint8_t response_bytes = _serial->read();
    }

    saveConfiguration();
}

void OrbisBR::saveConfiguration()
{
    for (byte command : OrbisCommands::UNLOCK_SEQUENCE)
    {
        _serial->write(command); delay(1);
    }

    _serial->write(OrbisCommands::SAVE_CONFIGURATION); delay(1);

    while (_serial->available()) {
        uint8_t response_bytes = _serial->read();
    }

    delay(OrbisConstants::REQUIRED_SAVE_CONFIGURATION_DELAY);
}



/* -------------------- Sample/Data Methods -------------------- */
void OrbisBR::sample()
{

    _lastConversion.errors = EncoderErrorFlags_s{};  // reset to all false
    _orbisErrors           = OrbisErrorFlags_s{};    // reset to all false

    _serial->write(OrbisCommands::DETAILED_POS_REQUEST); delay(1);
    // Detailed Position Request:  1 byte echo, 2 byte position, 1 byte detailed status

    while (_serial->available() < 4);

    if (_serial->available() < 4)     // check if received all 4 bytes
    {
        _lastConversion.errors.noData = true;
        _lastConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_ERROR;
        return;
    }

    uint8_t echo     = _serial->read();

    if (echo != OrbisCommands::DETAILED_POS_REQUEST)
    {
        _lastConversion.errors.noData = true;
        _lastConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_ERROR;
        return;
    }

    uint8_t general1 = _serial->read();
    uint8_t general2 = _serial->read();
    uint8_t detailed = _serial->read();

    // Extract general status bits from the raw data
    uint8_t general_status = general2 & OrbisConstants::SELF_CALIBRATION_STATUS_MASK;  // b1:b0 are general status

    // Decode errors using the extracted general status bits
    _decodeErrors(general_status, detailed);

    // Decode errors, detailed status bytes
    bool anyError =
     (
        _lastConversion.errors.generalError   ||
        _lastConversion.errors.noData         ||
        _orbisErrors.counterError             ||
        _orbisErrors.speedHigh                ||
        _orbisErrors.tempRange                ||
        _orbisErrors.distFar                  ||
        _orbisErrors.distNear
     );

    _lastConversion.status = anyError ? SteeringEncoderStatus_e::STEERING_ENCODER_ERROR : SteeringEncoderStatus_e::STEERING_ENCODER_NOMINAL;

    // Extract 14-bit position data
    uint16_t raw_position_data = (((uint16_t) general1) << OrbisConstants::POSITION_DATA_MASK_1) | (uint16_t) general2;
    uint16_t _position_data = raw_position_data >> OrbisConstants::POSITION_DATA_MASK_2;

    // Convert Position Data to Angle
    _lastConversion.raw = _position_data;

    float angle = ((float)_position_data / OrbisConstants::ENCODER_RESOLUTION ) * OrbisConstants::DEGREES_PER_REVOLUTION;
    if (angle > OrbisConstants::ANGLE_CONVERSION_MARKER) {
        angle -= OrbisConstants::ANGLE_CONVERSION;
    }
    // "_position_data" / "ENCODER_RESOLUTION" = fraction of one full rotation. Then multiple by "DEGREES_PER_REVOLUTION" (360), to get the exact angle.
    // We want the sensor to work such that the middle is 0, left goes to -180 degrees, and right goes to +180 degrees.
    // Thus, if the angle is > 180 (ANGLE_CONVERSION_MARKER), we want to subtract 360 (ANGLE_CONVERSION) so that it's negative.

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
void OrbisBR::_decodeErrors(uint8_t general, uint8_t detailed)
{
    // General bits error low (0)
    _lastConversion.errors.generalError     = !(general & OrbisErrorMasks::ORBIS_BR_BITMASK_GENERAL_ERROR);
    _lastConversion.errors.generalWarning   = !(general & OrbisErrorMasks::ORBIS_BR_BITMASK_GENERAL_WARNING);

    // Detailed bits are error high (1)
    _orbisErrors.counterError     = detailed & OrbisErrorMasks::ORBIS_BR_BITMASK_DETAILED_COUNTER_ERROR;
    _orbisErrors.speedHigh        = detailed & OrbisErrorMasks::ORBIS_BR_BITMASK_DETAILED_SPEED_HIGH;
    _orbisErrors.tempRange        = detailed & OrbisErrorMasks::ORBIS_BR_BITMASK_DETAILED_TEMP_RANGE;
    _orbisErrors.distFar          = detailed & OrbisErrorMasks::ORBIS_BR_BITMASK_DETAILED_DIST_FAR;
    _orbisErrors.distNear         = detailed & OrbisErrorMasks::ORBIS_BR_BITMASK_DETAILED_DIST_NEAR;
}



/* -------------------- Additional Functionalities -------------------- */
void OrbisBR::factoryReset()
{
    for (byte command : OrbisCommands::UNLOCK_SEQUENCE)
    {
         _serial->write(command); delay(1);
         _serial->read();
    }

    _serial->write(OrbisCommands::FACTORY_RESET);

    while (!_serial->available());
    uint8_t response_byte = Serial3.read();

    delay(OrbisConstants::REQUIRED_RESET_DELAY);

    //Serial.println("Factory Reset Done");       // Debug line
}

