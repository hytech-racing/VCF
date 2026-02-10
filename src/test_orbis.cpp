#include "Arduino.h"
#include "ORBIS_BR10.h"
#include "SteeringEncoderInterface.h"

const int BAUD_RATE = 115200;

// Globals
//OrbisBR10 encoder(&Serial3, BAUD_RATE);
unsigned long lastPrintTime = 0;


void setup()
{
    Serial.begin(BAUD_RATE);
    OrbisBRInstance::create(&Serial3);

    while(!Serial);
    Serial.println("Serial Monitor connected.");        // Debug line

    // OrbisBRInstance::instance().factoryReset(); delay(100);
    
    // Serial.println("Calling init");                  // Debug line
    // bool _isCalibrated = false;    // Assume sensor is not calibrated
    // while (!_isCalibrated)
    // {
    //     _isCalibrated = OrbisBRInstance::instance().performSelfCalibration();
    // }
    
    OrbisBRInstance::instance().sample(); delay(10);
    
    // OrbisBRInstance::instance().setEncoderOffset();

    // OrbisBRInstance::instance().saveConfiguration();

    // delay(1000);

    // OrbisBRInstance::instance().sample();

    // OrbisBRInstance::instance().setEncoderOffset();
    // OrbisBRInstance::instance().sample(); delay(10);
    
    
    /* --- Block For Debugging --- */
    // OrbisBRInstance::instance().sample();
    // Serial.print("\n\n");
}

void loop()
{
    if (millis() - lastPrintTime >= 1000)
    {
        lastPrintTime = millis();
        
        // Serial.println("Detailed Pos Req.");
        OrbisBRInstance::instance().sample();

        // Serial.println("\nShort Pos Req");
        // Serial3.write(OrbisCommands::SHORT_POS_REQUEST); delay(1);
        // Serial.println(Serial3.read(), HEX);
        // Serial.println(Serial3.read(), HEX);
    }

}