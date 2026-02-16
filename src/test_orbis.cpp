#include <Arduino.h>
#include "SteeringEncoderInterface.h"
#include "Orbis_BR.h"

constexpr int SAMPLE_RATE = 5000;

// Globals
unsigned long lastPrintTime = 0;


void setup()
{
    Serial.begin(ORBIS_BR_DEFAULT_BAUD_RATE);

    while(!Serial);
    Serial.println("Serial Monitor connected.");        // Debug line

    OrbisBRInstance::create(&Serial3, ORBIS_BR_DEFAULT_BAUD_RATE);

    
    Serial.println("Calling Self-Calib");                  // Debug line
    
    OrbisBRInstance::instance().sample();
    
    OrbisBRInstance::instance().factoryReset();

    bool _isCalibrated = false;    // Assume sensor is not calibrated
    while (!_isCalibrated) //NOLINT
    {
        _isCalibrated = OrbisBRInstance::instance().performSelfCalibration(); 
    }
    
    OrbisBRInstance::instance().setEncoderOffset();

    // OrbisBRInstance::instance().saveConfiguration(); delay(100);

    /* --- Block For Debugging --- */
    // OrbisBRInstance::instance().sample();
    // Serial.print("\n\n");
}

void loop()
{
    if (millis() - lastPrintTime >= SAMPLE_RATE) 
    {
        lastPrintTime = millis();
        
        OrbisBRInstance::instance().sample();

    }
}