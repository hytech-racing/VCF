#include "Arduino.h"
#include "ORBIS_BR10.h"
#include "SteeringEncoderInterface.h"

// Globals
OrbisBR10 encoder(&Serial3, 115200);
unsigned long lastPrintTime = 0;

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println("Initializing configuration...");
    encoder.init();
    Serial.println("Ready...\n");
}

void loop()
{
   if (millis() - lastPrintTime >= 500)
   {
        lastPrintTime = millis();
        SteeringEncoderConversion_s data = encoder.position();

        Serial.print("Raw: ");
        Serial.println(data.raw);
        Serial.print(" | Angle:");
        Serial.println(data.angle);
   }
}