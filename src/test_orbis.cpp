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

    OrbisBRInstance::create(&Serial3, BAUD_RATE);

    OrbisBRInstance::instance().init();

    delay(500);
    
    for (byte command : UNLOCK_SEQUENCE)
    {
        Serial3.write(command);
        delay(1);
    } // may need delay(1)
    
    for (byte command : CONTINUOUS_RESPONSE)
    {
        Serial3.write(command);
        delay(1);
    } // may need delay(1)

    delay(10);

    for (byte command : UNLOCK_SEQUENCE)
    {
        Serial3.write(command);
        delay(1);
    } // may need delay(1)

    Serial3.write('S');

    delay(10);

    for (byte command : UNLOCK_SEQUENCE)
    {
        Serial3.write(command);
        delay(1);
    } // may need delay(1)

    Serial3.write('P');

    delay(10);

    for (byte command : UNLOCK_SEQUENCE)
    {
        Serial3.write(command);
        delay(1);
    } // may need delay(1)

    Serial3.write(SAVE_CONFIG);

    // for (byte command : UNLOCK_SEQUENCE)
    // {
    //     Serial3.write(command);
    // } // may need delay(1)

    // Serial3.write('r');

    while(!Serial);
    Serial.println("Serial Monitor connected.");
}

void loop()
{
//    if (millis() - lastPrintTime >= 500)
//    {
//         lastPrintTime = millis();
//         // OrbisBRInstance::instance().sample();
//         // // SteeringEncoderConversion_s data = OrbisBRInstance::instance().position();

        
//         // Serial.print("Raw: ");
//         // Serial.println(OrbisBRInstance::instance().position().raw);
//         // Serial.print("Angle: ");
//         // Serial.println(OrbisBRInstance::instance().position().angle);
//         // Serial.print("\n\n");
//         if (Serial3.available()) {
//             Serial.println(Serial3.read(), HEX);
//         }
//    }
    if (Serial3.available()) {
        Serial.println(Serial3.read(), HEX);
    }
}