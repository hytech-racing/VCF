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

    while(!Serial);
    OrbisBRInstance::create(&Serial3, BAUD_RATE);

    Serial.println("Calling init");
    OrbisBRInstance::instance().init();
    delay(500);

    // for (byte command : UNLOCK_SEQUENCE)
    // {
    //     Serial3.write(command);
    //     delay(1);
    // } 

    // Serial3.write('r');
    
    // Serial.println("Factor Reset Done");
    
    // for (byte command : UNLOCK_SEQUENCE)
    // {
    //     Serial3.write(command);
    //     delay(1);
    // } // may need delay(1)
    
    // Serial.println("Starting Self-Calibration");
    // Serial3.write(SELF_CALIB_START);
    // delay(10000);

    // for (byte command : CONTINUOUS_RESPONSE)
    // {
    //     Serial3.write(command);
    //     delay(1);
    // } // may need delay(1)

    // delay(10);

    // for (byte command : UNLOCK_SEQUENCE)
    // {
    //     Serial3.write(command);
    //     delay(1);
    // } // may need delay(1)

    // Serial3.write('S');

    // delay(10);

    // for (byte command : UNLOCK_SEQUENCE)
    // {
    //     Serial3.write(command);
    //     delay(1);
    // } // may need delay(1)

    // Serial3.write('P');

    // delay(10);

    // for (byte command : UNLOCK_SEQUENCE)
    // {
    //     Serial3.write(command);
    //     delay(1);
    // } // may need delay(1)

    // Serial3.write(SAVE_CONFIG);

    // for (byte command : UNLOCK_SEQUENCE)
    // {
    //     Serial3.write(command);
    // } // may need delay(1)

    // Serial3.write('r');
    Serial.println("Serial Monitor connected.");
    OrbisBRInstance::instance().sample();
    Serial.print("Raw: ");
    Serial.println(OrbisBRInstance::instance().position().raw);
    Serial.print("Angle: ");
    Serial.println(OrbisBRInstance::instance().position().angle);
    Serial.print("\n\n");
}

void loop()
{
    //Serial.println("HELLO");
    if (millis() - lastPrintTime >= 500)
    {
        lastPrintTime = millis();
        // OrbisBRInstance::instance().sample();
        // Serial.println("In Loop");

        // bool read = false;
        
        // while (!read)
        // {
        //     if (Serial3.available() > 3) {
        //         Serial.println(Serial3.read(), HEX);
        //         Serial.println(Serial3.read(), HEX);
        //         Serial.println(Serial3.read(), HEX);
        //         Serial.println(Serial3.read(), HEX);
        //         read = true;
        //     }
        // }
        // SteeringEncoderConversion_s data = OrbisBRInstance::instance().position();

        //--
        // Serial.print("Raw: ");
        // Serial.println(OrbisBRInstance::instance().position().raw);
        // Serial.print("Angle: ");
        // Serial.println(OrbisBRInstance::instance().position().angle);
        // Serial.print("\n\n");
        // --

        // if (Serial3.available()) {
        //     Serial.println(Serial3.read(), HEX);
        // }
    }
}
    
    // Serial3.write(0xCD); delay(1);  // unlock sequence, delay is from datasheet. "Delay between each byte sent during programming must be a least 1 ms."
    // Serial3.write(0xEF); delay(1);  
    // Serial3.write(0x89); delay(1);  
    // Serial3.write(0xAB); delay(1); 
    // Serial3.write('P');
    // Serial.println("Sending P on the liner");

    // for (byte command : UNLOCK_SEQUENCE)
    // {
    //     Serial3.write(command); delay(1);
    // } // may need delay(1)
    
    // Serial3.write(0x64);
    // Serial.println("Sending Position on the liner");

    // if (Serial3.available()) {
    //     Serial.println(Serial3.read(), HEX);
    // }

    
    // for (byte command : UNLOCK_SEQUENCE)
    // {
    //     Serial3.write(command); delay(1);
    //     if (Serial3.available())
    //     {
    //         Serial.println(Serial3.read(),HEX);
    //     }
    // } // may need delay(1)
    

    // Serial3.write(0x64); delay(1);
    // Serial.println("Sending Position on the liner");

    // bool read = false;

    // while (!read)
    // {
    //     if (Serial3.available() > 3) {
    //         Serial.println(Serial3.read(), HEX);
    //         Serial.println(Serial3.read(), HEX);
    //         Serial.println(Serial3.read(), HEX);
    //         Serial.println(Serial3.read(), HEX);
    //         read = true;
    //     }
    // }

    // delay(1000);
// }