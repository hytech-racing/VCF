#include "ORBIS_BR10.h"

OrbisBR10 steering_digital(&Serial3);

const int delay_time = 1000;
const int serial_speed = 115200;

void setup() {
    Serial.begin(serial_speed);
    steering_digital.init();
}

void loop() {
    steering_digital.sample();
    SteeringEncoderConversion_s readings = steering_digital.convert();

    Serial.print("Raw Data: \t");
    Serial.println(readings.raw);
    // Serial.println(steering_digital.convert().raw);

    Serial.print("Steering Angle: \t");
    Serial.println(readings.angle);
    // Serial.println(steering_digital.convert().angle);
    Serial.print("Status: \t");
    Serial.println(static_cast<int>(readings.status));
    // Serial.println(static_cast<int>(steering_digital.convert().status));

    delay(delay_time);
}