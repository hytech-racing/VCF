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
    Serial.println("Raw Data: ");
    Serial.println(steering_digital.convert().raw);
    Serial.println("Steering Angle: ");
    Serial.println(steering_digital.convert().angle);
    Serial.println("Status: ");
    Serial.println(static_cast<int>(steering_digital.convert().status));

    delay(delay_time);
}