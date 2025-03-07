#ifdef ARDUINO
#include <Arduino.h>
#endif


/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"

/* Local includes */
#include "VCF_Globals.h"
#include "VCF_Constants.h"
#include "VCF_Tasks.h"
#include "PedalsSystem.h"
#include <stdio.h>



const PedalsParams params = {
    .min_pedal_1 = 1000,
    .min_pedal_2 = 2000,
    .max_pedal_1 = 2000,
    .max_pedal_2 = 1000,
    .activation_percentage = 0.05,
    .min_sensor_pedal_1 = 90,
    .min_sensor_pedal_2 = 90,
    .max_sensor_pedal_1 = 4000,
    .max_sensor_pedal_2 = 4000,
    .deadzone_margin = .03,
    .implausibility_margin = IMPLAUSIBILITY_PERCENT,
    .mechanical_activation_percentage = 0.05
};


void setup(){
    PedalsSystemInstance::create(params, params);
    const int begin_time = 115200;
    Serial.begin(begin_time);
    SPI.begin();
    init_adc_task();
}

void loop(){

    ADCsOnVCFInstance::instance().adc_2.tick();
    PedalSensorData_s pedal_sensor_data;
    
    
    pedal_sensor_data.accel_1 = ADCsOnVCFInstance::instance().adc_2.data.conversions[ACCEL_1_CHANNEL].conversion;
    pedal_sensor_data.accel_2 = ADCsOnVCFInstance::instance().adc_2.data.conversions[ACCEL_2_CHANNEL].conversion;
    pedal_sensor_data.brake_1 = ADCsOnVCFInstance::instance().adc_2.data.conversions[BRAKE_1_CHANNEL].conversion;
    pedal_sensor_data.brake_2 = ADCsOnVCFInstance::instance().adc_2.data.conversions[BRAKE_2_CHANNEL].conversion;
    
    PedalsSystemData_s data = PedalsSystemInstance::instance().evaluate_pedals(pedal_sensor_data, millis());
    Serial.print("Accel 1: ");
    Serial.println(pedal_sensor_data.accel_1);
    Serial.print("Accel 2: ");
    Serial.println(pedal_sensor_data.accel_2);
    Serial.print("Brake 1: ");
    Serial.println(pedal_sensor_data.brake_1);
    Serial.print("Brake 2: ");
    Serial.println(pedal_sensor_data.brake_2);
    //Serial.printf("Accel Percent: %f\n + Brake Percent: %f\n ", data.accel_percent, data.brake_percent);
    /*
    bool accel_is_implausible : 1; // Checks if either accel pedal is out of range OR they disagree by more than 10%
    bool brake_is_implausible : 1; // Checks if brake sensor is out of range.
    bool brake_is_pressed : 1; // True if brake pedal is pressed beyond the specified activationPercentage.
    bool accel_is_pressed : 1; // True if the accel pedal is pressed beyond the specified activationPercentage.
    bool mech_brake_is_active : 1; // True if the brake pedal is pressed beyond mechanical_activation_percentage.
    bool brake_and_accel_pressed_implausibility_high : 1; // If accel is pressed at all while mech_brake_is_active.
    bool implausibility_has_exceeded_max_duration : 1; // True if implausibility lasts more than 100ms
    float accel_percent;
    float brake_percent;
    float regen_percent;
    */
    Serial.println("Accel Implausible: ");
    Serial.println(data.accel_is_implausible);
    Serial.println("Brake Implausible: ");
    Serial.println(data.brake_is_implausible);
    Serial.println("Brake Pressed: ");
    Serial.println(data.brake_is_pressed);
    Serial.println("Accel Pressed: ");
    Serial.println(data.accel_is_pressed);
    Serial.println("Mech Brake Active: ");
    Serial.println(data.mech_brake_is_active);
    Serial.println("Brake and Accel Implausibility High: ");
    Serial.println(data.brake_and_accel_pressed_implausibility_high);
    Serial.println("Implausibility has exceeded max duration: ");
    Serial.println(data.implausibility_has_exceeded_max_duration);
    Serial.println("Accel Percent: ");
    Serial.println(data.accel_percent);
    Serial.println("Brake Percent: ");
    Serial.println(data.brake_percent);
    Serial.println("Regen Percent: ");
    Serial.println(data.regen_percent);    
    const int delay_time = 1000;
    delay(delay_time);
}