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
    PedalSensorData_s pedal_sensor_data = {};
    
    pedal_sensor_data.accel_1 = ADCsOnVCFInstance::instance().adc_2.data.conversions[ACCEL_1_CHANNEL].conversion;
    pedal_sensor_data.accel_2 = ADCsOnVCFInstance::instance().adc_2.data.conversions[ACCEL_2_CHANNEL].conversion;
    pedal_sensor_data.brake_1 = ADCsOnVCFInstance::instance().adc_2.data.conversions[BRAKE_1_CHANNEL].conversion;
    pedal_sensor_data.brake_2 = ADCsOnVCFInstance::instance().adc_2.data.conversions[BRAKE_2_CHANNEL].conversion;
    
    PedalsSystemData_s data = PedalsSystemInstance::instance().evaluate_pedals(pedal_sensor_data, millis());

    Serial.printf("Accel 1: %d\n", pedal_sensor_data.accel_1);
    Serial.printf("Accel 2: %d\n", pedal_sensor_data.accel_2);
    Serial.printf("Brake 1: %d\n", pedal_sensor_data.brake_1);
    Serial.printf("Brake 2: %d\n", pedal_sensor_data.brake_2);
    Serial.printf("Accel Implausible: %d\n", data.accel_is_implausible);
    Serial.printf("Brake Implausible: %d\n", data.brake_is_implausible);
    Serial.printf("Brake Pressed: %d\n", data.brake_is_pressed);
    Serial.printf("Accel Pressed: %d\n", data.accel_is_pressed);
    Serial.printf("Mech Brake Active: %d\n", data.mech_brake_is_active);
    Serial.printf("Brake and Accel Implausibility High: %d\n", data.brake_and_accel_pressed_implausibility_high);
    Serial.printf("Implausibility has exceeded max duration: %d\n", data.implausibility_has_exceeded_max_duration);
    Serial.printf("Accel Percent: %d\n", data.accel_percent);
    Serial.printf("Brake Percent: %d\n", data.brake_percent);
    Serial.printf("Mech Brake Percent: %d\n", data.mech_brake_percent);
    Serial.printf("Regen Percent: %d\n", data.regen_percent); 
    
    const int delay_time = 1000;
    delay(delay_time);
}