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

//accel params and brake params

const PedalsParams accel_params = {
    .min_pedal_1 = 1790,
    .min_pedal_2 = 1690,
    .max_pedal_1 = 2830,
    .max_pedal_2 = 670,
    .activation_percentage = 0.05,
    .min_sensor_pedal_1 = 90,
    .min_sensor_pedal_2 = 90,
    .max_sensor_pedal_1 = 4000,
    .max_sensor_pedal_2 = 4000,
    .deadzone_margin = .03,
    .implausibility_margin = IMPLAUSIBILITY_PERCENT,
    .mechanical_activation_percentage = 0.05
};

const PedalsParams brake_params = {
    .min_pedal_1 = 1180,
    .min_pedal_2 = 2500,
    .max_pedal_1 = 1660,
    .max_pedal_2 = 1770,
    .activation_percentage = 0.05,
    .min_sensor_pedal_1 = 90,
    .min_sensor_pedal_2 = 90,
    .max_sensor_pedal_1 = 4000,
    .max_sensor_pedal_2 = 4000,
    .deadzone_margin = .03,
    .implausibility_margin = IMPLAUSIBILITY_PERCENT,
    .mechanical_activation_percentage = 0.65
};


void setup(){
    PedalsSystemInstance::create(accel_params, brake_params); //pass in the two different params
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

    Serial.print("Accel 1: ");
    Serial.println(pedal_sensor_data.accel_1);
    Serial.print("Accel 2: ");
    Serial.println(pedal_sensor_data.accel_2);
    Serial.print("Brake 1: ");
    Serial.println(pedal_sensor_data.brake_1);
    Serial.print("Brake 2: ");
    Serial.println(pedal_sensor_data.brake_2);
    Serial.print("Accel Implausible: ");
    Serial.println(data.accel_is_implausible);
    Serial.print("Brake Implausible: ");
    Serial.println(data.brake_is_implausible);
    Serial.print("Brake Pressed: ");
    Serial.println(data.brake_is_pressed);
    Serial.print("Accel Pressed: ");
    Serial.println(data.accel_is_pressed);
    Serial.print("Mech Brake Active: ");
    Serial.println(data.mech_brake_is_active);
    Serial.print("Brake and Accel Implausibility High: ");
    Serial.println(data.brake_and_accel_pressed_implausibility_high);
    Serial.print("Implausibility has exceeded max duration: ");
    Serial.println(data.implausibility_has_exceeded_max_duration);
    Serial.print("Accel Percent: ");
    Serial.println(data.accel_percent);
    Serial.print("Brake Percent: ");
    Serial.println(data.brake_percent);
    Serial.print("Regen Percent: ");
    Serial.println(data.regen_percent);    
    
    const int delay_time = 1000;
    delay(delay_time);
}