#include "Arduino.h"
#include "ADCInterface.h"
#include "VCF_Constants.h"

#include "Logger.h"

unsigned long const DELAY = 100;
unsigned long last = millis();

void setup()
{
    SPI.begin();
    ADCInterfaceInstance::create(
        ADCPinout_s {
            VCFInterfaceConstants::ADC1_CS,
            VCFInterfaceConstants::ADC2_CS
        },
        ADCChannels_s {
            VCFInterfaceConstants::STEERING_1_CHANNEL,
            VCFInterfaceConstants::STEERING_2_CHANNEL,
            VCFInterfaceConstants::FR_LOADCELL_CHANNEL,
            VCFInterfaceConstants::FL_LOADCELL_CHANNEL,
            VCFInterfaceConstants::FR_SUS_POT_CHANNEL,
            VCFInterfaceConstants::FL_SUS_POT_CHANNEL,
            VCFInterfaceConstants::ACCEL_1_CHANNEL,
            VCFInterfaceConstants::ACCEL_2_CHANNEL,
            VCFInterfaceConstants::BRAKE_1_CHANNEL,
            VCFInterfaceConstants::BRAKE_2_CHANNEL
        },
        ADCScales_s { 
            VCFInterfaceConstants::STEERING_1_SCALE, 
            VCFInterfaceConstants::STEERING_2_SCALE, 
            VCFInterfaceConstants::FR_LOADCELL_SCALE,
            VCFInterfaceConstants::FL_LOADCELL_SCALE,
            VCFInterfaceConstants::FR_SUS_POT_SCALE,
            VCFInterfaceConstants::FL_SUS_POT_SCALE, 
            VCFInterfaceConstants::ACCEL_1_SCALE, 
            VCFInterfaceConstants::ACCEL_2_SCALE, 
            VCFInterfaceConstants::BRAKE_1_SCALE, 
            VCFInterfaceConstants::BRAKE_2_SCALE
        }, 
        ADCOffsets_s {
            VCFInterfaceConstants::STEERING_1_OFFSET,
            VCFInterfaceConstants::STEERING_2_OFFSET,
            VCFInterfaceConstants::FR_LOADCELL_OFFSET,
            VCFInterfaceConstants::FL_LOADCELL_OFFSET,
            VCFInterfaceConstants::FR_SUS_POT_OFFSET,
            VCFInterfaceConstants::FL_SUS_POT_OFFSET,
            VCFInterfaceConstants::ACCEL_1_OFFSET,
            VCFInterfaceConstants::ACCEL_2_OFFSET,
            VCFInterfaceConstants::BRAKE_1_OFFSET,
            VCFInterfaceConstants::BRAKE_2_OFFSET
        }
    );
}

void loop()
{
    if (millis() - DELAY > last) {
        ADCInterfaceInstance::instance().adc1_tick();
        hal_printf("ADC1 Front Right Load Cell Raw: %d\n", ADCInterfaceInstance::instance().FR_load_cell().raw);
        hal_printf("ADC1 Front Left Load Cell Raw:  %d\n", ADCInterfaceInstance::instance().FL_load_cell().raw);
        hal_printf("ADC1 Front Right Sus Pot Raw:   %d\n", ADCInterfaceInstance::instance().FR_sus_pot().raw);
        hal_printf("ADC1 Front Left Sus Pot Raw:    %d\n", ADCInterfaceInstance::instance().FL_sus_pot().raw);
        hal_printf("ADC1 Steering Degrees CCW Raw:  %d\n", ADCInterfaceInstance::instance().steering_degrees_ccw().raw);
        hal_printf("ADC1 Steering Degrees CW Raw:   %d\n", ADCInterfaceInstance::instance().steering_degrees_cw().raw);
        hal_printf("\n\n");

        ADCInterfaceInstance::instance().adc2_tick();
        hal_printf("ADC2 Acceleration 1 Raw: %d\n", ADCInterfaceInstance::instance().acceleration_1().raw);
        hal_printf("ADC2 Acceleration 2 Raw: %d\n", ADCInterfaceInstance::instance().acceleration_2().raw);
        hal_printf("ADC2 Brake 1 Raw:        %d\n", ADCInterfaceInstance::instance().brake_1().raw);
        hal_printf("ADC2 Brake 2 Raw:        %d\n", ADCInterfaceInstance::instance().brake_2().raw);
        hal_printf("\n\n");
        last = millis();
    }
}