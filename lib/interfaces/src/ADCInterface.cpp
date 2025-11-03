#include "ADCInterface.h"

ADCInterface::ADC1InterfaceData_s ADCInterface::read_adc1()
{
    // Samples all eight channels.
    adc1.tick();

    ADC1Data::instance().analog_steering_degrees = adc1.data.conversions[STEERING_1_CHANNEL].conversion; // Only using steering 1 for now
    ADC1Data::instance().FL_load_cell = ADCInterface::iir_filter(IIR_FILTER_ALPHA, ADC1Data::instance().FL_load_cell, adc1.data.conversions[FL_LOADCELL_CHANNEL].conversion);
    ADC1Data::instance().FR_load_cell = ADCInterface::iir_filter(IIR_FILTER_ALPHA, ADC1Data::instance().FR_load_cell, adc1.data.conversions[FR_LOADCELL_CHANNEL].conversion);
    ADC1Data::instance().FL_sus_pot = ADCInterface::iir_filter(IIR_FILTER_ALPHA, ADC1Data::instance().FL_sus_pot, adc1.data.conversions[FL_SUS_POT_CHANNEL].raw);
    ADC1Data::instance().FR_sus_pot = ADCInterface::iir_filter(IIR_FILTER_ALPHA, ADC1Data::instance().FR_sus_pot, adc1.data.conversions[FR_SUS_POT_CHANNEL].raw);

    return ADC1Data::instance();
}

ADCInterface::ADC2InterfaceData_s ADCInterface::read_adc2()
{
    // Samples all eight channels.
    adc2.tick();

    ADC2Data::instance().acceleration_1 = adc2.data.conversions[ACCEL_1_CHANNEL].conversion;
    ADC2Data::instance().acceleration_2 = adc2.data.conversions[ACCEL_2_CHANNEL].conversion;
    ADC2Data::instance().brake_1 = adc2.data.conversions[BRAKE_1_CHANNEL].conversion;
    ADC2Data::instance().brake_2 = adc2.data.conversions[BRAKE_2_CHANNEL].conversion;

    return ADC2Data::instance();
}

float ADCInterface::iir_filter(float alpha, float prev_value, float new_value)
{
    return (alpha * new_value) + (1 - alpha) * (prev_value);
}