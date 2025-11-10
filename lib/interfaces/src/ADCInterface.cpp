#include "ADCInterface.h"

std::array<float, adc_default_parameters::channels_within_mcp_adc> ADCInterface::adc1_scales() {
  std::array<float, adc_default_parameters::channels_within_mcp_adc> scales = {};

  scales.at(_adc_parameters.channels.steering_1_channel)    = _adc_parameters.scales.steering_1_scale; 
  scales.at(_adc_parameters.channels.steering_2_channel)    = _adc_parameters.scales.steering_2_scale;
  scales.at(_adc_parameters.channels.fr_loadcell_channel)   = _adc_parameters.scales.fr_loadcell_scale;
  scales.at(_adc_parameters.channels.fl_suspot_channel)     = _adc_parameters.scales.fl_loadcell_scale;
  scales.at(_adc_parameters.channels.fr_suspot_channel)     = _adc_parameters.scales.fr_suspot_scale;
  scales.at(_adc_parameters.channels.fl_suspot_channel)     = _adc_parameters.scales.fl_suspot_scale;
  
  return scales;
}

std::array<float, adc_default_parameters::channels_within_mcp_adc> ADCInterface::adc1_offsets() {
  std::array<float, adc_default_parameters::channels_within_mcp_adc> offsets = {};

  offsets.at(_adc_parameters.channels.steering_1_channel)   = _adc_parameters.offsets.steering_1_offset; 
  offsets.at(_adc_parameters.channels.steering_2_channel)   = _adc_parameters.offsets.steering_2_offset;
  offsets.at(_adc_parameters.channels.fr_loadcell_channel)  = _adc_parameters.offsets.fr_loadcell_offset;
  offsets.at(_adc_parameters.channels.fl_suspot_channel)    = _adc_parameters.offsets.fl_loadcell_offset;
  offsets.at(_adc_parameters.channels.fr_suspot_channel)    = _adc_parameters.offsets.fr_suspot_offset;
  offsets.at(_adc_parameters.channels.fl_suspot_channel)    = _adc_parameters.offsets.fl_suspot_offset;
  
  return offsets;
}

std::array<float, adc_default_parameters::channels_within_mcp_adc> ADCInterface::adc2_scales() {
  std::array<float, adc_default_parameters::channels_within_mcp_adc> scales = {};
  
  scales.at(_adc_parameters.channels.accel_1_channel)   = _adc_parameters.scales.accel_1_scale; 
  scales.at(_adc_parameters.channels.accel_2_channel)   = _adc_parameters.scales.accel_2_scale;
  scales.at(_adc_parameters.channels.brake_1_channel)   = _adc_parameters.scales.brake_1_scale;
  scales.at(_adc_parameters.channels.brake_2_channel)   = _adc_parameters.scales.brake_2_scale;
   
  return scales;
}

std::array<float, adc_default_parameters::channels_within_mcp_adc> ADCInterface::adc2_offsets() {
  std::array<float, adc_default_parameters::channels_within_mcp_adc> offsets = {};

  offsets.at(_adc_parameters.channels.accel_1_channel)  = _adc_parameters.offsets.accel_1_offset; 
  offsets.at(_adc_parameters.channels.accel_2_channel)  = _adc_parameters.offsets.accel_2_offset;
  offsets.at(_adc_parameters.channels.brake_1_channel)  = _adc_parameters.offsets.brake_1_offset;
  offsets.at(_adc_parameters.channels.brake_2_channel)  = _adc_parameters.offsets.brake_2_offset;
  
  return offsets;
}

void ADCInterface::adc1_tick() {
    _adc1.tick();
}

void ADCInterface::adc2_tick() {
    _adc2.tick();
} 

/* ------ ADC 1 ------ */
AnalogConversion_s ADCInterface::steering_degrees_1() {
    return _adc1.data.conversions[_adc_parameters.channels.steering_1_channel];
}

AnalogConversion_s ADCInterface::steering_degrees_2() {
    return _adc1.data.conversions[_adc_parameters.channels.steering_2_channel];
}

AnalogConversion_s ADCInterface::FL_load_cell() {
    return _adc1.data.conversions[_adc_parameters.channels.fl_loadcell_channel];
}

AnalogConversion_s ADCInterface::FR_load_cell() {
    return _adc1.data.conversions[_adc_parameters.channels.fl_loadcell_channel];
}

AnalogConversion_s ADCInterface::FL_sus_pot() {
    return _adc1.data.conversions[_adc_parameters.channels.fl_suspot_channel];
}

AnalogConversion_s ADCInterface::FR_sus_pot() {
    return _adc1.data.conversions[_adc_parameters.channels.fr_suspot_channel];
}

/* ------ ADC 2 ------ */
AnalogConversion_s ADCInterface::acceleration_1() {
    return _adc2.data.conversions[_adc_parameters.channels.accel_1_channel];
}

AnalogConversion_s ADCInterface::acceleration_2() {
    return _adc2.data.conversions[_adc_parameters.channels.accel_2_channel];
}

AnalogConversion_s ADCInterface::brake_1() {
    return _adc2.data.conversions[_adc_parameters.channels.brake_1_channel];
}

AnalogConversion_s ADCInterface::brake_2() {
    return _adc2.data.conversions[_adc_parameters.channels.brake_2_channel];
}

float ADCInterface::iir_filter(float alpha, float prev_value, float new_value)
{
    return (alpha * new_value) + (1 - alpha) * (prev_value);
}