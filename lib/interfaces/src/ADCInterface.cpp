#include "ADCInterface.h"


std::array<float, adc_default_parameters::channels_within_mcp_adc> ADCInterface::adc1_scales() {
  std::array<float, adc_default_parameters::channels_within_mcp_adc> scales = {};

  scales[_adc_parameters.channels.steering_cw_channel]   = _adc_parameters.scales.steering_cw_scale; 
  scales[_adc_parameters.channels.steering_ccw_channel]  = _adc_parameters.scales.steering_ccw_scale;
  scales[_adc_parameters.channels.fr_loadcell_channel]   = _adc_parameters.scales.fr_loadcell_scale;
  scales[_adc_parameters.channels.fl_loadcell_channel]     = _adc_parameters.scales.fl_loadcell_scale;
  scales[_adc_parameters.channels.fr_suspot_channel]     = _adc_parameters.scales.fr_suspot_scale;
  scales[_adc_parameters.channels.fl_suspot_channel]     = _adc_parameters.scales.fl_suspot_scale;
  
  return scales;
}

std::array<float, adc_default_parameters::channels_within_mcp_adc> ADCInterface::adc1_offsets() {
  std::array<float, adc_default_parameters::channels_within_mcp_adc> offsets = {};

  offsets[_adc_parameters.channels.steering_cw_channel]  = _adc_parameters.offsets.steering_cw_offset; 
  offsets[_adc_parameters.channels.steering_ccw_channel] = _adc_parameters.offsets.steering_ccw_offset;
  offsets[_adc_parameters.channels.fr_loadcell_channel]  = _adc_parameters.offsets.fr_loadcell_offset;
  offsets[_adc_parameters.channels.fl_loadcell_channel]    = _adc_parameters.offsets.fl_loadcell_offset;
  offsets[_adc_parameters.channels.fr_suspot_channel]    = _adc_parameters.offsets.fr_suspot_offset;
  offsets[_adc_parameters.channels.fl_suspot_channel]    = _adc_parameters.offsets.fl_suspot_offset;
  
  return offsets;
}

std::array<float, adc_default_parameters::channels_within_mcp_adc> ADCInterface::adc2_scales() {
  std::array<float, adc_default_parameters::channels_within_mcp_adc> scales = {};
  
  scales[_adc_parameters.channels.accel_1_channel]   = _adc_parameters.scales.accel_1_scale; 
  scales[_adc_parameters.channels.accel_2_channel]   = _adc_parameters.scales.accel_2_scale;
  scales[_adc_parameters.channels.brake_1_channel]   = _adc_parameters.scales.brake_1_scale;
  scales[_adc_parameters.channels.brake_2_channel]   = _adc_parameters.scales.brake_2_scale;
   
  return scales;
}

std::array<float, adc_default_parameters::channels_within_mcp_adc> ADCInterface::adc2_offsets() {
  std::array<float, adc_default_parameters::channels_within_mcp_adc> offsets = {};

  offsets[_adc_parameters.channels.accel_1_channel]  = _adc_parameters.offsets.accel_1_offset; 
  offsets[_adc_parameters.channels.accel_2_channel]  = _adc_parameters.offsets.accel_2_offset;
  offsets[_adc_parameters.channels.brake_1_channel]  = _adc_parameters.offsets.brake_1_offset;
  offsets[_adc_parameters.channels.brake_2_channel]  = _adc_parameters.offsets.brake_2_offset;
  
  return offsets;
}

float ADCInterface::iir_filter(float alpha, float prev_value, float new_value)
{
    return (alpha * new_value) + (1 - alpha) * (prev_value);
}



/* -------------------- ADC 1 Functions -------------------- */
void ADCInterface::adc1_tick() {
    _adc1.tick();
}

AnalogConversion_s ADCInterface::steering_degrees_cw() {
    return _adc1.data.conversions[_adc_parameters.channels.steering_cw_channel];
}

AnalogConversion_s ADCInterface::steering_degrees_ccw() {
    return _adc1.data.conversions[_adc_parameters.channels.steering_ccw_channel];
}

AnalogConversion_s ADCInterface::FL_load_cell() {
    return _adc1.data.conversions[_adc_parameters.channels.fl_loadcell_channel];
}

AnalogConversion_s ADCInterface::FR_load_cell() {
    return _adc1.data.conversions[_adc_parameters.channels.fr_loadcell_channel];
}

AnalogConversion_s ADCInterface::FL_sus_pot() {
    return _adc1.data.conversions[_adc_parameters.channels.fl_suspot_channel];
}

AnalogConversion_s ADCInterface::FR_sus_pot() {
    return _adc1.data.conversions[_adc_parameters.channels.fr_suspot_channel];
}

float ADCInterface::get_filtered_FL_load_cell() {
    return _filteredValues.FL_loadcell_analog;
}

float ADCInterface::get_filtered_FR_load_cell() {
    return _filteredValues.FR_loadcell_analog;
}

float ADCInterface::get_filtered_FL_sus_pot() {
    return _filteredValues.FL_sus_pot_analog;
}

float ADCInterface::get_filtered_FR_sus_pot() {
    return _filteredValues.FR_sus_pot_analog;
}

void ADCInterface::update_filtered_values(float alpha) {
    _filteredValues.FL_loadcell_analog = iir_filter(
        alpha,
        _filteredValues.FL_loadcell_analog,
        FL_load_cell().conversion
    );
    _filteredValues.FR_loadcell_analog = iir_filter(
        alpha,
        _filteredValues.FR_loadcell_analog,
        FR_load_cell().conversion
    );
    _filteredValues.FL_loadcell_analog = iir_filter(
        alpha,
        _filteredValues.FL_sus_pot_analog,
        FL_sus_pot().conversion
    );
    _filteredValues.FL_loadcell_analog = iir_filter(
        alpha,
        _filteredValues.FR_sus_pot_analog,
        FR_sus_pot().conversion
    );
}

// void ADCInterface::set_filtered_FR_load_cell() {
//     _filteredValues.FR_loadcell_analog = iir_filter(
//         VCFTaskConstants::LOADCELL_IIR_FILTER_ALPHA,
//         _filteredValues.FR_loadcell_analog,
//         FR_load_cell().conversion
//     );
// }

// void ADCInterface::set_filtered_FL_sus_pot() {
//     _filteredValues.FL_loadcell_analog = iir_filter(
//         VCFTaskConstants::LOADCELL_IIR_FILTER_ALPHA,
//         _filteredValues.FL_sus_pot_analog,
//         FL_sus_pot().conversion
//     );
// }

// void ADCInterface::set_filtered_FR_sus_pot() {
//     _filteredValues.FL_loadcell_analog = iir_filter(
//         VCFTaskConstants::LOADCELL_IIR_FILTER_ALPHA,
//         _filteredValues.FR_sus_pot_analog,
//         FR_sus_pot().conversion
//     );
// }

float filtered_FR_sus_pot();

/* -------------------- ADC 2 Functions -------------------- */
void ADCInterface::adc2_tick() {
    _adc2.tick();
} 

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