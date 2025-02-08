#include <math.h>
#include <tuple>
#include "PedalsSystem.h"

// VCF interface/system data - has all pedals data - global. Analog readings of 4 pedals sensors - all data that is in pedal system data
// Accel,brake, regen percent = know what each reps - use last years function - tic/update pedal system function. take in reference to vcf interface data
// analog value into the pedal systems stuff. 
// apps - 2 acceleration position sensors. they both have opposite slopes. determine what is positive and negative slope. Linear interpolation. 
//get rid of adc

float PedalsSystem::_scale_pedal_val(int raw_pedal_val, int min, int max)
{
    return static_cast<float>(raw_pedal_val)/static_cast<float>(max-min);
}

void PedalsSystem::tick(unsigned long curr_millis, PedalSensorData_s & pedals_data)
{
    _data = evaluate_pedals(pedals_data, curr_millis);
}
// method to evaluate pedals
PedalsSystemData_s PedalsSystem::evaluate_pedals(PedalSensorData_s pedals_data, unsigned long curr_millis)
{
    PedalsSystemData_s out = {};
    int accel_1 = pedals_data.accel_1; 
    int accel_2 = pedals_data.accel_2;
    int brake_1 = pedals_data.brake_1;
    int brake_2 = pedals_data.brake_2; 
    float _accel1_scaled_ = _scale_pedal_val(accel_1,_accelParams.min_pedal_1, _accelParams.max_pedal_1);
    float _accel2_scaled_ = _scale_pedal_val(accel_2,_accelParams.min_pedal_2, _accelParams.max_pedal_2);
    float _brake1_scaled_ = _scale_pedal_val(brake_1,_brakeParams.min_pedal_1,_brakeParams.max_pedal_1);
    float _brake2_scaled_ = _scale_pedal_val(brake_2,_brakeParams.min_pedal_2,_brakeParams.max_pedal_2);
    const float _implausibility = 0.1;
    out.brake_is_implausible = evaluate_pedal_implausibilities_(brake_1,brake_2,_brakeParams,_implausibility);
    const float _halfer = 2.0;
    out.accel_is_pressed = pedal_is_active_(_accel1_scaled_,_accel2_scaled_, _accelParams, false);
    out.brake_is_pressed = pedal_is_active_(_brake1_scaled_,_brake2_scaled_,_brakeParams,false);
    out.accel_is_implausible = evaluate_pedal_implausibilities_(accel_1, accel_2, _accelParams, _implausibility);
    out.brake_is_implausible = evaluate_pedal_implausibilities_(brake_1,brake_2,_brakeParams,_implausibility);
    out.brake_and_accel_pressed_implausibility_high = evaluate_brake_and_accel_pressed_(pedals_data);
    auto accel_percent = (out.accel_is_implausible) ? _accel1_scaled_ : (static_cast<float>(_accel1_scaled_ + _accel2_scaled_))/_halfer; // yeah this one too
    out.accel_percent = remove_deadzone_(accel_percent, _accelParams.deadzone_margin);
    out.accel_percent = std::max(out.accel_percent, 0.0f);
    auto brake_percent = (out.brake_is_implausible) ? _brake1_scaled_ : (static_cast<float>(_brake1_scaled_ + _brake2_scaled_))/_halfer; // whats up with the conditional?
    out.brake_percent = remove_deadzone_(brake_percent, _brakeParams.deadzone_margin);
    out.brake_percent = std::max(out.brake_percent, 0.0f);
    bool implausibility = (out.accel_is_implausible || out.brake_and_accel_pressed_implausibility_high || out.brake_is_implausible);
    const float accel_perc_lim = 0.05;
    if (implausibility && (_implausibilityStartTime ==0)){
        _implausibilityStartTime = curr_millis;
    }
    else if ((!implausibility) && ((out.accel_percent <= accel_perc_lim))){
        _implausibilityStartTime = 0;
    }
    bool oor = implausibility && (evaluate_pedal_oor(accel_1, _accelParams.min_pedal_1, _accelParams.max_pedal_1)
                                 || evaluate_pedal_oor(accel_2, _accelParams.min_pedal_2, _accelParams.max_pedal_2));
    out.accel_percent = (oor) ? 0 : out.accel_percent;
    out.brake_percent = (oor) ? 0 : out.brake_percent;
    out.mech_brake_is_active = out.brake_percent >= _brakeParams.mechanical_activation_percentage;
    out.regen_percent = std::max(std::min(out.brake_percent/_brakeParams.mechanical_activation_percentage, 1.0f), 0.0f);
    out.implausibility_has_exceeded_max_duration = max_duration_of_implausibility_exceeded_(curr_millis);
    return out;
}

bool PedalsSystem::max_duration_of_implausibility_exceeded_(unsigned long curr_millis)
{
    const int implausibility_duration = 100;
    if (_implausibilityStartTime != 0)
    {
        return ((curr_millis - _implausibilityStartTime) > implausibility_duration);
    }
    else
    {
        return false;
    }
}

bool PedalsSystem::evaluate_pedal_implausibilities_(int pedal_data1_analog, int pedal_data2_analog, const PedalsParams &params, float max_percent_diff){
    bool pedal1_min_max_implaus = evaluate_min_max_pedal_implausibilities_(pedal_data1_analog, params.min_pedal_1, params.max_pedal_1, params.implausibility_margin);
    bool pedal2_min_max_implaus = evaluate_min_max_pedal_implausibilities_(pedal_data2_analog, params.min_pedal_2, params.max_pedal_2, params.implausibility_margin);
    bool sens_not_within_req_percent = (fabs(pedal_data1_analog - pedal_data2_analog) > max_percent_diff);
    if (pedal1_min_max_implaus || pedal2_min_max_implaus)
    {
        return true;
    }
    else if (sens_not_within_req_percent)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool PedalsSystem::evaluate_min_max_pedal_implausibilities_(int pedal_data, int min, int max, float implaus_margin_scale){
    bool pedal_swapped = false;
    float pedal_margin = static_cast<float>(::abs(max-min)) * implaus_margin_scale;
    if(min>max){
        pedal_swapped = true;
    }
    // FSAE EV.5.5
    // FSAE T.4.2.10
    float float_pedal_data = float(pedal_data);
    float min_float = static_cast<float>(min);
    float max_float = static_cast<float>(max);
    bool pedal_less_than_min = pedal_swapped ? (float_pedal_data > (min_float+pedal_margin)) : (float_pedal_data < (max_float-pedal_margin));
    bool pedal_greater_than_max = pedal_swapped ? (float_pedal_data < (max_float-pedal_margin)) : (float_pedal_data > (min_float+pedal_margin));
    if (pedal_less_than_min)
    {
        return true;
    }
    else if (pedal_greater_than_max)
    {
        return true;
    }
    else
    {
        return false;
    }

}

bool PedalsSystem::pedal_is_active_(float pedal1ScaledData, float pedal2ScaledData, const PedalsParams& params, bool check_mech_activation)
{
    float val1_deadzone_removed = remove_deadzone_(pedal1ScaledData, params.deadzone_margin);
    float val2_deadzone_removed = remove_deadzone_(pedal2ScaledData, params.deadzone_margin);
    bool pedal_1_is_active = false; 
    bool pedal_2_is_active = false; 
    if(check_mech_activation)
    {
        pedal_1_is_active = val1_deadzone_removed >= params.mechanical_activation_percentage;
        pedal_2_is_active = val2_deadzone_removed >= params.mechanical_activation_percentage;
    } else {
        pedal_1_is_active = val1_deadzone_removed >= params.activation_percentage;
        pedal_2_is_active = val2_deadzone_removed >= params.activation_percentage;
    }
    return (pedal_1_is_active || pedal_2_is_active);
}

float PedalsSystem::remove_deadzone_(float conversion_input, float deadzone)
{
    const float onner = 1.0;
    float range = onner - (deadzone * 2);
    // e.g. vals from 0 to 1, deadzone is .05, range is .1
    // subtract deadzone to be -.05 to .95 & clamp at 0
    float out = std::max(conversion_input - deadzone, 0.0f);
    // values now are 0 to .95
    // divide by range of values to scale up (.9)
    out /= range;
    // values are now 0 to 1.0555...
    // clamp at 0 to 1
    out = std::min(out, 1.0f);

    return out;
}

bool PedalsSystem::evaluate_pedal_oor (int pedal_data, int min, int max){
    return(pedal_data<=min || pedal_data>=max);
}

//converts data by dividing it by 4096
bool PedalsSystem::evaluate_brake_and_accel_pressed_(PedalSensorData_s & pedals_data){
    int accel_1 = pedals_data.accel_1; 
    int accel_2 = pedals_data.accel_2;
    int brake_1 = pedals_data.brake_1;
    int brake_2 = pedals_data.brake_2; 
    float _accel1_scaled_ = _scale_pedal_val(accel_1,_accelParams.min_pedal_1, _accelParams.max_pedal_1);
    float _accel2_scaled_ = _scale_pedal_val(accel_2,_accelParams.min_pedal_2, _accelParams.max_pedal_2);
    float _brake1_scaled_ = _scale_pedal_val(brake_1,_brakeParams.min_pedal_1,_brakeParams.max_pedal_1);
    float _brake2_scaled_ = _scale_pedal_val(brake_2,_brakeParams.min_pedal_2,_brakeParams.max_pedal_2);
    bool accel_pressed = pedal_is_active_(_accel1_scaled_, _accel2_scaled_, _accelParams, false);
    bool mech_brake_pressed = pedal_is_active_(_brake1_scaled_,_brake2_scaled_,_brakeParams,true);
    bool both_pedals_implausible = (accel_pressed && mech_brake_pressed);
    return both_pedals_implausible;
}