#include <math.h>
#include "PedalsSystem.h"



float PedalsSystem:: _pedal_percentage(float pedal1val, float pedal2val, const PedalsParams& params)
{
    float pedal1percent = fabs((pedal1val - static_cast<float>(params.min_pedal_1)))/fabs(static_cast<float>(params.max_pedal_1 - (params.min_pedal_1)));
    float pedal2percent = fabs((pedal2val) - static_cast<float>(params.min_pedal_2))/fabs(static_cast<float>(params.max_pedal_2 - params.min_pedal_2));
    const float divider = 2.0;
    return (pedal1percent + pedal2percent)/divider;
}


PedalsSystemData_s PedalsSystem::evaluate_pedals(PedalSensorData_s pedals_data, unsigned long curr_millis)
{
    PedalsSystemData_s out = {};
    int accel_1 = static_cast<int>(pedals_data.accel_1); 
    int accel_2 = static_cast<int>(pedals_data.accel_2);
    int brake_1 = static_cast<int>(pedals_data.brake_1);
    int brake_2 = static_cast<int>(pedals_data.brake_1); // Copying Brake1 data into brake2 as we don't need 2 brakes anymore


    float _accel1_scaled_ = (fabs(static_cast<float>(accel_1 - std::min(_accelParams.min_pedal_1,_accelParams.max_pedal_1)))/fabs(static_cast<float>(_accelParams.max_pedal_1 - _accelParams.min_pedal_1)));
    float _accel2_scaled_ = (fabs(static_cast<float>(accel_2 - std::min(_accelParams.min_pedal_2,_accelParams.max_pedal_2)))/fabs(static_cast<float>(_accelParams.max_pedal_2 - _accelParams.min_pedal_2)));
    float _brake1_scaled_ = (fabs(static_cast<float>(brake_1 - std::min(_brakeParams.min_pedal_1,_brakeParams.max_pedal_1)))/fabs(static_cast<float>(_brakeParams.max_pedal_1 - _brakeParams.min_pedal_1)));
    float _brake2_scaled_ = _brake1_scaled_;

    // FSAE Rules T.4.2.4
    const float _implausibility = 0.1;
    out.brake_is_implausible = _evaluate_pedal_implausibilities(brake_1,brake_2,_brakeParams, 1.0);
    out.brake_is_implausible = _evaluate_pedal_implausibilities(brake_1,brake_2,_brakeParams,1.0);
    out.accel_is_pressed = _pedal_is_active(_accel1_scaled_, _accel2_scaled_, _accelParams, false);
    out.brake_is_pressed = _pedal_is_active( _brake1_scaled_, _brake2_scaled_,_brakeParams,false);
    out.accel_is_implausible = _evaluate_pedal_implausibilities(accel_1, accel_2, _accelParams, _implausibility);
    out.accel_is_implausible = _evaluate_pedal_implausibilities(accel_1, accel_2, _accelParams, IMPLAUSIBILITY_PERCENT);


    out.brake_and_accel_pressed_implausibility_high = _evaluate_brake_and_accel_pressed(pedals_data);
    float accel_percent = (out.accel_is_implausible) ? _accel1_scaled_ : _pedal_percentage(static_cast<float>(accel_1),static_cast<float>(accel_2),_accelParams); 
    
    out.accel_percent = _remove_deadzone(accel_percent, _accelParams.deadzone_margin);
    out.accel_percent = std::max(out.accel_percent, 0.0f);
    float brake_percent = (out.brake_is_implausible) ? _brake1_scaled_ : _pedal_percentage(static_cast<float>(brake_1),static_cast<float>(brake_2),_brakeParams); 
    out.brake_percent = _remove_deadzone(brake_percent, _brakeParams.deadzone_margin);
    out.brake_percent = std::max(out.brake_percent, 0.0f);
    bool implausibility = (out.accel_is_implausible || out.brake_and_accel_pressed_implausibility_high || out.brake_is_implausible);
    if (implausibility && (_implausibilityStartTime ==0)){
        _implausibilityStartTime = curr_millis;
    }
    else if ((!implausibility) && ((out.accel_percent <= ACCELERATION_PERCENT_LIMIT))){
        _implausibilityStartTime = 0;
    }
    bool oor = implausibility && (_evaluate_pedal_oor(accel_1, _accelParams.min_sensor_pedal_1, _accelParams.max_sensor_pedal_1)
                                 || _evaluate_pedal_oor(accel_2, _accelParams.min_sensor_pedal_2, _accelParams.max_sensor_pedal_2));
    out.accel_percent = (oor) ? 0 : out.accel_percent;
    out.brake_percent = (oor) ? 0 : out.brake_percent;
    out.mech_brake_is_active = out.brake_percent >= _brakeParams.mechanical_activation_percentage;
    out.implausibility_has_exceeded_max_duration = _max_duration_of_implausibility_exceeded(curr_millis);
    return out;
}

bool PedalsSystem::_max_duration_of_implausibility_exceeded(unsigned long curr_millis)
{

    if (_implausibilityStartTime != 0)
    {
        return ((curr_millis - _implausibilityStartTime) > IMPLAUSIBILITY_DURATION);
    }
    else
    {
        return false;
    }
}

bool PedalsSystem::_evaluate_pedal_implausibilities(int pedal_data1_analog, int pedal_data2_analog, const PedalsParams &params, float max_percent_diff){
    bool pedal1_min_max_implaus = _evaluate_min_max_pedal_implausibilities(pedal_data1_analog, params.min_pedal_1, params.max_pedal_1, params.implausibility_margin);
    bool pedal2_min_max_implaus = _evaluate_min_max_pedal_implausibilities(pedal_data2_analog, params.min_pedal_2, params.max_pedal_2, params.implausibility_margin);
    float pedal1_scaled = (fabs((static_cast<float>(pedal_data1_analog - params.min_pedal_1)) / fabs(static_cast<float>(params.max_pedal_1 - params.min_pedal_1))));
    float pedal2_scaled = (fabs((static_cast<float>(pedal_data2_analog - params.min_pedal_2)) / fabs(static_cast<float>(params.max_pedal_2 - params.min_pedal_2))));
    bool sens_not_within_req_percent = ((fabs(pedal1_scaled - pedal2_scaled)) > max_percent_diff); // DIVIDE BY 100
    return pedal1_min_max_implaus || pedal2_min_max_implaus || sens_not_within_req_percent;
}

bool PedalsSystem::_evaluate_min_max_pedal_implausibilities(int pedal_data, int min, int max, float implaus_margin_scale){
    bool pedal_swapped = false;
    float pedal_margin = static_cast<float>(::abs(max-min)) * implaus_margin_scale;
    if(min>max){
        pedal_swapped = true;
    }
    
    // FSAE EV.5.5
    // FSAE T.4.2.10
    float float_pedal_data = static_cast<float>(pedal_data);
    float min_float = static_cast<float>(min);
    float max_float = static_cast<float>(max);
    bool pedal_less_than_min = pedal_swapped ? (float_pedal_data > (min_float+pedal_margin)) : (float_pedal_data < (min_float-pedal_margin)); 
    bool pedal_greater_than_max = pedal_swapped ? (float_pedal_data < (max_float-pedal_margin)) : (float_pedal_data > (max_float+pedal_margin)); 
    return pedal_less_than_min || pedal_greater_than_max;

}

bool PedalsSystem::_pedal_is_active(float pedal1ScaledData, float pedal2ScaledData, const PedalsParams& params, bool check_mech_activation)
{
    float val1_deadzone_removed = _remove_deadzone(pedal1ScaledData, params.deadzone_margin);
    float val2_deadzone_removed = _remove_deadzone(pedal2ScaledData, params.deadzone_margin);
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

float PedalsSystem::_remove_deadzone(float conversion_input, float deadzone)
{
    // Your conversion input is basically pedal data over abs(max-min) to get it betwen 0-1. Then deadzone is removed from it. 
    const float onner = 1.0;
    float range = onner - (deadzone * 2);
    // e.g. vals from 0 to 1, deadzone is .05, range is .9
    // subtract deadzone to be -.05 to .95 & clamp at 0
    float out = std::max(conversion_input - deadzone, 0.0f); // max(1010 - 0.03, 0)
    // values now are 0 to .95
    // divide by range of values to scale up (.9)
    out /= range; // 1074.43617
    // values are now 0 to 1.0555...
    // clamp at 0 to 1
    out = std::min(out, 1.0f); // min(out, 1)

    return out;
}

bool PedalsSystem::_evaluate_pedal_oor(int pedal_data, int min, int max){
    return(pedal_data<=min || pedal_data>=max);
}

bool PedalsSystem::_evaluate_brake_and_accel_pressed(PedalSensorData_s & pedals_data){
    int accel_1 = static_cast<int>(pedals_data.accel_1); 
    int accel_2 = static_cast<int>(pedals_data.accel_2);
    int brake_1 = static_cast<int>(pedals_data.brake_1);
    int brake_2 = static_cast<int>(pedals_data.brake_1); 
    float _accel1_scaled_ = (fabs(static_cast<float>(accel_1 - std::min(_accelParams.min_pedal_1,_accelParams.max_pedal_1)))/fabs(static_cast<float>(_accelParams.max_pedal_1 - _accelParams.min_pedal_1)));
    float _accel2_scaled_ = (fabs(static_cast<float>(accel_2 - std::min(_accelParams.min_pedal_2,_accelParams.max_pedal_2)))/fabs(static_cast<float>(_accelParams.max_pedal_2 - _accelParams.min_pedal_2)));
    float _brake1_scaled_ = (fabs(static_cast<float>(brake_1 - std::min(_brakeParams.min_pedal_1,_brakeParams.max_pedal_1)))/fabs(static_cast<float>(_brakeParams.max_pedal_1 - _brakeParams.min_pedal_1)));
    float _brake2_scaled_ = _brake1_scaled_;
    bool accel_pressed = _pedal_is_active(_accel1_scaled_, _accel2_scaled_, _accelParams, false);
    bool mech_brake_pressed = _pedal_is_active(_brake1_scaled_, _brake2_scaled_,_brakeParams,true);
    bool both_pedals_implausible = (accel_pressed && mech_brake_pressed);
    return both_pedals_implausible;
}