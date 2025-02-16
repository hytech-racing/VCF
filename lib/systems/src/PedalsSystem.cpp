#include <math.h>
#include <tuple>
#include "PedalsSystem.h"

// VCF interface/system data - has all pedals data - global. Analog readings of 4 pedals sensors - all data that is in pedal system data
// Accel,brake, regen percent = know what each reps - use last years function - tic/update pedal system function. take in reference to vcf interface data
// analog value into the pedal systems stuff. 
// apps - 2 acceleration position sensors. they both have opposite slopes. determine what is positive and negative slope. Linear interpolation. 
//get rid of adc

void PedalsSystem::tick(unsigned long curr_millis, VCFInterfaceData_s & interface_data, bool use_both_brake_sensors)
{
    _data = evaluate_pedals(interface_data.pedals_data, curr_millis, use_both_brake_sensors);
}

//converts the paramater needed - eg. accel1, accel2, brake1, or brake2. takes in int32, returns int32. 
// the data inputted should be a raw reading- from a PedalSensorData_s reference
uint32_t conversion(PedalSensorData_s pedals_data, uint32_t data){
    if(data == pedals_data.accel_1){
        return pedals_data.accel_1;
    } else if (data == pedals_data.accel_2){
        return pedals_data.accel_2; 
    } else if (data == pedals_data.brake_1){
        return pedals_data.brake_1;
    } else if (data == pedals_data.brake_2){
        return pedals_data.brake_2;
    }
}
//convert accel to float
//What this is doing - analogConversion_s object reference given
// to accel1, accel2, brake1, brake2. This, instead of being converted from 0-1, can be processed raw. 
// Take in a reference to PedalsSystem_data struct - and obtain the analog values from there. Utilize helper methods to convert. 

PedalsSystemData_s PedalsSystem::evaluate_pedals(PedalSensorData_s pedals_data, unsigned long curr_millis, bool twobrakes)
{
    PedalsSystemData_s out = {};
    int accel_1 = conversion(pedals_data, pedals_data.accel_1);
    int accel_2 = conversion(pedals_data, pedals_data.accel_2);
    int brake_1 = conversion(pedals_data, pedals_data.brake_1);
    int brake_2 = conversion(pedals_data, pedals_data.brake_2);
    printf("accel1: %d, accel2: %d, brake1: %d, brake2: %d\n", accel_1, accel_2, brake_1, brake_2);
    out.accel_is_pressed = pedal_is_active_(accel_1,accel_2, _accelParams, false);
    out.accel_is_implausible = evaluate_pedal_implausibilities_(accel_1, accel_2, _accelParams, 0.1);
    float pedal1ScaledData = (float) (accel_1) / abs(_accelParams.max_sensor_pedal_1 - _accelParams.min_sensor_pedal_1);
    float pedal2ScaledData = (float) (accel_2) / abs(_accelParams.max_sensor_pedal_2 - _accelParams.min_sensor_pedal_2);
    float break1ScaledData = (float) (brake_1) / abs(_brakeParams.max_sensor_pedal_1 - _brakeParams.min_sensor_pedal_1);
    float break2ScaledData = (float) (brake_2) / abs(_brakeParams.max_sensor_pedal_2 - _brakeParams.min_sensor_pedal_2);
    auto percent = (out.accel_is_implausible) ? pedal1ScaledData : (pedal1ScaledData + pedal2ScaledData) / 2.0;
    percent /= 100.0;
    out.accel_percent = remove_deadzone_(percent, _accelParams.deadzone_margin);
    out.accel_percent = std::max(out.accel_percent, 0.0f);
    if(twobrakes){
        out.brake_is_implausible = evaluate_pedal_implausibilities_(brake_1,brake_2,_brakeParams, 0.1);
        out.brake_and_accel_pressed_implausibility_high = evaluate_brake_and_accel_pressed_(accel_1, accel_2, brake_1, brake_2);
        out.brake_percent = (brake_1 + brake_2) / 2;
        out.brake_is_pressed = pedal_is_active_(brake_1, brake_2, _brakeParams, false);
    } else{
        out.brake_is_implausible = evaluate_pedal_implausibilities_(break1ScaledData, _brakeParams);
        out.brake_and_accel_pressed_implausibility_high = evaluate_brake_and_accel_pressed_(pedal1ScaledData, pedal2ScaledData, break1ScaledData);
        out.brake_percent = break1ScaledData/2;
        out.brake_is_pressed = pedal_is_active_(break1ScaledData, _brakeParams, false);
    }
    bool implausibility = (out.accel_is_implausible || out.brake_and_accel_pressed_implausibility_high || out.brake_is_implausible);
    printf("accel_is_implausible: %d\n", out.accel_is_implausible);
    printf("brake_and_accel_pressed_implausibility_high: %d\n", out.brake_and_accel_pressed_implausibility_high);
    printf("brake_is_implausible: %d\n", out.brake_is_implausible);
    printf("implausibility: %d\n", implausibility);
    printf("Accel percent: %f\n", out.accel_percent);
    printf("Implaus Start Before If: %d\n", _implausibilityStartTime);
    if (implausibility && (_implausibilityStartTime ==0)){
        printf("start implausibility\n");
        _implausibilityStartTime = curr_millis;
    }
    else if ((!implausibility) && ((out.accel_percent <= 0.05))){
        printf("end implausibility\n");
        _implausibilityStartTime = 0;
    }
    printf("Implaus Start After If: %d\n", _implausibilityStartTime);
    bool oor = implausibility && (evaluate_pedal_oor(accel_1, _accelParams.min_pedal_1, _accelParams.max_pedal_1)
                                 || evaluate_pedal_oor(accel_2, _accelParams.min_pedal_2, _accelParams.max_pedal_2));
    out.accel_percent = (oor) ? 0 : out.accel_percent;
    out.brake_percent = remove_deadzone_(out.brake_percent, _brakeParams.deadzone_margin);
    out.mech_brake_is_active = out.brake_percent >= _brakeParams.mechanical_activation_percentage;
    out.regen_percent = std::max(std::min(out.brake_percent/_brakeParams.mechanical_activation_percentage, 1.0f), 0.0f);
    out.implausibility_has_exceeded_max_duration = max_duration_of_implausibility_exceeded_(curr_millis);
    return out;
}

bool PedalsSystem::max_duration_of_implausibility_exceeded_(unsigned long curr_millis)
{
    if (_implausibilityStartTime != 0)
    {
        return ((curr_millis - _implausibilityStartTime) > 100);
    }
    else
    {
        return false;
    }
}

bool PedalsSystem::evaluate_pedal_implausibilities_(int pedal_data1_analog, int pedal_data2_analog, const PedalsParams &params, float max_percent_diff){
    bool pedal1_min_max_implaus = evaluate_min_max_pedal_implausibilities_(pedal_data1_analog, params.min_pedal_1, params.max_pedal_1, params.implausibility_margin);
    bool pedal2_min_max_implaus = evaluate_min_max_pedal_implausibilities_(pedal_data2_analog, params.min_pedal_2, params.max_pedal_2, params.implausibility_margin);
    bool sens_not_within_req_percent = ((fabs(pedal_data1_analog - pedal_data2_analog))/100.0 > max_percent_diff);
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

bool PedalsSystem::evaluate_pedal_implausibilities_(int pedal_data1_analog, const PedalsParams &params){
    return evaluate_min_max_pedal_implausibilities_(pedal_data1_analog, params.min_pedal_1, params.max_pedal_1, params.implausibility_margin);
}



bool PedalsSystem::evaluate_min_max_pedal_implausibilities_(int pedal_data, int min, int max, float implaus_margin_scale){
    bool pedal_swapped = false;
    int pedal_margin = abs(max-min) * implaus_margin_scale;
    if(min>max){
        pedal_swapped = true;
    }
    // FSAE EV.5.5
    // FSAE T.4.2.10
    bool pedal_less_than_min = pedal_swapped ? (pedal_data > (min+pedal_margin)) : (pedal_data < (min-pedal_margin));
    bool pedal_greater_than_max = pedal_swapped ? (pedal_data < (max-pedal_margin)) : (pedal_data > (max+pedal_margin));
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
// pedal_is_active_(accel1, accel2, _accelParams, false)
bool PedalsSystem::pedal_is_active_(float pedal1ConvertedData, float pedal2ConvertedData, const PedalsParams& params, bool check_mech_activation)
{
    float pedal1ScaledData = (float) (pedal1ConvertedData) / abs(params.max_sensor_pedal_1 - params.min_sensor_pedal_1);
    float pedal2ScaledData = (float) (pedal2ConvertedData) / abs(params.max_sensor_pedal_2 - params.min_sensor_pedal_2);
    float val1_deadzone_removed = remove_deadzone_(pedal1ScaledData, params.deadzone_margin);
    float val2_deadzone_removed = remove_deadzone_(pedal2ScaledData, params.deadzone_margin);
    bool pedal_1_is_active;
    bool pedal_2_is_active;
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


bool PedalsSystem::pedal_is_active_(float pedal1ConvertedData, const PedalsParams& params, bool check_mech_activation)
{
    float val1_deadzone_removed = remove_deadzone_(pedal1ConvertedData, params.deadzone_margin);
    bool pedal_1_is_active;
    if(check_mech_activation)
    {
        pedal_1_is_active = val1_deadzone_removed >= params.mechanical_activation_percentage;
    } else {
        pedal_1_is_active = val1_deadzone_removed >= params.activation_percentage;
    }
    return (pedal_1_is_active);
}

// remove_deadzone_(accel1, .03)
float PedalsSystem::remove_deadzone_(float conversion_input, float deadzone)
{
    float range = 1.0 - (deadzone * 2); // 0.94
    // e.g. vals from 0 to 1, deadzone is .05, range is .1
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

bool PedalsSystem::evaluate_pedal_oor (int pedal_data, int min, int max){
    return(pedal_data<=min || pedal_data>=max);
}

// evaluate_brake_and_accel_pressed_(accel_1, accel_2, brake_1, brake_2)
bool PedalsSystem::evaluate_brake_and_accel_pressed_(int accelPedalData1_analog, int accelPedalData2_analog, int brakePedalData1_analog, int brakePedalData2_analog){
    bool accel_pressed = pedal_is_active_(accelPedalData1_analog, accelPedalData2_analog, _accelParams, false);
    bool mech_brake_pressed = pedal_is_active_(brakePedalData1_analog, brakePedalData2_analog, _brakeParams, true);
    bool both_pedals_implausible = (accel_pressed && mech_brake_pressed);
    return both_pedals_implausible;
}

bool PedalsSystem::evaluate_brake_and_accel_pressed_(int accelPedalData1_analog, int accelPedalData2_analog, int brakePedalData1_analog){
    bool accel_pressed = pedal_is_active_(accelPedalData1_analog, accelPedalData2_analog, _accelParams, false);
    float brake_pedal_real = remove_deadzone_(brakePedalData1_analog, _brakeParams.deadzone_margin);
    bool mech_brake_pressed = brake_pedal_real >= _brakeParams.mechanical_activation_percentage;
    bool both_pedals_implausible = (accel_pressed && mech_brake_pressed);
    return both_pedals_implausible;
}