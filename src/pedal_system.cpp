#include <math.h>
#include <tuple>
#include "pedal_system.h"
#include "VCF_Globals.h"
#include "SharedFirmwareTypes.h"
#include "SysClock.h"

// VCF interface/system data - has all pedals data - global. Analog readings of 4 pedals sensors - all data that is in pedal system data
// Accel,brake, regen percent = know what each reps - use last years function - tic/update pedal system function. take in reference to vcf interface data
// analog value into the pedal systems stuff. 
// apps - 2 acceleration position sensors. they both have opposite slopes. determine what is positive and negative slope. Linear interpolation. 
//get rid of adc


//these are tick methods - update the state of pedals. both methods r overloaded, 1/2 brakes. 
//check if this is correct - making accel1 accel 2 etc to float. 
void PedalsSystems::tick(const SysTick_s & tick, int accel1, 
int accel2, int brake1, int brake2)
{
    data_ = evaluate_pedals(accel1,accel2,brake1,brake2,tick.millis);
}

void PedalsSystems::tick(const SysTick_s & tick, const accel1, 
const PedalsSystemData_s & pedals_data, const PedalsSystemData_s & pedals_data)
{
    conversion(pedals_data);
    int accel1 = pedals_data.accel1;
    int accel2 = pedals_data.accel2;
    int brake1 = pedals_data.brake1;
    int brake2 = pedals_data.brake2;

    data_ = evaluate_pedals(accel1,accel2,brake1,brake2,tick.millis);
}

// New Logic - do ADC conversion directly
void conversion(const PedalSensorData_s & pedals_data){
    pedals_data.accel1 = (pedals_data.accel1 - ACCEL_1_OFFSET) * ACCEL_1_SCALE;
    pedals_data.accel2 = (pedals_data.accel2 - ACCEL_2_OFFSET) * ACCEL_2_SCALE;
    pedals_data.accel1 = (pedals_data.brake1 - ACCEL_1_OFFSET) * BRAKE_1_SCALE;
    pedals_data.brake2 = (pedals_data.brake2-BRAKE_2_OFFSET) * BRAKE_2_SCALE;

}


// Pedal Evaluation - One Brake (use brake 1 data) 
PedalsSystemData_s PedalsSystem::evaluate_pedals(const PedalSensorData_s & pedals_data, unsigned long curr_time)
{
    conversion(pedals_data);
    PedalsSystemData_s out;
    int accel1 = pedals_data.accel1;
    int accel2 = pedals_data.accel2;
    int brake1 = pedals_data.brake1;

    out.accelPressed = pedal_is_active_(accel1,accel2, accelParams_,false);
    out.accelImplausible = evaluate_pedal_implausibilities_(pedals_data.accel1(), pedals_data.accel2(),accelParams_,0.1);
    auto percent = (out.accelImplausible) ? accel1 : (accel1 + accel2) / 2.0;
    out.accelPercent = remove_deadzone_(percent, accelParams_.deadzone_margin);
    out.accelPercent = std::max(out.accelPercent, 0.0f);
    out.brakeImplausible = evaluate_pedal_implausibilities_(pedals_data.brake1(), brakeParams_, 0.25);
    out.brakeAndAccelPressedImplausibility = evaluate_brake_and_accel_pressed_(pedals_data.accel1(),pedals_data.accel2(), pedals_data.brake1());
    bool implausibility = (out.brakeAndAccelPressedImplausibility || out.brakeImplausible || out.accelImplausible);
    if (implausibility && (implausibilityStartTime_ == 0))
    {
        implausibilityStartTime_ = curr_time;
    }
    else if ((!implausibility) && ((out.accelPercent <= 0.05)))
    {
        implausibilityStartTime_ = 0;
    }

    bool oor = implausibility && (evaluate_pedal_oor(accel1, accelParams_.min_sensor_pedal_1, accelParams_.max_sensor_pedal_1) 
            || evaluate_pedal_oor(accel2, accelParams_.min_sensor_pedal_2, accelParams_.max_sensor_pedal_2));
    out.accelPercent = (oor) ? 0 : out.accelPercent;

    
    out.brakePercent = brake1; //Brake 1 or Brake 2 here? also this is to determine percent brake pressed using globals from VCF
    out.brakePercent = remove_deadzone_(out.brakePercent, brakeParams_.deadzone_margin);
    out.brakePressed = pedal_is_active_(brake1,brakeParams_, false);


    out.mechBrakeActive = out.brakePercent >= brakeParams_.mechanical_activation_percentage;
    out.regenPercent = std::max(std::min(out.brakePercent / brakeParams_.mechanical_activation_percentage, 1.0f), 0.0f);

    
    out.implausibilityExceededMaxDuration = max_duration_of_implausibility_exceeded_(curr_time);
    return out;

}

PedalsSystemData_s PedalsSystem::evaluate_pedals2(const PedalSensorData_s & pedals_data, unsigned long curr_time)
{
    PedalsSystemData_s out;
    conversion(pedals_data);
    int accel1 = pedals_data.accel1;
    int accel2 = pedals_data.accel2;
    int brake1 = pedals_data.brake1;
    int brake2 = pedals_data.brake2;
    out.accelPressed = pedal_is_active_(accel1,accel2, accelParams_,false);
    out.accelImplausible = evaluate_pedal_implausibilities_(accel1,accel2,accelParams_,0.1);
    auto percent = (out.accelImplausible) ? accel1 : (accel1 + accel2) / 2.0;
    out.accelPercent = remove_deadzone_(percent, accelParams_.deadzone_margin);
    out.accelPercent = std::max(out.accelPercent, 0.0f);
    out.brakeImplausible = evaluate_pedal_implausibilities_(brake1, brake2, brakeParams_, 0.25);
    out.brakeAndAccelPressedImplausibility = evaluate_brake_and_accel_pressed_(accel1, accel2, brake1, brake2);
    bool implausibility = (out.brakeAndAccelPressedImplausibility || out.brakeImplausible || out.accelImplausible);
    if (implausibility && (implausibilityStartTime_ == 0))
    {
        implausibilityStartTime_ = curr_time;
    }
    else if ((!implausibility) && ((out.accelPercent <= 0.05)))
    {
        implausibilityStartTime_ = 0;
    }

    bool oor = implausibility && (evaluate_pedal_oor(accel1, accelParams_.min_sensor_pedal_1, accelParams_.max_sensor_pedal_1) 
            || evaluate_pedal_oor(accel2, accelParams_.min_sensor_pedal_2, accelParams_.max_sensor_pedal_2));
    out.accelPercent = (oor) ? 0 : out.accelPercent;

    
    out.brakePercent = (brake1 +brake2)/2; //Brake 1 or Brake 2 here? also this is to determine percent brake pressed using globals from VCF
    out.brakePercent = remove_deadzone_(out.brakePercent, brakeParams_.deadzone_margin);
    out.brakePressed = pedal_is_active_(brake1,brake2,brakeParams_, false);


    out.mechBrakeActive = out.brakePercent >= brakeParams_.mechanical_activation_percentage;
    out.regenPercent = std::max(std::min(out.brakePercent / brakeParams_.mechanical_activation_percentage, 1.0f), 0.0f);

    
    out.implausibilityExceededMaxDuration = max_duration_of_implausibility_exceeded_(curr_time);
    return out;
}


bool PedalsSystem::max_duration_of_implausibility_exceeded_(unsigned long curr_time)
{
    if (implausibilityStartTime_ != 0)
    {
        return ((curr_time - implausibilityStartTime_) > 100);
    }
    else
    {
        return false;
    }
}

bool PedalsSystem::evaluate_pedal_implausibilities_(const PedalSensorData_s & pedals_data, const PedalsParams &params)
{
    return evaluate_min_max_pedal_implausibilities_(pedals_data, params.min_pedal_1, params.max_pedal_1, params.implausibility_margin);
}

bool PedalsSystem::evaluate_pedal_implausibilities_(const PedalSensorData_s & pedals_data, const PedalsParams &params)
{
    bool pedal1_min_max_implaus = evaluate_min_max_pedal_implausibilities_(pedalData1, params.min_pedal_1, params.max_pedal_1, params.implausibility_margin);
    bool pedal2_min_max_implaus = evaluate_min_max_pedal_implausibilities_(pedalData2, params.min_pedal_2, params.max_pedal_2, params.implausibility_margin);
    bool sens_not_within_req_percent = (fabs(pedalData1.conversion - pedalData2.conversion) > max_percent_diff);
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

bool PedalsSystem::evaluate_min_max_pedal_implausibilities_(const PedalSensorData_s &pedals_data,
                                                            int min,
                                                            int max,
                                                            float implaus_margin_scale)
{

    bool pedal_swapped = false;

    // get the pedal margin. The margin will be a percentage of the range of the measured max values
    int pedal_margin = abs(max - min) * implaus_margin_scale;

    if (min > max)
    {
        pedal_swapped = true;
        // swap the logic: need to check and see if it is greater than min and less than max
    }
    // FSAE EV.5.5
    // FSAE T.4.2.10
    // pedaldata.raw? what is this
    bool pedal_less_than_min = pedal_swapped ? (pedalData.raw > (min + pedal_margin))
                                             : (pedalData.raw < (min - pedal_margin));

    bool pedal_greater_than_max = pedal_swapped ? (pedalData.raw < (max - pedal_margin))
                                                : (pedalData.raw > (max + pedal_margin));

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

float PedalsSystem::remove_deadzone_(float conversion_input, float deadzone)
{
    float range = 1.0 - (deadzone * 2);
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
// method for 1 brake
bool PedalsSystem::evaluate_brake_and_accel_pressed_(const PedalsSystemData_s & pedals_data)
{
    conversion(pedals_data);
    int accel1 = pedals_data.accel1;
    int accel2 = pedals_data.accel2;
    int brake = pedals_data.brake1;
    bool accel_pressed = pedal_is_active_(accel1, accel2, accelParams_, false); // .1
    float brake_pedal_real = remove_deadzone_(brakePedalData.conversion, brakeParams_.deadzone_margin);
    bool mech_brake_pressed = brake_pedal_real >= brakeParams_.mechanical_activation_percentage;
    bool both_pedals_implausible = (accel_pressed && mech_brake_pressed);
    return both_pedals_implausible;
}
// methods for 2 brakes
bool PedalsSystem::evaluate_brake_and_accel_pressed_(const PedalsSystemData_s & pedals_data)
{

    conversion(pedals_data);
    int accel1 = pedals_data.accel1;
    int accel2 = pedals_data.accel2;
    int brake1 = pedals_data.brake1;
    int brake2 = pedals_data.brake2;
    bool accel_pressed = pedal_is_active_(accel1, accel2, accelParams_, false); // .1
    bool mech_brake_pressed = pedal_is_active_(brake1, brake2, brakeParams_, true);  // 0.40
    
    
    bool both_pedals_implausible = (accel_pressed && mech_brake_pressed);
    return both_pedals_implausible;
}

bool PedalsSystem::pedal_is_active_(float pedal1ConvertedData, float pedal2ConvertedData, const PedalsParams& params, bool check_mech_activation)
{
    float val1_deadzone_removed = remove_deadzone_(pedal1ConvertedData, params.deadzone_margin);
    float val2_deadzone_removed = remove_deadzone_(pedal2ConvertedData, params.deadzone_margin);
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

bool PedalsSystem::evaluate_pedal_oor(const PedalsSystemData_s & pedals_data,
                                      int min,
                                      int max)
{
    return (pedals_data.raw >= max || pedals_data.raw <= min);
    //add raw to pedalsdata struct
} 


