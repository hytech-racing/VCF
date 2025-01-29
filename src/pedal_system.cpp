#include <math.h>
#include <tuple>
#include "pedal_system.h"
#include "VCF_Globals.h"
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
const AnalogConversion_s & accel2, const AnalogConversion_s & brake)
{
    data_ = evaluate_pedals(accel1,accel2,brake1,brake2,tick.millis);
}

// New Logic - do ADC conversion directly


//convert accel to float
//What this is doing - analogConversion_s object reference given
// to accel1, accel2, brake1, brake2. This, instead of being converted from 0-1, can be processed raw. 

PedalsSystemData_s PedalsSystem::evaluate_pedals(const VCFInterfaceData_s &accel1,
                                                 const VCFInterfaceData_s &accel2,
                                                 const VCFInterfaceData_s &brake1,
                                                 const VCFInterfaceData_s &brake2,
                                                 unsigned long curr_time)
{
    PedalsSystemData_s out;
    accel1 = (ACCEL_1_CHANNEL + ACCEL_1_OFFSET) * ACCEL_1_SCALE;
    accel2 = (ACCEL_2_CHANNEL + ACCEL_2_OFFSET) * ACCEL_2_SCALE;
    brake1 = (BRAKE_1_CHANNEL + BRAKE_1_OFFSET) * BRAKE_1_SCALE;
    brake2 = (BRAKE_2_CHANNEL + BRAKE_2_OFFSET) * BRAKE_2_SCALE;
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

    
    out.brakePercent = brake1; //Brake 1 or Brake 2 here? also this is to determine percent brake pressed using globals from VCF
    out.brakePercent = remove_deadzone_(out.brakePercent, brakeParams_.deadzone_margin);
    out.brakePressed = brake1 >= brakeParams_.activation_percentage;

    out.mechBrakeActive = out.brakePercent >= brakeParams_.mechanical_activation_percentage;
    out.regenPercent = std::max(std::min(out.brakePercent / brakeParams_.mechanical_activation_percentage, 1.0f), 0.0f);

    
    out.implausibilityExceededMaxDuration = max_duration_of_implausibility_exceeded_(curr_time);
    return out;
}



