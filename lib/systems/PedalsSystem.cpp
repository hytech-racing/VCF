#include <math.h>
#include <tuple>
#include "PedalsSystem.h"

// VCF interface/system data - has all pedals data - global. Analog readings of 4 pedals sensors - all data that is in pedal system data
// Accel,brake, regen percent = know what each reps - use last years function - tic/update pedal system function. take in reference to vcf interface data
// analog value into the pedal systems stuff. 
// apps - 2 acceleration position sensors. they both have opposite slopes. determine what is positive and negative slope. Linear interpolation. 
//get rid of adc

void PedalsSystem::tick(unsigned long curr_millis, VCFInterfaceData_s &interface_data, bool use_both_brake_sensors)
{
    if (use_both_brake_sensors)
    {
        // _data = evaluate_pedals(interface_data.pedals_data.accel_1,
        //                         interface_data.pedals_data.accel_2,
        //                         interface_data.pedals_data.brake_1,
        //                         interface_data.pedals_data.brake_2,
        //                         curr_millis);
    }
    else
    {
        // _data = evaluate_pedals(interface_data.pedals_data.accel_1,
        //                         interface_data.pedals_data.accel_2,
        //                         interface_data.pedals_data.brake_1,
        //                         curr_millis);
    }
}

//convert accel to float
//What this is doing - analogConversion_s object reference given
// to accel1, accel2, brake1, brake2. This, instead of being converted from 0-1, can be processed raw. 

PedalsSystemData_s PedalsSystem::evaluate_pedals(uint32_t accel1_analog, uint32_t accel2_analog, uint32_t brake1_analog, uint32_t brake2_analog, unsigned long curr_millis)
{
    PedalsSystemData_s out = {};
    // Please redo this
    return out;
}



