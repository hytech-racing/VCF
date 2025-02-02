// VCF interface/system data - has all pedals data - global. Analog readings of 4 pedals sensors - all data that is in pedal system data
// Accel,brake, regen percent = know what each reps - use last years function - tic/update pedal system function. take in reference to vcf interface data
// analog value into the pesal systems stuff. 
// apps - 2 acceleration position sensors. they both have opposite slopes. determine what is positive and negative slope. Linear interpolation. 
// brake implausible - one positive one negative coeff. same as accel. 

#ifndef PEDALSSYSTEM
#define PEDALSSYSTEM
#include <math.h>
#include <tuple>

#include "SharedFirmwareTypes.h"

/// @brief Pedals params struct that will hold min / max that will be used for evaluateion.
struct PedalsParams
{
    int min_pedal_1;
    int min_pedal_2;
    int max_pedal_1;
    int max_pedal_2;
    int min_sensor_pedal_1;
    int min_sensor_pedal_2;
    int max_sensor_pedal_1;
    int max_sensor_pedal_2;
    float activation_percentage;
    float deadzone_margin;
    float implausibility_margin;
    float mechanical_activation_percentage;
};

class PedalsSystem
{
public:
    /// @brief pedals system class that evaluates pedals for both accel and
    ///        brake percent as well as implausibilities
    /// @param accelParams accel pedal parameters. by rules, 2 sensors must be used for redundancy and evaluated w.r.t each other
    /// @param brakeParams brake pedal params. when used with only one pedal sensor, the pedal parameter evaluation for brakes only looks at the min and max for min_pedal_1 / max_pedal_1
    PedalsSystem(const PedalsParams &accelParams,
                 const PedalsParams &brakeParams)
    {
        setParams(accelParams, brakeParams);
        _implausibilityStartTime = 0;
    }

    void setParams(const PedalsParams &accelParams,
                   const PedalsParams &brakeParams)
    {
        _accelParams = accelParams;
        _brakeParams = brakeParams;
    }

    const PedalsSystemData_s &getPedalsSystemData()
    {
        return _data;
    }

    PedalsSystemData_s getPedalsSystemDataCopy()
    {
        return _data;
    }

    float getMechBrakeActiveThreshold()
    {
        return _brakeParams.mechanical_activation_percentage;
    }

    /// @brief Tick function that runs the evaluation of the pedals system.
    ///        evaluates brake using only min and max params for sensor 1 (min_pedal_1 / max_pedal_1).
    /// @param curr_millis The current timestamp, in milliseconds.
    /// @param interface_data A reference to the interface_data global.
    /// @param use_both_brake_sensors True if we should use both brake sensors for implausibility. False if otherwise.
    void tick(unsigned long curr_millis, VCFInterfaceData_s &interface_data, bool use_both_brake_sensors);

    /// @brief Pedal evaluation function that takes in the direct analog values of the pedals and
    ///        returns all of the pedals system data.

    /// I want to change this - can we take in one PedalSystemData_s struct reference and obtain the values from that and just have a boolean for 1 or 2 brakes?

    PedalsSystemData_s evaluate_pedals(PedalSensorData_s pedal_data,
                                       unsigned long curr_millis,
                                       bool two_brakes);
    /// @brief Overloaded pedal evaluation function that takes in direct analog values of pedals (but
    //         ignores brake sensor 2) and returns the PedalsSystemData.
    // lets get rid of this we only need one method it makes the code so much cleaner most of the code is the same anyway
    /*
    
    PedalsSystemData_s evaluate_pedals(uint32_t accel1,
                                       uint32_t accel2,
                                       uint32_t brake,
                                       unsigned long curr_millis);

    */

private:
    PedalsSystemData_s _data{};
    PedalsParams _accelParams{};
    PedalsParams _brakeParams{};
    unsigned long _implausibilityStartTime;
    float remove_deadzone_(float conversion_input, float deadzone);
    bool max_duration_of_implausibility_exceeded_(unsigned long curr_time);

    /// @brief
    ///    Evaluate pedal implausibilities_ determines if there is a software implausibility
    ///    in the pedals caused by them going out of range.
    ///    Our max/min sensor ranges are calcuated from the pedal min/max values
    ///    The pedal min/max values are defined in MCU_defs and are the real world raw
    ///    values that we determine from the pedal output.
    ///    The max/min sensor values are then a certain percent higher than these real world
    ///    values as determined by the implausibility margin. This protects against physical
    ///    damage to the sensor or pedal, but will not accidentally trip implausibility if
    ///    pedal values fluctuate

    /// @param pedalData1
    /// @param pedalData2
    /// @param params
    /// @param max_percent_diff
    /// @return
    bool evaluate_pedal_implausibilities_(int pedalData1_analog,
                                          int pedalData2_analog,
                                          const PedalsParams &params,
                                          float max_percent_diff);

    /// @brief overloaded pedal implaus check that doesnt need to check for percent diff between sensors since only one sensor
    /// @param pedalData
    /// @param params
    /// @return
    bool evaluate_pedal_implausibilities_(int pedalData, const PedalsParams &params);

    /// @brief function to determine if the pedals and the brakes are pressed at the same time.
    ///        evaluates brake being pressed with mech brake activation threshold AFTER removing
    ///        deadzones for both brake and accel
    /// @param pedal_data
    /// @param twopedals
    bool evaluate_brake_and_accel_pressed_(PedalSensorData_s & pedal_data, bool twopedals);

    /// @brief This checks to see if any pedal sensor is out of range :(
    /// @param PedalData The analog pedal Value
    /// @return 
    bool evaluate_pedal_oor(int pedalData_analog,
                           int min,
                           int max);
    /// @brief
    /// @param pedalData
    /// @param min
    /// @param max
    /// @param implaus_margin_scale
    /// @return
    bool evaluate_min_max_pedal_implausibilities_(int pedalData_analog,
                                                  int min,
                                                  int max,
                                                  float implaus_margin_scale);

    /// @brief check whether or not pedal is active according to input parameters. returns true if either pedal is over threshold. removes the deadzone before checking.
    /// @param pedal1ConvertedData the value 0 to 1 of the first pedal without deadzone removed
    /// @param pedal2ConvertedData ... second pedal 0 to 1 val
    /// @param params the pedal parameters for this specific pedal
    /// @param check_mech_activation if this is true, function will check percentages against the mechanical activation percentage
    /// @return true or false accordingly
    bool pedal_is_active_(float pedal1ConvertedData, float pedal2ConvertedData, const PedalsParams &params, bool check_mech_activation);

    /// @brief check whether or not SINGLE pedal is active according to input. returns true if pedal is over threshold. removes deadzone.
    /// @param pedal1ConvertedData the value 0 to 1 of the first pedal without deadzone removed
    /// @param params the pedal parameters for this specific pedal
    /// @param check_mech_activation if this is true, function will check percentages against the mechanical activation percentage
    /// @return true or false accordingly
    bool pedal_is_active_(float pedal1ConvertedData, const PedalsParams &params, bool check_mech_activation);


};

#endif /* PEDALSSYSTEM */
