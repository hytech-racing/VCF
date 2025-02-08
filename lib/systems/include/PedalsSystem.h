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


// This struct has the raw data for the pedals
struct PedalSensorData_s
{
    int accel_1;
    int accel_2;
    int brake_1;
    int brake_2;
};

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
    struct PedalData_s {
        PedalSensorData_s raw_data;
        float accel_percent; // 0 to 1
        float brake_percent;
    };
    /// @brief pedals system class that evaluates pedals for both accel and
    ///        brake percent as well as implausibilities
    /// @param accelParams accel pedal parameters. by rules, 2 sensors must be used for redundancy and evaluated w.r.t each other
    /// @param brakeParams brake pedal params. when used with only one pedal sensor, the pedal parameter evaluation for brakes only looks at the min and max for min_pedal_1 / max_pedal_1
    PedalsSystem(const PedalsParams &accelParams,
                 const PedalsParams &brakeParams) : _accelParams(accelParams), _brakeParams(brakeParams), _implausibilityStartTime(0)
    { }

    void set_params(const PedalsParams &accelParams,
                   const PedalsParams &brakeParams)
    {
        _accelParams = accelParams;
        _brakeParams = brakeParams;
    }

    const PedalsSystemData_s &get_pedals_system_data()
    {
        return _data;
    }

    float get_mech_brake_activation_threshold()
    {
        return _brakeParams.mechanical_activation_percentage;
    }

    /// @brief Tick function that runs the evaluation of the pedals system.
    ///        evaluates brake using only min and max params for sensor 1 (min_pedal_1 / max_pedal_1).
    /// @param curr_millis The current timestamp, in milliseconds.
    /// @param pedals_data A reference to the PedalSensorData.
    void tick(unsigned long curr_millis, PedalSensorData_s  & pedals_data);

    /// @brief Pedal evaluation function that takes in the direct analog values of the pedals and
    ///        returns all of the pedals system data.

    /// I want to change this - can we take in one PedalSystemData_s struct reference and obtain the values from that and just have a boolean for 1 or 2 brakes?

    PedalsSystemData_s evaluate_pedals(PedalSensorData_s pedal_data, unsigned long curr_millis);

private:
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
    bool evaluate_pedal_implausibilities_(int pedal_1_raw,
                                          int pedal_2_raw,
                                          const PedalsParams &params,
                                          float max_percent_diff);

    /// @brief function to determine if the pedals and the brakes are pressed at the same time.
    ///        evaluates brake being pressed with mech brake activation threshold AFTER removing
    ///        deadzones for both brake and accel
    /// @param pedal_data
    /// @param twopedals
    bool evaluate_brake_and_accel_pressed_(PedalSensorData_s & pedal_data);

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
    
    float _scale_pedal_val(int raw_pedal_val, int min, int max);
private:
    PedalsSystemData_s _data{};
    PedalsParams _accelParams{};
    PedalsParams _brakeParams{};
    unsigned long _implausibilityStartTime;


};

#endif /* PEDALSSYSTEM */
