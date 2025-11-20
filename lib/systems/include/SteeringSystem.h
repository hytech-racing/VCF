#ifndef STEERINGSYSTEM
#define STEERINGSYSTEM
#include <etl/singleton.h>
#include "SharedFirmwareTypes.h"

struct SteeringParams {
    uint32_t min_steering_1;  // Raw ADC at leftmost (min) steering angle
    uint32_t max_steering_1;  // Raw ADC at rightmost (max) steering angle
    uint32_t min_steering_2;  // (optional) for redundant steering sensor
    uint32_t max_steering_2;

    uint32_t min_sensor_steering_1; // Min value that the sensor can output (if ADC reads less than this, sensor is likely unplugged)
    uint32_t min_sensor_steering_2; // Min value that the sensor can output (if ADC reads less than this, sensor is likely unplugged)
    uint32_t max_sensor_steering_1; // Max value that the sensor can output (if ADC reads more than this, sensor is likely unplugged)
    uint32_t max_sensor_steering_2; // Max value that the sensor can output (if ADC reads more than this, sensor is likely unplugged)
    
    float implausibility_margin; // Out-of-range implausibility margin (0.0 to 1.0). If margin is 0.10, then the steering can travel 10% beyond the measured min/max
    float max_dtheta_threshold; // Threshold to make sure sensor doesn't change too much in one tick
    float cross_sensor_tolerance_deg; // Max allowed difference between sensors
};

class SteeringSystem
{
public:
    SteeringSystem(const SteeringParams &params)
        : _params(params), _last_update_micros(0), _prev_steering_analog(0)
    { }

    void set_params(const SteeringParams &params) {
        _params = params;
    }

    const SteeringSystemData_s &get_steering_system_data() {
        return _data;
    }

    /// @brief Steering evaluation function that takes in the direct analog values of the steering and
    ///        returns all of the steering system data.
    SteeringSystemData_s evaluate_steering(SteeringSensorData_s steering_data,
                                  unsigned long curr_micros);

    SteeringParams get_params() {return _params;}

    /**
     * This is a way to force-update the calibrated min/max values.
     * WARNING: This should only be called when driver holds the button!
     * WARNING: This requires steering to be at min/max travel positions!
     */
    void recalibrate_min_max(SteeringSensorData_s &curr_values)
    {
        // If steering is at min travel and is closer to the observed max, then this sensor is a negative coefficient.
        uint32_t raw_value = static_cast<uint32_t>(curr_values.analog_steering_degrees);
        bool steering_1_flipped = std::abs((int) raw_value - (int) max_observed_steering_1) < std::abs((int) raw_value - (int) min_observed_steering_1);
        
        _params.min_steering_1 = steering_1_flipped ? max_observed_steering_1 : min_observed_steering_1;
        _params.max_steering_1 = steering_1_flipped ? min_observed_steering_1 : max_observed_steering_1;
        // Note: steering_2 not currently used, but keeping structure for future redundancy
        // bool steering_2_flipped = std::abs((int) raw_value_2 - (int) max_observed_steering_2) < std::abs((int) raw_value_2 - (int) min_observed_steering_2);
        // _params.min_steering_2 = steering_2_flipped ? max_observed_steering_2 : min_observed_steering_2;
        // _params.max_steering_2 = steering_2_flipped ? min_observed_steering_2 : max_observed_steering_2;
    }

    /**
     * From code startup, the SteeringSystem should constantly update what its observed max/min
     * values are. When the driver triggers a steering recalibration, these values will be written
     * to non-volatile memory (EEPROM). This should be called constantly to update the
     * observation.
     */
    void update_observed_steering_limits(SteeringSensorData_s &curr_values)
    {
        uint32_t raw_value = static_cast<uint32_t>(curr_values.analog_steering_degrees);
        min_observed_steering_1 = std::min(min_observed_steering_1, raw_value);
        max_observed_steering_1 = std::max(max_observed_steering_1, raw_value);
        // Note: steering_2 not currently used, but keeping for future redundancy
        // min_observed_steering_2 = std::min(min_observed_steering_2, raw_value_2);
        // max_observed_steering_2 = std::max(max_observed_steering_2, raw_value_2);
    }
    uint32_t min_observed_steering_1 = 4096;
    uint32_t max_observed_steering_1 = 0;
    uint32_t min_observed_steering_2 = 4095;
    uint32_t max_observed_steering_2 = 0;

private:
    /// @brief function to determine if the steering is out of range of the calibrated value for the steering
    /// @param steering_analog the value of the steering without deadzone removed
    /// @param min_sensor_value the min value of the steering -- min sensor value
    /// @param max_sensor_value the max value of the steering -- max sensor value
    bool _evaluate_steering_oor(int steering_analog,
        int min_sensor_value,
        int max_sensor_value);

    /// @brief function to determine if the steering has changed too much in one tick
    /// @param steering_analog the value of the steering
    /// @param dt the time since the last tick
    /// @return true if the steering has changed too much in one tick, false otherwise
    bool _evaluate_steering_dtheta(int steering_analog,
        float dt);
                            
    /// @brief
    /// @param steering_analog the value of the steering
    /// @param min_sensor_value the min value of the steering -- min sensor value
    /// @param max_sensor_value the max value of the steering -- max sensor value
    /// @param implaus_margin_scale the implausibility margin scale
    /// @return true if the steering is implausible, false otherwise
    bool _evaluate_steering_implausibilities(int steering_analog,
                                             int min_sensor_value,
                                             int max_sensor_value,
                                             int min_steering_value,
                                             int max_steering_value,
                                             float implausibility_margin,
                                             float dt);

    /// @brief
    /// @param steering_analog the value of the steering
    /// @param min the min value of the steering -- min sensor value
    /// @param max the max value of the steering -- max sensor value
    /// @param implaus_margin_scale the implausibility margin scale
    /// @return true if the steering is implausible, false otherwise
    bool _evaluate_min_max_steering_implausibilities(int steering_analog,
        int min_steering_value,
        int max_steering_value,
        float implaus_margin_scale);

private:
    SteeringSystemData_s _data{};
    SteeringParams _params{};
    bool _implaus_occured = false;
    unsigned long _last_update_micros = 0;
    uint32_t _prev_steering_analog = 0;
};

using SteeringSystemInstance = etl::singleton<SteeringSystem>;

#endif /* STEERINGSYSTEM */