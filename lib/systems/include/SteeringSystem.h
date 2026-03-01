#ifndef STEERING_SYSTEM_H
#define STEERING_SYSTEM_H

#include <etl/singleton.h>
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "SharedFirmwareTypes.h"

struct SteeringParams_s {
    // raw ADC input signals
    uint32_t min_steering_signal_analog = 0; //Raw ADC value from analog sensor at minimum (left) steering angle (calibration)
    uint32_t max_steering_signal_analog = 4095; //Raw ADC value from analog sensor at maximum (right) steering angle
    uint32_t min_steering_signal_digital; //Raw ADC value from digital sensor at minimum (left) steering angle
    uint32_t max_steering_signal_digital; //Raw ADC value from digital sensor at maximum (right) steering angle

    uint32_t span_signal_analog = 4095;
    uint32_t span_signal_digital;
    int32_t digital_midpoint;
    int32_t analog_midpoint;

    // calibration limits
    uint32_t min_observed_digital;
    uint32_t max_observed_digital;
    uint32_t min_observed_analog; // do we need to do min/max calibration for analog?
    uint32_t max_observed_analog; 

    // conversion rates
    // float deg_per_count_analog = 0.0439f; //hard coded for analog (180)
    float deg_per_count_analog = 0.087890625f;
    float deg_per_count_digital = 0.02197265625f; //based on digital readings

    // implausibility values
    float analog_tol = 0.005f; //+- 0.5% error
    float analog_tol_deg;
    float digital_tol_deg = 0.2f; // +- 0.2 degrees error
   
    // rate of angle change
    float max_dtheta_threshold; //maximum change in angle since last reading to consider the reading valid

    // difference rating
    float error_between_sensors_tolerance; //maximum difference between digital and analog sensor allowed
};

struct SteeringSystemData_s
{
    float analog_steering_angle; //in degrees
    float digital_steering_angle; //in degrees
    float output_steering_angle; // represents the better output of the two sensors or some combination of the values

    float analog_steering_velocity_deg_s; //in degrees per second
    float digital_steering_velocity_deg_s;

    bool digital_oor_implausibility;
    bool analog_oor_implausibility;
    bool sensor_disagreement_implausibility;
    bool dtheta_exceeded_analog;
    bool dtheta_exceeded_digital;
    bool both_sensors_fail;
};

class SteeringSystem {
public:
    SteeringSystem(const SteeringParams_s &steeringParams) : _steeringParams(steeringParams) {}

    // Functions
    void recalibrate_steering_digital(const SteeringSensorData_s &current_steering_data, bool calibration_is_on);
    
    SteeringSystemData_s evaluate_steering(const SteeringSensorData_s &current_steering_data, uint32_t current_millis);

    // Getters
    const SteeringParams_s &get_steering_params() const {
        return _steeringParams;
    }

    const SteeringSystemData_s &get_steering_system_data() const {
        return _steeringSystemData;
    }

    const SteeringSensorData_s &get_steering_sensor_data() const {
        return _steeringSensorData;
    }

    // Setters
    void set_steering_params(const SteeringParams_s &steeringParams) {
        _steeringParams = steeringParams;
    }

    void set_steering_system_data(const SteeringSystemData_s &steeringSystemData) {
        _steeringSystemData = steeringSystemData;
    }

    void set_steering_sensor_data(const SteeringSensorData_s &steeringSensorData) {
        _steeringSensorData = steeringSensorData;
    }
    
private:
    void update_observed_steering_limits(const SteeringSensorData_s &current_steering_data);

    float _convert_digital_sensor(const SteeringSensorData_s &current_steering_data);
   
    float _convert_analog_sensor(const SteeringSensorData_s &current_steering_data);
   
    //returns true if steering_analog is outside of the range defined by min and max sensor values
    bool _evaluate_steering_oor_analog(uint32_t steering_analog);
   
    //returns true if steering_digital is outside the range defined by min and max sensor values    
    bool _evaluate_steering_oor_digital(uint32_t steering_digital);

    //returns true if change in angle exceeds maximum change per reading ( max_dtheta_threshold )
    bool _evaluate_steering_dtheta_exceeded(float dtheta);

    SteeringSensorData_s _steeringSensorData {};
    SteeringSystemData_s _steeringSystemData {};
    SteeringParams_s _steeringParams;
    //track the state of our system from the previous tick to compare against current state for implausibility checks
    float _prev_analog_angle = 0.0f;
    float _prev_digital_angle = 0.0f;
    uint32_t _prev_timestamp = 0;
    bool _calibrating = false;
    bool _first_run = true; // skip dTheta check on the very first tick
};

using SteeringSystemInstance = etl::singleton<SteeringSystem>;

#endif
