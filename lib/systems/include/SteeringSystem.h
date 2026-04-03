#ifndef STEERING_SYSTEM_H
#define STEERING_SYSTEM_H


#include <etl/singleton.h>
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "SharedFirmwareTypes.h"
#include "SteeringEncoderInterface.h"

struct SteeringParams_s {
    // raw ADC input signals
    uint32_t min_steering_signal_analog; //Raw ADC value from analog sensor at minimum (left) steering angle (calibration)
    uint32_t max_steering_signal_analog; //Raw ADC value from analog sensor at maximum (right) steering angle
    uint32_t min_steering_signal_digital; //Raw ADC value from digital sensor at minimum (left) steering angle
    uint32_t max_steering_signal_digital; //Raw ADC value from digital sensor at maximum (right) steering angle

    int32_t analog_min_with_margins;
    int32_t analog_max_with_margins;
    int32_t digital_min_with_margins;
    int32_t digital_max_with_margins;

    uint32_t span_signal_analog;
    uint32_t span_signal_digital;
    int32_t digital_midpoint;
    int32_t analog_midpoint;

    // conversion rates
    // float deg_per_count_analog = 0.0439f; //hard coded for analog (180)
    float deg_per_count_analog;
    float deg_per_count_digital; //based on digital readings

    // implausibility values
    float analog_tol; //+- 0.5% error
    float analog_tol_deg;
    float digital_tol_deg; // +- 0.2 degrees error
   
    // rate of angle change
    float max_dtheta_threshold; //maximum change in angle since last reading to consider the reading valid

    // difference rating
    float error_between_sensors_tolerance; //maximum difference between digital and analog sensor allowed
};

struct SteeringSystemData_s
{
    uint32_t analog_raw;
    uint32_t digital_raw;

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
    bool interface_sensor_error;
};

class SteeringSystem {
public:
    SteeringSystem(const SteeringParams_s &steeringParams) : _steeringParams(steeringParams) {}

    // Functions
    void recalibrate_steering_digital(const uint32_t analog_raw, const uint32_t digital_raw);

    void begin_calibrating();

    void end_calibrating();

    bool is_calibrating();

    bool is_finished_calibrating();

    void evaluate_steering(const uint32_t analog_raw, const SteeringEncoderReading_s digital_data, const uint32_t current_millis);

    // Getters
    const SteeringParams_s &get_steering_params() const {
        return _steeringParams;
    }

    const SteeringSystemData_s &get_steering_system_data() const {
        return _steeringSystemData;
    }

    // Setters
    void set_steering_params(const SteeringParams_s &steeringParams) {
        _steeringParams = steeringParams;
    }

    void set_steering_system_data(const SteeringSystemData_s &steeringSystemData) {
        _steeringSystemData = steeringSystemData;
    }
   
    void update_observed_steering_limits(const uint32_t analog_raw, const uint32_t digital_raw);
private:

    float _convert_digital_sensor(const uint32_t digital_raw);
   
    float _convert_analog_sensor(const uint32_t analog_raw);
   
    //returns true if steering_analog is outside of the range defined by min and max sensor values
    bool _evaluate_steering_oor_analog(const uint32_t steering_analog);
   
    //returns true if steering_digital is outside the range defined by min and max sensor values    
    bool _evaluate_steering_oor_digital(const uint32_t steering_digital);

    //returns true if change in angle exceeds maximum change per reading ( max_dtheta_threshold )
    bool _evaluate_steering_dtheta_exceeded(float dtheta);

    SteeringSystemData_s _steeringSystemData {};
    SteeringParams_s _steeringParams;
    //track the state of our system from the previous tick to compare against current state for implausibility checks
    float _prev_analog_angle = 0.0f;
    float _prev_digital_angle = 0.0f;
    uint32_t _prev_timestamp = 0;
    bool _calibrating = false;
    bool _finished_calibrating = false;
    bool _first_run = true; // skip dTheta check on the very first tick

    uint32_t min_observed_digital = UINT32_MAX;
    uint32_t max_observed_digital = 0;
    uint32_t min_observed_analog = UINT32_MAX;
    uint32_t max_observed_analog = 0;
};

using SteeringSystemInstance = etl::singleton<SteeringSystem>;

#endif
