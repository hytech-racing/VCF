#ifndef STEERING_SYSTEM_H
#define STEERING_SYSTEM_H
#include <etl/singleton.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "SharedFirmwareTypes.h"



struct SteeringParams_s {
    uint32_t min_steering_signal_analog; //Raw ADC value from analog sensor at minimum (left) steering angle (calibration)
    uint32_t max_steering_signal_analog; //Raw ADC value from analog sensor at maximum (right) steering angle
    uint32_t min_steering_signal_digital; //Raw ADC value from digital sensor at minimum (left) steering angle
    uint32_t max_steering_signal_digital; //Raw ADC value from digital sensor at maximum (right) steering angle

    
    uint32_t min_steering_analog = 0; //minimum sensor output *analog 0
    uint32_t max_steering_analog = 4096; //maximimum sensor output *analog 4095
    uint32_t min_steering_digital = 0; // maximum sensor output *digital 4095
    uint32_t max_steering_digital = 4096; //maximum sensor output *digital0

    float deg_per_count; //degrees of steering angle per count of the sensor 
    float analog_tol_deg = 0.005f; //+- 0.5%
    float digital_tol_deg = 0.2f; // +- 0.2 degrees


    float max_dtheta_threshold; //maximum change in angle since last reading to consider the reading valid
    float error_between_sensors_tolerance; //maximum difference between digital and analog sensor allowed 
    
};


struct SteeringSystemData_s 
{
    float analog_steering_angle; //in degrees
    float digital_steering_angle; //in degrees
    float output_steering_angle; // represents the better output of the two sensors or some combination of the values

    float analog_steering_velocity_deg_s; //in degrees per second
    float digital_steering_velocity_deg_s;

    bool digital_sensor_error;
    bool sensor_difference_implausibility;
    bool digital_oor_implausibility;
    bool analog_oor_implausibility;

    bool dtheta_exceeded_analog;
    bool dtheta_exceeded_digital;
    bool analog_digital_disagreement;
};


class SteeringSystem 
{
public:
    SteeringSystem(const SteeringParams_s &steeringParams) : _steeringParams(steeringParams)
    {}

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

    // Getters
    const SteeringParams_s &get_steering_params() const;
    
    const SteeringSystemData_s &get_steering_system_data() const {
        return _steeringSystemData;
    }

    const SteeringSensorData_s &get_steering_sensor_data() const {
        return _steeringSensorData;
    }

    // Other Functions
    void recalibrate_steering(const SteeringSensorData_s &current_steering_data, bool calibration_is_on);

    //must have an input that says recalibration is on, then collect the abs min and max value 
    //from both sensors to establish a universal min & max
    
    SteeringSystemData_s evaluate_steering(const SteeringSensorData_s &current_steering_data, uint32_t current_millis);
    
    void update_steering_system();
    

private:

    //track the state of our system from the previous tick to compare against current state for implausibility checks
    float _prev_analog_angle = 0.0f;
    float _prev_digital_angle = 0.0f;
    uint32_t _prev_timestamp = 0; 
    bool _calibrating = false;
    bool _first_run = true; // skip dTheta check on the very first tick
   
    uint32_t _min_observed_analog; //These are class members so they persist during calibration
    uint32_t _max_observed_analog;
    uint32_t _min_observed_digital;
    uint32_t _max_observed_digital;



    float _convert_digital_sensor(const SteeringSensorData_s &current_steering_data);
    
    float _convert_analog_sensor(const SteeringSensorData_s &current_steering_data);


    
    //returns true if steering_analog is outside of the range defined by min and max sensor values
    bool _evaluate_steering_oor_analog(int steering_analog);
    
    //returns true if steering_digital is outside the range defined by min and max sensor values    
    bool _evaluate_steering_oor_digital(int steering_digital);


    //returns true if change in angle exceeds maximum change per reading ( max_dtheta_threshold )
    bool _evaluate_steering_dtheta_exceeded(float dtheta);

    


    SteeringSensorData_s _steeringSensorData{};
    SteeringSystemData_s _steeringSystemData{};
    SteeringParams_s _steeringParams{};
   
};

using SteeringSystemInstance = etl::singleton<SteeringSystem>;

#endif
