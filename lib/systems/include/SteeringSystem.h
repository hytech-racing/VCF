#ifndef STEERING_SYSTEM_H
#define STEERING_SYSTEM_H
#include <etl/singleton.h>

#include "SharedFirmwareTypes.h"



struct SteeringParams_s {
    uint32_t min_steering_signal_analog; //Raw ADC value from analog sensor at minimum (left) steering angle (calibration)
    uint32_t max_steering_signal_analog; //Raw ADC value from analog sensor at maximum (right) steering angle
    uint32_t min_steering_signal_digital; //Raw ADC value from digital sensor at minimum (left) steering angle
    uint32_t max_steering_signal_digital; //Raw ADC value from digital sensor at maximum (right) steering angle

    
    uint32_t min_steering_analog; //minimum sensor output *analog 0
    uint32_t max_steering_analog; //maximimum sensor output *analog 4095
    uint32_t min_steering_digital; // maximum sensor output *digital 4095
    uint32_t max_steering_digital; //maximum sensor output *digital0

    float deg_per_count; //degrees of steering angle per count of the sensor 


    float max_dtheta_threshold; //maximum change in angle since last reading to consider the reading valid
    float error_between_sensors_tolerance; //maximum difference between digital and analog sensor allowed 
    
};


struct SteeringSystemData_s 
{
    float analog_steering_angle; //in degrees
    float digital_steering_angle; //in degrees
    float output_steering_angle; // represents the better output of the two sensors or some combination of the values

    float steering_velocity_deg_s; //in degrees per second

    bool digital_sensor_error;
    bool sensor_difference_implausibility;
    bool digital_oor_implausibility;
    bool analog_oor_implausibility;
}


class SteeringSystem 
{
public:
    SteeringSystem(const SteeringParams_s &steeringParams) : _steeringParams(steeringParams)
    {}

    // Setters
    void set_steering_params(const SteeringParams_s &steeringParams)
    {
//parameters
    }


    void set_steering_system_data(const SteeringSystemData_s &steeringSystemData);
    {

        //output that comes once we've calibrated everything, we output this data
    }

    void set_steering_sensor_data(const SteeringSensorData_s &steeringSensorData);
    //get the current value from the sensor


    // Getters
    const SteeringParams_s &get_steering_params() const;
    
    const SteeringSystemData_s &get_steering_system_data() const;

    const SteeringSensorData_s &get_steering_sensor_data() const;

    // Other Functions
    void recalibrate_steering(const SteeringSensorData_s &current_steering_data);
    {
        uint32_t analog_raw_value = static_cast<uint32_t>(current_steering_data.analog_raw);
        uint32_t digital_raw_value = static_cast<uint32_t>(current_steering_data.digital_raw);
        bool steering_flipped = 

    }

    void evaluate_steering(const SteeringSensorData_s &current_steering_data);
    
    
    

private:
   
    float _convert_digital_sensor(const SteeringSensorData_s &current_steering_data);
        
    float _convert_analog_sensor(const SteeringSensorData_s &current_steering_data);


    
    //returns true if steering_analog is outside of the range defined by min and max sensor values
    bool _evaluate_steering_oor_analog(int steering_analog,
        int min_sensor_value_analog,
        int max_sensor_value_analog);
        
    //returns true if steering_digital is outside the range defined by min and max sensor values    
    bool _evaluate_steering_oor_digital(int steering_digital,
        int min_sensor_value_digital,
        int max_sensor_value_digital);

    //returns true if change in angle exceeds maximum change per reading ( max_dtheta_threshold )
    bool _evaluate_steering_dtheta_exceeded(int steering_analog,
        int steering_digital,
        float dt);

    

/*
    int32_t _prev_raw_adc = -1;  // Initialize to -1 so we know it's the first run
    int32_t _rotation_count = 0; // Tracks full 360-degree turns (e.g., -1, 0, 1, 2)

    -
    const int32_t MAX_ADC_LIMIT = 4095; // The rollover point
    const int32_t HALF_ADC_LIMIT = 2048; // Threshold to detect a wrap

    In order to check current total number of rotations (analog goes from max value back down to 0), we need to see if we exceeded a limit between readings
    leaving this commented for now because this conflicts with one of the implausibility checks (max change in angle per reading) and we can revisit this if we find a solution
*/
    SteeringSensorData_s _steeringSensorData{};
    SteeringSystemData_s _steeringSystemData{};
    SteeringParams_s _steeringParams{};
    bool _implaus
};

















using SteeringSystemInstance = etl::singleton<SteeringSystem>;

#endif
