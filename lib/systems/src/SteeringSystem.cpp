#include <math.h>
#include "SteeringSystem.h"


bool SteeringSystem::_evaluate_steering_oor_analog(int steering_analog,
        int min_sensor_value_analog,
        int max_sensor_value_analog,){
            if(steering_analog < min_sensor_value_analog || steering_analog > max_sensor_value_analog){
                SteeringParams.analog_oor_implausibility = true;
                return true;
            }
            return false;
        }

bool SteeringSystem::_evaluate_steering_oor_digital(int steering_digital,
        int min_sensor_value_digital,
        int max_sensor_value_digital){
            if(steering_digital < min_sensor_value_digital || steering_digital > max_sensor_value_analog){
                SteeringParams.digital_oor_implausibility = true;
                return true;
            }
            return false;
        }

bool SteeringSystem::_evaluate_steering_dtheta_exceeded(float dtheta){
            if(dtheta > SteeringParams.max_dtheta_threshold){
                return true;
            }
            return false;
        }




float SteeringSystem::_convert_analog_sensor(const SteeringSensorData_s &current_steering_data)
{
    // Get the raw value
    uint32_t raw_val = current_steering_data.steering_sensor_analog; // Check your struct for exact name

    //  Calculate how far we are from the minimum raw count delta
    int32_t count_delta = (int32_t)raw_val - (int32_t)_steeringParams.min_steering_signal_analog;

    //  Convert that delta into degrees
    float degrees_delta = (float)count_delta * _steeringParams.deg_per_count;

    // Add to the starting minimum angle to get absolute position
    return (float)_steeringParams.min_steering_analog + degrees_delta;
}


float SteeringSystem::_convert_digital_sensor(const SteeringSensorData_s &current_steering_data)
{
    // Same logic for digital
    uint32_t raw_val = current_steering_data.steering_sensor_digital; 

    int32_t count_delta = (int32_t)raw_val - (int32_t)_steeringParams.min_steering_signal_digital;

    float degrees_delta = (float)count_delta * _steeringParams.deg_per_count;

    return (float)_steeringParams.min_steering_digital + degrees_delta;
}

//_steeringParams.deg_per_count
