#include <math.h>
#include "SteeringSystem.h"


bool SteeringSystem::_evaluate_steering_oor_analog(int steering_analog,
        int min_sensor_value_analog,
        int max_sensor_value_analog){
            
            return (steering_analog < min_sensor_value_analog || steering_analog > max_sensor_value_analog);

        }

bool SteeringSystem::_evaluate_steering_oor_digital(int steering_digital,
        int min_sensor_value_digital,
        int max_sensor_value_digital){
            if(steering_digital < min_sensor_value_digital || steering_digital > max_sensor_value_digital){
                SteeringParams.digital_oor_implausibility = true;
                return true;
            }
            return false;
        }

bool SteeringSystem::_evaluate_steering_dtheta_exceeded(float dt){
            if(dt > SteeringParams.max_dtheta_threshold){
                return true;
            }
            return false;
        }


float SteeringSystem::_convert_digital_sensor(const SteeringSensorData_s &current_steering_data)




//_steeringParams.deg_per_count
