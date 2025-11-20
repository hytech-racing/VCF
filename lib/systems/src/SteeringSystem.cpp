#include <math.h>
#include "SteeringSystem.h"

SteeringSystemData_s SteeringSystem::evaluate_steering(SteeringSensorData_s steering_data,
                                              unsigned long curr_micros)
{
    float dt = (_last_update_micros == 0) ? 0.0f : (curr_micros - _last_update_micros) / 1e6f;
    _last_update_micros = curr_micros;

    SteeringSystemData_s out = {};
    out.steering_is_implausible = _evaluate_steering_implausibilities(static_cast<int>(steering_data.analog_steering_degrees), static_cast<int>(_params.min_sensor_steering_1), static_cast<int>(_params.max_sensor_steering_1), static_cast<int>(_params.min_steering_1), static_cast<int>(_params.max_steering_1), _params.implausibility_margin, dt);

    _prev_steering_analog = steering_data.analog_steering_degrees;

    return out;
}

bool SteeringSystem::_evaluate_steering_dtheta(int steering_analog, float dt)
{
    if (dt <= 0.0f) return false;
    float dtheta = fabs(steering_analog - _prev_steering_analog) / dt;
    return (dtheta > _params.max_dtheta_threshold);
}

bool SteeringSystem::_evaluate_steering_implausibilities(int steering_analog, int min_sensor_value, int max_sensor_value, int min_steering_value, int max_steering_value, float implausibility_margin, float dt)
{
    bool steering_oor = _evaluate_steering_oor(steering_analog, min_sensor_value, max_sensor_value);
    bool steering_min_max_implaus = _evaluate_min_max_steering_implausibilities(steering_analog, min_steering_value, max_steering_value, implausibility_margin);
    bool steering_dtheta = _evaluate_steering_dtheta(steering_analog, dt);
    return steering_oor || steering_min_max_implaus || steering_dtheta;
}

bool SteeringSystem::_evaluate_min_max_steering_implausibilities(int steering_analog, int min_steering_value, int max_steering_value, float implausibility_margin)
{
    bool steering_swapped = false;
    float steering_margin = static_cast<float>(fabs(max_steering_value - min_steering_value)) * implausibility_margin;
    if(min_steering_value > max_steering_value){
        steering_swapped = true;
    }

    float float_steering_analog = static_cast<float>(steering_analog);
    float min_float = static_cast<float>(min_steering_value);
    float max_float = static_cast<float>(max_steering_value);
    bool steering_less_than_min = steering_swapped ? (float_steering_analog > (min_float + steering_margin)) : (float_steering_analog < (min_float - steering_margin)); 
    bool steering_greater_than_max = steering_swapped ? (float_steering_analog < (max_float - steering_margin)) : (float_steering_analog > (max_float + steering_margin)); 
    return steering_less_than_min || steering_greater_than_max;
}

bool SteeringSystem::_evaluate_steering_oor(int steering_analog, int min_sensor_value, int max_sensor_value)
{
    return (steering_analog < min_sensor_value || steering_analog > max_sensor_value);
}