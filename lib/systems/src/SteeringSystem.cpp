#include <math.h>
#include "SteeringSystem.h"


void SteeringSystem::recalibrate_steering(const SteeringSensorData_s &current_steering_data, bool calibration_is_on){
    //get current raw angles
    const uint32_t analog_raw  = static_cast<uint32_t>(current_steering_data.steering_sensor_analog);
    const uint32_t digital_raw = static_cast<uint32_t>(current_steering_data.steering_sensor_digital);
   // _update_observed_limits(analog_raw, digital_raw);
    
    //button just pressed ->recalibration window
    if (calibration_is_on && !_calibrating){
        _calibrating = true;
        _min_observed_analog  = 4096;
        _max_observed_analog  = 0;
        _min_observed_digital = 4096;
        _max_observed_digital = 0;
    }
    
    //update relative extremeties
    if (calibration_is_on){
        _min_observed_analog = std::min(_min_observed_analog, analog_raw);
        _max_observed_analog = std::max(_max_observed_analog, analog_raw);
        _min_observed_digital = std::min(_min_observed_digital, digital_raw);
        _max_observed_digital = std::max(_max_observed_digital, digital_raw);
        return;
    }
    //button released ->commit the values
    if (!calibration_is_on && _calibrating) {
        _calibrating = false;
        _steeringParams.min_steering_signal_analog  = _min_observed_analog;
        _steeringParams.max_steering_signal_analog  = _max_observed_analog;
        _steeringParams.min_steering_signal_digital = _min_observed_digital;
        _steeringParams.max_steering_signal_digital = _max_observed_digital;
    }
        // swaps  min & max in the params if sensor is negative
    if (_steeringParams.min_steering_signal_analog > _steeringParams.max_steering_signal_analog) {
        std::swap(_steeringParams.min_steering_signal_analog, _steeringParams.max_steering_signal_analog);
    }
    if (_steeringParams.min_steering_signal_digital > _steeringParams.max_steering_signal_digital)
    {
        std::swap(_steeringParams.min_steering_signal_digital,_steeringParams.max_steering_signal_digital);
    }

    
}
bool SteeringSystem::_evaluate_steering_oor_analog(uint32_t steering_analog)
        {
            //check if the analog value is within bounds +- error
            const uint32_t span = _steeringParams.max_steering_signal_analog - _steeringParams.min_steering_signal_analog;
            const uint32_t analog_margin_counts = static_cast<uint32_t>(_steeringParams.analog_tol_deg * span);
            const uint32_t min_ok = _steeringParams.min_steering_signal_analog - analog_margin_counts;
            const uint32_t max_ok = _steeringParams.max_steering_signal_analog + analog_margin_counts;
            return (steering_analog < min_ok || steering_analog > max_ok);
        }

bool SteeringSystem::_evaluate_steering_oor_digital(uint32_t steering_digital,
        int min_sensor_value_digital,
        int max_sensor_value_digital)
        {
        //check if the digital value is within bounds +- error
        
            const uint32_t digital_margin_counts = static_cast<uint32_t>(_steeringParams.digital_tol_deg / _steeringParams.deg_per_count);
            const uint32_t min_ok = _steeringParams.min_steering_signal_digital - digital_margin_counts;
            const uint32_t max_ok = _steeringParams.max_steering_signal_digital + digital_margin_counts;
            return (steering_digital < min_ok || steering_digital > max_ok);
        }

bool SteeringSystem::_evaluate_steering_dtheta_exceeded(float dtheta){
           return (dtheta > _steeringParams.max_dtheta_threshold);
        }






float SteeringSystem::_convert_analog_sensor(const SteeringSensorData_s &current_steering_data)
{
    // Get the raw value
    uint32_t raw_val = current_steering_data.steering_sensor_analog;

    //  Calculate how far we are from the minimum raw count delta
    uint32_t count_delta = (int32_t)raw_val - (int32_t)_steeringParams.min_steering_signal_analog;

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

SteeringSystemData_s SteeringSystem::evaluate_steering(const SteeringSensorData_s &current_steering_data, uint32_t current_millis){
    SteeringSystemData_s out = {};// initialize struct to be blank


    //Conversion from raw ADC to degrees
    out.analog_steering_angle = _convert_analog_sensor(current_steering_data);
    out.digital_steering_angle = _convert_digital_sensor(current_steering_data);
    
    uint32_t dt = current_millis - _prev_timestamp;

    if(!_first_run && dt > 0){ //check that we not on the first run which would mean no previous data
        float dtheta_analog = fabsf(out.analog_steering_angle - _prev_analog_angle);
        float dtheta_digital = fabsf(out.digital_steering_angle - _prev_digital_angle);

        out.steering_velocity_deg_ms = dtheta_analog / dt; //need to add separate velocity for digital and analog, for now just have analog
        out.digital_steering_velocity_deg_ms = dtheta_digital / dt;


        //Check if either sensor moved too fast
        if (_evaluate_steering_dtheta_exceeded(dtheta_analog)){
            out.dtheta_exceeded_analog = true;
        }
        if (_evaluate_steering_dtheta_exceeded(dtheta_digital)) {
            out.dtheta_exceeded_digital = true;
        }   


        //Check if either sensor is out of range:

        out.analog_oor_implausibility = _evaluate_steering_oor_analog(current_steering_data.steering_sensor_analog, _steeringParams.min_steering_signal_analog, _steeringParams.max_steering_signal_analog);
        out.digital_oor_implausibility = _evaluate_steering_oor_digital(current_steering_data.steering_sensor_digital, _steeringParams.min_steering_signal_digital, _steeringParams.max_steering_signal_digital);


        //Check if there is too much of a difference between sensor values
        float difference = fabsf(out.analog_steering_angle - out.digital_steering_angle);

        const float full_scale_deg = fabsf(_steeringParams.max_steering_signal_analog - _steeringParams.min_steering_signal_analog);
        const float analog_tol_deg = _steeringParams.analog_tol_deg *full_scale_deg;
        const float digital_tol_deg = _steeringParams.digital_tol_deg;
        const float allowed_difference = analog_tol_deg+digital_tol_deg; //the allowed difference equals digital error+ analog error
        //digital error is +- 0.2 degrees, constant regardless of angle, while analog depends on steering angle at 0.5%
        bool sensors_agree = difference <= allowed_difference; //steeringParams.error

        
        
        //Still to do: create an algorithm/ checklist to determine which sensor we trust more,
        //or, if we should have an algorithm to have a weighted calculation based on both values
        bool analog_valid = !out.analog_oor_implausibility && !out.dtheta_exceeded_analog;
        bool digital_valid = !out.digital_oor_implausibility && !out.dtheta_exceeded_digital;
        if (analog_valid && digital_valid) 
        {
            //if sensors have acceptable difference, use average as steering angle, otherwise default to analog
            if (sensors_agree) 
            {
                out.output_steering_angle = 0.5f * (out.analog_steering_angle + out.digital_steering_angle);
            }
            else 
            {
                out.output_steering_angle = out.analog_steering_angle;
            }
        }
        else if (analog_valid) 
        { 
            out.output_steering_angle = out.analog_steering_angle;
        }
        else if (digital_valid) {
            out.output_steering_angle = out.digital_steering_angle;
        }
        else // if both sensors fail
        {
            // alert or something
        }

        
        
    }
    //Update states

        _prev_analog_angle = out.analog_steering_angle;
        _prev_digital_angle = out.digital_steering_angle;
        _prev_timestamp = current_millis;
        _first_run = false;

        return out;
}

void SteeringSystem::update_steering_system() {
    
    _steeringSystemData = evaluate_steering(/* real current_steering_data location*/);
}











