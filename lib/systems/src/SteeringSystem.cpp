#include <math.h>
#include "SteeringSystem.h"


void SteeringSystem::recalibrate_steering_digital(const SteeringSensorData_s &current_steering_data, bool calibration_is_on){
    //get current raw angles
    const uint32_t curr_digital_raw = static_cast<uint32_t>(current_steering_data.digital_steering_analog);
    
    //button just pressed ->recalibration window
    if (calibration_is_on && !_calibrating){
        _calibrating = true;
        _steeringParams._min_observed_digital = 10000000; //establishes a big number that will be greater than the readings
        _steeringParams._max_observed_digital = 0;
    }
    
    //update relative extremeties
    if (calibration_is_on){
        _steeringParams._min_observed_digital = std::min(_steeringParams._min_observed_digital, curr_digital_raw);
        _steeringParams._max_observed_digital = std::max(_steeringParams._max_observed_digital, curr_digital_raw);
        return;
    }
    //button released ->commit the values
    if (!calibration_is_on && _calibrating) {
        _calibrating = false;
        _steeringParams.min_steering_signal_digital = _steeringParams._min_observed_digital;
        _steeringParams.max_steering_signal_digital = _steeringParams._max_observed_digital;
        return;
    }
        // swaps  min & max in the params if sensor is flipped

    
    if (_steeringParams.min_steering_signal_digital > _steeringParams.max_steering_signal_digital)
    {
        std::swap(_steeringParams.min_steering_signal_digital,_steeringParams.max_steering_signal_digital);
        return;
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

bool SteeringSystem::_evaluate_steering_oor_digital(uint32_t steering_digital)
        {
        //check if the digital value is within bounds +- error
        
            const uint32_t digital_margin_counts = static_cast<uint32_t>(_steeringParams.digital_tol_deg);
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
    const uint32_t raw_val = current_steering_data.analog_steering_degrees; //is this adc input value?

    constexpr float kMindeg = -90.0f;

    //  Calculate how far we are from the minimum raw count delta
    const uint32_t count_delta = (int32_t)raw_val - (int32_t)_steeringParams.min_steering_signal_analog;

    //  Convert that delta into degrees
    const float degrees_delta = count_delta * _steeringParams.deg_per_count_analog;

    // Add to the starting minimum angle to get absolute position
    return kMindeg + degrees_delta;
}


float SteeringSystem::_convert_digital_sensor(const SteeringSensorData_s &current_steering_data)
{
    // Same logic for digital
    const uint32_t raw_val = current_steering_data.digital_steering_analog; //adc value
    //create a span for the digital sensor
    const uint32_t span_digital = static_cast<uint32_t>(_steeringParams.max_steering_digital) - static_cast<uint32_t>(_steeringParams.min_steering_digital);

    //degree span we are assuming:
    constexpr float kMindeg = -90.0f;
    constexpr float kSpan = 180.0f;

    _steeringParams.deg_per_count_digital = kSpan/(float)span_digital;

    const int32_t count_delta = (int32_t)raw_val - (int32_t)_steeringParams.min_steering_signal_digital;

    float degrees_delta = (float)count_delta * _steeringParams.deg_per_count_digital;
    //output in degreees_delta
    return kMindeg + degrees_delta;
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

        out.analog_steering_velocity_deg_s = dtheta_analog / dt; 
        out.digital_steering_velocity_deg_s = dtheta_digital / dt;


        //Check if either sensor moved too much in one tick
        if (_evaluate_steering_dtheta_exceeded(dtheta_analog)){
            out.dtheta_exceeded_analog = true;
        }
        if (_evaluate_steering_dtheta_exceeded(dtheta_digital)) {
            out.dtheta_exceeded_digital = true;
        }   


        //Check if either sensor is out of range

        out.analog_oor_implausibility = _evaluate_steering_oor_analog(current_steering_data.analog_steering_degrees);
        out.digital_oor_implausibility = _evaluate_steering_oor_digital(current_steering_data.digital_steering_analog);


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
            //if sensors have acceptable difference, use average as steering angle, otherwise default to digital
            if (sensors_agree) 
            {
                out.output_steering_angle = 0.5f * (out.analog_steering_angle + out.digital_steering_angle);
            }
            else 
            {
                out.output_steering_angle = out.digital_steering_angle;//default to original, but we need to consider what we really want to put here
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
            out.analog_digital_disagreement = true;
        }

        
        
    }
    //Update states

        _prev_analog_angle = out.analog_steering_angle;
        _prev_digital_angle = out.digital_steering_angle;
        _prev_timestamp = current_millis;
        _first_run = false;

        return out;
}

// void SteeringSystem::update_steering_system() {
    
//     _steeringSystemData = evaluate_steering(/* real current_steering_data location*/);
// }











