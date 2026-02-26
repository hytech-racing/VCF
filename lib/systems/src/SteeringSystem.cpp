#include <cmath>
#include "SteeringSystem.h"
#include <cstdint>




void SteeringSystem::recalibrate_steering_digital(const SteeringSensorData_s &current_steering_data, bool calibration_is_on) {
    //get current raw angles
    const uint32_t curr_digital_raw = static_cast<uint32_t>(current_steering_data.digital_steering_analog); //NOLINT will eventually be uint32
   
    //button just pressed ->recalibration window
    if (calibration_is_on && !_calibrating){
        _calibrating = true;
        _steeringParams.min_observed_digital = UINT32_MAX; //establishes a big number that will be greater than the readings
        _steeringParams.max_observed_digital = 0;
    }
    
    if (calibration_is_on && _calibrating) {
        update_observed_steering_limits(current_steering_data);
    }
    //update relative extremeties
    // assigning min and max observed during calibration only vs. constantly?
    // if (calibration_is_on){
    //     _steeringParams.min_observed_digital = std::min(_steeringParams.min_observed_digital, curr_digital_raw);
    //     _steeringParams.max_observed_digital = std::max(_steeringParams.max_observed_digital, curr_digital_raw);
    //     return;
    // }

    //button released ->commit the values
    if (!calibration_is_on && _calibrating) {

        _calibrating = false;
        _steeringParams.min_steering_signal_digital = _steeringParams.min_observed_digital;
        _steeringParams.max_steering_signal_digital = _steeringParams.max_observed_digital;
        // swaps  min & max in the params if sensor is flipped
        if (_steeringParams.min_steering_signal_digital > _steeringParams.max_steering_signal_digital)
        {
            std::swap(_steeringParams.min_steering_signal_digital,_steeringParams.max_steering_signal_digital);
        }
        _steeringParams.span_signal_digital = _steeringParams.max_steering_signal_digital-_steeringParams.min_steering_signal_digital;
        _steeringParams.analog_tol_deg = static_cast<float>(_steeringParams.span_signal_analog) * _steeringParams.analog_tol * _steeringParams.deg_per_count_analog;
    } 
}


bool SteeringSystem::_evaluate_steering_oor_analog(uint32_t steering_analog_raw) // RAW
        {

            //check if the analog value is within bounds +- error
            const int32_t analog_margin_counts = static_cast<int32_t>(_steeringParams.analog_tol * static_cast<float>(_steeringParams.span_signal_analog));
            const int32_t min_ok = static_cast<int32_t>(_steeringParams.min_steering_signal_analog) - analog_margin_counts;
            const int32_t max_ok = static_cast<int32_t>(_steeringParams.max_steering_signal_analog) + analog_margin_counts;
            return (static_cast<int32_t>(steering_analog_raw) < min_ok || static_cast<int32_t>(steering_analog_raw) > max_ok);
            
        }


bool SteeringSystem::_evaluate_steering_oor_digital(uint32_t steering_digital_raw) // RAW
        {
        //check if the digital value is within bounds +- error
            
            const int32_t digital_margin_counts = static_cast<int32_t>(_steeringParams.digital_tol_deg /_steeringParams.deg_per_count_digital);
            const int32_t min_ok = static_cast<int32_t>(_steeringParams.min_steering_signal_digital) - digital_margin_counts;
            const int32_t max_ok = static_cast<int32_t>(_steeringParams.max_steering_signal_digital) + digital_margin_counts;
            return (static_cast<int32_t>(steering_digital_raw) < min_ok || static_cast<int32_t>(steering_digital_raw) > max_ok);
        }


bool SteeringSystem::_evaluate_steering_dtheta_exceeded(float dtheta){
           return (fabs(dtheta) > _steeringParams.max_dtheta_threshold);
        }




float SteeringSystem::_convert_analog_sensor(const SteeringSensorData_s &current_steering_data)
{
    // Get the raw value
    const uint32_t curr_raw_analog = current_steering_data.analog_steering_degrees; //NOLINT should be uint32_t later
    const uint32_t analog_midpoint = (_steeringParams.max_steering_signal_analog + _steeringParams.min_steering_signal_analog) / 2;
   
    const int32_t curr_analog_offset = static_cast<int32_t>(curr_raw_analog) - static_cast<int32_t>(analog_midpoint);
   
    const float curr_analog_degrees = static_cast<float>(curr_analog_offset) * _steeringParams.deg_per_count_analog;


    return curr_analog_degrees;
    //  Calculate how far we are from the minimum raw count delta
    // const uint32_t count_delta = (int32_t)raw_val - (int32_t)_steeringParams.min_steering_signal_analog;


    // //  Convert that delta into degrees
    // const float degrees_delta = count_delta * _steeringParams.deg_per_count_analog;


    // // Add to the starting minimum angle to get absolute position
    // return kMindeg + degrees_delta;
}




float SteeringSystem::_convert_digital_sensor(const SteeringSensorData_s &current_steering_data)
{
    // Same logic for digital
    const uint32_t curr_raw_digital = current_steering_data.digital_steering_analog; //NOLINT should be uint32_t later
    //create a span for the digital sensor


    //degree span we are assuming:
    //_steeringParams.deg_per_count_digital = kSpan/(float)span_digital; -> we are assuming deg_per_count hardcoded from datasheet
    const uint32_t digital_midpoint = ((_steeringParams.max_steering_signal_digital+_steeringParams.min_steering_signal_digital)/2);


    const int32_t curr_digital_offset = static_cast<int32_t>(curr_raw_digital) - static_cast<int32_t>(digital_midpoint);
    float curr_digital_degrees = (float)curr_digital_offset * _steeringParams.deg_per_count_digital;
    return curr_digital_degrees;
}



void SteeringSystem::update_observed_steering_limits(const SteeringSensorData_s &current_steering_data) {
    _steeringParams.min_observed_digital = std::min(_steeringParams.min_observed_digital, current_steering_data.digital_steering_analog);
    _steeringParams.max_observed_digital = std::max(_steeringParams.max_observed_digital, current_steering_data.digital_steering_analog);
}

SteeringSystemData_s SteeringSystem::evaluate_steering(const SteeringSensorData_s &current_steering_data, const uint32_t current_millis){
    
    SteeringSystemData_s out = {};// initialize struct to be blank

    // Reset flags
    out.digital_oor_implausibility = false;
    out.analog_oor_implausibility = false;
    out.sensor_disagreement_implausibility = false;
    out.dtheta_exceeded_analog = false;
    out.dtheta_exceeded_digital = false;
    out.both_sensors_fail = false;

    //Conversion from raw ADC to degrees
    out.analog_steering_angle = _convert_analog_sensor(current_steering_data);
    out.digital_steering_angle = _convert_digital_sensor(current_steering_data);
    

    uint32_t dt = current_millis - _prev_timestamp; //current_millis is seperate data input


    if(!_first_run && dt > 0){ //check that we not on the first run which would mean no previous data
        float dtheta_analog = out.analog_steering_angle - _prev_analog_angle; //prev_angle established in last run
        float dtheta_digital = out.digital_steering_angle - _prev_digital_angle;
        out.analog_steering_velocity_deg_s = (dtheta_analog / dt) * 1000.0f; //NOLINT ms to s
        out.digital_steering_velocity_deg_s = (dtheta_digital / dt) * 1000.0f; //NOLINT ms to s


        //Check if either sensor moved too much in one tick
        out.dtheta_exceeded_analog = _evaluate_steering_dtheta_exceeded(dtheta_analog);
        out.dtheta_exceeded_digital = _evaluate_steering_dtheta_exceeded(dtheta_digital);


        //Check if either sensor is out of range (pass in raw)
        out.analog_oor_implausibility = _evaluate_steering_oor_analog(current_steering_data.analog_steering_degrees);
        out.digital_oor_implausibility = _evaluate_steering_oor_digital(current_steering_data.digital_steering_analog);

        //Check if there is too much of a difference between sensor values
        float sensor_difference = std::fabs(out.analog_steering_angle - out.digital_steering_angle);

        // const float full_scale_deg_analog = fabsf(_steeringParams.max_steering_signal_analog - _steeringParams.min_steering_signal_analog); //in adc
        // const float analog_tol = _steeringParams.analog_tol*full_scale_deg_analog*_steeringParams.deg_per_count_analog; // analog percentage*adc span *deg/adc count
        const float allowed_difference = _steeringParams.analog_tol_deg + _steeringParams.digital_tol_deg; //the allowed difference equals digital error+ analog error
        //digital error is +- 0.2 degrees, constant regardless of angle, while analog depends on steering angle at 0.5%
        bool sensors_agree = (sensor_difference <= allowed_difference); //steeringParams.error
        out.sensor_disagreement_implausibility = !sensors_agree;

    
    
        //create an algorithm/ checklist to determine which sensor we trust more,
        //or, if we should have an algorithm to have a weighted calculation based on both values
        bool analog_valid = !out.analog_oor_implausibility && !out.dtheta_exceeded_analog;
        bool digital_valid = !out.digital_oor_implausibility && !out.dtheta_exceeded_digital;

        if (analog_valid && digital_valid)
        {
            //if sensors have acceptable difference, use digital as steering angle
            if (sensors_agree)
            {
                out.output_steering_angle = out.digital_steering_angle;
            }
            else
            {
                out.output_steering_angle = out.digital_steering_angle; //default to original, but we need to consider what we really want to put here
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
            out.output_steering_angle = _prev_digital_angle;
            out.both_sensors_fail = true;
        }
    }
    //Update states


    _prev_analog_angle = out.analog_steering_angle;
    _prev_digital_angle = out.digital_steering_angle;
    _prev_timestamp = current_millis;
    _first_run = false;
    

    return out;
    
}




