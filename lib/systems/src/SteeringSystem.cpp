#include <cmath>
#include <cstdint>
#include "SteeringSystem.h"
#include "SteeringEncoderInterface.h"

void SteeringSystem::recalibrate_steering_digital(const uint32_t analog_raw, const uint32_t digital_raw) {
    //get current raw angles
    const uint32_t curr_digital_raw = digital_raw;

    _steeringParams.min_steering_signal_analog = min_observed_analog;
    _steeringParams.max_steering_signal_analog = max_observed_analog;
    _steeringParams.min_steering_signal_digital = min_observed_digital;
    _steeringParams.max_steering_signal_digital = max_observed_digital;
    // swaps  min & max in the params if sensor is flipped
    if (_steeringParams.min_steering_signal_digital > _steeringParams.max_steering_signal_digital) {
        std::swap(_steeringParams.min_steering_signal_digital, _steeringParams.max_steering_signal_digital);
    }
    if (_steeringParams.min_steering_signal_analog > _steeringParams.max_steering_signal_analog) {
        std::swap(_steeringParams.min_steering_signal_analog, _steeringParams.max_steering_signal_analog);
    }
    _steeringParams.span_signal_digital = _steeringParams.max_steering_signal_digital-_steeringParams.min_steering_signal_digital;
    _steeringParams.analog_tol_deg = static_cast<float>(_steeringParams.span_signal_analog) * _steeringParams.analog_tol * _steeringParams.deg_per_count_analog;
    _steeringParams.digital_midpoint = static_cast<int32_t>((_steeringParams.max_steering_signal_digital + _steeringParams.min_steering_signal_digital) / 2); //NOLINT
    _steeringParams.analog_midpoint = static_cast<int32_t>((_steeringParams.max_steering_signal_analog + _steeringParams.min_steering_signal_analog) / 2); //NOLINT
    const int32_t analog_margin_counts = static_cast<int32_t>(_steeringParams.analog_tol * static_cast<float>(_steeringParams.span_signal_analog)); //NOLINT
    const int32_t digital_margin_counts = static_cast<int32_t>(_steeringParams.digital_tol_deg /_steeringParams.deg_per_count_digital); //NOLINT
    _steeringParams.analog_min_with_margins = static_cast<int32_t>(_steeringParams.min_steering_signal_analog) - analog_margin_counts;
    _steeringParams.analog_max_with_margins = static_cast<int32_t>(_steeringParams.max_steering_signal_analog) + analog_margin_counts;
    _steeringParams.digital_min_with_margins = static_cast<int32_t>(_steeringParams.min_steering_signal_digital) - digital_margin_counts;
    _steeringParams.digital_max_with_margins = static_cast<int32_t>(_steeringParams.max_steering_signal_digital) + digital_margin_counts;
    _steeringParams.error_between_sensors_tolerance = _steeringParams.analog_tol_deg + _steeringParams.digital_tol_deg;

    _calibrating = false; //TODO: this and end_calibration_state likely redundant, either use this or have button call end_calibration_state

    // Reset observed values
    min_observed_digital = UINT32_MAX;
    max_observed_digital = 0;
    min_observed_analog = UINT32_MAX;
    max_observed_analog = 0;
}

void SteeringSystem::begin_calibration_state() {
    _calibrating = true;
}

void SteeringSystem::end_calibration_state() {
    _calibrating = false;
}

bool SteeringSystem::is_calibrating() {
    return _calibrating;
}

void SteeringSystem::evaluate_steering(const uint32_t analog_raw, const SteeringEncoderReading_s digital_data, const uint32_t current_millis) {
    // Reset flags
    _steeringSystemData.digital_oor_implausibility = false;
    _steeringSystemData.analog_oor_implausibility = false;
    _steeringSystemData.sensor_disagreement_implausibility = false;
    _steeringSystemData.dtheta_exceeded_analog = false;
    _steeringSystemData.dtheta_exceeded_digital = false;
    _steeringSystemData.both_sensors_fail = false;

    const uint32_t digital_raw = digital_data.rawValue;

    SteeringEncoderStatus_e digital_status = digital_data.status;
    bool digital_fault = (digital_status == SteeringEncoderStatus_e::ERROR);
    _steeringSystemData.interface_sensor_error = digital_fault;
    _steeringSystemData.digital_raw = digital_fault ? 0U : digital_raw;

    _steeringSystemData.analog_raw = analog_raw;

    //Conversion from raw ADC to degrees
    _steeringSystemData.analog_steering_angle = _convert_analog_sensor(analog_raw);
    _steeringSystemData.digital_steering_angle = digital_fault ? 0.0f : _convert_digital_sensor(digital_raw);
    
    uint32_t dt = current_millis - _prev_timestamp; //current_millis is seperate data input  

    if (!_first_run && dt > 0) { //check that we not on the first run which would mean no previous data
        float dtheta_analog = _steeringSystemData.analog_steering_angle - _prev_analog_angle; //prev_angle established in last run
        float dtheta_digital = _steeringSystemData.digital_steering_angle - _prev_digital_angle;
        _steeringSystemData.analog_steering_velocity_deg_s = (dtheta_analog / dt) * 1000.0f; //NOLINT ms to s
        _steeringSystemData.digital_steering_velocity_deg_s = (dtheta_digital / dt) * 1000.0f; //NOLINT ms to s

        //Check if either sensor moved too much in one tick
        _steeringSystemData.dtheta_exceeded_analog = _evaluate_steering_dtheta_exceeded(dtheta_analog);
        _steeringSystemData.dtheta_exceeded_digital = _evaluate_steering_dtheta_exceeded(dtheta_digital);

        //Check if either sensor is out of range (pass in raw)
        _steeringSystemData.analog_oor_implausibility = _evaluate_steering_oor_analog(static_cast<uint32_t>(analog_raw));
        _steeringSystemData.digital_oor_implausibility = _evaluate_steering_oor_digital(static_cast<uint32_t>(digital_raw)) || digital_fault;

        //Check if there is too much of a difference between sensor values
        float sensor_difference = std::fabs(_steeringSystemData.analog_steering_angle - _steeringSystemData.digital_steering_angle);
        bool sensors_agree = (sensor_difference <= _steeringParams.error_between_sensors_tolerance); //steeringParams.error
        _steeringSystemData.sensor_disagreement_implausibility = !sensors_agree;

        //create an algorithm/ checklist to determine which sensor we trust more,
        //or, if we should have an algorithm to have a weighted calculation based on both values
        bool analog_valid = !_steeringSystemData.analog_oor_implausibility && !_steeringSystemData.dtheta_exceeded_analog;
        bool digital_valid = !_steeringSystemData.digital_oor_implausibility && !_steeringSystemData.dtheta_exceeded_digital && !digital_fault;

        if (analog_valid && digital_valid) {
            //if sensors have acceptable difference, use digital as steering angle
            if (sensors_agree) {
                _steeringSystemData.output_steering_angle = _steeringSystemData.digital_steering_angle;
            } else {
                _steeringSystemData.output_steering_angle = _steeringSystemData.digital_steering_angle; //default to original, but we need to consider what we really want to put here
            }
        } else if (analog_valid) {
            _steeringSystemData.output_steering_angle = _steeringSystemData.analog_steering_angle;
        } else if (digital_valid) {
            _steeringSystemData.output_steering_angle = _steeringSystemData.digital_steering_angle;
        } else { // if both sensors fail
            _steeringSystemData.output_steering_angle = _prev_digital_angle;
            _steeringSystemData.both_sensors_fail = true;
        }
    }
    //Update states
    _prev_analog_angle = _steeringSystemData.analog_steering_angle;
    _prev_digital_angle = _steeringSystemData.digital_steering_angle;
    _prev_timestamp = current_millis;
    _first_run = false;
}

void SteeringSystem::update_observed_steering_limits(const uint32_t analog_raw, const uint32_t digital_raw) {
    min_observed_analog = std::min(min_observed_analog, static_cast<uint32_t>(analog_raw));
    max_observed_analog = std::max(max_observed_analog, static_cast<uint32_t>(analog_raw));
    min_observed_digital = std::min(min_observed_digital, static_cast<uint32_t>(digital_raw)); //NOLINT should both be uint32_t
    max_observed_digital = std::max(max_observed_digital, static_cast<uint32_t>(digital_raw)); //NOLINT ^
}

float SteeringSystem::_convert_digital_sensor(const uint32_t digital_raw) {
    // Same logic for digital
    const int32_t offset = static_cast<int32_t>(digital_raw) - _steeringParams.digital_midpoint; //NOLINT
    return static_cast<float>(offset) * _steeringParams.deg_per_count_digital;
}

float SteeringSystem::_convert_analog_sensor(const uint32_t analog_raw) {
    // Get the raw value
    const int32_t offset = static_cast<int32_t>(analog_raw) - _steeringParams.analog_midpoint; //NOLINT
    return static_cast<float>(offset) * _steeringParams.deg_per_count_analog;
}

bool SteeringSystem::_evaluate_steering_oor_analog(const uint32_t steering_analog_raw) { // RAW
    return (static_cast<int32_t>(steering_analog_raw) < _steeringParams.analog_min_with_margins || static_cast<int32_t>(steering_analog_raw) > _steeringParams.analog_max_with_margins);
}

bool SteeringSystem::_evaluate_steering_oor_digital(const uint32_t steering_digital_raw) {// RAW
    return (static_cast<int32_t>(steering_digital_raw) < _steeringParams.digital_min_with_margins || static_cast<int32_t>(steering_digital_raw) > _steeringParams.digital_max_with_margins);
}

bool SteeringSystem::_evaluate_steering_dtheta_exceeded(float dtheta){
    return (fabs(dtheta) > _steeringParams.max_dtheta_threshold);
}

