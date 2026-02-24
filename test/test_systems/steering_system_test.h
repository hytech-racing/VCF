#define STEERING_SYSTEM_TEST
#include <gtest/gtest.h>
#include <string>
#include "SteeringSystem.h"
#include "SharedFirmwareTypes.h"
#include <array>

#include <iostream>

SteeringParams_s gen_analog_and_digital_params(){
    SteeringParams_s params;
    //hard code the parmas for sensors
    params.min_steering_signal_analog = 0;
    params.max_steering_signal_digital = 4096;//actual hard coded
    params.min_steering_signal_digital = 0;
    params.max_steering_signal_digital = 8000; //testing values

    params.analog_tol_deg = 0.11377778f;
    params.digital_tol_deg = 0.2f;
    params.max_dtheta_threshold = 5;//change
    params.error_between_sensors_tolerance = 0.31377778f;
    return params;
}




void debug_print_steering(const SteeringSystemData_s& data){
    std::cout<<"analog_steering_angle: "<<data.analog_steering_angle<<" deg\n";
    std::cout<<"digital_steering_angle: "<<data.digital_steering_angle<<" deg\n";
    std::cout<<"output_steering_angle: "<<data.output_steering_angle<<" deg\n";
    std::cout<<"analog_oor_implausability: "<<data.analog_oor_implausability<<"\n";
    std::cout<<"digital_oor_implausability: "<<data.digital_oor_implausability<<"\n";
    std::cout<<"sensor_disagreement_implausability: "<<data.sensor_disagreement_implausability<<"\n";
    std::cout<<"dtheta_exceeded_analog: "<<data.dtheta_exceeded_analog<<"\n";
    std::cout<<"dtheta_exceeded_digital: "<<data.dtheta_exceeded_digital<<"\n";
    

}

TEST(SteeringSystemTesting, test_good_sensors)
{
    SteeringParams_s params;
    params.min_steering_signal_analog = 0;
    params.max_steering_signal_digital = 4096;//actual hard coded
    params.min_steering_signal_digital = 0;
    params.max_steering_signal_digital = 8000; //testing values

    params.analog_tol_deg = 0.11377778f;
    params.digital_tol_deg = 0.2f;
    params.max_dtheta_threshold = 5;//change
    params.error_between_sensors_tolerance = 0.31377778f;
    return params;
}
