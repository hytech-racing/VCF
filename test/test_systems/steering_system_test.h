#define STEERING_SYSTEM_TEST
#include <gtest/gtest.h>
#include <string>
#include "SteeringSystem.h"
#include "SharedFirmwareTypes.h"
#include <array>

#include <iostream>

SteeringParams_s gen_default_params(){
    SteeringParams_s params;
    //hard code the parmas for sensors
    params.min_steering_signal_analog = 0;
    params.max_steering_signal_analog = 4096;//actual hard coded
    
    params.min_steering_signal_digital = 0;
    params.max_steering_signal_digital = 8000; //testing values

    params.deg_per_count_analog = 0.087890625f;
    params.deg_per_count_digital = 0.02197265625f;

    params.analog_tol_deg = 0.11377778f;
    params.digital_tol_deg = 0.2f;

    params.max_dtheta_threshold = 5.0f;//change
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

static SteeringSensorData_s make_raw(uint32_t analog_adc, uint32_t digital_adc){
    SteeringSensorData_s raw{};
    raw.analog_steering_degrees = analog_adc;
    raw.digital_steering_analog = digital_adc;
}

static SteeringSystemData_s tick(SteeringSystem& sys,uint32_t analog_raw, uint32_t digital_raw, uint32_t t_ms){
    return sys.evaluate_steering(make_raw(analog_raw,digital_raw),t_ms);
}

static SteeringSystemData_s tick2(SteeringSystem & sys, uint32_t analog_raw, uint32_t digital_raw, uint32_t t0, uint32_t t1){
    (void)tick(sys,analog_raw,digital_raw,t0);
    return(tick(sys,analog_raw,digital_raw,t1)); //get the value of timestamp between evalute steerings
}

static float expected_angle_from_midpoint(uint32_t raw, uint32_t min, uint32_t max, float deg_per_count)
{
    uint32_t midpoint = (min + max) /2
    int32_t offset = static_cast<int32_t>(raw) - static_cast<int32_t>(midpoint);
    return static_cast<float>(offset) * deg_per_count
}


/* commented out initial draft because too general
TEST(SteeringSystemTesting, test_good_sensors)
{
    auto params = gen_default_params();
    SteeringSystem steering(params);
    SteeringSensorData_s sensor{};
    sensor.analog_steering_Degrees = 2048;
    sensor.digital_steering_analog = 4000;

    auto data = steering.evaluate_steering(sensor, 1000); //simulated timestamp for tests


    EXPECT_FALSE(data.sensor_disagreement_implausibility);

}
*/
TEST(SteeringSystemTesting, test_adc_to_degree_conversion)
{
    auto prams = gen_default_params();
    SteeringSystem steering(params);

    uint32_t analog_mid = (params.min_steering_signal_analog + params.max_steering_signal_analog) / 2;
}

TEST(SteeringSystemTesting, test_out_of_bounds_raw_signals){

    auto params = gen_default_params();
    SteeringSystem steering(params);


    //Check for a good value first
    SteeringSensorData_s good_val = {};
    good_val.analog_steering_degrees = 2100;
    good_val.digital_steering_analog = 4100;
    auto data = steering.evaluate_steering(good_val, 1000);
    EXPECT_FALSE(data.analog_oor_implausability);
    EXPECT_FALSE(data.digital_oor_implausability);

    //OOR High
    SteeringSensorData_s high_val = {5000,9000};
    data = steering.evaluate_steering(high_val, 1010);
    EXPECT_TRUE(data.analog_oor_implausability);
    EXPECT_TRUE(data.digital_oor_implausability);

    //OOR Low
    SteeringSensorData_s low_val = {0, 0};
    data = steering.evaluate_steering(low_val, 1020);
    EXPECT_TRUE(data.analog_oor_implausability);
    EXPECT_TRUE(data.digital_oor_implausability);

}

TEST(SteeringSystemTesting, test_detect_jumps_dtheta){
    auto params = gen_default_params();
    SteeringSystem steering(params);

    SteeringSensorData_s baseline = {2100,4100};

    //First run (just to verify that we don't get a dtheta exceeded on the first run since we have no previous data to compare to)
    auto data = steering.evaluate_steering(baseline, 1000);
    EXPECT_FALSE(data.dtheta_exceeded_analog);
    EXPECT_FALSE(data.dtheta_exceeded_digital);
    
    //Now verify again when dt is zero we don't get a dtheta exceeded since we can't divide by zero/
    SteeringSensorData_s massive_jump = {4100,8100};
    data = steering.evaluate_steering(massive_jump, 1000);
    EXPECT_FALSE(data.dtheta_exceeded_analog);
    EXPECT_FALSE(data.dtheta_exceeded_digital);

    //Small motion valid
    SteeringSensorData_s small_motion = {2110,4110};
    data = steering.evaluate_steering(small_motion, 1010); //advance time by 10 ms
    EXPECT_FALSE(data.dtheta_exceeded_analog);
    EXPECT_FALSE(data.dtheta_exceeded_digital);

    //Big motion NOT valid
    data = steering.evaluate_steering(massive_jump, 1020); //advance time by another 10 ms
    EXPECT_TRUE(data.dtheta_exceeded_analog);
    EXPECT_TRUE(data.dtheta_exceeded_digital);
}



TEST(SteeringSystemTesting, test_sensor_disagreemnet)
{
    auto params = gen_default_params();
    SteeringSystem steering(params);

    SteeringSensorData_s base{};
    base.analog_steering_degrees = 2000;
    base.digital_steering_analog = 4000;

    steering.evaluate_steering(base, 1000);

    //create disagreement between sensors to test
    SteeringSensorData_s disagree{};
    disagree.analog_steering_degrees = 2000; //stays same
    disagree.digital_steering_analog = 7000; //large offset from analog

    auto data = steering.evaluate_steering(disagree, 1100);
    EXPECT_TRUE(data.sensor_diagreement_implausibility);
} 


TEST(SteeringSystemTesting,test_sensor_output_logic){

    auto params = gen_standard_params();
    SteeringSystem steering(params);

    //When both valid and agreeing, we default to digital
    SteeringSensorData_s both_valid = {2100,4100};
    auto data = steering.evaluate_steering(both_valid, 1000);
    EXPECT_NEAR(data.output_steering)
    

}