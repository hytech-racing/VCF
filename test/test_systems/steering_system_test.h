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
/* helper functions, however we are asserting the values in each function as variables
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

*/

TEST(SteeringSystemTesting, test_adc_to_degree_conversion)
{
    auto params = gen_default_params();
    SteeringSystem steering(params);

    uint32_t analog_mid = (params.min_steering_signal_analog + params.max_steering_signal_analog) / 2;
    uint32_t digital_mid = (params.min_steering_signal_digital + params.max_steering_signal_digital) / 2;
    
    //midpoints
    SteeringSensorData_s midpoint{};
    midpoint.analog_steering_degrees = analog_mid;
    midpoint.digital_steering_analog = digital_mid;
    auto data = steering.evaluate_steering(midpoint, 1000);

    EXPECT_NEAR(data.analog_steering_angle, 0.0f, 0.001f);
    EXPECT_NEAR(data.digital_steering_angle, 0.0f, 0.001f);

    //min values
    SteeringSensorData_s min_val{};
    min_val.analog_steering_degrees = params.min_steering_signal_analog;
    min_val.digital_steering_analog = params.min_steering_signal_digital;
    
    data = steering.evaluate_steering(min_val, 1010);

    float expected_analog_min = (static_cast<int32_t>(params.min_steering_signal_analog) - analog_mid) * params.deg_per_count_analog;
    float expected_digital_min = (static_cast<int32_t>(params.min_steering_signal_digital) - digital_mid) * params.deg_per_count_digital;

    EXPECT_NEAR(data.analog_steering_angle, expected_analog_min, 0.001f);
    EXPECT_NEAR(data.digital_steering_angle, expected_digital_min, 0.001f);

    //max values
    SteeringSensorData_s max_val{};
    max_val.analog_steering_degrees = params.max_steering_signal_analog;
    max_val.digital_steering_analog = params.max_steering_signal_digital;

    data = steering.evaluate_steering(max_val, 1020);
    float expected_analog_max = (static_cast<int32_t>(params.max_steering_signal_analog) - analog_mid) * params.deg_per_count_analog;
    float expected_digital_max = (static_cast<int32_t>(params.max_steering_signal_digital) - digital_mid) * params.deg_per_count_digital;

    EXPECT_NEAR(data.analog_steering_angle, expected_analog_max, 0.001f);
    EXPECT_NEAR(data.digital_steering_angle, expected_digital_max, 0.001f);
    
}

TEST(SteeringSystemTesting, test_out_of_bounds_raw_signals){

    auto params = gen_default_params();
    SteeringSystem steering(params);

    //Check for a good value first
    SteeringSensorData_s good_val = {};
    good_val.analog_steering_degrees = 2048;
    good_val.digital_steering_analog = 4000;
    auto data = steering.evaluate_steering(good_val, 1000);
    EXPECT_FALSE(data.analog_oor_implausibility);
    EXPECT_FALSE(data.digital_oor_implausibility);

    //OOR High
    SteeringSensorData_s high_val = {5000, 9000};
    data = steering.evaluate_steering(high_val, 1010);
    EXPECT_TRUE(data.analog_oor_implausibility);
    EXPECT_TRUE(data.digital_oor_implausibility);

    //OOR Low
    SteeringSensorData_s low_val = {static_cast<uint32_t>(-1000), static_cast<uint32_t>(-1000)};
    data = steering.evaluate_steering(low_val, 1020);
    EXPECT_TRUE(data.analog_oor_implausibility);
    EXPECT_TRUE(data.digital_oor_implausibility);

}

TEST(SteeringSystemTesting, test_detect_jumps_dtheta){
    auto params = gen_default_params();
    SteeringSystem steering(params);

    SteeringSensorData_s baseline = {2048, 4000};

    //First run (just to verify that we don't get a dtheta exceeded on the first run since we have no previous data to compare to)
    auto data = steering.evaluate_steering(baseline, 1000);
    EXPECT_FALSE(data.dtheta_exceeded_analog);
    EXPECT_FALSE(data.dtheta_exceeded_digital);
    
    //Now verify again when dt is zero we don't get a dtheta exceeded since we can't divide by zero/
    SteeringSensorData_s massive_jump = {4096, 8000};
    data = steering.evaluate_steering(massive_jump, 1000);
    EXPECT_FALSE(data.dtheta_exceeded_analog);
    EXPECT_FALSE(data.dtheta_exceeded_digital);

    //Small motion valid
    SteeringSensorData_s small_motion = {2060, 4050};
    data = steering.evaluate_steering(small_motion, 1010); //advance time by 10 ms
    EXPECT_FALSE(data.dtheta_exceeded_analog);
    EXPECT_FALSE(data.dtheta_exceeded_digital);

    //Big motion NOT valid
    data = steering.evaluate_steering(massive_jump, 1020); //advance time by another 10 ms
    EXPECT_TRUE(data.dtheta_exceeded_analog);
    EXPECT_TRUE(data.dtheta_exceeded_digital);
}


TEST(SteeringSystemTesting, test_sensor_disagreement)
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

    auto params = gen_default_params();
    SteeringSystem steering(params);

    //When both valid and agreeing, we default to digital
    SteeringSensorData_s both_valid = {2048,4000};
    auto data = steering.evaluate_steering(both_valid, 1000);
    EXPECT_NEAR(data.output_steering_angle, data.digital_steering_angle, 0.001f);
    EXPECT_FALSE(data.both_sensors_fail);
    EXPECT_FALSE(data.sensor_disagreement_implausibility);

    //When both valid but disagreeing, we default to digital
    SteeringSensorData_s both_valid_disagree = {2048, 7000};
    auto data = steering.evaluate_steering(both_valid_disagree, 1010);

    EXPECT_TRUE(data.sensor_disagreement_implausibility);
    EXPECT_FALSE(data.analog_oor_implausibility);
    EXPECT_FALSE(data.digital_oor_implausibility);
    EXPECT_NEAR(data.output_steering_angle, data.digital_steering_angle, 0.001f);

    //When analog is good but digital is bad, we put analog
    SteeringSensorData_s digital_bad = {2048, 9000};
    data = steering.evaluate_steering(digital_bad, 1020);
    EXPECT_TRUE(data.digital_oor_implausibility);
    EXPECT_FALSE(data.analog_oor_implausibility);
    EXPECT_NEAR(data.output_steering_angle, data.analog_steering_angle, 0.001f);

    //When digital is good but analog is bad, we put digital
    SteeringSensorData_s analog_bad = {5000, 4000};
    data = steering.evaluate_steering(analog_bad, 1030);
    EXPECT_TRUE(data.analog_oor_implausibility);
    EXPECT_FALSE(data.digital_oor_implausibility);
    EXPECT_NEAR(data.output_steering_angle, data.digital_steering_angle, 0.001f);

    //When both bad, we flagging that error
    SteeringSensorData_s both_bad = {5000, 9000};
    auto data = steering.evaluate_steering(both_bad, 1040);
    EXPECT_TRUE(data.analog_oor_implausibility);
    EXPECT_TRUE(data.digital_oor_implausibility);
    EXPECT_TRUE(data.both_sensors_fail);

    

}