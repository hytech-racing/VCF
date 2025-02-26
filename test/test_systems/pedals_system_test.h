#define PEDALS_SYSTEM_TEST
#include <gtest/gtest.h>
#include <string>
#include "PedalsSystem.h"
#include "SharedFirmwareTypes.h"
#include <array>

float get_pedal_conversion_val(float min, float max, float data)
{
    float scale = 1.0f / (max - min);
    return ((data - min) * scale);
}

PedalsParams gen_positive_and_negative_slope_params()
{
    PedalsParams params;
    params.min_pedal_1 = 90;
    params.max_pedal_1 = 4000;
    params.min_pedal_2 = 4000;
    params.max_pedal_2 = 90;
    params.min_sensor_pedal_1 = 90;
    params.min_sensor_pedal_2 = 90;
    params.max_sensor_pedal_1 = 4000;
    params.max_sensor_pedal_2 = 4000;
    params.activation_percentage = 0.25;
    params.mechanical_activation_percentage = 0.4;
    params.deadzone_margin = .03;
    params.implausibility_margin = 0.1;
    return params;
}

PedalsParams gen_positive_slope_only_params()
{
    PedalsParams params;
    params.min_pedal_1 = 90;
    params.max_pedal_1 = 4000;
    params.min_pedal_2 = 90;
    params.max_pedal_2 = 4000;
    params.min_sensor_pedal_1 = 90;
    params.max_sensor_pedal_1 = 4000;
    params.min_sensor_pedal_2 = 90;
    params.max_sensor_pedal_2 = 4000;
    params.activation_percentage = 0.1;
    params.mechanical_activation_percentage = 0.4;
    params.deadzone_margin = 0.05;
    params.implausibility_margin = 0.1;
    return params;
}

// returns true if implausibility has exceeded max duration for double brake test
bool get_result_of_double_brake_test(PedalsSystem &pedals, const PedalSensorData_s &sensor_data)
{
    auto data = pedals.evaluate_pedals(sensor_data, 1000);
    data = pedals.evaluate_pedals(sensor_data, 1110);
    return data.implausibility_has_exceeded_max_duration;
}

// // resets implausibility time and returns true always
// bool reset_pedals_system_implaus_time(PedalsSystem &pedals)
// {
//     // Populating the test data with plausible values for the pedals
//     PedalSensorData_s test_pedal_data;
//     test_pedal_data.accel_1 = 90;
//     test_pedal_data.accel_2 = 3900;
//     test_pedal_data.brake_1 = 90;
//     test_pedal_data.brake_2 = 90;

//     auto data = pedals.evaluate_pedals(test_pedal_data, 1000);
//     data = pedals.evaluate_pedals(test_pedal_data, 1110);

//     // Always returns true because the plausible values were used
//     return (!data.implausibility_has_exceeded_max_duration);
// }

// resets implausibility time and returns true always
bool reset_pedals_system_implaus_time(PedalsSystem &pedals)
{
    // Populating the test data with plausible values for the pedals
    PedalSensorData_s test_pedal_data;
    test_pedal_data.accel_1 = 94;
    test_pedal_data.accel_2 = 3996;
    test_pedal_data.brake_1 = 94;
    test_pedal_data.brake_2 = 3996;

    auto data = pedals.evaluate_pedals(test_pedal_data, 1000);
    data = pedals.evaluate_pedals(test_pedal_data, 1110);

    // Always returns true because the plausible values were used
    return (!data.implausibility_has_exceeded_max_duration);
}


// T.4.3.4, T.4.2.7, T.4.2.10 FSAE rules 2025 v1
TEST(PedalsSystemTesting, test_accel_and_brake_limits_plausibility)
{
    auto params = gen_positive_and_negative_slope_params();
    PedalsSystem pedals(params, params);

    // Create test pedal data
    PedalSensorData_s test_pedal_good_val = {2045, 2045, 94, 3996}; // All values in a good range

    // Create a vector of test cases for acceleration sensor implausibility
    std::vector<PedalSensorData_s> accel_test_cases = {
        {0, 5000, 94, 3996},
        {5000, 0, 94, 3996}

    };

    // Create a vector of test cases for brake sensor implausibility
    std::vector<PedalSensorData_s> brake_test_cases = {
        {94, 3996, 0, 5000},
        {94, 3996, 5000, 0}
    };

    // T.4.2.4 and T.4.2.10 (accel out of ranges min/max) testing
    for (const auto& test : accel_test_cases)
    {
        // Test double brake mode
        bool t_4_2_7 = get_result_of_double_brake_test(pedals, test);
        EXPECT_TRUE(t_4_2_7); // Expecting implausibility duration to be exceed for all because all values are out of range and beyond 100 ms duration
        EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));
    }

    // Ensure that all good is still good for double brake mode
    bool t_4_2_7 = get_result_of_double_brake_test(pedals, test_pedal_good_val);
    EXPECT_FALSE(t_4_2_7);
    EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));

    // // T.4.3.4 brake testing
    for (const auto& test : brake_test_cases)
    {
        // Test double brake mode
        bool t_4_3_4 = get_result_of_double_brake_test(pedals, test);
        EXPECT_TRUE(t_4_3_4);
        EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));
    }

    // // Ensure that all good is still good for double brake mode
    bool t_4_3_4 = get_result_of_double_brake_test(pedals, test_pedal_good_val);
    EXPECT_FALSE(t_4_3_4);
    EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));
}

// T.4.2.4 FSAE rules 2025 v1 (accel vals not within 10 percent of each other)
TEST(PedalsSystemTesting, test_accel_and_brake_percentages_implausibility)
{
    auto accel_params = gen_positive_and_negative_slope_params();
    auto brake_params = gen_positive_and_negative_slope_params();
    PedalsSystem pedals(accel_params, brake_params);

    PedalSensorData_s test_pedal_half_pressed_one = {2045, 94, 94, 3996};
    PedalSensorData_s test_pedal_half_pressed_two = {94, 2045, 94, 3996};
    PedalSensorData_s test_pedal_not_pressed = {94, 3996, 94, 3996};
    PedalSensorData_s test_pedal_half_pressed = {2045, 2045, 94, 3996};

    std::vector<std::tuple<PedalSensorData_s, bool>> test_cases = {
        {test_pedal_half_pressed_one, true},
        {test_pedal_half_pressed_two, true},
        {test_pedal_not_pressed, false},
        {test_pedal_half_pressed, false},
    };

    for (const auto& test_case : test_cases)
    {
        const auto& sensor_data = std::get<0>(test_case);
        bool expected_result = std::get<1>(test_case);

        bool res = get_result_of_double_brake_test(pedals, sensor_data);
        EXPECT_EQ(res, expected_result);
        EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));
    }
}

// EV.4.7.1 and EV 4.7.2 FSAE rules 2025 v1
TEST(PedalsSystemTesting, test_accel_and_brake_pressed_at_same_time_and_activation)
{
    auto accel_params = gen_positive_and_negative_slope_params();
    auto brake_params = gen_positive_and_negative_slope_params();
    PedalsSystem pedals(accel_params, brake_params);

    // testing with example half pressed values
    PedalSensorData_s test_pedal_val_half_pressed = {2045, 2045, 2045, 2045};

    auto data = pedals.evaluate_pedals(test_pedal_val_half_pressed, 1000);
    EXPECT_TRUE(data.accel_is_pressed);
    EXPECT_TRUE(data.brake_is_pressed);
    EXPECT_TRUE(data.brake_and_accel_pressed_implausibility_high);
    
    // remove pressing both pedals
    PedalSensorData_s test_pedal_val_unpressed = {94, 3996, 94, 3996};
    data = pedals.evaluate_pedals(test_pedal_val_unpressed, 1100);
    EXPECT_FALSE(data.accel_is_pressed);
    EXPECT_FALSE(data.brake_is_pressed);
    EXPECT_FALSE(data.brake_and_accel_pressed_implausibility_high);

    // Future Implementation: test with real values from the car
}

// T.4.2.5 FSAE rules 2025 v1
TEST(PedalsSystemTesting, test_implausibility_duration)
{
    auto accel_params = gen_positive_and_negative_slope_params();
    auto brake_params = gen_positive_and_negative_slope_params();
    PedalsSystem pedals(accel_params, brake_params);

    PedalSensorData_s test_pedal_data = {2045, 2045, 2045, 2045};
    
    // Testing accel and brake pressed together
    auto data = pedals.evaluate_pedals(test_pedal_data, 1000);
    EXPECT_TRUE(data.brake_and_accel_pressed_implausibility_high);
    EXPECT_TRUE(data.brake_is_pressed);
    EXPECT_TRUE(data.accel_is_pressed);
    EXPECT_FALSE(data.implausibility_has_exceeded_max_duration);

    data = pedals.evaluate_pedals(test_pedal_data, 1110);
    EXPECT_TRUE(data.implausibility_has_exceeded_max_duration);
}

// EV.4.7.2 a FSAE rules 2025 v1
TEST(PedalsSystemTesting, implausibility_latching_until_accel_released_double_brake)
{
    auto accel_params = gen_positive_and_negative_slope_params();
    auto brake_params = gen_positive_and_negative_slope_params();
    PedalsSystem pedals(accel_params, brake_params);

    // create test data with both pedals pressed
    PedalSensorData_s test_pedal_data = {2045, 2045, 2045, 2045};

    // create an implausibility in the acceleration pedal
    EXPECT_TRUE(get_result_of_double_brake_test(pedals, test_pedal_data));

    // make acceleration pedal go out of range
    test_pedal_data.accel_1 = 5000;
    test_pedal_data.accel_2 = 0;
    EXPECT_TRUE(get_result_of_double_brake_test(pedals, test_pedal_data));

    // Release acceleration pedal and ensure implausibility sets to false
    test_pedal_data.accel_1 = 94;
    test_pedal_data.accel_2 = 3996;
    test_pedal_data.brake_1 = 94;
    test_pedal_data.brake_2 = 3996;
    EXPECT_FALSE(get_result_of_double_brake_test(pedals, test_pedal_data));
}

// testing accel and brake pedal percentage accuracies with deadzone
TEST(PedalsSystemTesting, deadzone_removal_calc_double_brake_ped)
{
    auto accel_params = gen_positive_and_negative_slope_params();
    auto brake_params = gen_positive_and_negative_slope_params();
    PedalsSystem params(accel_params, brake_params);

    // Test accel pedal with good values (0%, 100%)
    PedalSensorData_s test_pedal_data = {94, 3996, 94, 3996};
    auto data = params.evaluate_pedals(test_pedal_data, 1000);
    EXPECT_NEAR(data.accel_percent, 0.0, 0.001);

    test_pedal_data = {2045, 2045, 94, 3996};
    data = params.evaluate_pedals(test_pedal_data, 1100);
    EXPECT_NEAR(data.accel_percent, 0.5, 0.001);

    test_pedal_data = {3996, 94, 94, 3996};
    data = params.evaluate_pedals(test_pedal_data, 1200);
    EXPECT_NEAR(data.accel_percent, 1, .001);

    // Testing brake pedal with good values (0%, 50%, 100%)
    PedalSensorData_s test_brake_pedal_data = {94, 3996, 94, 3996};
    data = params.evaluate_pedals(test_brake_pedal_data, 1000);
    EXPECT_NEAR(data.brake_percent, 0.0, 0.001);

    test_brake_pedal_data = {94, 3996, 2045, 2045};
    data = params.evaluate_pedals(test_brake_pedal_data, 1100);
    EXPECT_NEAR(data.brake_percent, 0.5, .001);

    test_brake_pedal_data = {94, 3996, 3996, 94};
    data = params.evaluate_pedals(test_brake_pedal_data, 1200);
    EXPECT_NEAR(data.brake_percent, 1, .001);

}

// testing mechanical brake and brake activation
TEST(PedalsSystemTesting, brake_value_testing_double)
{
    PedalSensorData_s test_pedal_data = {94, 3996, 872, 3218};
    auto params = gen_positive_and_negative_slope_params();
    
    params.deadzone_margin = 0;
    PedalsSystem pedals(params, params);

    auto data = pedals.evaluate_pedals(test_pedal_data, 1000);
    EXPECT_NEAR(data.brake_percent, 0.2, 0.001);
    EXPECT_FALSE(data.brake_is_pressed);
    EXPECT_FALSE(data.mech_brake_is_active);

    test_pedal_data = {94, 3996, 2045, 2045};
    data = pedals.evaluate_pedals(test_pedal_data, 1100);
    EXPECT_NEAR(data.brake_percent, 0.5, 0.001);
    EXPECT_TRUE(data.brake_is_pressed);
    EXPECT_TRUE(data.mech_brake_is_active);
}

// checking to see that accel pedal can never be negative
TEST(PedalsSystemTesting, check_accel_never_negative_double)
{
    PedalSensorData_s test_pedal_data = {static_cast<uint32_t>(-1000), static_cast<uint32_t>(-3000), 94, 3996};
    auto params = gen_positive_and_negative_slope_params();

    PedalsSystem pedals(params, params);
    params.deadzone_margin = 0;

    auto data = pedals.evaluate_pedals(test_pedal_data, 1000);
    EXPECT_TRUE(data.accel_is_implausible);
    EXPECT_EQ(data.accel_percent, 0.0);
}

// testing that accel pedal marks as pressed
TEST(PedalsSystemTesting, check_accel_pressed)
{
    PedalSensorData_s test_pedal_data = {2045, 2045, 94, 3996};
    auto params = gen_positive_and_negative_slope_params();

    PedalsSystem pedals(params, params);

    auto data = pedals.evaluate_pedals(test_pedal_data, 1000);
    EXPECT_TRUE(data.accel_is_pressed);    

    // Is supposed to fail, will be 0.2
    test_pedal_data = {872, 3218, 94, 3996};
    auto params2 = gen_positive_and_negative_slope_params();

    PedalsSystem pedals2(params2, params2);
    params2.deadzone_margin = 0;

    data = pedals2.evaluate_pedals(test_pedal_data, 1000);
    EXPECT_FALSE(data.accel_is_pressed);
}

// testing that accel percent and accel implaus is marked when pedals are out of range
TEST(PedalsSystemTesting, check_accel_oor)
{
    PedalSensorData_s test_pedal_oor_hi_val_accel = {5000, 0, 94, 3996};
    PedalSensorData_s test_pedal_oor_lo_val_accel = {0, 5000, 94, 3996};

    auto params = gen_positive_and_negative_slope_params();
    PedalsSystem pedals(params, params);

    auto data_oor_hi = pedals.evaluate_pedals(test_pedal_oor_hi_val_accel, 1000);
    EXPECT_NEAR(data_oor_hi.accel_percent, 0.0, 0.001);
    EXPECT_TRUE(data_oor_hi.accel_is_implausible);
    auto data_oor_lo = pedals.evaluate_pedals(test_pedal_oor_lo_val_accel, 1000);
    EXPECT_NEAR(data_oor_lo.accel_percent, 0.0, 0.001);
    EXPECT_TRUE(data_oor_lo.accel_is_implausible);
}