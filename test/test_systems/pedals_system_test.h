#define PEDALS_SYSTEM_TEST
#include <gtest/gtest.h>
#include <string>
#include "PedalsSystem.h"
#include "SharedFirmwareTypes.h"
#include <array>

/* 
Struct from shared firmware types that has pedal system data
struct PedalsSystemData_s
{
    bool accel_is_implausible : 1; // Checks if either accel pedal is out of range OR they disagree by more than 10%
    bool brake_is_implausible : 1; // Checks if brake sensor is out of range.
    bool brake_is_pressed : 1; // True if brake pedal is pressed beyond the specified activationPercentage.
    bool accel_is_pressed : 1; // True if the accel pedal is pressed beyond the specified activationPercentage.
    bool mech_brake_is_active : 1; // True if the brake pedal is pressed beyond mechanical_activation_percentage.
    bool brake_and_accel_pressed_implausibility_high : 1; // If accel is pressed at all while mech_brake_is_active.
    bool implausibility_has_exceeded_max_duration : 1; // True if implausibility lasts more than 100ms
    float accel_percent;
    float brake_percent;
    float regen_percent; // When brake pedal is 0%, regen_percent is 0.0. When brakes are at mechanical_activation_percentage,
                         // regen_percent is at 1.0. For instance, if mech activation percentage was 60%, then when brake
                         // travel is at 40%, regen_percent would be 0.667. Beyond that, regen_percent is clamped to 1.0.
};

Struct from shared firmware types that has pedal sensor data
struct PedalSensorData_s
{
    uint32_t accel_1;
    uint32_t accel_2;
    uint32_t brake_1;
    uint32_t brake_2;
};
*/

float get_pedal_conversion_val(float min, float max, float data)
{
    float scale = 1.0f / (max - min);
    return ((data - min) * scale);
}

PedalsParams gen_positive_and_negative_slope_params()
{
    PedalsParams params;
    params.min_pedal_1 = 2000;
    params.max_pedal_1 = 1000;
    params.min_pedal_2 = 1000;
    params.max_pedal_2 = 2000;
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
    params.min_pedal_1 = 1000;
    params.max_pedal_1 = 2000;
    params.min_pedal_2 = 1000;
    params.max_pedal_2 = 2000;
    params.min_sensor_pedal_1 = 0;
    params.max_sensor_pedal_1 = 4000;
    params.min_sensor_pedal_2 = 0;
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
    auto data = pedals.evaluate_pedals(sensor_data, 1000, true);
    data = pedals.evaluate_pedals(sensor_data, 1110, true);
    return data.implausibility_has_exceeded_max_duration;
}

// returns true if implausibility has exceeded max duration for single brake test
bool get_result_of_single_brake_test(PedalsSystem &pedals, const PedalSensorData_s &sensor_data)
{
    auto data = pedals.evaluate_pedals(sensor_data, 1000, false);
    data = pedals.evaluate_pedals(sensor_data, 1110, false);
    return data.implausibility_has_exceeded_max_duration;
}

// resets implausibility time and returns true always
bool reset_pedals_system_implaus_time(PedalsSystem &pedals)
{
    // Populating the test data with plausible values for the pedals
    PedalSensorData_s test_pedal_data;
    test_pedal_data.accel_1 = 1010;
    test_pedal_data.accel_2 = 1010;
    test_pedal_data.brake_1 = 1010;
    test_pedal_data.brake_2 = 1010;

    auto data = pedals.evaluate_pedals(test_pedal_data, 1000, true);
    data = pedals.evaluate_pedals(test_pedal_data, 1110, true);

    // Always returns true because the plausible values were used
    return (!data.implausibility_has_exceeded_max_duration);
}

// T.4.3.4, T.4.2.7, T.4.2.9, T.4.2.10 FSAE rules 2024 v1
TEST(PedalsSystemTesting, test_accel_and_brake_limits_plausibility)
{
    auto params = gen_positive_and_negative_slope_params();
    PedalsSystem pedals(params, params);

    // Create test pedal data
    PedalSensorData_s test_pedal_good_val = {1200, 1200, 1200, 1200}; // All values in a good range

    // Create a vector of test cases for acceleration sensor implausibility
    std::vector<PedalSensorData_s> accel_test_cases = {
        {0, 1200, 1200, 1200},
        {4000, 1200, 1200, 1200},
        {1200, 0, 1200, 1200},
        {1200, 4000, 1200, 1200},
        {0, 4000, 1200, 1200},
        {4000, 0, 1200, 1200}
    };

    // Create a vector of test cases for brake sensor implausibility
    std::vector<PedalSensorData_s> brake_test_cases = {
        {1200, 1200, 0, 1200},
        {1200, 1200, 4000, 1200},
        {1200, 1200, 1200, 0},
        {1200, 1200, 1200, 4000},
        {1200, 1200, 0, 4000},
        {1200, 1200, 4000, 0}
    };

    // T.4.2.7 , T.4.2.9 and T.4.2.10 (accel out of ranges min/max) testing
    for (const auto& test : accel_test_cases)
    {
        // Test double brake mode
        bool t_4_2_7 = get_result_of_double_brake_test(pedals, test);
        EXPECT_TRUE(t_4_2_7); // Expecting implausibility duration to be exceed for all because all values are out of range and beyond 100 ms duration
        EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));

        // Test single brake mode
        t_4_2_7 = get_result_of_single_brake_test(pedals, test);
        EXPECT_TRUE(t_4_2_7);
        EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));
    }

     // Ensure that all good is still good for double brake mode
    bool t_4_2_7 = get_result_of_double_brake_test(pedals, test_pedal_good_val);
    EXPECT_FALSE(t_4_2_7);
    EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));

    // Ensure that all good is still good for single brake mode
    t_4_2_7 = get_result_of_single_brake_test(pedals, test_pedal_good_val);
    printf("t_4_2_7: %d\n", t_4_2_7);
    // EXPECT_FALSE(t_4_2_7);
    EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));

    // T.4.3.4 brake testing
    for (const auto& test : brake_test_cases)
    {
        // Test double brake mode
        bool t_4_3_4 = get_result_of_double_brake_test(pedals, test);
        EXPECT_TRUE(t_4_3_4);
        EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));

        // Test single brake mode
        t_4_3_4 = get_result_of_single_brake_test(pedals, test);
        printf("t_4_3_4: %d\n", t_4_3_4);
        EXPECT_TRUE(t_4_3_4);
        EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));
    }

    // // Ensure that all good is still good for double brake mode
    bool t_4_3_4 = get_result_of_double_brake_test(pedals, test_pedal_good_val);
    EXPECT_FALSE(t_4_3_4);
    EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));

    // // Ensure that all good is still good for single brake mode
    t_4_3_4 = get_result_of_single_brake_test(pedals, test_pedal_good_val);
    // EXPECT_FALSE(t_4_3_4);
    EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));
}

//T.4.2.4 FSAE rules 2024 v1 (accel vals not within 10 percent of each other)
TEST(PedalsSystemTesting, test_accel_and_brake_percentages_implausibility)
{
    auto accel_params = gen_positive_and_negative_slope_params();
    auto brake_params = gen_positive_slope_only_params();
    PedalsSystem pedals(accel_params, brake_params);

    PedalSensorData_s test_pedal_neg_slope_not_pressed = {2000, 1000, 1200, 1200};
    PedalSensorData_s test_pedal_half_pressed = {1500, 1500, 1200, 1200};
    PedalSensorData_s test_pedal_pos_slope_not_pressed = {1000, 2000, 1200, 1200};

    std::vector<std::tuple<PedalSensorData_s, bool>> test_cases = {
        {test_pedal_neg_slope_not_pressed, true},
        {test_pedal_pos_slope_not_pressed, true},
        // {test_pedal_half_pressed, false},
    };

    for (const auto& test_case : test_cases)
    {
        const auto& sensor_data = std::get<0>(test_case);
        bool expected_result = std::get<1>(test_case);

        bool res = get_result_of_double_brake_test(pedals, sensor_data);
        printf("Double brake res: %d\n", res);
        EXPECT_EQ(res, expected_result);
        EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));

        res = get_result_of_single_brake_test(pedals, sensor_data);
        printf("Single brake res: %d\n", res);
        EXPECT_EQ(res, expected_result);
        EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));
    }
}

TEST(PedalsSystemTesting, test_accel_and_brake_pressed_at_same_time_and_activation)
{
    auto accel_params = gen_positive_and_negative_slope_params();
    auto brake_params = gen_positive_slope_only_params();
    PedalsSystem pedals(accel_params, brake_params);

    // testing with example half pressed values
    PedalSensorData_s test_pedal_val_half_pressed = {1500, 1500, 1500, 1500};

    EXPECT_TRUE(get_result_of_double_brake_test(pedals, test_pedal_val_half_pressed)); // gives true because both pedals are pressed (brake_and_accel_pressed_implausibility_high)
    EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));
    EXPECT_TRUE(get_result_of_single_brake_test(pedals, test_pedal_val_half_pressed)); // gives true because both pedals are pressed (brake_and_accel_pressed_implausibility_high)
    EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));
}

