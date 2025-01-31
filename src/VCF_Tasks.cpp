#include "VCF_Tasks.h"
#include "VCF_Globals.h"



bool init_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    adc_1.setChannelScaleAndOffset(STEERING_1_CHANNEL, STEERING_1_SCALE, STEERING_1_OFFSET);
    adc_1.setChannelScaleAndOffset(STEERING_2_CHANNEL, STEERING_2_SCALE, STEERING_2_OFFSET);
    adc_1.setChannelScaleAndOffset(FR_SUS_POT_CHANNEL, FR_SUS_POT_SCALE, FR_SUS_POT_OFFSET);
    adc_1.setChannelScaleAndOffset(FR_LOADCELL_CHANNEL, FR_LOADCELL_SCALE, FR_LOADCELL_OFFSET);
    adc_1.setChannelScaleAndOffset(FL_SUS_POT_CHANNEL, FL_SUS_POT_SCALE, FL_SUS_POT_OFFSET);
    adc_1.setChannelScaleAndOffset(FL_LOADCELL_CHANNEL, FL_LOADCELL_SCALE, FL_LOADCELL_OFFSET);
    return true;
}
bool run_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    adc_1.sample(); // Samples all eight channels.
    adc_1.convert(); // Converts all eight channels.

    interface_data.steering_data.analog_steering_degrees = adc_1.data.conversions[STEERING_1_CHANNEL].conversion; // Only using steering 1 for now
    interface_data.front_loadcell_data.FL_loadcell_analog = adc_1.data.conversions[FL_LOADCELL_CHANNEL].conversion;
    interface_data.front_loadcell_data.FR_loadcell_analog = adc_1.data.conversions[FR_LOADCELL_CHANNEL].conversion;
    interface_data.front_suspot_data.FL_sus_pot_analog = adc_1.data.conversions[FL_SUS_POT_CHANNEL].raw; // Just use raw for suspots
    interface_data.front_suspot_data.FR_sus_pot_analog = adc_1.data.conversions[FR_SUS_POT_CHANNEL].raw; // Just use raw for suspots

    return true;
}
HT_TASK::Task read_adc1_task = HT_TASK::Task(init_read_adc1_task, run_read_adc1_task, 10, 1000UL); // 1000us is 1kHz //NOLINT



bool init_read_adc2_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    adc_2.setChannelScaleAndOffset(ACCEL_1_CHANNEL, ACCEL_1_SCALE, ACCEL_1_OFFSET);
    adc_2.setChannelScaleAndOffset(ACCEL_2_CHANNEL, ACCEL_2_SCALE, ACCEL_2_OFFSET);
    adc_2.setChannelScaleAndOffset(BRAKE_1_CHANNEL, BRAKE_1_SCALE, BRAKE_1_OFFSET);
    adc_2.setChannelScaleAndOffset(BRAKE_2_CHANNEL, BRAKE_2_SCALE, BRAKE_2_OFFSET);

    return true;
}
bool run_read_adc2_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    adc_2.sample(); // Samples all eight channels.
    adc_2.convert(); // Converts all eight channels.

    interface_data.pedals_data.accel_1 = adc_2.data.conversions[ACCEL_1_CHANNEL].conversion;
    interface_data.pedals_data.accel_2 = adc_2.data.conversions[ACCEL_2_CHANNEL].conversion;
    interface_data.pedals_data.brake_1 = adc_2.data.conversions[BRAKE_1_CHANNEL].conversion;
    interface_data.pedals_data.brake_2 = adc_2.data.conversions[BRAKE_2_CHANNEL].conversion;

    return true;
}

bool init_buzzer_control_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    pinMode(BUZZER_CONTROL_PIN, OUTPUT);

    return true;
}
bool run_buzzer_control_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    digitalWrite(BUZZER_CONTROL_PIN, vcr_system_data.buzzer_is_active);
    
    return true;
}

HT_TASK::Task read_adc2_task = HT_TASK::Task(init_read_adc2_task, run_read_adc2_task, 10, 1000UL); // 1000us is 1kHz //NOLINT
