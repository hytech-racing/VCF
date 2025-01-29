#include "VCF_Tasks.h"
#include "VCF_Globals.h"
#include "VCF_Constants.h"


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

    interface_data.steering_unfiltered.analog_steering_unfiltered_degrees = adc_1.data.conversions[STEERING_1_CHANNEL].conversion; // Only using steering 1 for now
    interface_data.front_loadcells_unfiltered.FL_loadcell_unfiltered_pounds = adc_1.data.conversions[FL_LOADCELL_CHANNEL].conversion;
    interface_data.front_loadcells_unfiltered.FR_loadcell_unfiltered_pounds = adc_1.data.conversions[FR_LOADCELL_CHANNEL].conversion;
    interface_data.front_suspots_unfiltered.FL_sus_pot_unfiltered_analog = adc_1.data.conversions[FL_SUS_POT_CHANNEL].raw; // Just use raw for suspots
    interface_data.front_suspots_unfiltered.FR_sus_pot_unfiltered_analog = adc_1.data.conversions[FR_SUS_POT_CHANNEL].raw; // Just use raw for suspots

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

    interface_data.pedals_unfiltered.accel1_unfiltered_percent = adc_2.data.conversions[ACCEL_1_CHANNEL].conversion;
    interface_data.pedals_unfiltered.accel2_unfiltered_percent = adc_2.data.conversions[ACCEL_2_CHANNEL].conversion;
    interface_data.pedals_unfiltered.brake1_unfiltered_percent = adc_2.data.conversions[BRAKE_1_CHANNEL].conversion;
    interface_data.pedals_unfiltered.brake1_unfiltered_percent = adc_2.data.conversions[BRAKE_2_CHANNEL].conversion;

    return true;
}

bool init_read_gpio_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // Setting digital/analog buttons D10-D6, A8 as inputs
    pinMode(BTN_DIM_READ, INPUT);
    pinMode(BTN_PRESET_READ, INPUT);
    pinMode(BTN_MC_CYCLE_READ, INPUT);
    pinMode(BTN_MODE_READ, INPUT);
    pinMode(BTN_START_READ, INPUT);
    // pinMode(BTN_DATA_READ, INPUT); this is an analog pin, so technically no need for pinMode as analogRead will setup the pin
    
    return true;
}
bool run_read_gpio_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // Doing digital read on all digital inputs
    int dimButton = digitalRead(BTN_DIM_READ);
    int presetButton = digitalRead(BTN_PRESET_READ);
    int mcCycleButton = digitalRead(BTN_MC_CYCLE_READ);
    int modeButton = digitalRead(BTN_MODE_READ);
    int startButton = digitalRead(BTN_START_READ);
    int dataButton = analogRead(BTN_DATA_READ); // this is on an analog pin, so technically need to use analogRead
    
    interface_data.dash_input_state.dim_btn_is_pressed = dimButton;
    interface_data.dash_input_state.preset_btn_is_pressed = presetButton;
    interface_data.dash_input_state.mc_cycle_btn_is_pressed = mcCycleButton;
    interface_data.dash_input_state.mode_btn_is_pressed = modeButton;
    interface_data.dash_input_state.start_btn_is_pressed = startButton;
    interface_data.dash_input_state.data_btn = dataButton;

    return true;
}

HT_TASK::Task read_adc2_task = HT_TASK::Task(init_read_adc2_task, run_read_adc2_task, 10, 1000UL); // 1000us is 1kHz //NOLINT
