#include "VCF_Tasks.h"
#include "VCF_Globals.h"
#include "VCF_Constants.h"


bool init_adc_task()
{
    // Initilize one at a time to remove dependence on implicit ordering to match channels.
    float adc_1_scales[channels_within_mcp_adc], adc_1_offsets[channels_within_mcp_adc], adc_2_scales[channels_within_mcp_adc], adc_2_offsets[channels_within_mcp_adc];
    adc_1_scales[STEERING_1_CHANNEL] = STEERING_1_SCALE;
    adc_1_offsets[STEERING_1_CHANNEL] = STEERING_1_OFFSET;
    adc_1_scales[STEERING_2_CHANNEL] = STEERING_2_SCALE;
    adc_1_offsets[STEERING_2_CHANNEL] = STEERING_2_OFFSET;
    adc_1_scales[FR_SUS_POT_CHANNEL] = FR_SUS_POT_SCALE;
    adc_1_offsets[FR_SUS_POT_CHANNEL] = FR_SUS_POT_OFFSET;
    adc_1_scales[FL_SUS_POT_CHANNEL] = FL_SUS_POT_SCALE;
    adc_1_offsets[FL_SUS_POT_CHANNEL] = FL_SUS_POT_OFFSET;
    adc_1_scales[FR_LOADCELL_CHANNEL] = FR_LOADCELL_SCALE;
    adc_1_offsets[FR_LOADCELL_CHANNEL] = FR_LOADCELL_OFFSET;
    adc_1_scales[FL_LOADCELL_CHANNEL] = FL_LOADCELL_SCALE;
    adc_1_offsets[FL_LOADCELL_CHANNEL] = FL_LOADCELL_OFFSET;

    adc_2_scales[ACCEL_1_CHANNEL] = ACCEL_1_SCALE;
    adc_2_offsets[ACCEL_1_CHANNEL] = ACCEL_1_OFFSET;
    adc_2_scales[ACCEL_2_CHANNEL] = ACCEL_2_SCALE;
    adc_2_offsets[ACCEL_2_CHANNEL] = ACCEL_2_OFFSET;
    adc_2_scales[BRAKE_1_CHANNEL] = BRAKE_1_SCALE;
    adc_2_offsets[BRAKE_1_CHANNEL] = BRAKE_1_OFFSET;
    adc_2_scales[BRAKE_2_CHANNEL] = BRAKE_2_SCALE;
    adc_2_offsets[BRAKE_2_CHANNEL] = BRAKE_2_OFFSET;

    ADCsOnVCFInstance::create(adc_1_scales, adc_1_offsets, adc_2_scales, adc_2_offsets);

    return true;
}
bool run_read_adc1_task()
{
    // Samples all eight channels.
    ADCsOnVCFInstance::instance().adc_1.tick();

    interface_data.steering_data.analog_steering_degrees = ADCsOnVCFInstance::instance().adc_1.data.conversions[STEERING_1_CHANNEL].conversion; // Only using steering 1 for now
    interface_data.front_loadcell_data.FL_loadcell_analog = ADCsOnVCFInstance::instance().adc_1.data.conversions[FL_LOADCELL_CHANNEL].conversion;
    interface_data.front_loadcell_data.FR_loadcell_analog = ADCsOnVCFInstance::instance().adc_1.data.conversions[FR_LOADCELL_CHANNEL].conversion;
    interface_data.front_suspot_data.FL_sus_pot_analog = ADCsOnVCFInstance::instance().adc_1.data.conversions[FL_SUS_POT_CHANNEL].raw; // Just use raw for suspots
    interface_data.front_suspot_data.FR_sus_pot_analog = ADCsOnVCFInstance::instance().adc_1.data.conversions[FR_SUS_POT_CHANNEL].raw; // Just use raw for suspots

    return true;
}

bool run_read_adc2_task()
{
    // Samples all eight channels.
    ADCsOnVCFInstance::instance().adc_2.tick();

    interface_data.pedal_sensor_data.accel_1 = ADCsOnVCFInstance::instance().adc_2.data.conversions[ACCEL_1_CHANNEL].conversion;
    interface_data.pedal_sensor_data.accel_2 = ADCsOnVCFInstance::instance().adc_2.data.conversions[ACCEL_2_CHANNEL].conversion;
    interface_data.pedal_sensor_data.brake_1 = ADCsOnVCFInstance::instance().adc_2.data.conversions[BRAKE_1_CHANNEL].conversion;
    interface_data.pedal_sensor_data.brake_2 = ADCsOnVCFInstance::instance().adc_2.data.conversions[BRAKE_2_CHANNEL].conversion;

    return true;
}

bool init_read_gpio_task()
{
    // Setting digital/analog buttons D10-D6, A8 as inputs
    pinMode(BTN_DIM_READ, INPUT);
    pinMode(BTN_PRESET_READ, INPUT);
    pinMode(BTN_MC_CYCLE_READ, INPUT);
    pinMode(BTN_MODE_READ, INPUT);
    pinMode(BTN_START_READ, INPUT);
    pinMode(BTN_DATA_READ, INPUT);
    
    return true;
}
bool run_read_gpio_task()
{
    // Doing digital read on all digital inputs
    int dimButton = digitalRead(BTN_DIM_READ);
    int presetButton = digitalRead(BTN_PRESET_READ);
    int mcCycleButton = digitalRead(BTN_MC_CYCLE_READ);
    int modeButton = digitalRead(BTN_MODE_READ);
    int startButton = digitalRead(BTN_START_READ);
    int dataButton = digitalRead(BTN_DATA_READ);
    
    interface_data.dash_input_state.dim_btn_is_pressed = dimButton;
    interface_data.dash_input_state.preset_btn_is_pressed = presetButton;
    interface_data.dash_input_state.mc_reset_btn_is_pressed = mcCycleButton;
    interface_data.dash_input_state.mode_btn_is_pressed = modeButton;
    interface_data.dash_input_state.start_btn_is_pressed = startButton;
    interface_data.dash_input_state.data_btn_is_pressed = dataButton;

    return true;
}

bool init_buzzer_control_task()
{
    pinMode(BUZZER_CONTROL_PIN, OUTPUT);

    return true;
}
bool run_buzzer_control_task()
{
    digitalWrite(BUZZER_CONTROL_PIN, vcr_system_data.buzzer_is_active);
    
    return true;
}
