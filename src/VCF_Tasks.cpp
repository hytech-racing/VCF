#include "VCF_Tasks.h"
#include "SharedFirmwareTypes.h"
#include "VCF_Globals.h"
#include "VCF_Constants.h"
#include <QNEthernet.h>
#include "ProtobufMsgInterface.h"
#include "EEPROMUtilities.h"
#include "ht_task.hpp"
#include "hytech.h"
#include "hytech_msgs.pb.h"
#include "VCFCANInterfaceImpl.h"
#include "CANInterface.h"
#include "VCRInterface.h"
#include "SystemTimeInterface.h"
#include "PedalsSystem.h"
#include "WatchdogSystem.h"

#include "WatchdogSystem.h"
#include "Arduino.h"

float apply_iir_filter(float alpha, float old_value, float new_value)
{
    return (alpha * new_value) + (1 - alpha) * (old_value);
}

HT_TASK::TaskResponse run_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // Samples all eight channels.
    ADCsOnVCFInstance::instance().adc_1.tick();

    VCFData_sInstance::instance().interface_data.steering_data.analog_steering_degrees = ADCsOnVCFInstance::instance().adc_1.data.conversions[STEERING_1_CHANNEL].conversion; // Only using steering 1 for now
    VCFData_sInstance::instance().interface_data.front_loadcell_data.FL_loadcell_analog = apply_iir_filter(LOADCELL_IIR_FILTER_ALPHA, VCFData_sInstance::instance().interface_data.front_loadcell_data.FL_loadcell_analog, ADCsOnVCFInstance::instance().adc_1.data.conversions[FL_LOADCELL_CHANNEL].conversion);
    VCFData_sInstance::instance().interface_data.front_loadcell_data.FR_loadcell_analog = apply_iir_filter(LOADCELL_IIR_FILTER_ALPHA, VCFData_sInstance::instance().interface_data.front_loadcell_data.FR_loadcell_analog, ADCsOnVCFInstance::instance().adc_1.data.conversions[FR_LOADCELL_CHANNEL].conversion);
    VCFData_sInstance::instance().interface_data.front_suspot_data.FL_sus_pot_analog = apply_iir_filter(LOADCELL_IIR_FILTER_ALPHA, VCFData_sInstance::instance().interface_data.front_suspot_data.FL_sus_pot_analog, ADCsOnVCFInstance::instance().adc_1.data.conversions[FL_SUS_POT_CHANNEL].raw);
    VCFData_sInstance::instance().interface_data.front_suspot_data.FR_sus_pot_analog = apply_iir_filter(LOADCELL_IIR_FILTER_ALPHA, VCFData_sInstance::instance().interface_data.front_suspot_data.FR_sus_pot_analog, ADCsOnVCFInstance::instance().adc_1.data.conversions[FR_SUS_POT_CHANNEL].raw);

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_read_adc2_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // Samples all eight channels.
    ADCsOnVCFInstance::instance().adc_2.tick();
    VCFData_sInstance::instance().interface_data.pedal_sensor_data.accel_1 = ADCsOnVCFInstance::instance().adc_2.data.conversions[ACCEL_1_CHANNEL].conversion;
    VCFData_sInstance::instance().interface_data.pedal_sensor_data.accel_2 = ADCsOnVCFInstance::instance().adc_2.data.conversions[ACCEL_2_CHANNEL].conversion;
    VCFData_sInstance::instance().interface_data.pedal_sensor_data.brake_1 = ADCsOnVCFInstance::instance().adc_2.data.conversions[BRAKE_1_CHANNEL].conversion;
    VCFData_sInstance::instance().interface_data.pedal_sensor_data.brake_2 = ADCsOnVCFInstance::instance().adc_2.data.conversions[BRAKE_2_CHANNEL].conversion;
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse init_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    WatchdogInstance::create(WATCHDOG_KICK_INTERVAL_MS); // NOLINT
    pinMode(WATCHDOG_PIN, OUTPUT);
    pinMode(SOFTWARE_OK_PIN, OUTPUT);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    digitalWrite(SOFTWARE_OK_PIN , HIGH);
    digitalWrite(WATCHDOG_PIN, WatchdogInstance::instance().get_watchdog_state(sys_time::hal_millis()));
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse update_pedals_calibration_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    // Observed pedal values (ONLY USED FOR RECALIBRATION)
    // WARNING: These are the true min/max observed values, NOT the "value at min travel" and "value at max travel"
    //          that are defined in the PedalsParam struct.
    PedalsSystemInstance::instance().update_observed_pedal_limits(VCFData_sInstance::instance().interface_data.pedal_sensor_data);

    if (VCRInterfaceInstance::instance().is_in_pedals_calibration_state())
    {
        PedalsSystemInstance::instance().recalibrate_min_max(VCFData_sInstance::instance().interface_data.pedal_sensor_data);
        EEPROMUtilities::write_eeprom_32bit(ACCEL_1_MIN_ADDR, PedalsSystemInstance::instance().get_accel_params().min_pedal_1);
        EEPROMUtilities::write_eeprom_32bit(ACCEL_1_MAX_ADDR, PedalsSystemInstance::instance().get_accel_params().max_pedal_1);
        EEPROMUtilities::write_eeprom_32bit(ACCEL_2_MIN_ADDR, PedalsSystemInstance::instance().get_accel_params().min_pedal_2);
        EEPROMUtilities::write_eeprom_32bit(ACCEL_2_MAX_ADDR, PedalsSystemInstance::instance().get_accel_params().max_pedal_2);
        EEPROMUtilities::write_eeprom_32bit(BRAKE_1_MIN_ADDR, PedalsSystemInstance::instance().get_brake_params().min_pedal_1);
        EEPROMUtilities::write_eeprom_32bit(BRAKE_1_MAX_ADDR, PedalsSystemInstance::instance().get_brake_params().max_pedal_1);
        EEPROMUtilities::write_eeprom_32bit(BRAKE_2_MIN_ADDR, PedalsSystemInstance::instance().get_brake_params().min_pedal_2);
        EEPROMUtilities::write_eeprom_32bit(BRAKE_2_MAX_ADDR, PedalsSystemInstance::instance().get_brake_params().max_pedal_2);
    }

    return HT_TASK::TaskResponse::YIELD;
}

// bool init_read_gpio_task()
// {
//     // Setting digital/analog buttons D10-D6, A8 as inputs
//     pinMode(BTN_DIM_READ, INPUT);
//     pinMode(BTN_PRESET_READ, INPUT);
//     pinMode(BTN_MC_CYCLE_READ, INPUT);
//     pinMode(BTN_MODE_READ, INPUT);
//     pinMode(BTN_START_READ, INPUT);
//     pinMode(BTN_DATA_READ, INPUT);
    
//     return HT_TASK::TaskResponse::YIELD;
// }
// bool run_read_gpio_task()
// {
//     // Doing digital read on all digital inputs
//     int dimButton = digitalRead(BTN_DIM_READ);
//     int presetButton = digitalRead(BTN_PRESET_READ);
//     int mcCycleButton = digitalRead(BTN_MC_CYCLE_READ);
//     int modeButton = digitalRead(BTN_MODE_READ);
//     int startButton = digitalRead(BTN_START_READ);
//     int dataButton = digitalRead(BTN_DATA_READ);
    
//     vcf_data.interface_data.dash_input_state.dim_btn_is_pressed = dimButton;
//     vcf_data.interface_data.dash_input_state.preset_btn_is_pressed = presetButton;
//     vcf_data.interface_data.dash_input_state.mc_reset_btn_is_pressed = mcCycleButton;
//     vcf_data.interface_data.dash_input_state.mode_btn_is_pressed = modeButton;
//     vcf_data.interface_data.dash_input_state.start_btn_is_pressed = startButton;
//     vcf_data.interface_data.dash_input_state.data_btn_is_pressed = dataButton;

//     return HT_TASK::TaskResponse::YIELD;
// }

HT_TASK::TaskResponse init_buzzer_control_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    pinMode(BUZZER_CONTROL_PIN, OUTPUT);

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_buzzer_control_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{

    bool buzzer_is_active = BuzzerController::getInstance().buzzer_is_active(sys_time::hal_millis()); //NOLINT

    digitalWrite(BUZZER_CONTROL_PIN, buzzer_is_active);
    
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse handle_CAN_send(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VCFCANInterfaceObjects& vcf_interface_objects = VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance();
    VCFCANInterfaceImpl::send_all_CAN_msgs(vcf_interface_objects.main_can_tx_buffer, vcf_interface_objects.MAIN_CAN);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse handle_CAN_receive(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VCFCANInterfaceObjects& vcf_interface_objects = VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance();
    CANInterfaces& vcf_can_interfaces = VCFCANInterfaceImpl::CANInterfacesInstance::instance(); 
    process_ring_buffer(vcf_interface_objects.main_can_rx_buffer, vcf_can_interfaces, sys_time::hal_millis(), vcf_interface_objects.can_recv_switch);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse send_dash_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{   
    CANInterfaces can_interfaces = VCFCANInterfaceImpl::CANInterfacesInstance::instance(); 
    DashInputState_s dash_outputs = can_interfaces.dash_interface.get_dashboard_outputs();

    DASH_INPUT_t msg_out;

    // msg_out.start_button = dash_outputs.start_btn_is_pressed;
    msg_out.preset_button = dash_outputs.preset_btn_is_pressed;
    msg_out.motor_controller_cycle_button = dash_outputs.mc_reset_btn_is_pressed;
    msg_out.mode_button = dash_outputs.mode_btn_is_pressed;
    msg_out.start_button = dash_outputs.start_btn_is_pressed;
    msg_out.data_button_is_pressed = dash_outputs.data_btn_is_pressed;
    msg_out.left_shifter_button = dash_outputs.left_paddle_is_pressed;
    msg_out.right_shifter_button = dash_outputs.right_paddle_is_pressed;   
    msg_out.led_dimmer_button = dash_outputs.dim_btn_is_pressed; 
    msg_out.dash_dial_mode = static_cast<int>(VCFData_sInstance::instance().interface_data.dash_input_state.dial_state);

//    Serial.printf("%d %d %d %d %d %d %d %d\n", msg_out.preset_button, msg_out.motor_controller_cycle_button, msg_out.mode_button, msg_out.start_button, msg_out.data_button_is_pressed, msg_out.left_shifter_button, msg_out.right_shifter_button, msg_out.led_dimmer_button);
    
    CAN_util::enqueue_msg(&msg_out, &Pack_DASH_INPUT_hytech, VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance().main_can_tx_buffer);
    
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_front_suspension_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) 
{
    CANInterfaces can_interface = VCFCANInterfaceImpl::CANInterfacesInstance::instance();
    FRONT_SUSPENSION_t msg_out;

    msg_out.fr_load_cell = VCFData_sInstance::instance().interface_data.front_loadcell_data.FR_loadcell_analog;
    CAN_util::enqueue_msg(&msg_out, &Pack_FRONT_SUSPENSION_hytech, VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance().main_can_tx_buffer);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_steering_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) 
{
    STEERING_DATA_t msg_out;

    msg_out.steering_analog_raw = VCFData_sInstance::instance().interface_data.steering_data.analog_steering_degrees;
    msg_out.steering_digital_raw = 0; //NOLINT VCFData_sInstance::instance().interface_data.steering_data.digital_steering_analog;

    CAN_util::enqueue_msg(&msg_out, &Pack_STEERING_DATA_hytech, VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance().main_can_tx_buffer);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse init_handle_send_vcf_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    qindesign::network::Ethernet.begin(EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().car_subnet, EthernetIPDefsInstance::instance().default_gateway);
    VCF_socket.begin(EthernetIPDefsInstance::instance().VCFData_port);

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_handle_send_vcf_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    hytech_msgs_VCFData_s msg = VCFEthernetInterface::make_vcf_data_msg(VCFData_sInstance::instance());
    if(handle_ethernet_socket_send_pb<hytech_msgs_VCFData_s_size, hytech_msgs_VCFData_s>
            (EthernetIPDefsInstance::instance().drivebrain_ip,
            EthernetIPDefsInstance::instance().VCFData_port,
            &VCF_socket,
            msg,
            hytech_msgs_VCFData_s_fields)) {
    }
    return HT_TASK::TaskResponse::YIELD;  
}

// HT_TASK::TaskResponse init_handle_receive_vcr_ethernet_data() {
//     VCF_socket.begin(EthernetIPDefsInstance::instance().VCFData_port);

//     return HT_TASK::TaskResponse::YIELD;
// }

// HT_TASK::TaskResponse run_handle_receive_vcr_ethernet_data() {
//     etl::optional<hytech_msgs_VCRData_s> protoc_struct = handle_ethernet_socket_receive<hytech_msgs_VCRData_s_size, hytech_msgs_VCRData_s>(&VCF_socket, &hytech_msgs_VCRData_s_msg);

//     return HT_TASK::TaskResponse::YIELD;
// }

HT_TASK::TaskResponse enqueue_pedals_data(const unsigned long &sys_micros, const HT_TASK::TaskInfo& task_info)
{
    PEDALS_SYSTEM_DATA_t pedals_data = {};
    pedals_data.accel_implausible = VCFData_sInstance::instance().system_data.pedals_system_data.accel_is_implausible;
    pedals_data.brake_implausible = VCFData_sInstance::instance().system_data.pedals_system_data.brake_is_implausible;
    pedals_data.brake_accel_implausibility = VCFData_sInstance::instance().system_data.pedals_system_data.brake_and_accel_pressed_implausibility_high;

    pedals_data.accel_pedal_active = VCFData_sInstance::instance().system_data.pedals_system_data.accel_is_pressed;
    pedals_data.brake_pedal_active = VCFData_sInstance::instance().system_data.pedals_system_data.brake_is_pressed;
    pedals_data.mechanical_brake_active = VCFData_sInstance::instance().system_data.pedals_system_data.mech_brake_is_active;
    pedals_data.implaus_exceeded_max_duration = VCFData_sInstance::instance().system_data.pedals_system_data.implausibility_has_exceeded_max_duration;

    
    pedals_data.accel_pedal_ro = HYTECH_accel_pedal_ro_toS(VCFData_sInstance::instance().system_data.pedals_system_data.accel_percent);
    pedals_data.brake_pedal_ro = HYTECH_brake_pedal_ro_toS(VCFData_sInstance::instance().system_data.pedals_system_data.brake_percent);
    // Serial.println(pedals_data.brake_pedal_ro);
    // Serial.println(pedals_data.accel_pedal_ro);
    CAN_util::enqueue_msg(&pedals_data, &Pack_PEDALS_SYSTEM_DATA_hytech, VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance().main_can_tx_buffer);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_dash_GPIOs_task(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
{
    bool was_dim_btn_pressed = VCFData_sInstance::instance().interface_data.dash_input_state.dim_btn_is_pressed; //NOLINT (linter thinks variable uninitialized)

    VCFData_sInstance::instance().interface_data.dash_input_state = DashboardInterfaceInstance::instance().get_dashboard_outputs();

    if (!VCFData_sInstance::instance().interface_data.dash_input_state.preset_btn_is_pressed)
    {
        VCRInterfaceInstance::instance().disable_calibration_state();
    }

    if (was_dim_btn_pressed && !VCFData_sInstance::instance().interface_data.dash_input_state.dim_btn_is_pressed)
    {
        NeopixelControllerInstance::instance().dim_neopixels();
    }

    // if (VCFData_sInstance::instance().interface_data.dash_input_state.start_btn_is_pressed) { // Test code to ensure buzzer timing works (on benchtop)
    //     BuzzerController::getInstance().activate(sys_micros / 1000);
    // }
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse create_ioexpander(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
{
    Wire2.begin();
    IOExpanderInstance::create(0x20, Wire2);
    IOExpanderInstance::instance().init();

    IOExpanderInstance::instance().portMode(MCP23017Port::A, 0b00000000);
    IOExpanderInstance::instance().portMode(MCP23017Port::B, 0b01111111);

    // IOExpanderInstance::instance().writeRegister(MCP23017Register::GPIO_A, 0x00);  //Reset port A 
    // IOExpanderInstance::instance().writeRegister(MCP23017Register::GPIO_B, 0x00);  //Reset port B

    IOExpanderInstance::instance().writeRegister(MCP23017Register::GPPU_B, 0xFF);  //Internal pull-ups

    IOExpanderInstance::instance().writeRegister(MCP23017Register::IPOL_B, 0xFF);  //Polarity (inverted)

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse read_ioexpander(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
{
    uint16_t in = IOExpanderInstance::instance().read(); // NOLINT (linter thinks variable uninitialized)
    // for (int i = 0; i < 8; ++i) {
    //     Serial.printf("%d ", in & 0x01);
    //     in = in >> 1;
    // }
    // Serial.println("");

    // Yes, I know there are magic numbers here. I just trial-and-errored it from the dash connector pinning.
    if (IOExpanderUtils::getBit(in, 1, 2)) {
        VCFData_sInstance::instance().interface_data.dash_input_state.dial_state = ControllerMode_e::MODE_0;
    } else if (IOExpanderUtils::getBit(in, 1, 1)) {
        VCFData_sInstance::instance().interface_data.dash_input_state.dial_state = ControllerMode_e::MODE_1;
    } else if (IOExpanderUtils::getBit(in, 1, 0)) {
        VCFData_sInstance::instance().interface_data.dash_input_state.dial_state = ControllerMode_e::MODE_2;
    } else if (IOExpanderUtils::getBit(in, 1, 5)) { // NOLINT (pin is magic number)
        VCFData_sInstance::instance().interface_data.dash_input_state.dial_state = ControllerMode_e::MODE_3;
    } else if (IOExpanderUtils::getBit(in, 1, 4)) { // NOLINT (pin is magic number)
        VCFData_sInstance::instance().interface_data.dash_input_state.dial_state = ControllerMode_e::MODE_4;
    } else if (IOExpanderUtils::getBit(in, 1, 3)) { // NOLINT (pin is magic number)
        VCFData_sInstance::instance().interface_data.dash_input_state.dial_state = ControllerMode_e::MODE_5;
    }

    ControllerMode_e state = VCFData_sInstance::instance().interface_data.dash_input_state.dial_state; // NOLINT (linter thinks state uninitialized)
    switch (state) {
        case ControllerMode_e::MODE_0:
        {
            IOExpanderInstance::instance().writePort(MCP23017Port::A, 0b00000010);
            break;
        }
        case ControllerMode_e::MODE_1:
        {
            IOExpanderInstance::instance().writePort(MCP23017Port::A, 0b01010111);
            break;
        }
        case ControllerMode_e::MODE_2:
        {
            IOExpanderInstance::instance().writePort(MCP23017Port::A, 0b00011000);
            break;
        }
        case ControllerMode_e::MODE_3:
        {
            IOExpanderInstance::instance().writePort(MCP23017Port::A, 0b00010100);
            break;
        }
        case ControllerMode_e::MODE_4:
        {
            IOExpanderInstance::instance().writePort(MCP23017Port::A, 0b01000101);
            break;
        }
        case ControllerMode_e::MODE_5:
        {
            IOExpanderInstance::instance().writePort(MCP23017Port::A, 0b00100100);
            break;
        }
        default:
        {
            IOExpanderInstance::instance().writePort(MCP23017Port::A, 0b11110000);
            break;
        }
    }

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse init_neopixels_task(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
{
    NeopixelControllerInstance::create(NEOPIXEL_COUNT, NEOPIXEL_CONTROL_PIN);
    NeopixelControllerInstance::instance().init_neopixels();
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_update_neopixels_task(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
{
    NeopixelControllerInstance::instance().refresh_neopixels(VCFData_sInstance::instance(), VCRData_sInstance::instance(), VCFCANInterfaceImpl::CANInterfacesInstance::instance());
    return HT_TASK::TaskResponse::YIELD;
}

namespace async_tasks 
{
    // these are async tasks. we want these to run as fast as possible p much
    void handle_async_CAN_receive() //NOLINT caps for CAN
    {
        VCFCANInterfaceObjects& vcf_interface_objects = VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance();
        CANInterfaces& vcf_can_interfaces = VCFCANInterfaceImpl::CANInterfacesInstance::instance(); 
        process_ring_buffer(vcf_interface_objects.main_can_rx_buffer, vcf_can_interfaces, sys_time::hal_millis(), vcf_interface_objects.can_recv_switch);
    }

    void handle_async_recvs()
    {
        // ethernet, etc...
        
        handle_async_CAN_receive();
    }
    HT_TASK::TaskResponse handle_async_main(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
    {
        handle_async_recvs();
        // Serial.println("test");
        VCFData_sInstance::instance().system_data.pedals_system_data = PedalsSystemInstance::instance().evaluate_pedals(VCFData_sInstance::instance().interface_data.pedal_sensor_data, sys_time::hal_millis());
        // Serial.println(VCFData_sInstance::instance().system_data.pedals_system_data.accel_percent);
        return HT_TASK::TaskResponse::YIELD;
    }
};
