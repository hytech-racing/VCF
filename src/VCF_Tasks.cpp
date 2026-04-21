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
#include "DashboardInterface.h"
#include "VCFEthernetInterface.h"
#include "ACUInterface.h"
#include <EEPROM.h>
#include "FlexCAN_T4.h"
#include "Orbis_BR.h"

#include "WatchdogSystem.h"
#include "Arduino.h"

HT_TASK::TaskResponse run_read_adc0_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // Updates all eight channels.
    ADCInterfaceInstance::instance().adc0_tick();
    PedalsSystemInstance::instance().set_pedals_sensor_data(PedalSensorData_s{
        .accel_1 = static_cast<uint32_t>(ADCInterfaceInstance::instance().acceleration_1().conversion),
        .accel_2 = static_cast<uint32_t>(ADCInterfaceInstance::instance().acceleration_2().conversion),
        .brake_1 = static_cast<uint32_t>(ADCInterfaceInstance::instance().brake_1().conversion),
        .brake_2 = static_cast<uint32_t>(ADCInterfaceInstance::instance().brake_2().conversion)
    });

    // sample digital steering too TODO: move this to its own task maybe?
    // OrbisBRInstance::instance().sample();
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // Samples all eight channels.
    ADCInterfaceInstance::instance().adc1_tick();
    ADCInterfaceInstance::instance().update_filtered_values(VCFTaskConstants::LOADCELL_IIR_FILTER_ALPHA);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse init_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    WatchdogInstance::create(VCFInterfaceConstants::WATCHDOG_KICK_INTERVAL_MS); // NOLINT
    pinMode(VCFInterfaceConstants::WATCHDOG_PIN, OUTPUT);
    pinMode(VCFInterfaceConstants::SOFTWARE_OK_PIN, OUTPUT);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    digitalWrite(VCFInterfaceConstants::SOFTWARE_OK_PIN , HIGH);
    digitalWrite(VCFInterfaceConstants::WATCHDOG_PIN, WatchdogInstance::instance().get_watchdog_state(sys_time::hal_millis()));
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse update_pedals_calibration_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    // Observed pedal values (ONLY USED FOR RECALIBRATION)
    // WARNING: These are the true min/max observed values, NOT the "value at min travel" and "value at max travel"
    //          that are defined in the PedalsParam struct.
    PedalsSystemInstance::instance().update_observed_pedal_limits(PedalsSystemInstance::instance().get_pedals_sensor_data());

    if (VCRInterfaceInstance::instance().is_in_pedals_calibration_state())
    {
        // PedalsSystemInstance::instance().recalibrate_min_max(VCFData_sInstance::instance().interface_data.pedal_sensor_data);
        PedalsSystemInstance::instance().recalibrate_min_max(PedalsSystemInstance::instance().get_pedals_sensor_data());
        EEPROMUtilities::write_eeprom_32bit(VCFSystemConstants::ACCEL_1_MIN_ADDR, PedalsSystemInstance::instance().get_accel_params().min_pedal_1);
        EEPROMUtilities::write_eeprom_32bit(VCFSystemConstants::ACCEL_1_MAX_ADDR, PedalsSystemInstance::instance().get_accel_params().max_pedal_1);
        EEPROMUtilities::write_eeprom_32bit(VCFSystemConstants::ACCEL_2_MIN_ADDR, PedalsSystemInstance::instance().get_accel_params().min_pedal_2);
        EEPROMUtilities::write_eeprom_32bit(VCFSystemConstants::ACCEL_2_MAX_ADDR, PedalsSystemInstance::instance().get_accel_params().max_pedal_2);
        EEPROMUtilities::write_eeprom_32bit(VCFSystemConstants::BRAKE_1_MIN_ADDR, PedalsSystemInstance::instance().get_brake_params().min_pedal_1);
        EEPROMUtilities::write_eeprom_32bit(VCFSystemConstants::BRAKE_1_MAX_ADDR, PedalsSystemInstance::instance().get_brake_params().max_pedal_1);
        EEPROMUtilities::write_eeprom_32bit(VCFSystemConstants::BRAKE_2_MIN_ADDR, PedalsSystemInstance::instance().get_brake_params().min_pedal_2);
        EEPROMUtilities::write_eeprom_32bit(VCFSystemConstants::BRAKE_2_MAX_ADDR, PedalsSystemInstance::instance().get_brake_params().max_pedal_2);
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
    pinMode(VCFInterfaceConstants::BUZZER_CONTROL_PIN, OUTPUT);

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_buzzer_control_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{

    bool buzzer_is_active = BuzzerController::getInstance().buzzer_is_active(sys_time::hal_millis()); //NOLINT

    digitalWrite(VCFInterfaceConstants::BUZZER_CONTROL_PIN, buzzer_is_active);

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
    process_ring_buffer(vcf_interface_objects.main_can_rx_buffer, vcf_can_interfaces, sys_time::hal_millis(), vcf_interface_objects.can_recv_switch, CANInterfaceType_e::TELEM);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse send_dash_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    CANInterfaces can_interfaces = VCFCANInterfaceImpl::CANInterfacesInstance::instance();
    DashInputState_s dash_outputs = can_interfaces.dash_interface.get_dashboard_outputs();

    DASH_INPUT_t msg_out;

    msg_out.preset_button = dash_outputs.preset_btn_is_pressed;
    msg_out.mode_button = 0; // dont exist but i dont wanna bother changing can msgs

    msg_out.motor_controller_cycle_button = dash_outputs.mc_reset_btn_is_pressed;
    msg_out.start_button = dash_outputs.start_btn_is_pressed;
    msg_out.data_button_is_pressed = dash_outputs.data_btn_is_pressed;
    msg_out.left_shifter_button = 0;
    msg_out.right_shifter_button = dash_outputs.BUTTON_2;
    msg_out.led_dimmer_button = dash_outputs.brightness_ctrl_btn_is_pressed;
    msg_out.dash_dial_mode = static_cast<int>(DashboardInterfaceInstance::instance().get_dashboard_outputs().dial_state);

    CAN_util::enqueue_msg(&msg_out, &Pack_DASH_INPUT_hytech, VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance().main_can_tx_buffer);

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_front_suspension_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    CANInterfaces can_interface = VCFCANInterfaceImpl::CANInterfacesInstance::instance();
    FRONT_SUSPENSION_t msg_out;

    msg_out.fr_load_cell = ADCInterfaceInstance::instance().get_filtered_FR_load_cell();
    msg_out.fl_load_cell = ADCInterfaceInstance::instance().get_filtered_FL_load_cell();
    msg_out.fr_shock_pot_ro = HYTECH_fr_shock_pot_ro_toS(ADCInterfaceInstance::instance().get_filtered_FR_sus_pot());
    msg_out.fl_shock_pot_ro = HYTECH_fl_shock_pot_ro_toS(ADCInterfaceInstance::instance().get_filtered_FL_sus_pot());

    CAN_util::enqueue_msg(&msg_out, &Pack_FRONT_SUSPENSION_hytech, VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance().main_can_tx_buffer);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_steering_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    STEERING_DATA_t msg_out = {};

    // msg_out.steering_analog_raw = ADCInterfaceInstance::instance().get_steering_degrees_cw().raw;
    // msg_out.steering_digital_raw = ADCInterfaceInstance::instance().get_steering_degrees_ccw().raw; //NOLINT TODO: once digital steering sensor works, this needs to be changed accordingly
    
    // TODO: change these to actually grab values from steering system
    msg_out.steering_analog_oor = 0;
    msg_out.steering_analog_raw = ADCInterfaceInstance::instance().get_steering_degrees_cw().raw;
    msg_out.steering_both_sensors_fail = 0;
    msg_out.steering_digital_oor = 0;
    msg_out.steering_digital_raw = OrbisBRInstance::instance().getLastReading().rawValue;
    msg_out.steering_dtheta_exceeded_analog = 0;
    msg_out.steering_dtheta_exceeded_digital = 0;
    msg_out.steering_interface_sensor_error = 0;
    msg_out.steering_output_steering_angle_ro = 0;
    msg_out.steering_sensor_disagreement = 0;

    CAN_util::enqueue_msg(&msg_out, &Pack_STEERING_DATA_hytech, VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance().main_can_tx_buffer);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse init_handle_send_vcf_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    qindesign::network::Ethernet.begin(EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().car_subnet, EthernetIPDefsInstance::instance().default_gateway);
    VCF_socket.begin(EthernetIPDefsInstance::instance().VCFData_port);

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_handle_send_vcf_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    hytech_msgs_VCFData_s msg = VCFEthernetInterface::make_vcf_data_msg(ADCInterfaceInstance::instance(), DashboardInterfaceInstance::instance(), PedalsSystemInstance::instance());
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

    pedals_data.accel_implausible = PedalsSystemInstance::instance().get_pedals_system_data().accel_is_implausible;
    pedals_data.brake_implausible = PedalsSystemInstance::instance().get_pedals_system_data().brake_is_implausible;
    pedals_data.brake_accel_implausibility = PedalsSystemInstance::instance().get_pedals_system_data().brake_and_accel_pressed_implausibility_high;

    pedals_data.accel_pedal_active = PedalsSystemInstance::instance().get_pedals_system_data().accel_is_pressed;
    pedals_data.brake_pedal_active = PedalsSystemInstance::instance().get_pedals_system_data().brake_is_pressed;
    pedals_data.mechanical_brake_active = PedalsSystemInstance::instance().get_pedals_system_data().mech_brake_is_active;
    pedals_data.implaus_exceeded_max_duration = PedalsSystemInstance::instance().get_pedals_system_data().implausibility_has_exceeded_max_duration;


    pedals_data.accel_pedal_ro = HYTECH_accel_pedal_ro_toS(PedalsSystemInstance::instance().get_pedals_system_data().accel_percent);
    pedals_data.brake_pedal_ro = HYTECH_brake_pedal_ro_toS(PedalsSystemInstance::instance().get_pedals_system_data().brake_percent);
    // Serial.println(pedals_data.brake_pedal_ro);
    // Serial.println(pedals_data.accel_pedal_ro);

    // TODO: Need to add brake pressure data to CAN msg
    CAN_util::enqueue_msg(&pedals_data, &Pack_PEDALS_SYSTEM_DATA_hytech, VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance().main_can_tx_buffer);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_dash_GPIOs_task(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
{
    bool was_dim_btn_pressed = DashboardInterfaceInstance::instance().get_dashboard_stored_state().brightness_ctrl_btn_is_pressed; //NOLINT (linter thinks variable uninitialized)
    DashInputState_s current_state = DashboardInterfaceInstance::instance().get_dashboard_outputs();

    if (!current_state.preset_btn_is_pressed) //preset_btn_is_pressed doesnt exist anymore
    {
        VCRInterfaceInstance::instance().disable_calibration_state();
    }

    // Checks if dim btn has been clicked (falling edge)
    if (was_dim_btn_pressed && !current_state.brightness_ctrl_btn_is_pressed)
    {
        NeopixelControllerInstance::instance().dim_neopixels();
    }

    DashboardInterfaceInstance::instance().sync_dashboard_stored_state();

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

    //TODO: Double check the harnessing is not wrong this time
    if (IOExpanderUtils::getBit(in, 1, 0)) {
        DashboardInterfaceInstance::instance().set_dial_state(ControllerMode_e::MODE_0);
    } else if (IOExpanderUtils::getBit(in, 1, 1)) {
        DashboardInterfaceInstance::instance().set_dial_state(ControllerMode_e::MODE_1);
    } else if (IOExpanderUtils::getBit(in, 1, 2)) {
        DashboardInterfaceInstance::instance().set_dial_state(ControllerMode_e::MODE_2);
    } else if (IOExpanderUtils::getBit(in, 1, 3)) { // NOLINT (pin is magic number)
        DashboardInterfaceInstance::instance().set_dial_state(ControllerMode_e::MODE_3);
    } else if (IOExpanderUtils::getBit(in, 1, 4)) { // NOLINT (pin is magic number)
        DashboardInterfaceInstance::instance().set_dial_state(ControllerMode_e::MODE_4);
    } else if (IOExpanderUtils::getBit(in, 1, 5)) { // NOLINT (pin is magic number)
        DashboardInterfaceInstance::instance().set_dial_state(ControllerMode_e::MODE_5);
    }

    ControllerMode_e state = DashboardInterfaceInstance::instance().get_dashboard_outputs().dial_state; // NOLINT (linter thinks state uninitialized)
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
    NeopixelControllerInstance::create(VCFInterfaceConstants::NEOPIXEL_COUNT, VCFInterfaceConstants::NEOPIXEL_CONTROL_PIN);
    NeopixelControllerInstance::instance().init_neopixels();
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_update_neopixels_task(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
{
    NeopixelControllerInstance::instance().refresh_neopixels(PedalsSystemInstance::instance().get_pedals_system_data(), VCFCANInterfaceImpl::CANInterfacesInstance::instance());
    return HT_TASK::TaskResponse::YIELD;
}

namespace async_tasks
{
    // these are async tasks. we want these to run as fast as possible p much
    void handle_async_CAN_receive() //NOLINT caps for CAN
    {
        VCFCANInterfaceObjects& vcf_interface_objects = VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance();
        CANInterfaces& vcf_can_interfaces = VCFCANInterfaceImpl::CANInterfacesInstance::instance();
        process_ring_buffer(vcf_interface_objects.main_can_rx_buffer, vcf_can_interfaces, sys_time::hal_millis(), vcf_interface_objects.can_recv_switch, CANInterfaceType_e::TELEM);
    }

    void handle_async_recvs()
    {
        // ethernet, etc...

        handle_async_CAN_receive();
    }
    HT_TASK::TaskResponse handle_async_main(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
    {
        handle_async_recvs();
        PedalsSystemInstance::instance().evaluate_pedals(
            PedalsSystemInstance::instance().get_pedals_sensor_data(),
            sys_time::hal_millis()
        );
        return HT_TASK::TaskResponse::YIELD;
    }
};

HT_TASK::TaskResponse debug_print(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    /* Pedals Info */
    Serial.println("\n\nPedals Info:");
    Serial.println("\tPercent Pressed Implaus Min 1 \tMax 1 \tMin 2 \tMax 2");
    // Accel
    Serial.print("Accel: \t");
    Serial.print(PedalsSystemInstance::instance().get_pedals_system_data().accel_percent); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_pedals_system_data().accel_is_pressed); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_pedals_system_data().accel_is_implausible); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_accel_params().min_pedal_1); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_accel_params().max_pedal_1); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_accel_params().min_pedal_2); Serial.print("\t");
    Serial.println(PedalsSystemInstance::instance().get_accel_params().max_pedal_2);
    // Brake
    Serial.print("Brake: \t");
    Serial.print(PedalsSystemInstance::instance().get_pedals_system_data().brake_percent); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_pedals_system_data().brake_is_pressed); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_pedals_system_data().brake_is_implausible); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_brake_params().min_pedal_1); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_brake_params().max_pedal_1); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_brake_params().min_pedal_2); Serial.print("\t");
    Serial.println(PedalsSystemInstance::instance().get_brake_params().max_pedal_2);

    /* ADC Values */
    Serial.println("\nADC Vals:");
    // ADC 0
    Serial.println("ADC 0\t\t  Steering");
    Serial.println("\t2V5 Ref CW \tCCW \tAccel 1 Accel 2 Brake 1 Brake 2");
    // Raw values
    Serial.print("Raw\t");
    Serial.print(ADCInterfaceInstance::instance().pedal_reference().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_steering_degrees_cw().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_steering_degrees_ccw().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().acceleration_1().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().acceleration_2().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().brake_1().raw); Serial.print("\t");
    Serial.println(ADCInterfaceInstance::instance().brake_2().raw);
    // Converted values
    Serial.print("Convert\t");
    Serial.print(ADCInterfaceInstance::instance().pedal_reference().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_steering_degrees_cw().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_steering_degrees_ccw().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().acceleration_1().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().acceleration_2().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().brake_1().conversion); Serial.print("\t");
    Serial.println(ADCInterfaceInstance::instance().brake_2().conversion);

    // ADC 1
    Serial.println("\nADC 1\t\t\t  Load Cells \t  Sus Pots \t Brake Pressure");
    Serial.println("\tSHDN H \tSHDN D \tFL \tFR \tFR \tFL \tFront \tRear");
    // Raw ADC
    Serial.print("Raw\t");
    Serial.print(ADCInterfaceInstance::instance().shdn_h().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().shdn_d().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().FL_load_cell().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().FR_load_cell().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().FR_sus_pot().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().FL_sus_pot().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_brake_pressure_front().raw); Serial.print("\t");
    Serial.println(ADCInterfaceInstance::instance().get_brake_pressure_rear().raw);
    // Conversion ADC
    Serial.print("Convert\t");
    Serial.print(ADCInterfaceInstance::instance().shdn_h().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().shdn_d().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_filtered_FL_load_cell()); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_filtered_FR_load_cell()); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_filtered_FR_sus_pot()); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_filtered_FL_sus_pot()); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_brake_pressure_front().conversion); Serial.print("\t");
    Serial.println(ADCInterfaceInstance::instance().get_brake_pressure_rear().conversion);

    /* Dashboard Info */
    Serial.println("\nDash Buttons / Buzzer:");
    Serial.println("Preset \tReset \tStart \tData \tBuzzer");
    Serial.print(DashboardInterfaceInstance::instance().get_dashboard_outputs().preset_btn_is_pressed); Serial.print("\t");
    Serial.print(DashboardInterfaceInstance::instance().get_dashboard_outputs().mc_reset_btn_is_pressed); Serial.print("\t");
    Serial.print(DashboardInterfaceInstance::instance().get_dashboard_outputs().start_btn_is_pressed); Serial.print("\t");
    Serial.print(DashboardInterfaceInstance::instance().get_dashboard_outputs().data_btn_is_pressed); Serial.print("\t");
    Serial.println(BuzzerController::getInstance().buzzer_is_active(sys_time::hal_millis()));

    return HT_TASK::TaskResponse::YIELD;
}

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> VCFCANInterfaceImpl::main_can;

void setup_all_interfaces() {
    SPI.begin();
    Serial.begin(VCFTaskConstants::SERIAL_BAUDRATE); // NOLINT

    // Initialize all singletons

    // Create ADC interface singleton
    ADCInterfaceInstance::create(
    ADCPinout_s {
        VCFInterfaceConstants::ADC0_CS,
        VCFInterfaceConstants::ADC1_CS
    },
    ADCChannels_s {
        VCFInterfaceConstants::PEDAL_REF_2V5_CHANNEL,
        VCFInterfaceConstants::STEERING_1_CHANNEL,
        VCFInterfaceConstants::STEERING_2_CHANNEL,
        VCFInterfaceConstants::ACCEL_1_CHANNEL,
        VCFInterfaceConstants::ACCEL_2_CHANNEL,
        VCFInterfaceConstants::BRAKE_1_CHANNEL,
        VCFInterfaceConstants::BRAKE_2_CHANNEL,

        VCFInterfaceConstants::SHDN_H_CHANNEL,
        VCFInterfaceConstants::SHDN_D_CHANNEL,
        VCFInterfaceConstants::FL_LOADCELL_CHANNEL,
        VCFInterfaceConstants::FR_LOADCELL_CHANNEL,
        VCFInterfaceConstants::FR_SUS_POT_CHANNEL,
        VCFInterfaceConstants::FL_SUS_POT_CHANNEL,
        VCFInterfaceConstants::BRAKE_PRESSURE_FRONT_CHANNEL,
        VCFInterfaceConstants::BRAKE_PRESSURE_REAR_CHANNEL
    },
    ADCScales_s {
        VCFInterfaceConstants::PEDAL_REF_2V5_SCALE,
        VCFInterfaceConstants::STEERING_1_SCALE,
        VCFInterfaceConstants::STEERING_2_SCALE,
        VCFInterfaceConstants::ACCEL_1_SCALE,
        VCFInterfaceConstants::ACCEL_2_SCALE,
        VCFInterfaceConstants::BRAKE_1_SCALE,
        VCFInterfaceConstants::BRAKE_2_SCALE,

        VCFInterfaceConstants::SHDN_H_SCALE,
        VCFInterfaceConstants::SHDN_D_SCALE,
        VCFInterfaceConstants::FL_LOADCELL_SCALE,
        VCFInterfaceConstants::FR_LOADCELL_SCALE,
        VCFInterfaceConstants::FR_SUS_POT_SCALE,
        VCFInterfaceConstants::FL_SUS_POT_SCALE,
        VCFInterfaceConstants::BRAKE_PRESSURE_FRONT_SCALE,
        VCFInterfaceConstants::BRAKE_PRESSURE_REAR_SCALE
    },
    ADCOffsets_s {
        VCFInterfaceConstants::PEDAL_REF_2V5_OFFSET,
        VCFInterfaceConstants::STEERING_1_OFFSET,
        VCFInterfaceConstants::STEERING_2_OFFSET,
        VCFInterfaceConstants::ACCEL_1_OFFSET,
        VCFInterfaceConstants::ACCEL_2_OFFSET,
        VCFInterfaceConstants::BRAKE_1_OFFSET,
        VCFInterfaceConstants::BRAKE_2_OFFSET,

        VCFInterfaceConstants::SHDN_H_OFFSET,
        VCFInterfaceConstants::SHDN_D_OFFSET,
        VCFInterfaceConstants::FL_LOADCELL_OFFSET,
        VCFInterfaceConstants::FR_LOADCELL_OFFSET,
        VCFInterfaceConstants::FR_SUS_POT_OFFSET,
        VCFInterfaceConstants::FL_SUS_POT_OFFSET,
        VCFInterfaceConstants::BRAKE_PRESSURE_FRONT_OFFSET,
        VCFInterfaceConstants::BRAKE_PRESSURE_REAR_OFFSET
    });

    EthernetIPDefsInstance::create();

    // Create pedals singleton
    PedalsParams accel_params = {
        .min_pedal_1 = EEPROMUtilities::read_eeprom_32bit(VCFSystemConstants::ACCEL_1_MIN_ADDR),
        .min_pedal_2 = EEPROMUtilities::read_eeprom_32bit(VCFSystemConstants::ACCEL_2_MIN_ADDR),
        .max_pedal_1 = EEPROMUtilities::read_eeprom_32bit(VCFSystemConstants::ACCEL_1_MAX_ADDR),
        .max_pedal_2 = EEPROMUtilities::read_eeprom_32bit(VCFSystemConstants::ACCEL_2_MAX_ADDR),
        .activation_percentage = VCFSystemConstants::ACCEL_ACTIVATION_PERCENTAGE,
        .min_sensor_pedal_1 = VCFSystemConstants::ACCEL_MIN_SENSOR_PEDAL_1,
        .min_sensor_pedal_2 = VCFSystemConstants::ACCEL_MIN_SENSOR_PEDAL_2,
        .max_sensor_pedal_1 = VCFSystemConstants::ACCEL_MAX_SENSOR_PEDAL_1,
        .max_sensor_pedal_2 = VCFSystemConstants::ACCEL_MAX_SENSOR_PEDAL_2,
        .deadzone_margin = VCFSystemConstants::ACCEL_DEADZONE_MARGIN,
        .implausibility_margin = IMPLAUSIBILITY_PERCENT,
        .mechanical_activation_percentage = VCFSystemConstants::ACCEL_MECHANICAL_ACTIVATION_PERCENTAGE
    };

    PedalsParams brake_params = {
        .min_pedal_1 = EEPROMUtilities::read_eeprom_32bit(VCFSystemConstants::BRAKE_1_MIN_ADDR),
        .min_pedal_2 = EEPROMUtilities::read_eeprom_32bit(VCFSystemConstants::BRAKE_2_MIN_ADDR),
        .max_pedal_1 = EEPROMUtilities::read_eeprom_32bit(VCFSystemConstants::BRAKE_1_MAX_ADDR),
        .max_pedal_2 = EEPROMUtilities::read_eeprom_32bit(VCFSystemConstants::BRAKE_2_MAX_ADDR),
        .activation_percentage = VCFSystemConstants::BRAKE_ACTIVATION_PERCENTAGE,
        .min_sensor_pedal_1 = VCFSystemConstants::BRAKE_MIN_SENSOR_PEDAL_1,
        .min_sensor_pedal_2 = VCFSystemConstants::BRAKE_MIN_SENSOR_PEDAL_2,
        .max_sensor_pedal_1 = VCFSystemConstants::BRAKE_MAX_SENSOR_PEDAL_1,
        .max_sensor_pedal_2 = VCFSystemConstants::BRAKE_MAX_SENSOR_PEDAL_2,
        .deadzone_margin = VCFSystemConstants::BRAKE_DEADZONE_MARGIN,
        .implausibility_margin = IMPLAUSIBILITY_PERCENT,
        .mechanical_activation_percentage = VCFSystemConstants::BRAKE_MECHANICAL_ACTIVATION_PERCENTAGE
    };

    PedalsSystemInstance::create(accel_params, brake_params); //pass in the two different params

    // Create Digital Steering Sensor singleton
    OrbisBRInstance::create(&Serial2);

    // Create dashboard singleton
    DashboardGPIOs_s dashboard_gpios = {
        .BRIGHTNESS_CONTROL_PIN = VCFInterfaceConstants::BRIGHTNESS_CONTROL_PIN,
        .PRESET_BUTTON = VCFInterfaceConstants::BTN_PRESET_READ,
        .MC_CYCLE_BUTTON = VCFInterfaceConstants::BTN_MC_CYCLE_READ,
        .START_BUTTON = VCFInterfaceConstants::BTN_START_READ,
        .DATA_BUTTON = VCFInterfaceConstants::BTN_DATA_READ,
        .BUTTON_2 = VCFInterfaceConstants::BUTTON_2
    };

    DashboardInterfaceInstance::create(dashboard_gpios); //NOLINT
    ACUInterfaceInstance::create();
    VCRInterfaceInstance::create();
    // Create can singletons
    VCFCANInterfaceImpl::CANInterfacesInstance::create(DashboardInterfaceInstance::instance(), ACUInterfaceInstance::instance(), VCRInterfaceInstance::instance());
    auto main_can_recv = etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long, CANInterfaceType_e)>::create<VCFCANInterfaceImpl::vcf_recv_switch>();
    VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::create(main_can_recv, &VCFCANInterfaceImpl::main_can);

    const uint32_t CAN_baudrate = 1000000;
    handle_CAN_setup(VCFCANInterfaceImpl::main_can, CAN_baudrate, &VCFCANInterfaceImpl::on_main_can_recv);

    EthernetIPDefsInstance::create();
    uint8_t mac[6]; // NOLINT (mac addresses are always 6 bytes)
    qindesign::network::Ethernet.macAddress(&mac[0]);
    qindesign::network::Ethernet.begin(mac, EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().default_dns, EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);

}