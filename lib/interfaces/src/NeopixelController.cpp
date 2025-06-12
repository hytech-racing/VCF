#include "NeopixelController.h"

void NeopixelController::init_neopixels() {
    _neopixels.begin();
    _neopixels.setBrightness(_current_brightness);
    //set init color for every led
    for (int i = 0; i < _neopixel_count; i++) {
        // BMS and IMD are off according to rules
        if (i == LED_ID_e::BMS || i == LED_ID_e::IMD){
            _neopixels.setPixelColor(i, (uint32_t) LED_color_e::OFF);
        } else {
            _neopixels.setPixelColor(i, (uint32_t) LED_color_e::INIT_COLOR);
        }
    }
    // write data to neopixels
    _neopixels.show();
}

void NeopixelController::dim_neopixels() {
    _current_brightness -= STEP_BRIGHTNESS;
    // set current brightness to 0xFF (255) if less than min brightness - sid :) DO NOT CHANGE
    if (_current_brightness < MIN_BRIGHTNESS) { _current_brightness |= 0xFF; } // NOLINT (bitmask with 255)
    _neopixels.setBrightness(_current_brightness);
}

void NeopixelController::set_neopixel(uint16_t id, uint32_t c) {
    _neopixels.setPixelColor(id, c);
}

void NeopixelController::refresh_neopixels(VCFData_s &vcf_data, VCRData_s &vcr_data, CANInterfaces &interfaces) {

    // If we are in pedals recalibration state, LIGHT UP DASHBOARD ALL RED.
    if (interfaces.vcr_interface.is_in_pedals_calibration_state()) {
        set_neopixel_color(LED_ID_e::BRAKE, LED_color_e::RED);
        set_neopixel_color(LED_ID_e::TORQUE_MODE, LED_color_e::RED);
        set_neopixel_color(LED_ID_e::LAUNCH_CTRL, LED_color_e::RED);
        set_neopixel_color(LED_ID_e::CRIT_CHARGE, LED_color_e::RED);
        set_neopixel_color(LED_ID_e::INERTIA, LED_color_e::RED);
        set_neopixel_color(LED_ID_e::COCKPIT_BRB, LED_color_e::RED);
        set_neopixel_color(LED_ID_e::BOTS, LED_color_e::RED);
        set_neopixel_color(LED_ID_e::MC_ERR, LED_color_e::RED);
        set_neopixel_color(LED_ID_e::RDY_DRIVE, LED_color_e::RED);
        set_neopixel_color(LED_ID_e::GLV, LED_color_e::RED);
        _neopixels.show();
        return;
    }

    LED_color_e brake_light_color = LED_color_e::OFF;
    if (vcf_data.system_data.pedals_system_data.brake_is_pressed && !vcf_data.system_data.pedals_system_data.implausibility_has_exceeded_max_duration) {
        brake_light_color = LED_color_e::GREEN;
    } else if (vcf_data.system_data.pedals_system_data.implausibility_has_exceeded_max_duration) {
        brake_light_color = LED_color_e::RED;
    }

    LED_color_e torque_mode_color = LED_color_e::OFF;
    switch (interfaces.vcr_interface.get_torque_limit_mode())
    {
        case TorqueLimit_e::TCMUX_LOW_TORQUE:
        {
            torque_mode_color = LED_color_e::RED;
            break;
        }
        case TorqueLimit_e::TCMUX_MID_TORQUE:
        {
            torque_mode_color = LED_color_e::YELLOW;
            break;
        }
        case TorqueLimit_e::TCMUX_FULL_TORQUE:
        {
            torque_mode_color = LED_color_e::GREEN;
            break;
        }
        default:
        {
            torque_mode_color = LED_color_e::OFF;
            break;
        }
    }

    LED_color_e ready_drive_color = LED_color_e::OFF;
    switch (interfaces.vcr_interface.get_vehicle_state())
    {
        case static_cast<uint8_t>(VehicleState_e::READY_TO_DRIVE):
        {
            interfaces.vcr_interface.get_db_in_ctrl() ? LED_color_e::BLUE : LED_color_e::GREEN;
            break;
        }
        case static_cast<uint8_t>(VehicleState_e::WANTING_READY_TO_DRIVE):
        {
            LED_color_e::YELLOW;
            break;
        }
        default:
        {
            LED_color_e::RED;
            break;
        }
    }
    constexpr float glv_critical_voltage = 22.0f;

    set_neopixel_color(LED_ID_e::BRAKE, brake_light_color);
    set_neopixel_color(LED_ID_e::LAUNCH_CTRL, LED_color_e::OFF); // Unused for now
    set_neopixel_color(LED_ID_e::CRIT_CHARGE, interfaces.acu_interface.get_voltages_not_critical() ? LED_color_e::GREEN : LED_color_e::RED); // Unused for now
    set_neopixel_color(LED_ID_e::INERTIA, LED_color_e::OFF); // Unused for now
    set_neopixel_color(LED_ID_e::COCKPIT_BRB, LED_color_e::OFF); // Unused for now
    set_neopixel_color(LED_ID_e::BOTS, LED_color_e::OFF); // Unused for now
    set_neopixel_color(LED_ID_e::IMD, interfaces.dash_interface.imd_ok ? LED_color_e::GREEN : LED_color_e::RED);
    set_neopixel_color(LED_ID_e::BMS, interfaces.dash_interface.bms_ok ? LED_color_e::GREEN : LED_color_e::RED);
    set_neopixel_color(LED_ID_e::MC_ERR, (vcr_data.interface_data.inverter_data.FL.error || vcr_data.interface_data.inverter_data.FR.error || vcr_data.interface_data.inverter_data.RL.error || vcr_data.interface_data.inverter_data.RR.error) ? LED_color_e::RED : LED_color_e::OFF);
    set_neopixel_color(LED_ID_e::RDY_DRIVE, ready_drive_color);
    set_neopixel_color(LED_ID_e::GLV, vcr_data.interface_data.current_sensor_data.twentyfour_volt_sensor > glv_critical_voltage ? LED_color_e::GREEN : LED_color_e::OFF); // No sensor there yet
    set_neopixel_color(LED_ID_e::TORQUE_MODE, torque_mode_color);

    _neopixels.show();

}

void NeopixelController::set_neopixel_color(LED_ID_e led, LED_color_e color)
{
    _neopixels.setPixelColor(led, (uint32_t) color);
}