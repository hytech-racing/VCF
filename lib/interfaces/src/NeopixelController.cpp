#include "NeopixelController.h"

void NeopixelController::dim_neopixels() {
    _current_brightness -= STEP_BRIGHTNESS;
    // set current brightness to 0xFF (255) if less than min brightness - sid :) DO NOT CHANGE
    if (_current_brightness < MIN_BRIGHTNESS) { _current_brightness |= 0xFF; }
    _neopixels.setBrightness(_current_brightness);
}

void NeopixelController::set_neopixel(uint16_t id, uint32_t c) {
    _neopixels.setPixelColor(id, c);
}

void NeopixelController::refresh_neopixels(VCFData_s vcf_data, VCRData_s vcr_data) {

    // // only refresh if updates were found in the CAN message
    // // This does not work for some reason on initial message
    // // replace Metro timer if it is figured out
    // if (CAN->mcu_state_update || pixel_refresh.check()) {

    //     set_neopixel_color(LED_ID_e::BOTS, CAN->dash_mcu_state.bots_led);
    //     set_neopixel_color(LED_ID_e::LAUNCH_CTRL, CAN->dash_mcu_state.launch_control_led);
    //     set_neopixel_color(LED_ID_e::TORQUE_MODE, CAN->dash_mcu_state.mode_led);
    //     set_neopixel_color(LED_ID_e::COCKPIT_BRB, CAN->brb_read ? 1 : 3);
    //     set_neopixel_color(LED_ID_e::INERTIA, CAN->inertia_read ? 1 : 3);
    //     set_neopixel_color(LED_ID_e::RDY_DRIVE, CAN->dash_mcu_state.start_status_led);
    //     set_neopixel_color(LED_ID_e::MC_ERR, CAN->dash_mcu_state.motor_controller_error_led);
    //     set_neopixel_color(LED_ID_e::IMD, CAN->dash_mcu_state.imd_led);
    //     set_neopixel_color(LED_ID_e::AMS, CAN->dash_mcu_state.ams_led);
    //     set_neopixel_color(LED_ID_e::GLV, 0);
    //     // set_neopixel_color_gradient(LED_ID_e::GLV, CAN->dash_mcu_state.glv_led);
    //     set_neopixel_color_gradient(LED_ID_e::CRIT_CHARGE, CAN->dash_mcu_state.pack_charge_led);
    //     SerialUSB.println(CAN->mcu_status.no_brake_implausibility);
    //     if (!CAN->mcu_status.no_brake_implausibility) {
    //         set_neopixel_color(LED_ID_e::BRAKE_ENGAGE, 3);
    //         if (blink()) { set_neopixel_color(LED_ID_e::BRAKE_ENGAGE, 0); }
    //     } else {
    //         set_neopixel_color(LED_ID_e::BRAKE_ENGAGE, CAN->dash_mcu_state.mechanical_brake_led);
    //     }
    //     _neopixels.show();
    //     CAN->mcu_state_update = false;

    //     pixel_refresh.reset();
    // }

    // DEBUG CODE ONLY
    for  (int i = 0; i < 12; ++i) {
        set_neopixel_color((LED_ID_e) i, LED_color_e::GREEN);
    }
    _neopixels.show();

}

void NeopixelController::set_neopixel_color(LED_ID_e led, LED_color_e color)
{
    _neopixels.setPixelColor(led, (uint32_t) color);
}