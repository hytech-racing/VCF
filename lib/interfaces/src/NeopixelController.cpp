#include "NeopixelController.h"

void NeopixelController::init_neopixels() {
    _neopixels.begin();
    _neopixels.setBrightness(_current_brightness);
    //set init color for every led
    for (int i = 0; i < _neopixel_count; i++) {
        _neopixels.setPixelColor(i, (uint32_t) LED_color_e::INIT_COLOR);
        // BMS and IMD are off according to rules
        if (i == LED_ID_e::BMS || i == LED_ID_e::IMD){
            _neopixels.setPixelColor(i, (uint32_t) LED_color_e::OFF);
        }
    }
    // write data to neopixels
    _neopixels.show();
}

void NeopixelController::dim_neopixels() {
    _current_brightness -= STEP_BRIGHTNESS;
    // set current brightness to 0xFF (255) if less than min brightness - sid :) DO NOT CHANGE
    if (_current_brightness < MIN_BRIGHTNESS) { _current_brightness |= 0xFF; }
    _neopixels.setBrightness(_current_brightness);
}

void NeopixelController::set_neopixel(uint16_t id, uint32_t c) {
    _neopixels.setPixelColor(id, c);
}

void NeopixelController::refresh_neopixels(VCFData_s &vcf_data, VCRData_s &vcr_data) {

    set_neopixel_color(LED_ID_e::BRAKE, vcf_data.system_data.pedals_system_data.brake_is_pressed ? LED_color_e::RED : LED_color_e::OFF);
    set_neopixel_color(LED_ID_e::TORQUE_MODE, LED_color_e::OFF); // Unused for now
    set_neopixel_color(LED_ID_e::LAUNCH_CTRL, LED_color_e::OFF); // Unused for now
    set_neopixel_color(LED_ID_e::CRIT_CHARGE, LED_color_e::OFF); // Unused for now
    set_neopixel_color(LED_ID_e::INERTIA, LED_color_e::OFF); // Unused for now
    set_neopixel_color(LED_ID_e::COCKPIT_BRB, LED_color_e::OFF); // Unused for now
    set_neopixel_color(LED_ID_e::BOTS, LED_color_e::OFF); // Unused for now
    set_neopixel_color(LED_ID_e::IMD, vcr_data.interface_data.shutdown_sensing_data.imd_is_ok ? LED_color_e::GREEN : LED_color_e::RED);
    set_neopixel_color(LED_ID_e::BMS, vcr_data.interface_data.shutdown_sensing_data.bms_is_ok ? LED_color_e::GREEN : LED_color_e::RED);
    set_neopixel_color(LED_ID_e::MC_ERR, (vcr_data.interface_data.inverter_data.FL.error || vcr_data.interface_data.inverter_data.FR.error || vcr_data.interface_data.inverter_data.RL.error || vcr_data.interface_data.inverter_data.RR.error) ? LED_color_e::RED : LED_color_e::OFF);
    set_neopixel_color(LED_ID_e::RDY_DRIVE, LED_color_e::RED);
    set_neopixel_color(LED_ID_e::GLV, vcr_data.interface_data.current_sensor_data.twentyfour_volt_sensor > 22.0f ? LED_color_e::GREEN : LED_color_e::OFF); // No sensor there yet
    
    _neopixels.show();

}

void NeopixelController::set_neopixel_color(LED_ID_e led, LED_color_e color)
{
    _neopixels.setPixelColor(led, (uint32_t) color);
}