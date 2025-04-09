#ifndef NEOPIXEL_CONTROLLER_H
#define NEOPIXEL_CONTROLLER_H

/* Neopixel controller defines */
#define MAX_BRIGHTNESS 255
#define MIN_BRIGHTNESS 3
#define BRIGHTNESS_STEPS 4
#define STEP_BRIGHTNESS (MAX_BRIGHTNESS - MIN_BRIGHTNESS) / BRIGHTNESS_STEPS
// Note from Justin: I know that this sort of breaks the paradigm that we've put
// in place for most of our code, but
// - other libraries do this all the time
// - I don't really see these as externally configurable, so I don't see why we should
// define a new type of struct, add an arg to the constructor, etc.
// - this is how it was implemented on STM32 dash and I want to be fast :)

#include "Adafruit_NeoPixel.h"
#include "SharedFirmwareTypes.h"

enum LED_ID_e
{
    BRAKE = 0,
    COCKPIT_BRB = 1,
    BOTS = 2,
    INERTIA = 3,
    LAUNCH_CTRL = 4,
    TORQUE_MODE = 5,
    CRIT_CHARGE = 6,
    GLV = 7,
    RDY_DRIVE = 8,
    MC_ERR = 9,
    IMD = 10,
    BMS = 11
};

enum class LED_color_e
{
    OFF = 0x00000000,
    GREEN = 0x0000FF00,
    YELLOW = 0x00FFFF00,
    RED = 0x00FF0000,
};

class NeopixelController
{
    public:
    NeopixelController(uint32_t neopixel_count, uint32_t neopixel_pin) :
        _neopixels(neopixel_count, neopixel_pin),
        _current_brightness(UINT8_MAX)
    {};

    NeopixelController() = delete;
    
    void dim_neopixels();
    void set_neopixel(uint16_t id, uint32_t c);
    void NeopixelController::refresh_neopixels(VCFData_s vcf_data, VCRData_s vcr_data);
    void NeopixelController::set_neopixel_color(LED_ID_e led, LED_color_e color);

    private:
    
    Adafruit_NeoPixel _neopixels;
    uint8_t _current_brightness;

};

#endif /* NEOPIXEL_CONTROLLER_H */