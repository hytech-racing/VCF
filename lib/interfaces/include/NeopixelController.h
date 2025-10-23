#ifndef NEOPIXEL_CONTROLLER_H
#define NEOPIXEL_CONTROLLER_H

/* Neopixel controller defines */
#define MAX_BRIGHTNESS 255
#define MIN_BRIGHTNESS 3
#define BRIGHTNESS_STEPS 4
#define STEP_BRIGHTNESS ((MAX_BRIGHTNESS - MIN_BRIGHTNESS) / BRIGHTNESS_STEPS)
// Note from Justin: I know that this sort of breaks the paradigm that we've put
// in place for most of our code, but
// - other libraries do this all the time
// - I don't really see these as externally configurable, so I don't see why we should
// define a new type of struct, add an arg to the constructor, etc.
// - this is how it was implemented on STM32 dash and I want to be fast :)

#include "Adafruit_NeoPixel.h"
#include "SharedFirmwareTypes.h"
#include "etl/singleton.h"
#include "VCFCANInterfaceImpl.h"


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
    OFF = 0x00,
    GREEN = 0xFF00,
    YELLOW = 0xFFFF00,
    RED = 0xFF0000,
    INIT_COLOR = 0xFF007F,
    BLUE = 0xFF,
    PURPLE = 0x703fab,
    ORANGE = 0xf5a742,
};


class NeopixelController
{
    public:
    NeopixelController(uint32_t neopixel_count, uint32_t neopixel_pin) :
        _neopixels(neopixel_count, neopixel_pin, NEO_GRBW + NEO_KHZ800),
        _current_brightness(64),
        _neopixel_count(neopixel_count)
    {};

    NeopixelController() = delete;
    
    void init_neopixels();
    void dim_neopixels();
    void set_neopixel(uint16_t id, uint32_t c);
    void refresh_neopixels(VCFData_s &vcf_data, VCRData_s &vcr_data, CANInterfaces &interfaces);
    void set_neopixel_color(LED_ID_e led, LED_color_e color);

    private:
    
    Adafruit_NeoPixel _neopixels;
    uint8_t _current_brightness;
    uint8_t _neopixel_count;

};

using NeopixelControllerInstance = etl::singleton<NeopixelController>;

#endif /* NEOPIXEL_CONTROLLER_H */