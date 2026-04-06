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


struct MinCellMonitoringThresholds_s
{
    float max_level = 3.8;
    float second_level = 3.7;
    float third_level = 3.6;
    float fourth_level = 3.5;
    float fifth_level = 3.45;
    float critical_charge_level = 3.4;
};

enum LED_ID_e
{
    SHUTDOWN = 0,
    INVERTER_ERR = 1,
    TORQUE_MODE = 2,
    BRAKE = 3,
    BMS = 4,
    GLV = 5,
    PACK = 6,
    IMD = 7,
    IMPLAUSE = 8,
    RDY_DRIVE = 9,
    LATCH = 10,
    CRIT_CHARGE = 11
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
    void refresh_neopixels(const PedalsSystemData_s &pedals_data, CANInterfaces &interfaces);
    void set_neopixel_color(LED_ID_e led, LED_color_e color);

    private:
    
    Adafruit_NeoPixel _neopixels;
    uint8_t _current_brightness;
    uint8_t _neopixel_count;
    const uint8_t _hv_threshold_voltage = 60;
    MinCellMonitoringThresholds_s _min_cell_thresholds;
};

using NeopixelControllerInstance = etl::singleton<NeopixelController>;

#endif /* NEOPIXEL_CONTROLLER_H */