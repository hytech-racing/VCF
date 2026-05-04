#ifndef BRAKEROTORTEMP_H
#define BRAKEROTORTEMP_H

/* External libraries */
#include "FlexCAN_T4.h"
#include "etl/singleton.h"


namespace BrakeRotorTempDefaultParams {
    constexpr size_t channels_within_brake_temp_sensor = 16; // TODO: check if our sensors are actually 16 channel
}

struct BrakeTempSensorData_s {
    float max_temp;
    float avg_temp;

    std::array<float, BrakeRotorTempDefaultParams::channels_within_brake_temp_sensor> channel_data;
};

struct BrakeTempData_s {
    BrakeTempSensorData_s fl_sensor;
    BrakeTempSensorData_s fr_sensor;
};

class BrakeRotorTemp
{
    public:
        // default empty constructor will init state to zeros
        BrakeRotorTemp() {}

        /**
         * Retrieves the latest data that has been sent from the sensors
         * @return the latest temp data
         */
        BrakeTempData_s getBrakeRotorTempData() const;
        
        /**
         * CAN receive function to parse the new CAN msg and update internal state
         * Called by VCF's recv switch
         * @param msg the CAN msg to parse
         */
        void receiveBrakeRotorTempData(const CAN_message_t &msg);
    
    private:
        BrakeTempData_s _temp_data;

        void _updateCalculatedValues(bool sensor);
};

using BrakeRotorTempInstance = etl::singleton<BrakeRotorTemp>;

#endif // BRAKEROTORTEMP_H