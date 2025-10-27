#ifndef PEDALPRESSUREINTERFACE_H
#define PEDALPRESSUREINTERFACE_H

#include "hytech.h"
#include "VCFCANInterfaceImpl.h"

class PedalPressureInterface {
public:
    void send_pedal_pressure_CAN(uint16_t pedal_pressure);

private:
    PEDAL_PRESSURE_t msg_data;
};
#endif  // PEDALPRESSUREINTERFACE_H