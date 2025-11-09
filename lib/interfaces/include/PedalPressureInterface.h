#ifndef PEDALPRESSUREINTERFACE_H
#define PEDALPRESSUREINTERFACE_H

#include "hytech.h"
#include "VCFCANInterfaceImpl.h"
#include "SharedFirmwareTypes.h"

class PedalPressureInterface {
public:
    void send_pedal_pressure_CAN(uint16_t pedal_pressure);
};
#endif  // PEDALPRESSUREINTERFACE_H