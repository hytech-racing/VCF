#include "PedalPressureInterface.h"

void PedalPressureInterface::send_pedal_pressure_CAN(uint16_t pedal_pressure)
{
     msg_data.Pedal_Pressure = pedal_pressure;

    uint8_t buf[8];
    uint8_t len = 0;
    uint8_t ide = 0;

    Pack_PEDAL_PRESSURE_hytech(&msg_data, buf, &len, &ide);

    // Push to VCF CAN TX buffer
    VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance().main_can_tx_buffer.push_back(buf, len);
}
