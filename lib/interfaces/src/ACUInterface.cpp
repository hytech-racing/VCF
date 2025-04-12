#include "ACUInterface.h"

void ACUInterface::receive_ACU_voltages(const CAN_message_t &can_msg)
{
    BMS_VOLTAGES_t unpacked_msg;
    Unpack_BMS_VOLTAGES_hytech(&unpacked_msg, can_msg.buf, can_msg.len);
    
    _voltages_not_critical = HYTECH_low_voltage_ro_fromS(unpacked_msg.low_voltage_ro) > 3.3f;
}
