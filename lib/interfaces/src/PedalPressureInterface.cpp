#include "PedalPressureInterface.h"
#include <string.h>  // for memcpy

#define PEDAL_PRESSURE_CANID 0x0CBU
#define PEDAL_PRESSURE_DLC   2U  // 16-bit pedal pressure

void PedalPressureInterface::send_pedal_pressure_CAN(uint16_t pedal_pressure)
{
    // Create the CAN frame
    CAN_message_t msg;
    msg.id = PEDAL_PRESSURE_CANID;
    msg.len = PEDAL_PRESSURE_DLC;
    msg.flags.extended = 0; // standard frame
    msg.buf[0] = static_cast<uint8_t>(pedal_pressure & 0xFF);        // low byte
    msg.buf[1] = static_cast<uint8_t>((pedal_pressure >> 8) & 0xFF); // high byte

    // Copy the message into a raw byte buffer
    uint8_t raw_buf[sizeof(CAN_message_t)];
    memcpy(raw_buf, &msg, sizeof(CAN_message_t));

    // Push to the VCF CAN TX buffer
    auto &can_objs = VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance();
    can_objs.main_can_tx_buffer.push_back(raw_buf, sizeof(CAN_message_t));

    // Use the helper to send all messages
    VCFCANInterfaceImpl::send_all_CAN_msgs(can_objs.main_can_tx_buffer, can_objs.MAIN_CAN);
}
