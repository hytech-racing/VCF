#include "PedalPressureInterface.h"
#include "VCFCANInterfaceImpl.h"
#include <cstdint>
#include <cstring> // for memcpy

// Constants
constexpr uint32_t PEDAL_PRESSURE_CANID = 0x0CBU;
constexpr uint8_t PEDAL_PRESSURE_DLC = 2U;  // 16-bit pedal pressure
constexpr uint8_t BYTE_MASK = 0xFF;
constexpr size_t CAN_MSG_SIZE = sizeof(CAN_message_t);

void PedalPressureInterface::send_pedal_pressure_CAN(uint16_t pedal_pressure)
{
    // Construct the CAN message
    CAN_message_t msg{};
    msg.id = PEDAL_PRESSURE_CANID;
    msg.len = PEDAL_PRESSURE_DLC;
    msg.flags.extended = 0;

    // Encode the 16-bit pedal pressure
    msg.buf[0] = static_cast<uint8_t>(pedal_pressure & BYTE_MASK);        // low byte
    msg.buf[1] = static_cast<uint8_t>((pedal_pressure >> 8) & BYTE_MASK); // high byte

    // Copy to raw buffer for TX queue
    uint8_t raw_buf[CAN_MSG_SIZE];
    std::memcpy(static_cast<void*>(raw_buf), &msg, CAN_MSG_SIZE);

    // Push to TX buffer using VCF helper
    auto &can_objs = VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance();
    can_objs.main_can_tx_buffer.push_back(static_cast<uint8_t*>(raw_buf), CAN_MSG_SIZE);

    // Send all messages on the main CAN interface
    VCFCANInterfaceImpl::send_all_CAN_msgs(can_objs.main_can_tx_buffer, can_objs.MAIN_CAN);
}

