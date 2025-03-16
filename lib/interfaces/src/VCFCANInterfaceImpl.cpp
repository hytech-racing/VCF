#include "VCFCANInterfaceImpl.h"

namespace VCFCANInterfaceImpl {
    void on_main_can_recv(const CAN_message_t &msg)
    {
        uint8_t buf[sizeof(CAN_message_t)];
        memmove(buf, &msg, sizeof(msg)); // NOLINT (memory operations are fine)
        VCFCANInterfaceObjectsInstance::instance().main_can_rx_buffer.push_back(buf, sizeof(CAN_message_t));
    }

    void vcf_recv_switch(CANInterfaces &interfaces, const CAN_message_t &msg, unsigned long millis)
    {
        // TODO Implement this later
        switch (msg.id) 
        {
            default:
                break;
        }
    }

    void send_all_CAN_msgs(CANTXBufferType &buffer, FlexCAN_T4_Base *can_interface)
    {
        CAN_message_t msg;
        while (buffer.available()) {
            CAN_message_t msg;
            uint8_t buf[sizeof(CAN_message_t)];
            buffer.pop_front(buf, sizeof(CAN_message_t));
            memmove(&msg, buf, sizeof(msg)); // NOLINT (memory operations are fine)
            can_interface->write(msg);
        }
    }
}
