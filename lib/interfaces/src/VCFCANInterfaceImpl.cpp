#include "VCFCANInterfaceImpl.h"
#include "BuzzerController.h"

namespace VCFCANInterfaceImpl {
    void on_main_can_recv(const CAN_message_t &msg)
    {
        uint8_t buf[sizeof(CAN_message_t)];
        memmove(buf, &msg, sizeof(msg)); // NOLINT (memory operations are fine)
        VCFCANInterfaceObjectsInstance::instance().main_can_rx_buffer.push_back(buf, sizeof(CAN_message_t));
    }

    void vcf_recv_switch(CANInterfaces &interfaces, const CAN_message_t &msg, unsigned long millis)
    {
        switch (msg.id) 
        {
            case DASHBOARD_BUZZER_CONTROL_CANID:
            {
                interfaces.vcr_interface.receive_dash_control_data(msg);
                break;
            } 
            case BMS_VOLTAGES_CANID:
            {
                interfaces.acu_interface.receive_ACU_voltages(msg);
                break;
            }
            case ACU_OK_CANID: 
            {
                interfaces.dash_interface.receive_ACU_OK(msg);
                break;
            }    
            case CAR_STATES_CANID:
            {
                interfaces.vcr_interface.receive_car_states_data(msg);
                break;
            }
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
