#include "VCFCANInterfaceImpl.h"
#include "BuzzerController.h"

namespace VCFCANInterfaceImpl {

    CANRXBufferType telem_can_rx_buffer;
    CANTXBufferType telem_can_tx_buffer;

    CANRXBufferType faux_can_rx_buffer;
    CANTXBufferType faux_can_tx_buffer;
    
    void on_main_can_recv(const CAN_message_t &msg)
    {
        uint8_t buf[sizeof(CAN_message_t)];
        memmove(buf, &msg, sizeof(msg)); // NOLINT (memory operations are fine)
        telem_can_rx_buffer.push_back(buf, sizeof(CAN_message_t));
    }

    void on_faux_can_recv(const CAN_message_t &msg)
    {
        TELEM_CAN.write(msg); //immediately forward onto telem can to view data
        uint8_t buf[sizeof(CAN_message_t)];
        memmove(buf, &msg, sizeof(msg)); // NOLINT (memory operations are fine)
        faux_can_rx_buffer.push_back(buf, sizeof(CAN_message_t));

        Serial.println("msg recvd");
        Serial.print("MB: "); Serial.print(msg.mb);
        Serial.print("  ID: 0x"); Serial.print(msg.id, HEX);
        Serial.print("  EXT: "); Serial.print(msg.flags.extended);
        Serial.print("  LEN: "); Serial.print(msg.len);
        Serial.print(" DATA: ");
        for ( uint8_t i = 0; i < 8; i++ ) {
        Serial.print(msg.buf[i]); Serial.print(" ");
        }
        Serial.print("  TS: "); Serial.println(msg.timestamp);
    }

    void vcf_recv_switch(CANInterfaces &interfaces, const CAN_message_t &msg, unsigned long millis, CANInterfaceType_e interface_type)
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
            case INV1_STATUS_CANID:
            {
                interfaces.vcr_interface.receive_inverter_status_1(msg);
                break;
            }
            case INV2_STATUS_CANID:
            {
                interfaces.vcr_interface.receive_inverter_status_2(msg);
                break;
            }
            case INV3_STATUS_CANID:
            {
                interfaces.vcr_interface.receive_inverter_status_3(msg);
                break;
            }
            case INV4_STATUS_CANID:
            {
                interfaces.vcr_interface.receive_inverter_status_4(msg);
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
