#include "VCFCANInterfaceImpl.h"

#include <Arduino.h>
#include "hytech.h"
#include <cstdint>
// // global forwards
CANRXBufferType CAN1_rxBuffer;
CANRXBufferType inverter_can_rx_buffer;
CANRXBufferType telem_can_rx_buffer;

CANTXBufferType CAN1_txBuffer;
CANTXBufferType inverter_can_tx_buffer;
CANTXBufferType telem_can_tx_buffer;


void on_can2_receive(const CAN_message_t &msg)
{
    uint8_t buf[sizeof(CAN_message_t)];
    memmove(buf, &msg, sizeof(msg));
    CAN1_rxBuffer.push_back(&buf[0], sizeof(CAN_message_t));
}

void on_telem_can_receive(const CAN_message_t &msg)
{
    uint8_t buf[sizeof(CAN_message_t)];
    memmove(buf, &msg, sizeof(msg));
    telem_can_rx_buffer.push_back(&buf[0], sizeof(CAN_message_t));
}

struct CANinterfaces
{

};
namespace VCFCANInterfaceImpl
{

void vcf_CAN_recv(CANInterfaces& interfaces, const CAN_message_t& msg, unsigned long millis)
{
    switch(msg.id)
    {
        case INV1_DYNAMICS_CANID:
        {
            INV1_DYNAMICS_t test_msg;
            
            Unpack_INV1_DYNAMICS_hytech(&test_msg, &msg.buf[0], msg.len);
            Serial.print("recvd rpm: ");
            Serial.println(test_msg.actual_speed_rpm);
            break;
        }
        default:
        {
            break;
        }
        
    }
    }

    void send_all_CAN_msgs(CANTXBufferType &buffer, FlexCAN_T4_Base *can_interface){
        CAN_message_t msg;
        while (buffer.available())
        {
            CAN_message_t msg;
            uint8_t buf[sizeof(CAN_message_t)];
            buffer.pop_front(buf, sizeof(CAN_message_t));
            memmove(&msg, buf, sizeof(msg));
            can_interface->write(msg);
        }
    }
}

