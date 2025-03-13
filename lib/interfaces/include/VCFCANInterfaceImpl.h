#ifndef __VCFCANINTERFACEIMPL_H__
#define __VCFCANINTERFACEIMPL_H__


#include "FlexCAN_T4.h"

#include "etl/delegate.h"
#include "etl/singleton.h"

#include "CANInterface.h"

#include "SharedFirmwareTypes.h"

#include "hytech.h" // generated CAN library

/* Globally accessible types */
using CANRXBufferType = Circular_Buffer<uint8_t, (uint32_t)16, sizeof(CAN_message_t)>;
using CANTXBufferType = Circular_Buffer<uint8_t, (uint32_t)128, sizeof(CAN_message_t)>;
template <CAN_DEV_TABLE CAN_DEV> using FlexCAN_Type = FlexCAN_T4<CAN_DEV, RX_SIZE_256, TX_SIZE_16>;

/* Interfaces accessible to this one */
struct CANInterfaces {
    explicit CANInterfaces() {}
};

struct VCFCANInterfaceObjects {

    VCFCANInterfaceObjects(etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> recv_switch) {
        can_recv_switch = recv_switch;
    } 

    FlexCAN_Type<CAN1> MAIN_CAN;
    CANRXBufferType main_can_rx_buffer;
    CANTXBufferType main_can_tx_buffer;
    etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> can_recv_switch;
    
};

namespace VCFCANInterfaceImpl {
    void on_main_can_recv(const CAN_message_t &msg);
    void vcf_recv_switch(CANInterfaces &interfaces, const CAN_message_t &msg, unsigned long millis);
    void send_all_CAN_msgs(CANTXBufferType &buffer, FlexCAN_T4_Base *can_interface);

    using VCFCANInterfaceObjectsInstance = etl::singleton<VCFCANInterfaceObjects>;
    using CANInterfacesInstance = etl::singleton<CANInterfaces>;
}

#endif // __VCFCANINTERFACEIMPL_H__