#ifndef VCFCANINTERFACEIMPL_H
#define VCFCANINTERFACEIMPL_H


#include "FlexCAN_T4.h"

#include "etl/delegate.h"
#include "etl/singleton.h"

#include "CANInterface.h"

#include "DashboardInterface.h"
#include "ACUInterface.h"
#include "VCRInterface.h"

#include "SharedFirmwareTypes.h"

#include "hytech.h" // generated CAN library

/* Globally accessible types */
using CANRXBufferType = Circular_Buffer<uint8_t, (uint32_t)16, sizeof(CAN_message_t)>;
using CANTXBufferType = Circular_Buffer<uint8_t, (uint32_t)128, sizeof(CAN_message_t)>;
template <CAN_DEV_TABLE CAN_DEV> using FlexCAN_Type = FlexCAN_T4<CAN_DEV, RX_SIZE_256, TX_SIZE_16>;

/* Interfaces accessible to this one */
struct CANInterfaces {
    explicit CANInterfaces(DashboardInterface &dash_int, ACUInterface &acu_int, VCRInterface &vcr_int)
        : dash_interface(dash_int),
          acu_interface(acu_int),
          vcr_interface(vcr_int) {}

    DashboardInterface &dash_interface;
    ACUInterface &acu_interface;
    VCRInterface &vcr_interface;
};

namespace VCFCANInterfaceImpl {
    extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> TELEM_CAN;
    extern FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> FAUX_CAN;
    
    extern CANRXBufferType telem_can_rx_buffer;
    extern CANTXBufferType telem_can_tx_buffer;
    
    extern CANRXBufferType faux_can_rx_buffer;
    extern CANTXBufferType faux_can_tx_buffer;
    
    void on_main_can_recv(const CAN_message_t &msg);
    void on_faux_can_recv(const CAN_message_t &msg);
    
    void vcf_recv_switch(CANInterfaces &interfaces, const CAN_message_t &msg, unsigned long millis, CANInterfaceType_e interface_type); //vcf can receive
    void send_all_CAN_msgs(CANTXBufferType &buffer, FlexCAN_T4_Base *can_interface);

    using CANInterfacesInstance = etl::singleton<CANInterfaces>;
}

#endif // VCFCANINTERFACEIMPL_H
