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

struct VCFCANInterfaceObjects {

    VCFCANInterfaceObjects(etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long, CANInterfaceType_e)> recv_switch, FlexCAN_T4_Base * main_can): MAIN_CAN(main_can),can_recv_switch(recv_switch) 
    {} 

    
    FlexCAN_T4_Base* MAIN_CAN;
    // TODO fix this. needs to be a pointer to an in-main global created instance of CAN3. this also cannot be a base type.
    CANRXBufferType main_can_rx_buffer;
    CANTXBufferType main_can_tx_buffer;
    etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long, CANInterfaceType_e)> can_recv_switch;
    
};

namespace VCFCANInterfaceImpl {
    void on_main_can_recv(const CAN_message_t &msg);
    void vcf_recv_switch(CANInterfaces &interfaces, const CAN_message_t &msg, unsigned long millis, CANInterfaceType_e interface_type);
    void send_all_CAN_msgs(CANTXBufferType &buffer, FlexCAN_T4_Base *can_interface);

    extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> main_can;

    using VCFCANInterfaceObjectsInstance = etl::singleton<VCFCANInterfaceObjects>;
    using CANInterfacesInstance = etl::singleton<CANInterfaces>;
}

#endif // VCFCANINTERFACEIMPL_H
