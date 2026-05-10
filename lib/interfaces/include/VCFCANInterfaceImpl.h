#ifndef VCFCANINTERFACEIMPL_H
#define VCFCANINTERFACEIMPL_H


#include "FlexCAN_T4.h"

#include "etl/delegate.h"
#include "etl/singleton.h"

#include "CANInterface.h"

#include "DashboardInterface.h"
#include "ACUInterface.h"
#include "VCRInterface.h"
#include "BrakeRotorTemp.h"

#include "SharedFirmwareTypes.h"

#include "hytech.h" // generated CAN library

/* Globally accessible types */
using CANRXBufferType = Circular_Buffer<uint8_t, (uint32_t)16, sizeof(CAN_message_t)>;
using CANTXBufferType = Circular_Buffer<uint8_t, (uint32_t)128, sizeof(CAN_message_t)>;

// Definitions of VCF CAN bus types
using TelemCAN_t = FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>;
using FrontAuxCAN_t = FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>;

/* Interfaces accessible to this one */
struct CANInterfaces {
    explicit CANInterfaces(DashboardInterface &dash_int, ACUInterface &acu_int, VCRInterface &vcr_int, BrakeRotorTemp &brake_rotor_temp_int)
        : dash_interface(dash_int),
          acu_interface(acu_int),
          vcr_interface(vcr_int),
          brake_rotor_temp_interface(brake_rotor_temp_int) {}

    DashboardInterface &dash_interface;
    ACUInterface &acu_interface;
    VCRInterface &vcr_interface;
    BrakeRotorTemp &brake_rotor_temp_interface;
};
using CANInterfacesInstance = etl::singleton<CANInterfaces>;

struct VCFCANInterface {
    VCFCANInterface(etl::delegate<void (CANInterfaces &, const CAN_message_t &, unsigned long, CANInterfaceType_e)> recv_switch_func) 
        : can_recv_switch(recv_switch_func) 
        {}

    TelemCAN_t TELEM_CAN;

    CANRXBufferType telem_can_rx_buffer;
    CANTXBufferType telem_can_tx_buffer;

    FrontAuxCAN_t FRONT_AUX_CAN;

    CANRXBufferType front_aux_can_rx_buffer;
    CANTXBufferType front_aux_can_tx_buffer;

    etl::delegate<void (CANInterfaces &, const CAN_message_t &, unsigned long, CANInterfaceType_e)> can_recv_switch;
};
using VCFCANInterfaceInstance = etl::singleton<VCFCANInterface>;

namespace VCFCANInterfaceImpl {
    void on_main_can_recv(const CAN_message_t &msg);
    void on_faux_can_recv(const CAN_message_t &msg);
    
    void vcf_recv_switch(CANInterfaces &interfaces, const CAN_message_t &msg, unsigned long millis, CANInterfaceType_e interface_type); //vcf can receive
    void send_all_CAN_msgs(CANTXBufferType &buffer, FlexCAN_T4_Base *can_interface);
}

#endif // VCFCANINTERFACEIMPL_H
