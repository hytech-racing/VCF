#ifdef ARDUINO
#include <Arduino.h>
#endif


/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"
#include "EthernetAddressDefs.h"

/* From HT_SCHED libdep */
#include "ht_sched.hpp"

/* From Arduino Libraries */
#include "QNEthernet.h"

/* Local includes */
#include "VCF_Globals.h"
#include "VCF_Constants.h"
#include "VCF_Tasks.h"
#include "PedalsSystem.h"
#include "VCFEthernetInterface.h"

/* CAN Interface stuff */
#include "VCFCANInterfaceImpl.h"
#include "CANInterface.h"

/* Scheduler setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

using namespace qindesign::network;

etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> main_can_recv = etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)>::create<VCFCANInterfaceImpl::vcf_recv_switch>();


void setup() {
    const uint32_t CAN_baudrate = 500000;

    VCFCANInterfaceImpl::CANInterfacesInstance::create(); 
    VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::create(main_can_recv);

    VCFCANInterfaceObjects can_interface_objects = VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance();
    handle_CAN_setup(can_interface_objects.MAIN_CAN, CAN_baudrate, VCFCANInterfaceImpl::on_main_can_recv);

    Ethernet.begin(EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().default_dns, EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);
}

void loop() {
    

}