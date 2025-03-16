#ifdef ARDUINO
#endif


/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"
#include "EthernetAddressDefs.h"

/* From HT_SCHED libdep */
// #include "ht_sched.hpp"

#include <Arduino.h>

/* From Arduino Libraries */
#include "QNEthernet.h"

/* Local includes */
#include "VCF_Globals.h"
#include "VCF_Constants.h"
#include "VCF_Tasks.h"
#include "PedalsSystem.h"
#include "DashboardInterface.h"
// #include "VCFEthernetInterface.h"
#include "VCFCANInterfaceImpl.h"
#include "ht_sched.hpp"
#include "ht_task.hpp"

#include "hytech.h"

/* Scheduler setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

using namespace qindesign::network;

DashboardGPIOs_s dashboard_gpios {
    .START_BUTTON = 6
};

etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> main_can_recv = etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)>::create<VCFCANInterfaceImpl::vcf_recv_switch>();

// Send Periods
constexpr unsigned long dash_send_period = 4000;             // 4 000 us = 250 Hz
constexpr unsigned long dash_receive_period = 4000;

// Tasks
HT_TASK::Task CAN_receive(HT_TASK::DUMMY_FUNCTION, handle_CAN_receive, 0);
HT_TASK::Task CAN_send(HT_TASK::DUMMY_FUNCTION, handle_CAN_send, 5);
HT_TASK::Task dash_CAN_enqueue(HT_TASK::DUMMY_FUNCTION, send_dash_data, 5, dash_send_period);
HT_TASK::Task dash_CAN_receive(HT_TASK::DUMMY_FUNCTION, receive_dash_inputs, 5, dash_receive_period);

void setup() {
    Serial.begin(115200);

    const uint32_t CAN_baudrate = 500000;

    // Setup scheduler
    scheduler.setTimingFunction(micros);

    // Create dashboard singleton
    DashboardInterfaceInstance::create(dashboard_gpios);

    // Create can singletons
    VCFCANInterfaceImpl::CANInterfacesInstance::create(DashboardInterfaceInstance::instance()); 
    VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::create(main_can_recv);

    VCFCANInterfaceObjects can_interface_objects = VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance();

    // Setup CAN
    handle_CAN_setup(can_interface_objects.MAIN_CAN, CAN_baudrate, VCFCANInterfaceImpl::on_main_can_recv);

    // Schedule Tasks
    scheduler.schedule(CAN_receive); 
    scheduler.schedule(CAN_send); 
    scheduler.schedule(dash_CAN_enqueue);
    scheduler.schedMon(dash_CAN_receive);

    Ethernet.begin(EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().default_dns, EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);
    DashboardInterfaceInstance::create(dashboard_gpios); 
}

void loop() {
    scheduler.run();
    // DashboardInterface inst = DashboardInterfaceInstance::instance(); 
    // DashInputState_s dash_outputs = inst.get_dashboard_outputs(); 
    // Serial.println(dash_outputs.start_btn_is_pressed);
    // handle_CAN_receive(); 
    // handle_CAN_send();
    // send_dash_data(); 
    // // CAN_message_t msg;
    // // msg.id = 3;
    // // VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance().MAIN_CAN.write(msg);
    // // Serial.println("pluh");
}