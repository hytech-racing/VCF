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

// Tasks
HT_TASK::Task CAN_receive(HT_TASK::DUMMY_FUNCTION, handle_CAN_receive, CAN_RECV_PRIORITY);
HT_TASK::Task CAN_send(HT_TASK::DUMMY_FUNCTION, handle_CAN_send, CAN_SEND_PRIORITY);
HT_TASK::Task dash_CAN_enqueue(HT_TASK::DUMMY_FUNCTION, send_dash_data, DASH_SEND_PRIORITY, DASH_SEND_PERIOD);


void setup() {
    Serial.begin(115200); // NOLINT (common baud rate)

    const uint32_t CAN_baudrate = 500000;

    // Setup scheduler
    scheduler.setTimingFunction(micros);

    // Create dashboard singleton
    DashboardGPIOs_s dashboard_gpios = {
        .DIM_BUTTON = BTN_DIM_READ,
        .PRESET_BUTTON = BTN_PRESET_READ,
        .MC_CYCLE_BUTTON = BTN_MC_CYCLE_READ,
        .MODE_BUTTON = BTN_MODE_READ,
        .START_BUTTON = BTN_START_READ,
        .DATA_BUTTON = BTN_DATA_READ,
        .LEFT_SHIFTER_BUTTON = LEFT_SHIFTER,
        .RIGHT_SHIFTER_BUTTON = RIGHT_SHIFTER,
        .DIAL_SDA = I2C_SDA,
        .DIAL_SCL = I2C_SCL
    };
    DashboardInterfaceInstance::create(dashboard_gpios); // NOLINT (idk why it's saying this is uninitialized. It definitely is.)

    // Create can singletons
    VCFCANInterfaceImpl::CANInterfacesInstance::create(DashboardInterfaceInstance::instance()); 
    
    etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> main_can_recv;
    main_can_recv = etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)>::create<VCFCANInterfaceImpl::vcf_recv_switch>();
    VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::create(main_can_recv); // NOLINT (Not sure why it's uninitialized. I think it is.)

    VCFCANInterfaceObjects can_interface_objects = VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance();

    // Setup CAN
    handle_CAN_setup(can_interface_objects.MAIN_CAN, CAN_baudrate, VCFCANInterfaceImpl::on_main_can_recv);

    // Schedule Tasks
    scheduler.schedule(CAN_receive); 
    scheduler.schedule(CAN_send); 
    scheduler.schedule(dash_CAN_enqueue);

    Ethernet.begin(EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().default_dns, EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);
}

void loop() {
    scheduler.run();
}