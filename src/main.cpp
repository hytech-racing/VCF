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

/* Scheduler setup */
// HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

using namespace qindesign::network;

DashboardGPIOs_s dashboard_gpios {
    .START_BUTTON = 6
};

void setup() {
    Serial.begin(115200);
    Ethernet.begin(EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().default_dns, EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);
    DashboardInterfaceInstance::create(dashboard_gpios); 
}

void loop() {
    DashboardInterface inst = DashboardInterfaceInstance::instance();
    DashboardOutputs_s dash_outputs = inst.get_dashboard_outputs(); 
    Serial.println(dash_outputs.START_BUTTON);

}