#include "CANInterface.h"
#include "VCFCANInterfaceImpl.h"
#include "SystemTimeInterface.h"
// #include "VCF_Tasks.h"
#include "VCFDrivebrainInterface.h"
#include "hytech.h"
#include <cstdint>

// VCRData_s vcr_data;
REAR_SUSPENSION_t rear_sus_msg;
   DrivebrainInterface& db = DrivebrainInterfaceInstance::instance();
   VCRInterface vcr;
   CANInterfaces interfaces(vcr, db);


//CANFD_timings_t tmgs;

FlexCAN_Type<CAN2> INV_CAN;
FlexCAN_Type<CAN3> TELEM_CAN;

etl::delegate<void(CANInterfaces& , const CAN_message_t& , unsigned long )> recv_call;
//CAN_util::enqueue_msg(&rear_sus_msg, &Pack_REAR_SUSPENSION_hytech, telem_can_tx_buffer);


void receive_can_message() {
}

void send_can_message() {
    //VCFCANInterfaceImpl::send_all_CAN_msgs(inverter_can_tx_buffer, &TELEM_CAN);
    //CAN_util::enqueue_msg(&rear_sus_msg, &Pack_REAR_SUSPENSION_hytech, telem_can_tx_buffer);
    CAN_message_t msg;
    PEDALS_SYSTEM_DATA_t pedals = {};
    pedals.accel_pedal_ro = HYTECH_accel_pedal_ro_toS(1.0);
    pedals.brake_pedal_ro = HYTECH_brake_pedal_ro_toS(0.69);
   // pedals.
    msg.id = Pack_PEDALS_SYSTEM_DATA_hytech(&pedals, &msg.buf[0], &msg.len, (uint8_t*) &msg.flags.extended);

    TELEM_CAN.write(msg);
}

void setup()
{
    recv_call = etl::delegate<void(CANInterfaces& , const CAN_message_t& , unsigned long )>::create<VCFCANInterfaceImpl::vcf_CAN_recv>();
    // DrivebrainInterfaceInstance::create(vcf_data., vcr_data.interface_data.rear_suspot_data); 

    const uint32_t CAN_baudrate = 500000;
    // from CANInterfaceon_inverter_can_receive
    handle_CAN_setup(INV_CAN, CAN_baudrate, on_can2_receive);
    handle_CAN_setup(TELEM_CAN, CAN_baudrate, on_telem_can_receive);
    delay(2);
}


void loop()
{
    send_can_message();
    delay(10);
    process_ring_buffer(telem_can_rx_buffer, interfaces, millis(), recv_call);
} 