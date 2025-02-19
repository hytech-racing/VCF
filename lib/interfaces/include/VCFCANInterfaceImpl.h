#ifndef __VCFCANINTERFACEIMPL_H__
#define __VCFCANINTERFACEIMPL_H__

#include <cstdint>

#include <tuple>
#include <utility>

#include "FlexCAN_T4.h"

#include "etl/delegate.h"

#include "CANInterface.h"


#include "InverterInterface.h"

#include "SharedFirmwareTypes.h"

#include "hytech.h" // generated CAN library
#include "VCRInterface.h"
#include "DrivebrainInterface.h"

//#include "VCRInterface.h"
//#include "DrivebrainInterface.h"

using CANRXBufferType = Circular_Buffer<uint8_t, (uint32_t)16, sizeof(CAN_message_t)>;
using CANTXBufferType = Circular_Buffer<uint8_t, (uint32_t)128, sizeof(CAN_message_t)>;

/* RX buffers for CAN extern declarations*/
extern CANRXBufferType CAN1_rxBuffer;
extern CANRXBufferType inverter_can_rx_buffer;
extern CANRXBufferType telem_can_rx_buffer;

/* TX buffer for CAN1 */
extern CANTXBufferType CAN1_txBuffer;
/* TX buffer for CAN2 */
extern CANTXBufferType inverter_can_tx_buffer;
/* TX buffer for CAN3 */
extern CANTXBufferType telem_can_tx_buffer;

template <CAN_DEV_TABLE CAN_DEV>
using FlexCAN_Type = FlexCAN_T4<CAN_DEV, RX_SIZE_256, TX_SIZE_16>;

// this is being done to send immediately from the inverter CAN line to the TELEM CAN every inverter 
extern FlexCAN_Type<CAN3> TELEM_CAN; // gets defined in main as of right now

void on_can2_receive(const CAN_message_t &msg);
//void on_inverter_can_receive(const CAN_message_t &msg);
void on_telem_can_receive(const CAN_message_t &msg);

struct CANInterfaces
{
    explicit CANInterfaces(VCRInterface& vcr_int, DrivebrainInterface& db_int): vcr_interface(vcr_int), db_interface(db_int) {}

    VCRInterface& vcr_interface;
    DrivebrainInterface &db_interface;
};

namespace VCFCANInterfaceImpl
{
    void vcf_CAN_recv(CANInterfaces& interfaces, const CAN_message_t& msg, unsigned long millis);
    void send_all_CAN_msgs(CANTXBufferType &buffer, FlexCAN_T4_Base *can_interface);

};


#endif // __VCFCANINTERFACEIMPL_H__