#include "CANInterface.h"
#include "VCFCANInterfaceImpl.h"
#include "SystemTimeInterface.h"
#include "VCF_Tasks.h"


/* From shared-systems-lib */
#include "Logger.h"

/* From shared_firmware_types library */ 
#include "SharedFirmwareTypes.h"

/* Local includes */
#include "VCF_Constants.h"
#include "BuzzerController.h"
#include "VCF_Globals.h"

// #include "VehicleStateMachine.h"

#include "DrivebrainInterface.h"

// CAN send tasks

// adds rear suspension and vcr status CAN messages to the sent on next mega loop run 
void handle_send_suspension_CAN_data()
{
    DrivebrainInterfaceInstance::instance().send_suspension_CAN_data();
}

void handle_send_all_data()
{
    VCFCANInterfaceImpl::send_all_CAN_msgs(telem_can_tx_buffer, &TELEM_CAN);
}

