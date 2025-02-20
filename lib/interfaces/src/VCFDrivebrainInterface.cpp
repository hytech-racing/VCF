#include "VCFDrivebrainInterface.h"

#include "FlexCAN_T4.h"

#include "CANInterface.h"
#include "VCFCANInterfaceImpl.h"
#include "hytech.h"

DrivebrainInterface::DrivebrainInterface(const FrontLoadCellData_s &front_load_cell_data,
                                         const FrontSusPotData_s &front_suspot_data)
    : _suspension_data{.front_load_cell_data = front_load_cell_data,
                       .front_suspot_data = front_suspot_data} {}

void DrivebrainInterface::send_suspension_CAN_data() {
    FRONT_SUSPENSION_t front_sus_msg;
    front_sus_msg.fl_load_cell = _suspension_data.front_load_cell_data.FL_loadcell_analog;
    front_sus_msg.fr_load_cell = _suspension_data.front_load_cell_data.FR_loadcell_analog;
    front_sus_msg.fl_shock_pot = _suspension_data.front_suspot_data.FL_sus_pot_analog;
    front_sus_msg.fr_shock_pot = _suspension_data.front_suspot_data.FR_sus_pot_analog;

    CAN_util::enqueue_msg(&front_sus_msg, &Pack_FRONT_SUSPENSION_hytech, telem_can_tx_buffer);
}