#include "DrivebrainInterface.h"

void DrivebrainInterface::receive_vn_status(const CAN_message_t &can_msg)
{
    DRIVEBRAIN_STATE_DATA_t unpacked_msg;
    Unpack_DRIVEBRAIN_STATE_DATA_hytech(&unpacked_msg, can_msg.buf, can_msg.len); //NOLINT
    _vn_status = unpacked_msg.vn_gps_status;
}