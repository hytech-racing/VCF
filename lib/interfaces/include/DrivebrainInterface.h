#ifndef DRIVEBRAIN_INTERFACE_H
#define DRIVEBRAIN_INTERFACE_H

#include "SharedFirmwareTypes.h"
#include "Arduino.h"
#include "BuzzerController.h"
#include "etl/singleton.h"
#include "hytech.h"
#include "FlexCAN_T4.h"

class DrivebrainInterface 
{
    public:
        void receive_vn_status(const CAN_message_t &can_msg);

        uint8_t get_vn_status() { return _vn_status;}

    private:
        uint8_t _vn_status;
};

using DrivebrainInterfaceInstance = etl::singleton<DrivebrainInterface>;

#endif /* DRIVEBRAIN_INTERFACE_H */