#ifndef ACU_INTERFACE_H
#define ACU_INTERFACE_H

#include "SharedFirmwareTypes.h"
#include "Arduino.h"
#include "etl/singleton.h"
#include "hytech.h"
#include "FlexCAN_T4.h"

class ACUInterface 
{
    public:

        ACUCoreData_s get_last_recvd_data() {return _last_recvd_data;}

        bool get_voltages_not_critical() {return _voltages_not_critical;}

        void receive_ACU_voltages(const CAN_message_t &can_msg);
    
    private: 

        ACUCoreData_s _last_recvd_data;
        bool _voltages_not_critical = false;

};



using ACUInterfaceInstance = etl::singleton<ACUInterface>;

#endif /* ACU_INTERFACE_H */