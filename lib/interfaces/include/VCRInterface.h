#ifndef __VCRINTERFACE_H__
#define __VCRINTERFACE_H__

#include "FlexCAN_T4.h"


#include "SharedFirmwareTypes.h"
// this struct just contains the data we need from pedals 
// within VCR. the implaus check is done in the state machine.

struct LoadCellData_s : TimestampedData_s
{
    
};

struct VCRCANInterfaceData_s {
    RearLoadCellData_s rear_loadcell_data;
};

class VCRInterface {
public:
    VCRInterface() = default;

    void receive_rearloadcells_message(const CAN_message_t& msg, unsigned long curr_millis);
    
    VCRCANInterfaceData_s get_latest_data();

private:

    VCRCANInterfaceData_s _curr_data;
    
};
#endif // __VCRINTERFACE_H__