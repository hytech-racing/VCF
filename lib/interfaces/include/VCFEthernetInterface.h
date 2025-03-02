#ifndef VCF_ETHERNET_INTERFACE_H
#define VCF_ETHERNET_INTERFACE_H

#include "hytech_msgs.pb.h"
#include "SharedFirmwareTypes.h"

namespace VCFEthernetInterface 
{
    /**
     * Function to transform our struct from shared_data_types into the protoc struct hytech_msgs_VCFData_s.
     * 
     * @param shared_state The current VCF state, which includes both interface and system data.
     * @return A populated instance of the outgoing protoc struct.
     */
    hytech_msgs_VCFData_s make_vcf_data_msg(VCFData_s &shared_state);

    /**
     * Function to take a populated protoc struct from VCR and update the VCF state. This is ONLY critical
     * for buzzer control!
     * 
     * @param msg_in A reference to a populated protoc struct.
     * @param shared_state A reference to the VCF state.
     * 
     * @post After this function completes, shared_state will contain the updated buzzer control.
     */
    void receive_pb_msg_vcr(const hytech_msgs_VCRData_s &msg_in, VCFData_s &shared_state, unsigned long curr_millis);

}

#endif /* VCF_ETHERNET_INTERFACE_H */