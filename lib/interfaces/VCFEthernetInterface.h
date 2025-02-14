#include "VCREthernetInterface.h"
#include "SharedFirmwareTypes.h"

hytech_msgs_VCRData_s VCFEthernetInterface::make_vcf_data_msg(VCFData_s &shared_state)
{
	hytech_msgs_VCFData_s out;

    // Load cells
    out.front_loadcell_data.FL_loadcell_analog = shared_state.front_loadcell_data.FL_loadcell_analog
    out.front_loadcell_data.FR_loadcell_analog = shared_state.front_loadcell_data.FR_loadcell_analog
    // Sus pots
    out.front_suspot_data.FL_sus_pot_analog = shared_state.front_suspot_data.FL_sus_pot_analog;
    out.front_suspot_data.FL_sus_pot_analog = shared_state.front_suspot_data.FL_sus_pot_analog;
    // Steering
    // Dash
    // Ethernet link data
    // pedals system
}