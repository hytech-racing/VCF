#ifndef __VCFDRIVEBRAININTERFACE_H__
#define __VCFDRIVEBRAININTERFACE_H__


#include "etl/singleton.h"

#include "SharedFirmwareTypes.h"

#include "FlexCAN_T4.h"
class DrivebrainInterface {
  public:
    DrivebrainInterface(const FrontLoadCellData_s& front_load_cell_data, const FrontSusPotData_s& front_suspot_data);

  void send_suspension_CAN_data();
  private:
    struct {
      const FrontLoadCellData_s& front_load_cell_data;
      const FrontSusPotData_s& front_suspot_data;
    } _suspension_data;
    
    StampedDrivetrainCommand_s _latest_drivebrain_command = {};
};

using DrivebrainInterfaceInstance = etl::singleton<DrivebrainInterface>;

#endif // __VCFDRIVEBRAININTERFACE_H__