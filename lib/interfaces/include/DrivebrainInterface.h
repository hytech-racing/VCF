#ifndef __DRIVEBRAININTERFACE_H__
#define __DRIVEBRAININTERFACE_H__

#include "etl/singleton.h"

#include "SharedFirmwareTypes.h"

#include "FlexCAN_T4.h"
class DrivebrainInterface {
  public:
    DrivebrainInterface(const RearLoadCellData_s& rear_load_cell_data, const RearSusPotData_s& rear_suspot_data);
    void receive_drivebrain_speed_command(const CAN_message_t &msg,
                                          unsigned long curr_millis);
    void receive_drivebrain_torque_lim_command(const CAN_message_t &msg,
                                               unsigned long curr_millis);

    void send_suspension_CAN_data();
    StampedDrivetrainCommand_s get_latest_data();

  private:
    struct {
      const RearLoadCellData_s& rear_load_cell_data;
      const RearSusPotData_s& rear_suspot_data;
    } _suspension_data;
    StampedDrivetrainCommand_s _latest_drivebrain_command = {};
};

using DrivebrainInterfaceInstance = etl::singleton<DrivebrainInterface>;
#endif // __DRIVEBRAININTERFACE_H__