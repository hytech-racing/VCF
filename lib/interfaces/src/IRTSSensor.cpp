#include "IRTSSensorInterface.h"
#include "FlexCAN_T4.h"
#include "hytech.h"

// LF wheel
void IRTSSensorInterface::receive_IRTS_sensor_temp_lf_ch1_ch4(const CAN_message_t &msg, unsigned long curr_millis)
{
    UNPACK_AND_POPULATE_TEMP(FL, 1, 1, 2, 3, 4);
}
void IRTSSensorInterface::receive_IRTS_sensor_temp_lf_ch5_ch8(const CAN_message_t &msg, unsigned long curr_millis)
{
    UNPACK_AND_POPULATE_TEMP(FL, 2, 5, 6, 7, 8);
}
void IRTSSensorInterface::receive_IRTS_sensor_temp_lf_ch9_ch12(const CAN_message_t &msg, unsigned long curr_millis)
{
    UNPACK_AND_POPULATE_TEMP(FL, 3, 9, 10, 11, 12);
}
void IRTSSensorInterface::receive_IRTS_sensor_temp_lf_ch13_ch16(const CAN_message_t &msg, unsigned long curr_millis)
{
    UNPACK_AND_POPULATE_TEMP(FL, 4, 13, 14, 15, 16);
}

// RF wheel
void IRTSSensorInterface::receive_IRTS_sensor_temp_rf_ch1_ch4(const CAN_message_t &msg, unsigned long curr_millis)
{
    UNPACK_AND_POPULATE_TEMP(RF, 1, 1, 2, 3, 4);
}
void IRTSSensorInterface::receive_IRTS_sensor_temp_rf_ch5_ch8(const CAN_message_t &msg, unsigned long curr_millis)
{
    UNPACK_AND_POPULATE_TEMP(RF, 2, 5, 6, 7, 8);
}
void IRTSSensorInterface::receive_IRTS_sensor_temp_rf_ch9_ch12(const CAN_message_t &msg, unsigned long curr_millis)
{
    UNPACK_AND_POPULATE_TEMP(RF, 3, 9, 10, 11, 12);
}
void IRTSSensorInterface::receive_IRTS_sensor_temp_rf_ch13_ch16(const CAN_message_t &msg, unsigned long curr_millis)
{
    UNPACK_AND_POPULATE_TEMP(RF, 4, 13, 14, 15, 16);
}


// LFR wheel
void IRTSSensorInterface::receive_IRTS_sensor_temp_lr_ch1_ch4(const CAN_message_t &msg, unsigned long curr_millis)
{
    UNPACK_AND_POPULATE_TEMP(LR, 1, 1, 2, 3, 4);
}
void IRTSSensorInterface::receive_IRTS_sensor_temp_lr_ch5_ch8(const CAN_message_t &msg, unsigned long curr_millis)
{
    UNPACK_AND_POPULATE_TEMP(LR, 2, 5, 6, 7, 8);
}
void IRTSSensorInterface::receive_IRTS_sensor_temp_lr_ch9_ch12(const CAN_message_t &msg, unsigned long curr_millis)
{
    UNPACK_AND_POPULATE_TEMP(LR, 3, 9, 10, 11, 12);
}
void IRTSSensorInterface::receive_IRTS_sensor_temp_lr_ch13_ch16(const CAN_message_t &msg, unsigned long curr_millis)
{
    UNPACK_AND_POPULATE_TEMP(LR, 4, 13, 14, 15, 16);
}

// RR wheel
void IRTSSensorInterface::receive_IRTS_sensor_temp_rr_ch1_ch4(const CAN_message_t &msg, unsigned long curr_millis)
{
    UNPACK_AND_POPULATE_TEMP(RR, 1, 1, 2, 3, 4);
}
void IRTSSensorInterface::receive_IRTS_sensor_temp_rr_ch5_ch8(const CAN_message_t &msg, unsigned long curr_millis)
{
    UNPACK_AND_POPULATE_TEMP(RR, 2, 5, 6, 7, 8);
}
void IRTSSensorInterface::receive_IRTS_sensor_temp_rr_ch9_ch12(const CAN_message_t &msg, unsigned long curr_millis)
{
    UNPACK_AND_POPULATE_TEMP(RR, 3, 9, 10, 11, 12);
}
void IRTSSensorInterface::receive_IRTS_sensor_temp_rr_ch13_ch16(const CAN_message_t &msg, unsigned long curr_millis)
{
    UNPACK_AND_POPULATE_TEMP(RR, 4, 13, 14, 15, 16);
}

IRTSSensorData_s IRTSSensorInterface::get_latest_sensor_data() const
{
    return latest_sensor_data;
}