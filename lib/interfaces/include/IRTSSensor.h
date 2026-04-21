#ifndef IRTSSENSORINTERFACE_H
#define IRTSSENSORINTERFACE_H

#include "FlexCAN_T4.h"
#include "SharedFirmwareTypes.h"

#define UNPACK_AND_POPULATE_TEMP(WHEEL, MSGNUM, CH1, CH2, CH3, CH4) \
    WHEEL##_IRTS_##MSGNUM##_t unpacked_sensor_data; \
    Unpack_##WHEEL##_IRTS_##MSGNUM##_hytech(&unpacked_sensor_data, msg.buf, msg.len); \
    latest_sensor_data.infrared_temp[CH1 - 1]     = unpacked_sensor_data.WHEEL##IRTS_T##CH1##_ro; \
    latest_sensor_data.infrared_temp[CH1]         = unpacked_sensor_data.WHEEL##IRTS_T##CH2##_ro; \
    latest_sensor_data.infrared_temp[CH1 + 1]     = unpacked_sensor_data.WHEEL##IRTS_T##CH3##_ro; \
    latest_sensor_data.infrared_temp[CH1 + 2]     = unpacked_sensor_data.WHEEL##IRTS_T##CH4##_ro; \

class IRTSSensorInterface
{
    public:
        // LF wheel
        void receive_IRTS_sensor_temp_lf_ch1_ch4(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_IRTS_sensor_temp_lf_ch5_ch8(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_IRTS_sensor_temp_lf_ch9_ch12(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_IRTS_sensor_temp_lf_ch13_ch16(const CAN_message_t &msg, unsigned long curr_millis);

        // RF wheel
        void receive_IRTS_sensor_temp_rf_ch1_ch4(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_IRTS_sensor_temp_rf_ch5_ch8(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_IRTS_sensor_temp_rf_ch9_ch12(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_IRTS_sensor_temp_rf_ch13_ch16(const CAN_message_t &msg, unsigned long curr_millis);

        // LR wheel
        void receive_IRTS_sensor_temp_lr_ch1_ch4(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_IRTS_sensor_temp_lr_ch5_ch8(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_IRTS_sensor_temp_lr_ch9_ch12(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_IRTS_sensor_temp_lr_ch13_ch16(const CAN_message_t &msg, unsigned long curr_millis);

        // RR wheel
        void receive_IRTS_sensor_temp_rr_ch1_ch4(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_IRTS_sensor_temp_rr_ch5_ch8(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_IRTS_sensor_temp_rr_ch9_ch12(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_IRTS_sensor_temp_rr_ch13_ch16(const CAN_message_t &msg, unsigned long curr_millis);

        IRTSSensorData_s get_latest_sensor_data() const;

    private:
        IRTSSensorData_s latest_sensor_data;
};

#endif