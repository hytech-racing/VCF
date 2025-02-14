#include "VCF_Tasks.h"
#include "VCF_Globals.h"
#include "ProtobufMsgInterface.h"
#include "hytech_msgs.pb.h"
#include "etl/optional.h"
#include "VCFEthernetInterface.h"



bool init_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    adc_1.setChannelScaleAndOffset(STEERING_1_CHANNEL, STEERING_1_SCALE, STEERING_1_OFFSET);
    adc_1.setChannelScaleAndOffset(STEERING_2_CHANNEL, STEERING_2_SCALE, STEERING_2_OFFSET);
    adc_1.setChannelScaleAndOffset(FR_SUS_POT_CHANNEL, FR_SUS_POT_SCALE, FR_SUS_POT_OFFSET);
    adc_1.setChannelScaleAndOffset(FR_LOADCELL_CHANNEL, FR_LOADCELL_SCALE, FR_LOADCELL_OFFSET);
    adc_1.setChannelScaleAndOffset(FL_SUS_POT_CHANNEL, FL_SUS_POT_SCALE, FL_SUS_POT_OFFSET);
    adc_1.setChannelScaleAndOffset(FL_LOADCELL_CHANNEL, FL_LOADCELL_SCALE, FL_LOADCELL_OFFSET);
    return true;
}
bool run_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    adc_1.sample(); // Samples all eight channels.
    adc_1.convert(); // Converts all eight channels.

    vcf_data.interface_data.steering_data.analog_steering_degrees = adc_1.data.conversions[STEERING_1_CHANNEL].conversion; // Only using steering 1 for now
    vcf_data.interface_data.front_loadcell_data.FL_loadcell_analog = adc_1.data.conversions[FL_LOADCELL_CHANNEL].conversion;
    vcf_data.interface_data.front_loadcell_data.FR_loadcell_analog = adc_1.data.conversions[FR_LOADCELL_CHANNEL].conversion;
    vcf_data.interface_data.front_suspot_data.FL_sus_pot_analog = adc_1.data.conversions[FL_SUS_POT_CHANNEL].raw; // Just use raw for suspots
    vcf_data.interface_data.front_suspot_data.FR_sus_pot_analog = adc_1.data.conversions[FR_SUS_POT_CHANNEL].raw; // Just use raw for suspots

    return true;
}
HT_TASK::Task read_adc1_task = HT_TASK::Task(init_read_adc1_task, run_read_adc1_task, 10, 1000UL); // 1000us is 1kHz //NOLINT



bool init_read_adc2_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    adc_2.setChannelScaleAndOffset(ACCEL_1_CHANNEL, ACCEL_1_SCALE, ACCEL_1_OFFSET);
    adc_2.setChannelScaleAndOffset(ACCEL_2_CHANNEL, ACCEL_2_SCALE, ACCEL_2_OFFSET);
    adc_2.setChannelScaleAndOffset(BRAKE_1_CHANNEL, BRAKE_1_SCALE, BRAKE_1_OFFSET);
    adc_2.setChannelScaleAndOffset(BRAKE_2_CHANNEL, BRAKE_2_SCALE, BRAKE_2_OFFSET);

    return true;
}
bool run_read_adc2_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    adc_2.sample(); // Samples all eight channels.
    adc_2.convert(); // Converts all eight channels.

    vcf_data.interface_data.pedal_sensor_data.accel_1 = adc_2.data.conversions[ACCEL_1_CHANNEL].conversion;
    vcf_data.interface_data.pedal_sensor_data.accel_2 = adc_2.data.conversions[ACCEL_2_CHANNEL].conversion;
    vcf_data.interface_data.pedal_sensor_data.brake_1 = adc_2.data.conversions[BRAKE_1_CHANNEL].conversion;
    vcf_data.interface_data.pedal_sensor_data.brake_2 = adc_2.data.conversions[BRAKE_2_CHANNEL].conversion;

    return true;
}
HT_TASK::Task read_adc2_task = HT_TASK::Task(init_read_adc2_task, run_read_adc2_task, 10, 1000UL); // 1000us is 1kHz //NOLINT

bool init_buzzer_control_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    pinMode(BUZZER_CONTROL_PIN, OUTPUT);

    return true;
}
bool run_buzzer_control_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    digitalWrite(BUZZER_CONTROL_PIN, vcf_data.system_data.buzzer_is_active);
    
    return true;
}
HT_TASK::Task buzzer_control_task = HT_TASK::Task(init_buzzer_control_task, run_buzzer_control_task, 10, 1000UL); // 1000us is 1kHz //NOLINT

bool run_send_vcf_data_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{

    vcf_data.system_data.pedals_system_data.accel_percent = (taskInfo.executions % 100) / 100.0f;

    hytech_msgs_VCFData_s protoc_struct = VCFEthernetInterface::make_vcf_data_msg(vcf_data);

    handle_ethernet_socket_send_pb<hytech_msgs_VCFData_s, hytech_msgs_VCFData_s_size>(debug_ip, VCF_SEND_PORT, &protobuf_send_socket, protoc_struct, &hytech_msgs_VCFData_s_msg);

    return true;
}
HT_TASK::Task send_vcf_data_task = HT_TASK::Task(HT_TASK::DUMMY_FUNCTION, run_send_vcf_data_task, 11, 1000000UL); // 1 000 000 us is 1Hz //NOLINT

bool run_recv_vcr_data_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    etl::optional<hytech_msgs_VCRData_s> protoc_struct = handle_ethernet_socket_receive<hytech_msgs_VCRData_s_size, hytech_msgs_VCRData_s>(&protobuf_recv_socket, &hytech_msgs_VCRData_s_msg);
    if (protoc_struct)
    {
        Serial.println("Received protobuf message!");
        
    }

    return true;
}
HT_TASK::Task recv_vcr_data_task = HT_TASK::Task(HT_TASK::DUMMY_FUNCTION, run_recv_vcr_data_task, 11, 1000UL); // 1 000 us is 1kHz //NOLINT

