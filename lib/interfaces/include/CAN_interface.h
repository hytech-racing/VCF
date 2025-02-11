#ifndef __VCFCANINTERFACEIMPL_H__
#define __VCFCANINTERFACEIMPL_H__

#include <cstdint>


#include <stdint.h>
#include "FlexCAN_T4.h"
#include "HyTech_CAN.h"
#include "hytech.h"
#include "MessageQueueDefine.h"
#include "PedalsSystem.h"
#include "SharedDataTypes.h"
#include "SharedFirmwareTypes.h"
#include "InverterInterface.h"
//#include "VCFInterface.h"


using CANRXBufferType = Circular_Buffer<uint8_t, (uint32_t)16, sizeof(CAN_message_t)>;
using CANTXBufferType = Circular_Buffer<uint8_t, (uint32_t)128, sizeof(CAN_message_t)>;

/* RX buffers for CAN extern declarations*/
extern CANRXBufferType CAN1_rxBuffer;
extern CANRXBufferType CAN2_rxBuffer;
extern CANRXBufferType CAN3_rxBuffer;

/* TX buffer for CAN1 */
extern CANTXBufferType CAN1_txBuffer;
/* TX buffer for CAN2 */
extern CANTXBufferType CAN2_txBuffer;
/* TX buffer for CAN3 */
extern CANTXBufferType CAN3_txBuffer;

void on_can1_receive(const CAN_message_t &msg);
void on_can2_receive(const CAN_message_t &msg);
void on_can3_receive(const CAN_message_t &msg);

//Control Word? Got to put a Tick method here
//This Struct is used for all periodic ticker signals - not sure if this is correct
// Also - are these control messages already implemented with AMKs?
struct ticker
{
    //Control Word - sent every 50ms, minimum expectation to be sent every 75ms
        bool inverter_enable;
        bool hv_enable;
        bool driver_enable;
        bool remove_error;

    //Control Input Message.
    //will be attempted to be sent every 5ms, monitored by the inverter to be expected at least every 20ms
        int16_t speed_setpoint;
        int16_t positive_torque_limit;
        int16_t negative_torque_limit;
        int16_t torque_setpoint;

    //Control Parameter Message: 
    //will be sent intermitently, not monitored by the inverter to be expected at any regular period
        uint16_t speed_control_kp;
        uint16_t speed_control_ki;
        uint16_t speed_control_kd;
};

// This method is meant to be a timer with the above data
// takes in a reference to the struct above, where
void tick (const ticker & AMK_tick_params);


struct inverter_messages
{
    //INVERTER MESSAGES:
    bool system_ready;
    bool error;
    bool warning;
    bool quit_dc_on;
    bool dc_on;
    bool quit_inverter_on;
    bool inverter_on;
    bool derating_on;
    uint16_t dc_bus_voltage;
    uint16_t diagnostic_number;

    //inverter temps - sent periodically every 20 ms

    int16_t motor_temp;
    int16_t inverter_temp;
    int16_t igbt_temp;
    uint32_t actual_power_w;
    int16_t actual_torque_nm;
    int16_t actual_speed_rpm;

    //Inverter electrical power data
    int32_t active_power_w;
    int32_t reactive_power_var;

    //inverter parameter feedback
    uint16_t speed_control_kp;
    uint16_t speed_control_ki;
    uint16_t speed_control_kd;

};

void read_inverter_messages(const inverter_messages & messages_);

//


struct CANInterfaces
{
    explicit CANInterfaces(VCFInterfaceData_s & vcf_int): vcf_interface(vcf_int) {}

    // pedal_data CAN Packet - this data is a pedalsystemdatas struct
    /*
    bool accel_implausible;
    bool brake_implausible;
    bool brake_pressed;
    bool accel_pressed;
    bool mech_brake_active;
    bool brake_and_accel_pressed_implausibility;
    bool implausibility_exceeded_duration;
    //implausibility prsenet for too long - 200 or 180 ms

    //data - this includes brake and accel 32 - u16b
    uint16_t brake;
    uint16_t accel;
    */
    void update_vcf_status_CAN_pedals(const PedalsSystemData_s & pedals);
        //start engaging:

    // ANALOG/DIGITAL steering sensor values
    void update_vcf_status_CAN_steering(const VCFInterfaceData_s & steering);
    //Fl/fr load cell and shockpot values. 
    void update_vcf_status_CAN_suspension(const VCFInterfaceData_s & suspension);
};

void enqueue_CAN_vcf_status();
struct VCFOutputs{


    //UserInputs:
    // -requested drive mode (0-5)
    // requesting drivebrain/VCR in control mode
    // requested torque limit mode
    // requesting drivetrain error reset. 
    


    //STATUS:
    bool buzzerstatus;
    
    //FIRMWARE VERSION: 
    bool dirty; 
};

#endif // __VCRCANINTERFACEIMPL_H__