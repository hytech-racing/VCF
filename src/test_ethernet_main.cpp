/*

#include <Arduino.h>
#include <cstdint>
#include <QNEthernet.h>

#include <array>
#include <cstring>

// #include "ProtobufMsgInterface.h"
// #include "MCU_rev15_defs.h"
using namespace qindesign::network;
EthernetUDP send_socket; // SENDING 6969.2
EthernetUDP recv_socket; // recv 6969.4
// ParameterInterface params = ParameterInterface();
// ETHInterfaces ethernet_interfaces = {&params};

const IPAddress default_VCF_ip(192, 168, 1, 30);
const IPAddress default_PC_ip(192, 168, 1, 31);
const IPAddress default_dns(192, 168, 1, 1);
const IPAddress default_gateway(192, 168, 1, 1);
const IPAddress car_subnet(255, 255, 255, 0);

// uint8_t default_MCU_MAC_address[6] = 
//     {0x04, 0xe9, 0xe5, 0x10, 0x1f, 0x22};

void init_ethernet_device()
{
    Ethernet.begin(default_VCF_ip, default_dns, default_gateway, car_subnet);
    send_socket.begin(4444);
    recv_socket.begin(5555);
}

void test_ethernet()
{
    int packet_size = recv_socket.parsePacket();
    if (packet_size > 0)
    {
        float read_float;
        // uint8_t buffer[4];
        std::array<uint8_t, 4> buffer;
        recv_socket.read(buffer.data(), sizeof(float));

        // read_float = buffer[0] | buffer[1] 
        memcpy(&read_float, buffer.data(), sizeof(float));
        Serial.print("got float: ");
        Serial.println(read_float);
    }
    float send_float = 6969.2;
    std::array<uint8_t, 4> data;
    memcpy(data.data(), &send_float, sizeof(float));
    send_socket.send(default_PC_ip, 4444, data.data(), sizeof(float));
}

void setup()
{
    init_ethernet_device();
}

void loop()
{
    test_ethernet();
    Serial.println("loopin");
}
*/

#include <Arduino.h>
#include <cstdint>
#include <QNEthernet.h>

#include <array>
#include <cstring>
//
using namespace qindesign::network;
EthernetUDP socket; 
//EthernetUDP recv_socket; 
// ParameterInterface params = ParameterInterface();
// ETHInterfaces ethernet_interfaces = {&params};

const IPAddress default_VCR_ip(192, 168, 1, 30);
//const IPAddress default_PC_ip(192, 168, 1, 31);
const IPAddress multicast_ip1(234, 2, 1, 1);      // First multicast group
const IPAddress multicast_ip2(239, 1, 1, 2);      // Second multicast group
const IPAddress default_dns(192, 168, 1, 1);
const IPAddress default_gateway(192, 168, 1, 1);
const IPAddress car_subnet(255, 255, 0, 0);
uint16_t my_port = 62000; 

// uint8_t default_MCU_MAC_address[6] = 
//     {0x04, 0xe9, 0xe5, 0x10, 0x1f, 0x22};

void init_ethernet_device()
{
    while(!Serial) {}
    Ethernet.begin(default_VCR_ip, default_gateway, car_subnet);

    //for receiving
    socket.begin(my_port); //binds socket to specific port
    
    if(Ethernet.joinGroup(multicast_ip1))
    {
        Serial.println("yo");
    } else {
        Serial.println("fucl");
    }
    // Ethernet.joinGroup(multicast_ip2);

    //socket.beginMulticast(default_VCR_ip, my_port);
    



    //can also call this several times for each ip address beginMulticast(ip, port)
    //send_socket.begin(4444);
    //recv_socket.begin(5555);
}

void test_ethernet()
{
    /*
    int packet_size = recv_socket.parsePacket();
    if (packet_size > 0)
    {
        float read_float;
        // uint8_t buffer[4];
        std::array<uint8_t, 4> buffer;
        recv_socket.read(buffer.data(), sizeof(float));

        // read_float = buffer[0] | buffer[1] 
        memcpy(&read_float, buffer.data(), sizeof(float));
        Serial.print("got float: ");
        Serial.println(read_float);
    }
    float send_float = 6969.2;
    std::array<uint8_t, 4> data;
    memcpy(data.data(), &send_float, sizeof(float));
    send_socket.send(default_PC_ip, 4444, data.data(), sizeof(float));
    */

   //will receive any messages sent from another device to the given IP addresses

   int packet_size = socket.parsePacket();
   if (packet_size > 0) {
        float read_float;
        std::array<uint8_t, 4> buffer;
        //IPAddress sender_ip = socket.remoteIP();
        socket.read(buffer.data(), sizeof(float));
        memcpy(&read_float, buffer.data(), sizeof(float));

        Serial.print("received float ");
        Serial.println(read_float);
   }
    float send_float = 42.42;
    std::array<uint8_t, 4> data;
    memcpy(data.data(), &send_float, sizeof(float));
    // socket.send(default_VCR_ip, 5555, data.data(), sizeof(float)); //ip can be anything
    //Serial.print("sent float ");
    //Serial.println(send_float);
}

void setup()
{
    init_ethernet_device();
}

void loop()
{
    test_ethernet();
     // Serial.println("loopin");
}