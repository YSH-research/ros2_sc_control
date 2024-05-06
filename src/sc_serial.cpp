#include "sc_serial.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <bitset>

using namespace std;
serial::Serial ser;

int alive = 0;

bool valid()
{
    if (ser.isOpen())
        return true;
    else
        return false;
}

bool connect(std::string port_name, int baudrate)
{
    if (valid())
        return false;
    try
    {
        ser.setPort(port_name);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
        ROS_INFO("open port ");
        return true;
    }
    catch (serial::IOException &e)
    {
        ROS_INFO("CAN'T OPEN PORT ");
        ros::shutdown();
        return false;
    }
}

SC_Serial::SC_Serial(std::string port_name, int baudrate)
{
    connect(port_name, baudrate);

    // SerialProcessing(0, 0,1,0);
    Tx_Vel = 0;
    Tx_Steer = 0;
    Tx_break = 1;
    Tx_gear = 0;

    pubAorM = node_.advertise<std_msgs::String>("/MSG_CON/Rx_AorM", 1);
    pubEstop = node_.advertise<std_msgs::String>("/MSG_CON/Rx_Estop", 1);
    pubGear = node_.advertise<std_msgs::String>("/MSG_CON/Rx_Gear", 1);
    pubBreak = node_.advertise<std_msgs::Int8>("/MSG_CON/Rx_Break", 1);
    pubVel = node_.advertise<std_msgs::Int16>("/MSG_CON/Rx_Vel", 1);
    pubSteer = node_.advertise<std_msgs::Int16>("/MSG_CON/Rx_Steer", 1);
    pubEnc = node_.advertise<std_msgs::Int32>("/MSG_CON/Rx_Enc", 1);

    Rx_Vel.data = 0;
    Rx_Steer.data = 0;
    Rx_Enc.data = 0;
}

void SC_Serial::SendData(void)
{
    // STX
    input[0] = 0x53;
    input[1] = 0x54;
    input[2] = 0x58;

    // Data
    input[3] = 0x01; // A or M
    input[4] = 0x00; // E-Stop

    input[5] = this->Tx_gear & 0xFF; //0x0;      // Gear : 0~2

    input[6] = (this->Tx_Vel >> 8) & 0xFF; // Speed0
    input[7] = this->Tx_Vel & 0xFF;        // Speed1

    input[8] = (this->Tx_Steer >> 8) & 0xFF;
    input[9] = this->Tx_Steer & 0xFF;

    input[10] = this->Tx_break & 0xFF; //0x01;     // Break : 1 - 33
    //std::cout << "Tx_break : " << Tx_break << "  input[10]: " << (short)input[10] << std::endl;

    input[11] = (BYTE)alive; // Alive

    input[12] = 0x0D; // ETX0
    input[13] = 0x0A; // ETX1

    string sersend = "";

    //cout << "Send Data" << endl;
    for (int i = 0; i < 14; i++)
    {
        sersend += input[i];
        //cout << hex << "Tx[" << i << "] : " << (int)input[i] << endl;
    }
    ser.write(sersend);

    alive++;
    if (alive % 255 == 0)
        alive = 0;
}

void SC_Serial::ReceiveData(std::string str)
{
    if (str[0] == 'S')
    {
        uint8_t AorM = (uint8_t)str.c_str()[3];
        ROS_INFO("ERP MODE (M/A) : %d, %X(HEX)", AorM, AorM);
        uint8_t estop = (uint8_t)str.c_str()[4];
        ROS_INFO("ERP ESTOP (OFF/ON) : %d, %X(HEX)", estop ,estop);
        uint8_t gear = (uint8_t)str.c_str()[5];
        ROS_INFO("ERP GEAR (F/N/B) : %d %X(HEX)", gear, gear);

        uint16_t vel = (uint8_t)str.c_str()[6] | (uint8_t)str.c_str()[7] << 8;
        ROS_INFO("ERP VEL (km/h * 10) : %d %X %X(HEX)", vel, (uint8_t)str.c_str()[6], (uint8_t)str.c_str()[7]);

        int16_t steer = (uint8_t)str.c_str()[8] | (uint8_t)str.c_str()[9] << 8;
        ROS_INFO("ERP STEER : %d %X %X(HEX)", steer, (uint8_t)str.c_str()[8], (uint8_t)str.c_str()[9]);

        uint8_t brk = (uint8_t)str.c_str()[10];
        ROS_INFO("ERP BREAK : %d, %X(HEX)", brk, brk);

        int32_t enc = (uint8_t)str.c_str()[11] | (uint8_t)str.c_str()[12] << 8 |
                       (uint8_t)str.c_str()[13] << 16 | (uint8_t)str.c_str()[14] << 24;
        ROS_INFO("ERP ENC : %d, %X %X %X %X(HEX)", enc,
                 (uint8_t)str.c_str()[11], (uint8_t)str.c_str()[12], (uint8_t)str.c_str()[13], (uint8_t)str.c_str()[14]);

        Rx_AorM.data = AorM;
        Rx_Estop.data = estop;
        Rx_Gear.data = gear;
        Rx_Vel.data = vel;
        Rx_Steer.data = -steer;
        Rx_Break.data = brk;
        Rx_Enc.data = enc;

        pubAorM.publish(Rx_AorM);
        pubEstop.publish(Rx_Estop);
        pubGear.publish(Rx_Gear);
        pubVel.publish(Rx_Vel);
        pubSteer.publish(Rx_Steer);
        pubBreak.publish(Rx_Break);
        pubEnc.publish(Rx_Enc);
    }
}

void SC_Serial::SerialProcessing(short Tx_Vel, short Tx_Steer, short Tx_break, short Tx_gear)
{
    string result1;
    //    ros::spinOnce();

    if (ser.available())
    {
        this->Tx_Vel = Tx_Vel;
        this->Tx_Steer = Tx_Steer;
        this->Tx_break = Tx_break;
        this->Tx_gear = Tx_gear;
        SendData();
        //  cout << "process this " << this->Tx_break << "input break" << Tx_break <<  endl;

        ser.readline(result1);
        ReceiveData(result1);
    } //else
      // ROS_INFO("NO SERIAL DATA ");
}
int SC_Serial::HexStringToDec(char A, char B)
{
    //16진수로 된 char 배열 값을 정수로 변경하여 반환한다
    cout << "A:" << A << " B: " << B << endl;
    int sum = 0;
    int temp = 0;

    sum = sum * 16 + (A & 0xf0U) >> 4;
    sum = sum * 16 + (A & 0x0fU);

    sum = sum * 16 + (B & 0xf0U) >> 4;
    sum = sum * 16 + (B & 0x0fU);

    return sum;
}
