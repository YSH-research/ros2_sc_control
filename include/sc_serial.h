#include <string.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
typedef char BYTE;

class SC_Serial
{
     public:
        SC_Serial(std::string port_name, int baudrate);
        void SerialProcessing(short Tx_Vel, short Tx_Steer, short Tx_break, short Tx_gear);
        short Tx_Vel;
        short Tx_Steer;
        short Tx_gear;
        short Tx_break;
        std_msgs::String Rx_AorM;
        std_msgs::String Rx_Estop;
        std_msgs::String Rx_Gear;
        std_msgs::Int8 Rx_Break;
        std_msgs::Int16 Rx_Vel;
        std_msgs::Int16 Rx_Steer;
        std_msgs::Int32 Rx_Enc;


    private:
        void SendData(void);
        void ReceiveData(std::string str);
        BYTE input[14];
        int HexStringToDec(char A, char B);

        ros::NodeHandle node_;
        ros::Publisher pubAorM;
        ros::Publisher pubEstop;
        ros::Publisher pubGear;
        ros::Publisher pubVel;
        ros::Publisher pubSteer;
        ros::Publisher pubBreak;
        ros::Publisher pubEnc;
};

