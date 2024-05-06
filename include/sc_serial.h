#include <string.h>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/Int16.hpp>
#include <std_msgs/msg/Int8.hpp>
#include <std_msgs/msg/Int32.hpp>
#include <std_msgs/msg/string.hpp>
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

        rclcpp::NodeHandle node_;
        rclcpp::Publisher pubAorM;
        rclcpp::Publisher pubEstop;
        rclcpp::Publisher pubGear;
        rclcpp::Publisher pubVel;
        rclcpp::Publisher pubSteer;
        rclcpp::Publisher pubBreak;
        rclcpp::Publisher pubEnc;
};

