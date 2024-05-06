#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <queue>
#include "sc_serial.h"
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>

using namespace std;
using namespace ros;

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

#define EncoderPerMeter 1 /* pi */
#define KP 1              //0.85
#define KI 0              //0.6409
#define KD 0
#define STEER_KP 1
#define iteration_time 0.02
#define M_PI 3.14159265358979323846

int joy_break = 0;
int enc_flag = 0;
int pre_enc;
float enc_based_vel;

float error_prior = 0;
double cumulative_errors = 0;
float derivative;
int pid_out = 0;
int steering_out = 0;
short break_value;
short gear_value;
ros::Time current_time;
ros::Time prev_time;
ros::Time cmd_vel_time;
ros::Time cal_time;
float prev_angular = 0;

geometry_msgs::Twist cmd_vel;

void cmdCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    float angular_to_degree;
    cmd_vel.linear.x = msg->linear.x;
    cmd_vel.angular.z = msg->angular.z / M_PI * 180 * 71;
}

void CalculateVelocity(std_msgs::Int32 Rx_Enc)
{
    float enc = 0;
    // float steer=0;

    if (enc_flag == 0)
    {
        pre_enc = Rx_Enc.data;
        enc_flag = 1;
    }
    else
    {

        if (pre_enc - Rx_Enc.data > -100 && pre_enc - Rx_Enc.data < 100)
        {
            enc = pre_enc - Rx_Enc.data;
            pre_enc = Rx_Enc.data;
        }
    }

    enc_based_vel = enc * EncoderPerMeter * 50; // enc : 50hz     ,   m/s
}

void LinarVelocityPid(float desired_value, float current_vel)
{
    if (desired_value > 0) //forward
    {
        gear_value = 0;
        if ((current_vel + 20) < desired_value) //accel
        {
            pid_out = 200;
            break_value = 1;
        }
        else if ((current_vel - 20) > desired_value)
        {
            int tmp_break = current_vel - desired_value; // 200  ~ 20

            pid_out = 0;
            break_value = tmp_break /6.06; //TODO 필요함.1.25; 
        }
        else
        {
            pid_out = desired_value;
            break_value = 1;
        }
    }
    else //backward
    {
        pid_out = -desired_value;
        break_value = 1;
        gear_value = 2;
    }

    //deaccel
    if (desired_value == 0)
    {

        if (current_vel > 100)
            break_value = 16; //100
        else
            break_value = 33; //200
    }
}

void SteeringPid(float desired_angle)
{

    float steer_error;

    steer_error = desired_angle;
    steering_out = STEER_KP * steer_error; //+ STEER_KI*steer_integral;

    if (steering_out >= 2000)
        steering_out = 2000;
    if (steering_out <= -2000)
        steering_out = -2000;
}

void joyCallback(const sensor_msgs::Joy joyMsg)
{

    if (joyMsg.buttons.at(3) == 1)
    {
        break_value = 200;
        joy_break = 1;
    }

    if (joyMsg.buttons.at(5) == 1)
    {

        break_value = 1;
        joy_break = 0;
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Controller_data");
    ros::NodeHandle node_;
    ros::NodeHandle pnh("~");

    ros::Subscriber joy_subs = node_.subscribe("/joy", 1, joyCallback);
    ros::Subscriber cmd_subs = node_.subscribe("/cmd_vel", 1, cmdCallback);

    std::string port_name;
    int baud_rate;

    pnh.param<std::string>("port_name", port_name, "/dev/ttyUSB0");
    pnh.param<int>("baud_rate", baud_rate, 115200);

    SC_Serial SCS(port_name, baud_rate);

    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;

    //init
    enc_flag = 0;
    error_prior = 0;
    cumulative_errors = 0;
    break_value = 1;
    gear_value = 0;

    current_time = ros::Time::now();
    prev_time = ros::Time::now();

    ros::Rate r(70.0); //50hz
    while (node_.ok())
    {
        current_time = ros::Time::now();
        SCS.SerialProcessing(pid_out, steering_out, break_value, gear_value); // send data
        CalculateVelocity(SCS.Rx_Enc);

        LinarVelocityPid(cmd_vel.linear.x, SCS.Rx_Vel.data); // enc_based_vel
        SteeringPid(cmd_vel.angular.z);

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
