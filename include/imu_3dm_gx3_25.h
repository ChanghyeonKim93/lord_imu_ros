#ifndef _IMU_3DM_GX3_25_H_
#define _IMU_3DM_GX3_25_H_
#include <iostream>
#include <string>
#include <exception>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>

#include <Eigen/Dense>

#define REPLY_LENGTH 4

class IMU_3DM_GX3_25{
public:
    IMU_3DM_GX3_25(ros::NodeHandle& nh);
    ~IMU_3DM_GX3_25();

private:
    void stream();

private:
    bool validateChecksum(const unsigned char* data, unsigned short len);
    void openSerialPort(const std::string& portname, const int& baudrate);

    float extractFloat(unsigned char* addr);
    int extractInt(unsigned char* addr);


// Related boost::asio::serial
private:
    boost::asio::serial_port* serial_;
    std::string portname_;
    int baudrate_;

private:
    char stop[3];
    char mode[4];
    unsigned char reply[REPLY_LENGTH];

// ROS related 
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_imu_;
    sensor_msgs::Imu msg_imu_;
    std::string topicname_imu_;

};

#endif