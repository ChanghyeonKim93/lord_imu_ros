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

#include "util/ros_print_in_color.h"

#define REPLY_LENGTH 4
#define GRAVITY_MAGNITUDE 9.81

class IMU_3DM_GX3_25{
public:
    IMU_3DM_GX3_25(ros::NodeHandle& nh);
    ~IMU_3DM_GX3_25();

private:
    void stream();

private:
    bool validateChecksum(const unsigned char* data, unsigned short len);
    void openSerialPort(const std::string& portname, const int& baudrate);

    void setSamplingRate(uint32_t freq);
    void setBaudRate(uint32_t baudrate);

// Related boost::asio::serial
private:
    boost::asio::serial_port* serial_;

    boost::asio::io_service io_service;
    boost::asio::deadline_timer timeout;

    std::string portname_;
    int baudrate_;
    int imu_rate_;

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

    ros::Publisher pub_mag_;
    sensor_msgs::MagneticField msg_mag_;
    std::string topicname_mag_;

    union FLOAT_UNION{
        float float_;
        unsigned char uchar_[4];
    };
    union INT_UNION{
        int32_t int_;
        unsigned char uchar_[4];
    };
    union UINT_UNION{
        uint32_t uint_;
        unsigned char uchar_[4];
    };

    FLOAT_UNION acc_[3];
    FLOAT_UNION gyro_[3];
    FLOAT_UNION mag_[3];
    FLOAT_UNION R_[9];
    INT_UNION   time_;
};

#endif