#include "imu_3dm_gx3_25.h"


IMU_3DM_GX3_25::IMU_3DM_GX3_25(ros::NodeHandle& nh)
: nh_(nh), stop({'\xFA','\x75','\xB4'}), mode({'\xD4','\xA3','\x47','\x00'}){

    ROS_INFO_STREAM("IMU_3DM_GX3_25 - starts");

    topicname_imu_ = "/microstarain/imu";
    portname_      = "/dev/ttyACM0";
    baudrate_      = 115200;

    // Open the serial port
    this->openSerialPort(portname_, baudrate_);

    // Ros publisher
    pub_imu_ = nh_.advertise<sensor_msgs::Imu>(topicname_imu_, 10);

    // Steram
    this->stream();            
};

IMU_3DM_GX3_25::~IMU_3DM_GX3_25(){

    // Stop continous and close device
    boost::asio::write(*serial_, boost::asio::buffer(stop, 3));
    ROS_WARN("Wait 0.5s"); 
    ros::Duration(0.5).sleep();
    serial_->close(); 

    ROS_INFO_STREAM("IMU_3DM_GX3_25 - terminates.");
};


void IMU_3DM_GX3_25::openSerialPort(const std::string& portname, const int& baudrate){

    // Generate serial port.
    serial_ = nullptr;
    boost::asio::io_service io_service;
    serial_ = new boost::asio::serial_port(io_service);

    // Try to open the serial port
    try{
      serial_->open(portname);
    }
    catch (boost::system::system_error& error){
        std::cout << "Port [" << portname.c_str() << "] is not opened. Error message:" << error.what() << std::endl;
        throw std::runtime_error(error.what());
    }

    // If serial port cannot be opened, 
    if ( !serial_->is_open() ) {
      std::cout << "[" << portname <<"] is not opened. terminate the node\n";
      throw std::runtime_error("");
    }

    // Set serial port spec.
    boost::asio::serial_port_base::baud_rate baud_rate_option(baudrate);
    boost::asio::serial_port_base::flow_control flow_control(boost::asio::serial_port_base::flow_control::none);
    boost::asio::serial_port_base::parity parity(boost::asio::serial_port_base::parity::none);
    boost::asio::serial_port_base::stop_bits stop_bits(boost::asio::serial_port_base::stop_bits::one);

    serial_->set_option(baud_rate_option);
    serial_->set_option(flow_control);
    serial_->set_option(parity);
    serial_->set_option(stop_bits);


    // From this, we set the IMU settings
    bool flag_retry = false;

    // Check the mode
    boost::asio::write(*serial_, boost::asio::buffer(mode, 4));
    boost::asio::read(*serial_, boost::asio::buffer(reply, REPLY_LENGTH));
    if (!validateChecksum(reply, REPLY_LENGTH) ) {
        ROS_ERROR("%s: failed to get mode", portname.c_str());
        if (serial_->is_open()) serial_->close();

        flag_retry = true;
    }
    else {

    }

    // If we are not in active mode, change it
    if (reply[2] != '\x01') {
        mode[3] = '\x01';
        boost::asio::write(*serial_, boost::asio::buffer(mode, 4));
        boost::asio::read(*serial_, boost::asio::buffer(reply, REPLY_LENGTH));
        if (!validateChecksum(reply, REPLY_LENGTH)) {
            ROS_ERROR("%s: failed to set mode to active", portname.c_str());
            if (serial_->is_open()) serial_->close();
            throw std::runtime_error("failed to set active mode.");
        }
    }

    // Set the continous preset mode
    const char preset[4] = {'\xD6','\xC6','\x6B','\xCC'};
    boost::asio::write(*serial_, boost::asio::buffer(preset, 4));
    boost::asio::read(*serial_,  boost::asio::buffer(reply, REPLY_LENGTH));
    if (!validateChecksum(reply, REPLY_LENGTH)) {
        ROS_ERROR("%s: failed to set continuous mode preset", portname.c_str());
        if (serial_->is_open()) serial_->close();
        throw std::runtime_error("failed to set continuous mode preset.");
    }

    // Set the mode to continous output
    mode[3] = '\x02';
    boost::asio::write(*serial_, boost::asio::buffer(mode, 4));
    boost::asio::read( *serial_, boost::asio::buffer(reply, REPLY_LENGTH));
    if (!validateChecksum(reply, REPLY_LENGTH)) {
        ROS_ERROR("%s: failed to set continuous output", portname.c_str());
        if (serial_->is_open()) serial_->close();
        throw std::runtime_error("failed to set continuous output");
    }

    // Set Timer
    char set_timer[8] = {'\xD7','\xC1','\x29','\x01','\x00','\x00','\x00','\x00'};
    unsigned char reply_timer[7];
    boost::asio::write(*serial_, boost::asio::buffer(set_timer, 8));
    boost::asio::read(*serial_, boost::asio::buffer(reply_timer, 7));

    ROS_INFO_STREAM("Serial port is set.");
};  


void IMU_3DM_GX3_25::stream(){

    ROS_WARN("Streaming Data...");
    unsigned short data_length = 79;
    unsigned char  data[data_length];

    ros::Time t0 = ros::Time::now();  

    while (ros::ok()) {
        int len_read = serial_->read_some(boost::asio::buffer(data, data_length));
        if (len_read == -1) {
            if (errno == EINTR) continue;
        }
        std::cout << "len read : " << len_read << std::endl;
        if (!validateChecksum(data, data_length)) {
            ROS_ERROR("%s: checksum failed on message", portname_.c_str());
            continue;
        }



        unsigned int k = 1;
        float acc[3];
        float ang_vel[3];
        float mag[3];
        float M[9];
        double T;
        for (uint8_t i = 0; i < 3; i++, k += 4) acc[i]     = extractFloat(&(data[k]));
        for (uint8_t i = 0; i < 3; i++, k += 4) ang_vel[i] = extractFloat(&(data[k]));
        for (uint8_t i = 0; i < 3; i++, k += 4) mag[i]     = extractFloat(&(data[k]));
        for (uint8_t i = 0; i < 9; i++, k += 4) M[i]       = extractFloat(&(data[k]));
        T = extractInt(&(data[k])) / 62500.0;

        std::cout << " ACC: ";
        for(int i = 0; i < 3; ++i) std::cout << acc[i] <<" ";
        std::cout << std::endl;
        std::cout << "GYRO: ";
        for(int i = 0; i < 3; ++i) std::cout << ang_vel[i] <<" ";
        std::cout << std::endl;

        double delay = 0.0;
        msg_imu_.header.stamp    = t0 + ros::Duration(T) - ros::Duration(delay);
        msg_imu_.header.frame_id = "body";
        msg_imu_.angular_velocity.x    = ang_vel[0];
        msg_imu_.angular_velocity.y    = ang_vel[1];
        msg_imu_.angular_velocity.z    = ang_vel[2];
        msg_imu_.linear_acceleration.x = acc[0] * 9.81;
        msg_imu_.linear_acceleration.y = acc[1] * 9.81;
        msg_imu_.linear_acceleration.z = acc[2] * 9.81;


        Eigen::Matrix3d R;
        for (unsigned int i = 0; i < 3; i++)
            for (unsigned int j = 0; j < 3; j++)
            R(i,j) = M[j*3+i];
            
        Eigen::Quaternion<double> q(R);
        msg_imu_.orientation.w = (double)q.w();// q(0);
        msg_imu_.orientation.x = (double)q.x();// q(1);
        msg_imu_.orientation.y = (double)q.y();// q(2);
        msg_imu_.orientation.z = (double)q.z();// q(3);
        msg_imu_.orientation_covariance[0] = mag[0];
        msg_imu_.orientation_covariance[1] = mag[1];
        msg_imu_.orientation_covariance[2] = mag[2];
        pub_imu_.publish(msg_imu_);
    }
};




bool IMU_3DM_GX3_25::validateChecksum(const unsigned char* data, unsigned short len)
{
    unsigned short checksum_calc = 0;
    unsigned short checksum_recv = 0;

    //-------- Calculate the checksum
    for (unsigned short i = 0; i < len - 2; i++)
        checksum_calc += data[i];

    //-------- Extract the big-endian checksum from reply
    checksum_recv = data[len - 2] << 8;
    checksum_recv += data[len - 1];

    //-------- Compare the checksums
    return (checksum_calc == checksum_recv);
}

float IMU_3DM_GX3_25::extractFloat(unsigned char* addr) {
  float tmp;

  *((unsigned char*)(&tmp) + 3) = *(addr);
  *((unsigned char*)(&tmp) + 2) = *(addr+1);
  *((unsigned char*)(&tmp) + 1) = *(addr+2);
  *((unsigned char*)(&tmp)) = *(addr+3);

  return tmp;
}

int IMU_3DM_GX3_25::extractInt(unsigned char* addr) {
  int tmp;

  *((unsigned char*)(&tmp) + 3) = *(addr);
  *((unsigned char*)(&tmp) + 2) = *(addr+1);
  *((unsigned char*)(&tmp) + 1) = *(addr+2);
  *((unsigned char*)(&tmp)) = *(addr+3);

  return tmp;
}
