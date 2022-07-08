#include "imu_3dm_gx3_25.h"

IMU_3DM_GX3_25::IMU_3DM_GX3_25(ros::NodeHandle& nh)
: nh_(nh), stop({'\xFA','\x75','\xB4'}), mode({'\xD4','\xA3','\x47','\x00'}),
io_service(),timeout(io_service)
{

    ROS_INFO_STREAM("IMU_3DM_GX3_25 - starts");

    topicname_imu_ = "/lord_3dm_gx3_25/imu";
    topicname_mag_ = "/lord_3dm_gx3_25/mag";
    portname_      = "/dev/ttyACM0";
    baudrate_      = 115200;

    // Open the serial port
    this->openSerialPort(portname_, baudrate_);

    // Ros publisher
    pub_imu_ = nh_.advertise<sensor_msgs::Imu>(topicname_imu_, 1);
    pub_mag_ = nh_.advertise<sensor_msgs::MagneticField>(topicname_mag_, 1);

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


    // Serial baudrate 
    // boost::asio::serial_port_base::baud_rate baud_rate_option2(460800);
    // serial_->set_option(baud_rate_option2);
    ROS_INFO_STREAM("Serial port is set.");
};  


void IMU_3DM_GX3_25::stream(){

    ROS_WARN("Streaming Data...");

    // setting the serial ..
    UINT_UNION baud_rate_imu;
    baud_rate_imu.uint_ = 921600;
    char set_baudrate[11] = {'\xD9',
        '\xC3','\x55','\x01','\x01',
        baud_rate_imu.uchar_[3],baud_rate_imu.uchar_[2],baud_rate_imu.uchar_[1],baud_rate_imu.uchar_[0],
        '\x00','\x00'};
    unsigned char reply_baudrate[10];
    boost::asio::write(*serial_, boost::asio::buffer(set_baudrate, 11));
    boost::asio::read(*serial_, boost::asio::buffer(reply_baudrate, 10));

    baud_rate_imu.uchar_[3] = reply_baudrate[2];
    baud_rate_imu.uchar_[2] = reply_baudrate[3];
    baud_rate_imu.uchar_[1] = reply_baudrate[4];
    baud_rate_imu.uchar_[0] = reply_baudrate[5];

    std::cout << " baudrate response: " << baud_rate_imu.uint_ << std::endl;


    boost::asio::serial_port_base::baud_rate baud_rate_option2(921600);
    serial_->set_option(baud_rate_option2);
    boost::asio::serial_port_base::baud_rate baud_rate_option;
    serial_->get_option(baud_rate_option);
    std::cout << " baudrate set serialport :" << baud_rate_option.value() << std::endl;

    unsigned short data_length = 79;
    unsigned char  data[data_length];

    ros::Time t0 = ros::Time::now();  


    ros::Rate rate(2000);
    while (ros::ok()) {
        int len_read = -1;
        try {
            len_read = serial_->read_some(boost::asio::buffer(data, data_length));
        }
        catch (std::exception& e) {
            if (errno == EINTR) continue;
        }

        if (len_read == -1) continue;

        if (!validateChecksum(data, data_length)) {
            ROS_ERROR("%s: checksum failed on message", portname_.c_str());
            continue;
        }

        if(len_read > 0){
            if (!validateChecksum(data, data_length)) {
                ROS_ERROR("%s: checksum failed on message", portname_.c_str());
                continue;
            }

            // acc
            unsigned char* ptr = data;
            acc_[0].uchar_[3] = *(++ptr); acc_[0].uchar_[2] = *(++ptr); acc_[0].uchar_[1] = *(++ptr); acc_[0].uchar_[0] = *(++ptr);  
            acc_[1].uchar_[3] = *(++ptr); acc_[1].uchar_[2] = *(++ptr); acc_[1].uchar_[1] = *(++ptr); acc_[1].uchar_[0] = *(++ptr);  
            acc_[2].uchar_[3] = *(++ptr); acc_[2].uchar_[2] = *(++ptr); acc_[2].uchar_[1] = *(++ptr); acc_[2].uchar_[0] = *(++ptr);  
            
            gyro_[0].uchar_[3] = *(++ptr); gyro_[0].uchar_[2] = *(++ptr); gyro_[0].uchar_[1] = *(++ptr); gyro_[0].uchar_[0] = *(++ptr);  
            gyro_[1].uchar_[3] = *(++ptr); gyro_[1].uchar_[2] = *(++ptr); gyro_[1].uchar_[1] = *(++ptr); gyro_[1].uchar_[0] = *(++ptr);  
            gyro_[2].uchar_[3] = *(++ptr); gyro_[2].uchar_[2] = *(++ptr); gyro_[2].uchar_[1] = *(++ptr); gyro_[2].uchar_[0] = *(++ptr);  
            
            mag_[0].uchar_[3] = *(++ptr); mag_[0].uchar_[2] = *(++ptr); mag_[0].uchar_[1] = *(++ptr); mag_[0].uchar_[0] = *(++ptr);  
            mag_[1].uchar_[3] = *(++ptr); mag_[1].uchar_[2] = *(++ptr); mag_[1].uchar_[1] = *(++ptr); mag_[1].uchar_[0] = *(++ptr);  
            mag_[2].uchar_[3] = *(++ptr); mag_[2].uchar_[2] = *(++ptr); mag_[2].uchar_[1] = *(++ptr); mag_[2].uchar_[0] = *(++ptr);  
            
            time_.uchar_[3] = data[37]; time_.uchar_[2] = data[38]; time_.uchar_[1] = data[39]; time_.uchar_[0] = data[40];
            double timestamp_from_imu = time_.int_ / 62500.0;

            R_[0].uchar_[3] = *(++ptr); R_[0].uchar_[2] = *(++ptr); R_[0].uchar_[1] = *(++ptr); R_[0].uchar_[0] = *(++ptr);  
            R_[1].uchar_[3] = *(++ptr); R_[1].uchar_[2] = *(++ptr); R_[1].uchar_[1] = *(++ptr); R_[1].uchar_[0] = *(++ptr);  
            R_[2].uchar_[3] = *(++ptr); R_[2].uchar_[2] = *(++ptr); R_[2].uchar_[1] = *(++ptr); R_[2].uchar_[0] = *(++ptr);  
            R_[3].uchar_[3] = *(++ptr); R_[3].uchar_[2] = *(++ptr); R_[3].uchar_[1] = *(++ptr); R_[3].uchar_[0] = *(++ptr);  
            R_[4].uchar_[3] = *(++ptr); R_[4].uchar_[2] = *(++ptr); R_[4].uchar_[1] = *(++ptr); R_[4].uchar_[0] = *(++ptr);  
            R_[5].uchar_[3] = *(++ptr); R_[5].uchar_[2] = *(++ptr); R_[5].uchar_[1] = *(++ptr); R_[5].uchar_[0] = *(++ptr);  
            R_[6].uchar_[3] = *(++ptr); R_[6].uchar_[2] = *(++ptr); R_[6].uchar_[1] = *(++ptr); R_[6].uchar_[0] = *(++ptr);  
            R_[7].uchar_[3] = *(++ptr); R_[7].uchar_[2] = *(++ptr); R_[7].uchar_[1] = *(++ptr); R_[7].uchar_[0] = *(++ptr);  
            R_[8].uchar_[3] = *(++ptr); R_[8].uchar_[2] = *(++ptr); R_[8].uchar_[1] = *(++ptr); R_[8].uchar_[0] = *(++ptr);  

            double delay = 0.0;
            msg_imu_.header.stamp    = t0 + ros::Duration(timestamp_from_imu) - ros::Duration(delay);
            msg_imu_.header.frame_id = "body";
            msg_imu_.angular_velocity.x    = gyro_[0].float_;
            msg_imu_.angular_velocity.y    = gyro_[1].float_;
            msg_imu_.angular_velocity.z    = gyro_[2].float_;
            msg_imu_.linear_acceleration.x = acc_[0].float_ * 9.81;
            msg_imu_.linear_acceleration.y = acc_[1].float_ * 9.81;
            msg_imu_.linear_acceleration.z = acc_[2].float_ * 9.81;


            Eigen::Matrix3d R;
            for (size_t i = 0; i < 3; ++i)
                for (size_t j = 0; j < 3; ++j)
                    R(i,j) = R_[j*3+i].float_;
                
            Eigen::Quaternion<double> q(R);
            msg_imu_.orientation.w = (double)q.w();// q(0);
            msg_imu_.orientation.x = (double)q.x();// q(1);
            msg_imu_.orientation.y = (double)q.y();// q(2);
            msg_imu_.orientation.z = (double)q.z();// q(3);

            msg_mag_.header.stamp    = msg_imu_.header.stamp;
            msg_mag_.header.frame_id = msg_imu_.header.frame_id;
            msg_mag_.magnetic_field.x = mag_[0].float_;
            msg_mag_.magnetic_field.y = mag_[1].float_;
            msg_mag_.magnetic_field.z = mag_[2].float_;
            
            pub_imu_.publish(msg_imu_);
            pub_mag_.publish(msg_mag_);
        }

        ros::spinOnce();
        // rate.sleep();
    }
};




bool IMU_3DM_GX3_25::validateChecksum(const unsigned char* data, unsigned short len)
{
    unsigned short checksum_calc = 0;
    unsigned short checksum_recv = 0;

    //-------- Calculate the checksum
    for (unsigned short i = 0; i < len - 2; i++) checksum_calc += data[i];

    //-------- Extract the big-endian checksum from reply
    checksum_recv  = data[len - 2] << 8;
    checksum_recv += data[len - 1];

    //-------- Compare the checksums
    return (checksum_calc == checksum_recv);
}

float IMU_3DM_GX3_25::extractFloat(unsigned char* addr) {
  float tmp;
  *((unsigned char*)(&tmp) + 3) = *(addr);
  *((unsigned char*)(&tmp) + 2) = *(addr+1);
  *((unsigned char*)(&tmp) + 1) = *(addr+2);
  *((unsigned char*)(&tmp))     = *(addr+3);
  return tmp;
}

int IMU_3DM_GX3_25::extractInt(unsigned char* addr) {
  int tmp;
  *((unsigned char*)(&tmp) + 3) = *(addr);
  *((unsigned char*)(&tmp) + 2) = *(addr+1);
  *((unsigned char*)(&tmp) + 1) = *(addr+2);
  *((unsigned char*)(&tmp))     = *(addr+3);
  return tmp;
}


