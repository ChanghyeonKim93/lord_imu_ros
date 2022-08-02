#include <iostream>
#include <string>
#include <exception>

#include <ros/ros.h>

#include "imu_3dm_gx3_25.h"
#include "util/signal_handler_linux.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "lord_3dm_gx3_25_node");
    // ros::init(argc, argv, "lord_3dm_gx3_25_node", ros::init_options::NoSigintHandler);
    // SignalHandle::initSignalHandler();

    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("lord_3dm_gx3_25_node - STARTS.");

	try{
        std::shared_ptr<IMU_3DM_GX3_25> imu;
		if(ros::ok()){
            imu = std::make_shared<IMU_3DM_GX3_25>(nh);
		}
		else throw std::runtime_error("ROS not ok");
	}
	catch (std::exception& e){
		std::cout << "errno: " << errno <<"\n";
		ROS_ERROR(e.what());
	}

    ROS_INFO_STREAM("lord_3dm_gx3_25_node - TERMINATED.");
	return 0;
}