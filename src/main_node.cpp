#include <stdio.h>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "serial/serial.h"

using namespace std;
serial::Serial serial_port("/dev/ttyAMA0", 9600, 
	serial::Timeout::simpleTimeout(1000));

void cameraCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	ROS_INFO("Got an image");
	serial_port.write("Got an image");
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"mast");
	ros::NodeHandle n;
	ros::Subscriber camera_sub = n.subscribe("raspicam",1000, 
		cameraCallback);
	if(serial_port.isOpen()) {
		cout << "Serial Port opened\n";
	} else {
		cout << "Serial Port FAILED\n";
	}

	
	ros::spin();	
	return 0;
}
