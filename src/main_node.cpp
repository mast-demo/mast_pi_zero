#include <stdio.h>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

using namespace std;

void cameraCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	ROS_INFO("Got an image");
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"mast");
	ros::NodeHandle n;
	ros::Subscriber camera_sub = n.subscribe("raspicam",1000, 
		cameraCallback);
	ros::spin();	
	return 0;
}
