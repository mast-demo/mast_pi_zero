/***********************
node to manage the overall modes and operation
of the robot.
***********************/
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc,argv,"mast");
	ros::NodeHandle n;

	ros::spin();	
	return 0;
}
