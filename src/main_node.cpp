/***********************
node to manage the overall modes and operation
of the robot.
***********************/
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Empty.h"
//#include <tf/transform_datatypes.h>

using namespace std;
vector<geometry_msgs::Pose> goals;
int currentGoalIdx = 0;

ros::Publisher goalPub;
ros::Publisher cancelPub;

int main(int argc, char** argv)
{
	ros::init(argc,argv,"mast");
	ros::NodeHandle n;
	goalPub = n.advertise<geometry_msgs::Pose>("goal", 1);
	cancelPub = n.advertise<std_msgs::Empty>("cancel", 1);

	geometry_msgs::Pose goal;
	goal.position.x = 0.0;
	goal.position.y = 0.0;
	goals.push_back(goal);
	goal.position.x = 1.0;
	goal.position.y = 1.0;
	goals.push_back(goal);
	goal.position.x = -1.0;
	goal.position.y = 1.0;
	goals.push_back(goal);
	goal.position.x = -1.0;
	goal.position.y = -1.0;
	goals.push_back(goal);
	goal.position.x = -1.0;
	goal.position.y = 1.0;
	goals.push_back(goal);
	
	ros::spin();	
	return 0;
}
