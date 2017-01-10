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
#include "geometry_msgs/TransformStamped.h"
//#include <tf/transform_datatypes.h>

using namespace std;
vector<geometry_msgs::Pose> goals;
int currentGoalIdx = 0;

ros::Publisher cmdPub;
#define ANGLE_DEADBAND 0.2
#define DISTANCE_DEADBAND 0.1

void myPoseCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	geometry_msgs::Pose goal = goals.at(currentGoalIdx);
	float dx = goal.position.x - msg->transform.translation.x ;
	float dy = goal.position.y - msg->transform.translation.y;
	float heading = atan2(dy,dx);
	float d = sqrt(dx*dx+dy*dy);
	ROS_INFO_STREAM(" heading = " << heading << " distance = " << d);
	geometry_msgs::Quaternion q = msg->transform.rotation;
	float yaw   =atan2(2.0 * (q.z * q.w + q.x * q.y) , - 1.0 + 2.0 * (q.w * q.w + q.x * q.x));
	ROS_INFO_STREAM(" myYaw = " << yaw);
	float heading_error = heading - yaw;
	geometry_msgs::Twist cmd;
	if(d < DISTANCE_DEADBAND) {
		ROS_INFO("ARRIVED");
		cmd.linear.x = 0.0;	
		cmd.angular.z = 0.0;	
	} else if(heading_error > ANGLE_DEADBAND) {
		ROS_INFO("LEFT");
		cmd.linear.x = 0.0;	
		cmd.angular.z = 1.0;	
	} else if(heading_error < -ANGLE_DEADBAND) {
		ROS_INFO("RIGHT");
		cmd.linear.x = 0.0;	
		cmd.angular.z = -1.0;	
	} else {
		ROS_INFO("FORWARD");
		cmd.linear.x = 1.0;	
		cmd.angular.z = 0.0;	
	}
	cmdPub.publish(cmd);
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"mast");
	ros::NodeHandle n;
	cmdPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Subscriber sub = n.subscribe("vicon", 1000, myPoseCallback); 

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
