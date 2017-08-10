/***********************
node to create cmd_vel to move to next goal
subscribes to:
	should probably be made into an action client
 pose = current robot pose should be published at reasonable rate
 goal = current goal.  will attempt to go to this pose when receieved
 cancel = cancel current operations

publishes:
	cmd_vel

***********************/
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
//#include <tf/transform_datatypes.h>

using namespace std;
vector<geometry_msgs::Pose> goals;
int currentGoalIdx = 0;

bool busy = false;
ros::Publisher cmdPub;
geometry_msgs::Pose goal;
#define ANGLE_DEADBAND 0.2
#define DISTANCE_DEADBAND 0.1
#define FORWARD_VEL 0.5
#define ANGULAR_VEL 0.5

void cancelCallback(const std_msgs::Empty::ConstPtr& msg)
{
	busy = false;
}
void goalCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	if(busy == false) busy = true;
	goal = *msg;
}
void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	if(!busy) return;
	float dx = goal.position.x - msg->pose.pose.position.x ;
	float dy = goal.position.y - msg->pose.pose.position.y;
	float heading = atan2(dy,dx);
	float d = sqrt(dx*dx+dy*dy);
	ROS_INFO_STREAM(" heading = " << heading << " distance = " << d);
	geometry_msgs::Quaternion q = msg->pose.pose.orientation;
	float yaw   =atan2(2.0 * (q.z * q.w + q.x * q.y) , - 1.0 + 2.0 * (q.w * q.w + q.x * q.x));
	ROS_INFO_STREAM(" myYaw = " << yaw);
	float heading_error = heading - yaw;
	geometry_msgs::Twist cmd;
	if(d < DISTANCE_DEADBAND) {
		ROS_INFO("ARRIVED");
		busy = false;
		cmd.linear.x = 0.0;	
		cmd.angular.z = 0.0;	
	} else if(heading_error > ANGLE_DEADBAND) {
		ROS_INFO("LEFT");
		cmd.linear.x = 0.0;	
		cmd.angular.z = ANGULAR_VEL;	
	} else if(heading_error < -ANGLE_DEADBAND) {
		ROS_INFO("RIGHT");
		cmd.linear.x = 0.0;	
		cmd.angular.z = -ANGULAR_VEL;	
	} else {
		ROS_INFO("FORWARD");
		cmd.linear.x = FORWARD_VEL;	
		cmd.angular.z = 0.0;	
	}
	cmdPub.publish(cmd);
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"move_controller");
	ros::NodeHandle n;
	cmdPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	ros::Subscriber poseSub = n.subscribe("pose", 1, poseCallback); 
	ros::Subscriber goalSub = n.subscribe("goal", 1, goalCallback); 
	ros::Subscriber cancelSub = n.subscribe("cancel", 1, cancelCallback); 
	ros::spin();	
	return 0;
}
