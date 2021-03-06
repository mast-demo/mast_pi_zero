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
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
//#include <tf/transform_datatypes.h>

using namespace std;
//vector<geometry_msgs::Pose> goals;
//int currentGoalIdx = 0;
float linear_v = 1.7;
//float angular_v = 0;
float P = 1;
float heading_error = 0;
float heading_effort = 0;
bool busy = false;
ros::Publisher cmdPub;
//ros::Publisher stopFlagPub;
//geometry_msgs::Pose goal;
#define ANGLE_DEADBAND 0.2
//#define ANGLE_TURN_IN_PLACE 0.6
#define DISTANCE_DEADBAND 0.25
#define FORWARD_VEL 0.5 
//#define ANGULAR_VEL 2.5
int n = 2;
float xWaypoints [2] = {0.3,0.3};
float yWaypoints [2] = {-0.5,-0.5};
int current = 0;
/*
void linVelCallback(const std_msgs::Float32::ConstPtr& msg)
{
	linear_v = msg->data;
	ROS_INFO("Changing linear velocity to %f", linear_v);
}
void angVelCallback(const std_msgs::Float32::ConstPtr& msg)
{
	angular_v = msg->data;
	ROS_INFO("Changing angular velocity to %f",angular_v);
}
*/
void cancelCallback(const std_msgs::Empty::ConstPtr& msg)
{
	ROS_INFO("Cancelling Move command");
	busy = false;
	geometry_msgs::Twist cmd;
	cmd.linear.x = 0;
	cmd.angular.z = 0;
	cmdPub.publish(cmd);
}
void goalCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	ROS_INFO("Recevied new goal");
	if(busy == false) busy = true;
	//goal = *msg;
}
void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	if(!busy) return;
	float dx = xWaypoints[current] - msg->pose.pose.position.x;//goal.position.x - msg->pose.pose.position.x ;
	float dy = yWaypoints[current] - msg->pose.pose.position.y;//goal.position.y - msg->pose.pose.position.y;
	float heading = atan2(dy,dx);
	float d = sqrt(dx*dx+dy*dy);
	ROS_INFO_STREAM(" heading = " << heading << " distance = " << d);
	geometry_msgs::Quaternion q = msg->pose.pose.orientation;
	float yaw   =atan2(2.0 * (q.z * q.w + q.x * q.y) , - 1.0 + 2.0 * (q.w * q.w + q.x * q.x));
	ROS_INFO_STREAM(" myYaw = " << yaw);

	float heading_error = heading - yaw;
	
	if ( heading_error < -3*M_PI/2 || heading_error > 3*M_PI/2 )
	    heading_effort = -0.5*heading_error;
	  else
	    heading_effort = P*heading_error;
	if(heading_effort < -1.5)
		heading_effort = -1.5;
	if(heading_effort > 1.5)
		heading_effort = 1.5;

	ROS_INFO_STREAM(" heading_error = " << heading_error);
	ROS_INFO_STREAM(" heading_effort = " << heading_effort);
	//while(heading_error > M_PI) heading_error-=2.0*M_PI;
	//while(heading_error < -M_PI) heading_error+=2.0*M_PI;

	geometry_msgs::Twist cmd;
	
	if(d < DISTANCE_DEADBAND) {
		ROS_INFO("ARRIVED");
		cmd.linear.x = 0.0;	
		cmd.angular.z = 0.0; 
		current = current + 1;
		ROS_INFO_STREAM("Current Waypoint" << current);
		if (current == n) busy = false;
		ROS_INFO_STREAM("DONE");
	}	
	else {
		ROS_INFO("FORWARD");
		cmd.linear.x = 0.5;// linear speed in m/s 0.8;//linear_v;	
		cmd.angular.z = heading_effort;	//rad/s
	}
	cmdPub.publish(cmd);
}
/*
void callback(const ros::TimerEvent& event)
{
std_msgs::Bool flag;
flag.data = 1;
stopFlagPub.publish(flag);
ros::Duration(2).sleep();
flag.data = 0;
stopFlagPub.publish(flag);
}
*/
int main(int argc, char** argv)
{
	ros::init(argc,argv,"move_controller");
	ros::NodeHandle n;
	cmdPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	//stopFlagPub = n.advertise<std_msgs::Bool>("stop_flag", 1);
	ros::Subscriber poseSub = n.subscribe("pose", 1, poseCallback); 
	ros::Subscriber goalSub = n.subscribe("goal", 1, goalCallback); 
	ros::Subscriber cancelSub = n.subscribe("cancel", 1, cancelCallback); 
	//ros::Subscriber linVelSub = n.subscribe("linear_vel", 1, linVelCallback); 
	//ros::Subscriber angVelSub = n.subscribe("angular_vel", 1, angVelCallback); 
	ros::NodeHandle rn("~");
	rn.param<float>("linear_v", linear_v, FORWARD_VEL);
	rn.param<float>("P", P, 1);
	ROS_INFO_STREAM("Linear_v = "<<linear_v<<" P = "<< P);
	//ros::Timer timer = n.createTimer(ros::Duration(2), callback);
	ros::spin();	
	return 0;
}
