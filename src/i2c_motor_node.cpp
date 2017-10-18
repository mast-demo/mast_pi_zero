/******************
	node to handle onboard serial port
	to talk to SparkFun Serial Controller Motor Driver ROB 13911
	motor control board
	
	Subscribes to cmd_vel Twist messages and produces the 
	corresponding commands on the serial port

 ******************/

#include <stdio.h>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"
//#include "wiringPiI2C.h"
#include "string"
#include "cmath"	//std::abs
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>

using namespace std;

//Attempt to use Linux I2C
int file;
int adapter_nr = 1;
char filename[20];

//Initialize I2C system with given device identifier.
int devId = 0x5F;
int devId_w = 0xBE;
int SCMD;
int buff;

// velocity for 100% effort. used to scale input from m/s to %
#define MAX_VEL 1.0
// distance from center to side point of contact to ground
#define DS 0.3 
ros::Time lastCommandTime;
ros::Duration watchDogLimit = ros::Duration(1.0);
bool motorsEnabled = false;

float limit(float val, float limit)
{
	if(val < -limit) return -limit;
	if(val > limit) return limit;
	return val;
}
void enableMotors(bool ena)
{

	char buf[3];

	if(ena) 
	{
		motorsEnabled = true;

	buf[0] = 0x70;
	buf[1] = 0x01;
	
	if(write(file, buf, 2) != 2){
		ROS_INFO_STREAM("Error Writing Enable");	
	}

		ROS_INFO("enabling motors");
	} 
	else 
	{
		motorsEnabled = false;

	  buf[0] = 0x70;
		buf[1] = 0x00;

		if(write(file,buf, 2) != 2){
			ROS_INFO_STREAM("Error Writing Disable");
		}
		ROS_INFO("disabling motors");
	}
}

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	if(!motorsEnabled) 
	{
		enableMotors(true);
	}
	float v = msg->linear.x;
	float omega = msg->angular.z;
	float vl,vr;
	float vl_turn, vr_turn;

	char buf_1[2];
	char buf_2[2];

	vl_turn = -omega*DS;
	vr_turn = -vl_turn;
	vl = limit(v+vl_turn, MAX_VEL);
	vr = limit(v+vr_turn, MAX_VEL);

	int ml = vl*128.0 / MAX_VEL;
	int mr = vr*128.0 / MAX_VEL;
	char str[25];
	sprintf(str,"ml: %i mr: %i\n",ml,mr);
	ROS_INFO_STREAM(str);
	
	buf_1[0] = 0x20;
	buf_1[1] = (0x80+ml);
	//sprintf(str,"0x%02X 0x%02X", buf_1[0],buf_1[1]);
	//ROS_INFO_STREAM(str);
	
	buf_2[0] = 0x21;
	buf_2[1] = (0x80+mr);		
	
	int length = 2;
	if(write(file, buf_1,length) != length){
		ROS_INFO_STREAM("Error Writing M0");
	}
	if(write(file, buf_2, length) != length){
		ROS_INFO_STREAM("Error Writing M1");
	}

	lastCommandTime = ros::Time::now();
}

void timerCallback(const ros::TimerEvent&)
{
	if( motorsEnabled && 
		((ros::Time::now() - lastCommandTime)  > watchDogLimit ) )
	{
		ROS_INFO("cmd_vel timed out, stopping motors");
		enableMotors(false);
	}
}

int main(int argc, char** argv)
{

	ros::init(argc,argv,"mast");
	ros::NodeHandle n;
	// get the parameters from parameter server
	ros::NodeHandle private_node_handle("~");

	ros::Subscriber cmd_sub = n.subscribe("cmd_vel",1000, cmdCallback);
	ros::Timer timer = n.createTimer(ros::Duration(0.2), timerCallback);


  ROS_INFO_STREAM("I2C Setup");
	sprintf(filename,"/dev/i2c-1");
	file = open(filename, O_RDWR);
	if (file < 0){
		ROS_INFO_STREAM("I2C Setup Error");
	}


	if(ioctl(file, I2C_SLAVE, devId) < 0) {
		ROS_INFO("Unable to connect to Device: %d",devId);
	}
	enableMotors(false);

	//Invert motor A
	char buf[0];
	buf[0] = 0x12;
	buf[1] = 0x01;

	int length = 2;
	if(write(file, buf,length) != length){
		ROS_INFO_STREAM("Error Inverting M0");
	}	


	lastCommandTime = ros::Time::now();

	ros::spin();	

	return 0;
}
