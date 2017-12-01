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
#include "geometry_msgs/Point.h"
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

float v,omega,vl,vr;
float vl_turn, vr_turn;

float w_r,w_l,dot_error_r,dot_error_l,error_r,error_l,P_r,P_l,D_r,D_l,cmd_vel_r,cmd_vel_l,mr_raw,ml_raw;
int mr,ml;

ros::Publisher cmd_pub; 
// velocity for 100% effort. used to scale input from m/s to %
#define MAX_VEL 0.5
// distance from center to side point of contact to ground
#define DS 0.05 

double t;
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
/*
void leftFeedBack(const geometry_msgs::Point::ConstPtr& msg){

	float error = vl/MAX_VEL*4.2 - msg->y; //Convert vr reference (m/s) into voltage and subtract from feedback VI_Sense/right.y (bus voltage)
	// error = vl*100/MAX_VEL*4.2 - msg->y; removed the *100 
	float P_l = 0.7;

	float cmd_vel_l = P_l * error;

	//int ml = cmd_vel_l*128.0 / MAX_VEL;
	float ml_raw = cmd_vel_l*128.0/4.2;
	int ml = limit(ml_raw,128.0);

	char buf_1[2];	
	buf_1[0] = 0x20;
	buf_1[1] = (0x80+ml);
	
	int length = 2;
	if(write(file, buf_1,length) != length){
		ROS_INFO_STREAM("Error Writing M0");
	}
	ROS_INFO("error_l: %f",error);
	ROS_INFO("ml_raw: %f",ml_raw);
	ROS_INFO("ml: %d", ml);
}

void rightFeedBack(const geometry_msgs::Point::ConstPtr& msg){
	
	float error = vr/MAX_VEL*4.2 - msg->y; //Convert vr reference (m/s) into voltage and subtract from feedback VI_Sense/right.y (bus voltage)
	float P_r = 0.7;
	float cmd_vel_r = P_r * error;

	//int mr = cmd_vel_r*128.0 / MAX_VEL; //Convert vr (m/s) into motor command (percentage of full speed)
	float mr_raw = cmd_vel_r*128.0/4.2;
	int mr = limit(mr_raw,128.0);
	
	char buf_2[2];
	buf_2[0] = 0x21;
	buf_2[1] = (0x80+mr);	//Convert to motor command	
	
	int length = 2;
	if(write(file, buf_2, length) != length){
		ROS_INFO_STREAM("Error Writing M1");
	}
 	ROS_INFO("error_r: %f",error);
	ROS_INFO("mr_raw: %f",mr_raw);
	ROS_INFO("mr: %d", mr);  
}
*/
void w_Feedback(const geometry_msgs::Point::ConstPtr& msg){

	w_l = msg->x;
	w_r = msg->y;

	ROS_INFO_STREAM("w_r: " << w_r << " w_l: " << w_l);

	dot_error_r = ((vr - w_r) - error_r)/(ros::Time::now().toSec()-t);
	dot_error_l = ((vl - w_l) - error_l)/(ros::Time::now().toSec()-t);

	error_r = vr - w_r;
	error_l = vl - w_l;
	t = ros::Time::now().toSec();
	//P_r = 0.039;
	//P_l = 0.05;
	//D_r = 0.0002;
	//D_l = 0.0002;
		
	cmd_vel_r = P_r * error_r + D_r * dot_error_r;
	cmd_vel_l = P_l * error_l + D_l * dot_error_l;
	
	ROS_INFO_STREAM("cmd_vel_r: " << cmd_vel_r << " cmd_vel_l: " << cmd_vel_l);

	mr_raw = cmd_vel_r*128.0/4.2;
	ml_raw = cmd_vel_l*128.0/4.2;

	mr = limit(mr_raw,128.0);
	ml = limit(ml_raw,128.0); 
	
	if(vr == 0 || vl == 0){
		mr = 0;
		ml = 0;
	}
	char buf_2[2];
	buf_2[0] = 0x21; //Motor B
	buf_2[1] = (0x80+mr);

	char buf_1[2];
	buf_1[0] = 0x20;  //Motor A 
	buf_1[1] = (0x80+ml);
	
	int length = 2;
	if(write(file,buf_2, length) != length){
		ROS_INFO_STREAM("Error Writing MB");
	}
	if(write(file,buf_1, length) != length){
		ROS_INFO_STREAM("Error Writing MA");
	}
	ROS_INFO_STREAM("mr: " << mr << " ml: " << ml);
	//t = ros::Time::now().toSec();
}

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	if(!motorsEnabled) 
	{
		enableMotors(true);
	}
	
	v = msg->linear.x;
	omega = msg->angular.z;
	
	geometry_msgs::Point cmd;
	cmd.x = v;
	cmd.y = omega;
	cmd.z = ros::Time::now().toSec();
	cmd_pub.publish(cmd);

	float R = 0.01;
	float L = 0.1;
	float velocity_gain = 1; //0.2
	vr = (2*v/R+omega*L/R)/2/velocity_gain;
	vl = -L/R*omega/velocity_gain+vr;
	ROS_INFO_STREAM("vr: " << vr << " vl: " << vl);
/*
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
*/
	lastCommandTime = ros::Time::now();
}

void timerCallback(const ros::TimerEvent&)
{
	if( motorsEnabled && ((ros::Time::now() - lastCommandTime) > watchDogLimit))
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
	private_node_handle.param("P_r",P_r,float(0.039));
	private_node_handle.param("P_l",P_l,float(0.05));
	private_node_handle.param("D_r",D_r,float(0.0));
	private_node_handle.param("D_l",D_l,float(0.0));
	
	cmd_pub = n.advertise<geometry_msgs::Point>("cmd",1000); 		
	ros::Subscriber cmd_sub = n.subscribe("cmd_vel",1, cmdCallback);
	//ros::Subscriber left_sub = n.subscribe("VI_Sense/left", 10, leftFeedBack);
	//ros::Subscriber right_sub = n.subscribe("VI_Sense/right",10,rightFeedBack);
	ros::Subscriber w_sub = n.subscribe("w",1,w_Feedback);
	ros::Timer timer = n.createTimer(ros::Duration(1), timerCallback);


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
	buf[0] = 0x12; //MOTOR_A_INVERT (LEFT MOTOR)
	buf[1] = 0x01;

	int length = 2;
	if(write(file, buf,length) != length){
		ROS_INFO_STREAM("Error Inverting MA");
	}	


	lastCommandTime = ros::Time::now();

	ros::spin();	

	return 0;
}
