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

using namespace std;
serial::Serial *serial_port;
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
	if(ena) 
	{
		motorsEnabled = true;
		serial_port->write("E\n");
		ROS_INFO("enabling motors");
	} 
	else 
	{
		motorsEnabled = false;
		serial_port->write("D\n");
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

	vl_turn = -omega*DS;
	vr_turn = -vl_turn;
	vl = limit(v+vl_turn, MAX_VEL);
	vr = limit(v+vr_turn, MAX_VEL);

	int ml = vl*100.0 / MAX_VEL;
	int mr = vr*100.0 / MAX_VEL;
	char str[25];
	if(ml>0) {
		sprintf(str,"M0F%i\n",ml);
	} else {
		sprintf(str,"M0R%i\n",-ml);
	}
	ROS_INFO_STREAM(str);
	serial_port->write(str);
	if(mr>0) {
		sprintf(str,"M1F%i\n",mr);
	} else {
		sprintf(str,"M1R%i\n",-mr);
	}
	serial_port->write(str);
	ROS_INFO_STREAM(str);
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
	string port;
	int baudrate;

	ros::init(argc,argv,"mast");
	ros::NodeHandle n;
	// get the parameters from parameter server
	ros::NodeHandle private_node_handle("~");
	private_node_handle.param("port", port, string("/dev/ttyS0"));
	private_node_handle.param("baudrate", baudrate, int(115200));

	ros::Subscriber cmd_sub = n.subscribe("cmd_vel",1000, cmdCallback);
	ros::Timer timer = n.createTimer(ros::Duration(0.2), timerCallback);

	cout << "Opening " << port << " for serial com at " << baudrate << " baud\n";

	serial_port = new serial::Serial(port, baudrate, 
			serial::Timeout::simpleTimeout(1000));
	//	serial_port->setTimeout(10, 1, 10, 1, 10);
	if(serial_port->isOpen()) {
		cout << "Serial Port opened\n";
	} else {
		cout << "Serial Port FAILED\n";
	}
	serial_port->write("M0I\n"); // invert M0 since it is on the left side
//	serial_port->write("E\n"); // enable motors
	enableMotors(false);

	lastCommandTime = ros::Time::now();

	while(ros::ok()) {
		ros::spinOnce();	
	}
	return 0;
}
