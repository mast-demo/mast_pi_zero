/******************
	node to handle onboard serial port
  to talk to SparkFun Serial Controller Motor Driver ROB 13911
  motor control board

 ******************/

#include <stdio.h>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "serial/serial.h"

using namespace std;
serial::Serial *serial_port;
// velocity for 100% effort. used to scale input from m/s to %
#define MAX_VEL 1.0
// distance from center to side point of contact to ground
#define DS 0.03 

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  float v = msg->linear.x;
  float omega = msg->angular.z;
  if(omgea < 0.01) {
    vl = vr = v;
  } else {
    float r0 = v/omega;
    float rl = r0 - DS;
    float rr = r0 + DS;
    vl = v * (rl/r0);
    vr = v * (rr/r0);
  }
  int ml = vl / MAX_VEL;
  int mr = vr / MAX_VEL;
  char str[25];
  if(ml>0) {
    sprintf(str,"M0F%i\n",ml);
  } else {
    sprintf(str,"M0R%i\n",ml);
  }
  serial_port->write(str);
  if(mr>0) {
    sprintf(str,"M0F%i\n",mr);
  } else {
    sprintf(str,"M0R%i\n",mr);
  }
  serial_port->write(str);
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
  serial_port->write("E\n"); // enable motors

	while(ros::ok()) {
		ros::spinOnce();	
	}
	return 0;
}
