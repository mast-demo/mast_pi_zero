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
#include "geometry_msgs/Vector3.h"
//#include "wiringPiI2C.h"
#include "string"
#include "cmath"	//std::abs
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "Adafruit_INA219.h"

Adafruit_INA219 ina219;

using namespace std;

//Attempt to use Linux I2C
//int file;
//int adapter_nr = 1;
//char filename[20];

//Initialize I2C system with given device identifier. 
// Right Current Sense: 0x40
// Left Current Sense: 0x41
int devId_r = 0x40;

int main(int argc, char** argv)
{

	ros::init(argc,argv,"mast");
	ros::NodeHandle n;

	ros::Publisher right_current_pub = n.advertise<geometry_msgs::Vector3>("current_sense_right",1000);

	ina219.begin(devId_r);
	//16V 400mA set by default in begin()
	//ina219.setCalibration_16V_400mA();
	ros::Rate loop_rate(1);
	
	while(ros::ok()){
		
		float shuntvoltage = 0;
		float busvoltage = 0;
		float current_mA = 0;
		float loadvoltage = 0;
		
		geometry_msgs::Vector3 right_current_sense;
		
		shuntvoltage = ina219.getShuntVoltage_mV();
		busvoltage = ina219.getBusVoltage_V();
		current_mA = ina219.getCurrent_mA();
		loadvoltage = busvoltage + (shuntvoltage/1000);
		
		right_current_sense.x = current_mA;
		right_current_sense.y = busvoltage;
		right_current_sense.z = loadvoltage;
		
		right_current_pub.publish(right_current_sense);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
