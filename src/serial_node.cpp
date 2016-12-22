/******************
node to handle onboard serial port
This should do all of the read adn write on the serial port

currently subscribes to image and sends it out serial port
also prints num bytes received

create a very simple packet system
<HEADER> (4 bytes) = 0xABCDEF56
<LENGTH> (4 bytes) = number of total bytes including 12 for hdr, num, ftr
<DATA>
<FOOTER> (4 bytes) = 0xBAADF00D

so serial looks for HEADER in stream. 
 then reades LENGTH
 then reads next LENGTH-8 bytes
 then checks that next 4 byes are FOOTER

******************/

#include <stdio.h>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "serial/serial.h"

using namespace std;
serial::Serial *serial_port;

#define MAX_FRAME_SIZE 1024

void cameraCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
	ROS_INFO_STREAM("Got an image.  size = " << msg->data.size() << " bytes");
//	serial_port->write("Got an image");
	serial_port->write(msg->data);
}

int main(int argc, char** argv)
{
	string port;
	int baudrate;

	ros::init(argc,argv,"mast");
	ros::NodeHandle n;
	// get the parameters from parameter server
	ros::NodeHandle private_node_handle("~");
	private_node_handle.param("port", port, string("/dev/ttyAMA0"));
	private_node_handle.param("baudrate", baudrate, int(1000000));

	ros::Subscriber camera_sub = n.subscribe("camera/image/compressed",1000, 
		cameraCallback);

	cout << "Opening " << port << " for serial com at " << baudrate << " baud\n";
	
	serial_port = new serial::Serial(port, baudrate, 
		serial::Timeout::simpleTimeout(1000));
//	serial_port->setTimeout(10, 1, 10, 1, 10);
	if(serial_port->isOpen()) {
		cout << "Serial Port opened\n";
	} else {
		cout << "Serial Port FAILED\n";
	}
	vector<uint8_t> buffer(MAX_FRAME_SIZE);
	size_t numBytes;
	int bytesRemaining = 0;
	int pktBytesReceived = 0;
	while(ros::ok()) {
		numBytes = serial_port->read(buffer, MAX_FRAME_SIZE);
		if(numBytes > 0) {
			ROS_INFO_STREAM("Serial Rx " << numBytes << " bytes");
		}
		ros::spinOnce();	
	}
	return 0;
}
