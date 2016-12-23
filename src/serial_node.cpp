/******************
	node to handle onboard serial port
	This should do all of the read adn write on the serial port

	currently subscribes to image and sends it out serial port
	also prints num bytes received

	create a very simple packet system
	<HEADER> (4 bytes) 
	<LENGTH> (4 bytes) = number of total bytes including 12 for hdr, num, ftr
	<DATA>
	<FOOTER> (4 bytes)

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

// maximum number of bytes to read on each serial port read
#define MAX_PKT_SIZE 1024
// maximum total image file size
#define MAX_IMAGE_SIZE 1000000
uint32_t header=0xBEEFFACE;
uint32_t footer=0xBAADF00D;

void cameraCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
	ROS_INFO_STREAM("Transmiting image size = " << msg->data.size() << " bytes");
	//	serial_port->write("Got an image");
	uint32_t numBytes = msg->data.size()+12;
	serial_port->write((uint8_t*)&header, 4);
	serial_port->write((uint8_t*)&numBytes, 4);
	serial_port->write(msg->data);
	serial_port->write((uint8_t*)&footer, 4);
}

size_t pktBytesReceived = 0;
uint32_t imageSize = 0;
uint8_t imageBuffer[MAX_IMAGE_SIZE];
uint8_t tmpPktBuffer[MAX_PKT_SIZE*2];
size_t tmpPktCount = 0;
/***************************
	this is a recursive function to parse the serial stream
	it skips unitl it finds a header value. then stores it
	then stores the image size
	then stores that many bytes 
	then looks for footer

returns:
BUSY if image in progress
IMG_DONE if image is complete

 ****************************/
#define PKT_IDLE 0
#define PKT_BUSY 1
#define PKT_DONE 2

int findPktStart(void);
int findImageSize(void);
int collectData(void);
int findFooter(void);
void removeBytes(size_t num)
{
	memcpy(tmpPktBuffer, &tmpPktBuffer[num], tmpPktCount - num);
	tmpPktCount -= num;
}
int findPktStart(void)
{
	/* 
		 look for header discard anything before it
		 when it is found, remove header from buffer and look for size
	 */
	if(tmpPktCount <4) {
		return PKT_BUSY;
	}
	uint32_t val;
	for(int i=0;i<tmpPktCount-3;++i) {
		val = *(uint32_t*)&tmpPktBuffer[i];
		if(val == header) {
			ROS_INFO("found Serial Rx header");
			pktBytesReceived = 4;
			if(tmpPktCount > i+3) {
				removeBytes(i+4);
				return findImageSize();
			} else {
				return PKT_BUSY;
			}
		}
	}
	/* otherwise flush buffer */
	//ROS_INFO("No header, buffer flushed");
	tmpPktCount = 0;
	return PKT_IDLE;
}
/*
	 assuming header was found, size must be next four bytes in stream
 */
int findImageSize(void)
{
	if(tmpPktCount < 4) {
		/* if not enough bytes then return */
		return PKT_BUSY;
	} else {
		/* 
			 if there are enough bytes to include the size then read it 
			 and then remove those bytes and look for data
		 */
		imageSize = *(uint32_t*)tmpPktBuffer - 12;
		//ROS_INFO_STREAM("image size = " << imageSize << " bytes");
		pktBytesReceived+=4;
		removeBytes(4);
		return collectData();
	}
}
/*
	 collecting data into image buffer.  Just keep copying until image size is reached
 */
int collectData(void)
{
	size_t bytesRemaining = imageSize - (pktBytesReceived - 8);

	//ROS_INFO_STREAM("Serial Rx " << tmpPktCount << " bytes.  have "<< pktBytesReceived << " need " << imageSize << " remiaing " << bytesRemaining);
	if(tmpPktCount > bytesRemaining) {
		memcpy(&imageBuffer[pktBytesReceived-8], tmpPktBuffer, bytesRemaining);
		removeBytes(bytesRemaining);
		pktBytesReceived += bytesRemaining;
		return findFooter();
	} else {
		memcpy(&imageBuffer[pktBytesReceived-8], tmpPktBuffer, tmpPktCount);
		pktBytesReceived += tmpPktCount;
		tmpPktCount=0;
		return PKT_BUSY;
	}
}

/*
	 footer needs to be the next 4 bytes
 */
int findFooter(void)
{
	if(tmpPktCount < 4) {
		/* if not enough bytes then return */
		return PKT_BUSY;
	} else {
		/* 
			 if there are enough bytes to include the footer then read it 
		 */
		uint32_t val = *(uint32_t*)tmpPktBuffer;
		if(val == footer) {
			ROS_INFO_STREAM("footer found, pkt success!");
		} else {
			ROS_INFO_STREAM("footer invalid.  read "<< hex << val << " expected " << hex << footer << " pkt FAILED!");
		}
	}
	pktBytesReceived=0;
	tmpPktCount = 0;
	return PKT_DONE;
}
/*
	 called when serial data arrives.  First append data to temporary buffer which may have old data from
	 partial read last time.  then call the appropriate function based on state of packet.
 */
int parse_pkt(uint8_t *data, size_t numBytes) 
{
	memcpy(&tmpPktBuffer[tmpPktCount] , data, numBytes);
	tmpPktCount += numBytes;
	if(pktBytesReceived == 0)  { 
		return findPktStart();
	} else if(pktBytesReceived == 4) { // expecting pkt size
		return findImageSize();
	} else if(pktBytesReceived+8 < imageSize) {
		return collectData();
	} else {
		return findFooter();
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
	uint8_t serial_buffer[MAX_PKT_SIZE];
	size_t numBytes;
	int bytesRemaining = 0;
	while(ros::ok()) {
		numBytes = serial_port->read(serial_buffer, MAX_PKT_SIZE);
		if(numBytes > 0) {
			//ROS_INFO_STREAM("Serial Rx "<<numBytes<<" bytes");
			parse_pkt(serial_buffer, numBytes);
		}
		ros::spinOnce();	
	}
	return 0;
}
