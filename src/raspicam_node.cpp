#include <stdio.h>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"


#include "raspicam.h"
using namespace std;
void saveImage ( string filepath,unsigned char *data,raspicam::RaspiCam &Camera ) {
    std::ofstream outFile ( filepath.c_str(),std::ios::binary );
    if ( Camera.getFormat()==raspicam::RASPICAM_FORMAT_BGR ||  Camera.getFormat()==raspicam::RASPICAM_FORMAT_RGB ) {
        outFile<<"P6\n";
    } else if ( Camera.getFormat()==raspicam::RASPICAM_FORMAT_GRAY ) {
        outFile<<"P5\n";
    } else if ( Camera.getFormat()==raspicam::RASPICAM_FORMAT_YUV420 ) { //made up format
        outFile<<"P7\n";
    }
    outFile<<Camera.getWidth() <<" "<<Camera.getHeight() <<" 255\n";
    outFile.write ( ( char* ) data,Camera.getImageBufferSize() );
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"raspicam");
	ros::NodeHandle n;
	ros::Publisher raspicam_pub = n.advertise<sensor_msgs::Image>("raspicam",1000);
	ros::Rate loop_rate(1);

	raspicam::RaspiCam Camera;
	cout<<"Connecting to camera"<<endl;

        Camera.setWidth( 1280 );
	Camera.setHeight(960);
	Camera.setBrightness(50);
	Camera.setSharpness(0);
	Camera.setContrast(0);
	Camera.setSaturation(0);
	Camera.setShutterSpeed(0);
	Camera.setISO(400);
	Camera.setExposureCompensation(0);
	Camera.setExposure(raspicam::RASPICAM_EXPOSURE_AUTO);
	Camera.setAWB_RB(1,1);

	if ( !Camera.open() ) {
		cerr<<"Error opening camera"<<endl;
		return -1;
	}
	cout<<"Connected to camera ="<<Camera.getId() <<" bufs="<<Camera.getImageBufferSize( )<<endl;
	unsigned char *data=new unsigned char[  Camera.getImageBufferSize( )];
	while(ros::ok()) 
	{
		Camera.grab();
		Camera.retrieve ( data );
		//saveImage ( "image.jpg",data,Camera );
		sensor_msgs::Image img_msg;
		raspicam_pub.publish(img_msg);
	}

	Camera.release();
	return 0;
}
