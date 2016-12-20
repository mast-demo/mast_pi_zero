/************************
node to interface camera

tried to used raspicam_node whcih should be faster, but could 
not compile all of the dependencies.  Will try to improve later.

currently very slow and CU intensive due to openCV jpeg compression.
***********************/

#include <iostream>
#include <cstdlib>
#include <fstream>
#include "raspicam_cv.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>
#include <mars_msgs/mars_camera.h>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main ( int argc,char **argv ) {

	ros::init(argc,argv,"raspicam");
	ros::NodeHandle n;
	ros::Publisher image_pub = n.advertise<sensor_msgs::CompressedImage>(
		"raspicam/compressed",1000);
	ros::NodeHandle private_node_handle("~");
	float framerate;
	int quality;
	private_node_handle.param("framerate", framerate, float(10));
	private_node_handle.param("quality", quality, int(50));
	ros::Rate loop_rate(framerate);

	raspicam::RaspiCam_Cv Camera;
	cout<<"Connecting to camera"<<endl;
	if ( !Camera.open() ) {
		cerr<<"Error opening camera"<<endl;
		return -1;
	}
	cout<<"Connected to camera ="<<Camera.getId() <<endl;
	cout << "Framerate = " << framerate << " quality = " << quality << endl;
	cv_bridge::CvImage cv_image;
	cv_image.encoding = "rgb8";
	sensor_msgs::CompressedImage image_msg;
	image_msg.format = std::string("jpeg");
	int i=0;
	cv::Size cameraResolution(2592,1944);

	int count = 0;
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_params.push_back(quality);
	cv::Size sz(324*2,243*2);
	while(ros::ok()) 
	{
		Camera.grab();
		Camera.retrieve ( cv_image.image );
		cv::resize(cv_image.image,cv_image.image,sz);
		cv::cvtColor(cv_image.image, cv_image.image, COLOR_BGR2RGB);
		cv::imencode(".jpg",cv_image.image, image_msg.data, 
				compression_params);
		image_pub.publish(image_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
	Camera.release();
	return 0;
}
