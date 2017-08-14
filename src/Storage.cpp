/** \file 
 */
//#include <mast_ros/StorageControl.h>
#include "Storage.h"
void Storage::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if(isOpen && recordEnable) {
    ros::Time t = ros::Time::now();
    bag.write("image", t, *msg);
    bag.write("pose", t, pose);
    used = bag.getSize();
    remaining = (capacity-used)/capacity*100.0;
    storageMsg.data = remaining;
    storagePub.publish(storageMsg);
    if(remaining<10) {
      ROS_INFO_STREAM("Storage at "<<remaining<<"%");
    }
  }
}
void Storage::open(void)
{
  ros::Time t = ros::Time::now();
  std::ostringstream ss;
  ss << filePath << fileName << "_" << t.sec << ".bag";
  ROS_INFO_STREAM("saving data to " << ss.str());
  bag.open(ss.str().c_str(), rosbag::bagmode::Write);
  isOpen = true;
}
void Storage::close(void)
{
  if(isOpen) {
    bag.close();
    isOpen = false;
		ROS_INFO_STREAM("Closing file");
  }
}
void Storage::openCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if(msg->data) {
    open();
  } else {
    close();
  }
}

void Storage::recordCallback(const std_msgs::Bool::ConstPtr& msg)
{
	recordEnable = msg->data;
	ROS_INFO_STREAM("Recording = "<<recordEnable);
}

void Storage::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  pose = *msg;
}
Storage::Storage(ros::NodeHandle _n)
{
  n = _n; 
  ros::NodeHandle nh("~");
  nh.param<std::string>("imageTopic", imageTopic, "camera/image");
  nh.param<std::string>("poseTopic", poseTopic, "pose");
  nh.param<std::string>("openTopic", openTopic, "open");
  nh.param<std::string>("recordTopic", recordTopic, "record");
  nh.param<std::string>("fileName", fileName, "rosfile");
  nh.param<std::string>("filePath", filePath, ".");
  image_transport::ImageTransport it(n);
  sub = it.subscribe(imageTopic, 1, &Storage::imageCallback, this);
  poseSub = n.subscribe(poseTopic, 1, &Storage::poseCallback, this);
  openSub = n.subscribe(openTopic, 1, &Storage::openCallback, this);
  recordSub = n.subscribe(recordTopic, 1, &Storage::recordCallback, this);
	recordEnable = isOpen = false;
}
