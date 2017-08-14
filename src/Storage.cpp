/** \file 
 */
#include "mast.h"
#include "mast_ros/CheckConnection.h"
#include <mast_ros/StorageControl.h>
#include "Storage.h"
void Storage::load(YAML::Node y)
{
  capacity = y["capacity"].as<double>();
  used = y["used"].as<double>();
}
/*
   bool Agent::startBag(void)
   {
   mast_ros::StorageControl srv;
   srv.request.command = "start";
   if(!storageClient.call(srv)) {
   ROS_WARN_STREAM("Could not start recorder: "<<srv.response.ok);
   bagOpen = false;
   } else {
   bagOpen = true;
   }
   }
   bool Agent::stopBag(void)
   {
   bagOpen = false;
   mast_ros::StorageControl srv;
   srv.request.command = "stop";
   if(!storageClient.call(srv)) {
   ROS_WARN_STREAM("Could not stop recorder: "<<srv.response.ok);
   } else {
   bagOpen = false;
   }
   }
 */
void Storage::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if(isOpen) {
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
  }
}
bool Storage::control(mast_ros::StorageControl::Request &req,
    mast_ros::StorageControl::Response & res)
{
  if(req.command == "start") {
    open();
    res.ok = true;
    return true;
  } else if(req.command == "stop") {
    close();
    res.ok = true;
    return true;
  } else {
    ROS_ERROR_STREAM(" Invalid Command "<<req.command);
    res.ok = false;
    return false;
  }
}
void Storage::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  pose = *msg;
}
Storage::Storage(ros::NodeHandle _n)
{
  n = _n; 
  ros::NodeHandle nh("~");
  nh.param<std::string>("configFile", configFile, "config.yaml");
  nh.param<std::string>("imageTopic", imageTopic, "camera/image");
  nh.param<std::string>("poseTopic", poseTopic, "pose");
  nh.param<std::string>("storageTopic", storageTopic, "storage");
  nh.param<std::string>("fileName", fileName, "rosfile");
  nh.param<std::string>("filePath", filePath, "/tmp/");
  image_transport::ImageTransport it(n);
  sub = it.subscribe(imageTopic, 1, &Storage::imageCallback, this);
  poseSub = n.subscribe(poseTopic, 1, &Storage::poseCallback, this);
  storagePub = n.advertise<std_msgs::Float32>(storageTopic,1);
  controlSrv = n.advertiseService(
      "storage_node/control", &Storage::control, this);
  YAML::Node config = YAML::LoadFile(configFile);
  load(config["storage"]);
  ROS_INFO_STREAM("===================storage capacity = "<< capacity);
}
