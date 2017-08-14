/***
  subscribe to image and pose messages and store the images and corresponding
  poses to a rosbag file
  publish the storage used
 */
#include <ros/ros.h>
#include "Storage.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "com");
  ros::NodeHandle n;
  Storage storage(n);
  ros::spin();
  return 0;
}
