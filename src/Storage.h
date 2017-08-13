/*!
  @file
 */
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <rosbag/bag.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mast_ros/StorageControl.h>
#include <image_transport/image_transport.h>

class Storage {
  public:
    ros::NodeHandle n;
    Storage(ros::NodeHandle n);
    void load(YAML::Node y);
    float capacity; // storage bytes
    float used;			// bytes currently used
    float remaining; // percent space available

    ros::Publisher storagePub;
    rosbag::Bag bag;
    geometry_msgs::PoseWithCovarianceStamped pose;
    std_msgs::Float32 storageMsg;
    std::string imageTopic, poseTopic,storageTopic; 
    std::string fileName, filePath, configFile;
    bool isOpen=false;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void open(void);
    void close(void);
    bool control(mast_ros::StorageControl::Request &req,
        mast_ros::StorageControl::Response & res);
    void poseCallback(
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    ros::Subscriber poseSub;
    ros::ServiceServer controlSrv;
  image_transport::Subscriber sub;
};
