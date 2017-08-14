/*!
  @file
 */
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <mast_ros/StorageControl.h>
#include <image_transport/image_transport.h>

class Storage {
  public:
    ros::NodeHandle n;
    Storage(ros::NodeHandle n);
    float capacity; // storage bytes
    float used;			// bytes currently used
    float remaining; // percent space available

    ros::Publisher storagePub;
    rosbag::Bag bag;
    geometry_msgs::PoseWithCovarianceStamped pose;
    std_msgs::Float32 storageMsg;
    std::string imageTopic, poseTopic,storageTopic, openTopic, recordTopic; 
    std::string fileName, filePath, configFile;
    bool isOpen;
		bool recordEnable;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void open(void);
    void close(void);
    void poseCallback(
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
		void openCallback(const std_msgs::Bool::ConstPtr& msg);
		void recordCallback(const std_msgs::Bool::ConstPtr& msg);
    ros::Subscriber poseSub;
    ros::Subscriber openSub;
    ros::Subscriber recordSub;
  image_transport::Subscriber sub;
};
