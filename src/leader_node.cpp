#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <ros/console.h>
#include <geometry_msgs/Pose.h>
#include <cmath>

ros::Publisher cmd_vel_pub;
ros::Subscriber sub;

float x,y;
float e_l,e_y;
float kp_l = 0.25;
float kp_y = 0.7;
float yaw_d = 0; 
float r = 0.4;
int n = 0;
int no_of_waypoints = 4;
float pi = 3.14159;

/*
float waypoints_x[] = {-0.5,  -0.3536,   0,    0.3536,   0.5,  0.3536,     0, -0.3536};
float waypoints_y[] = {0   ,  0.3536,  0.5,   0.3536,   0,  -0.3536, -0.5, -0.3536 };
*/

//float waypoints_x[] = {-r,  -r*cos(pi/6),    -r*cos(pi/3),    0,   r*cos(pi/3),  r*cos(pi/6),     r,  r*cos(pi/6), r*cos(pi/3), 0, -r*cos(pi/3), -r*cos(pi/6) };
//float waypoints_y[] = {0   ,  r*sin(pi/6),  r*sin(pi/3),   r,   r*sin(pi/3), r*sin(pi/6), 0, -r*sin(pi/6),-r*sin(pi/3), -r, -r*sin(pi/3), -r*sin(pi/6) };

float waypoints_x[] = {0.2,-0.2,-0.2,0.2};
float waypoints_y[] = {0.1,0.1,-0.1,-0.1};

float x_d = waypoints_x[n];
float y_d = waypoints_y[n];

std::string follower;


double roll, pitch, yaw;

void chatterCallback(const geometry_msgs::Pose& msg);

int main(int argc, char** argv){
  ros::init(argc, argv, "leader");
  ROS_INFO("Publisher");
  ros::NodeHandle node;

  ros::param::get("~follower", follower);

  cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  sub = node.subscribe("/operator_cmd"+follower, 1000, chatterCallback);

  ROS_INFO("Waking up Leader Node.");
  ros::spin(); // Run until interupted 
};

void chatterCallback(const geometry_msgs::Pose& msg)
{
   ROS_INFO("Updating Leader Node");

    geometry_msgs::Twist base_cmd;

	    x = msg.position.x;
	    y = msg.position.y;

	    //tf::Quaternion q(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);

	    //tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	    roll = msg.orientation.x;
	    pitch = msg.orientation.y;
            yaw = msg.orientation.z;

	    ROS_INFO("x: %f,y: %f,yaw: %f",x,y,yaw);


	    yaw_d = -1*atan2((x_d-x) , (y_d-y));
	    //Controller
	    e_l = sqrt((x_d - x)*(x_d - x)+(y_d-y)*(y_d-y));

	  if ( (yaw_d-yaw) < -3*pi/2 || (yaw_d-yaw) > 3*pi/2 )
	    e_y = -0.2*(yaw_d-yaw);
	  else
	    e_y = 1*(yaw_d-yaw);


	    ROS_INFO("e_l: %f",e_l);
	    ROS_INFO("e_yaw: %f",e_y);


	    //check if Way-point reached
	    ROS_INFO("x_d: %f,y_d: %f",x_d,y_d);
	    ROS_INFO("yaw_d: %f",yaw_d);


	
	    if(e_l < 0.0065 && e_l > -0.0065){
		    ROS_INFO("abs(e_l) < 0.045");



		    ROS_INFO("Desired positions updated.");
	        ROS_INFO("x_d: %f,y_d: %f",x_d,y_d);
	        ROS_INFO("yaw_d: %f",yaw_d);
		
		    base_cmd.linear.x = 0.00;
		    base_cmd.angular.z = 0.00;

 		    n = n+1;
		    if(n == no_of_waypoints){
		        n = 0;
		    }
		    ROS_INFO("n: %i",n);

		    x_d = waypoints_x[n];
		    y_d = waypoints_y[n];	
	    }
	   else {
	      //we will be sending commands of type "twist"
	      base_cmd.linear.x = 0.7;//kp_l*e_l;
	      base_cmd.angular.z = kp_y*e_y;
	    }

	    ROS_INFO("Linear Cmd: %f",base_cmd.linear.x);
	    ROS_INFO("Angular Cmd: %f",base_cmd.angular.z);

	    cmd_vel_pub.publish(base_cmd);
	    ROS_INFO("n: %i",n);
  

}
