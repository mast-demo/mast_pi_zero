#!/bin/bash
#give network time to DHCP.  otherwise ROS_IP is empty and ROS will not work on network
export ROS_IP=`hostname -I`
if [[ -z $ROS_IP ]] ; then
	echo "ROS_IP is empty, no network?"
	echo "Sleeping and trying again"
	sleep 10s
	export ROS_IP=`hostname -I`
	echo "ROS_IP = $ROS_IP"
else
	echo "ROS_IP = $ROS_IP"
fi
if [[ ! -z $ROS_IP ]] ; then
	source ~/ros_catkin_ws/devel/setup.bash
	roslaunch mast_pi_zero mast_pi_zero.launch name:=`hostname`
fi
