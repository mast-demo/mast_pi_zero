#!/bin/bash
export ROS_IP=`hostname -I`
roslaunch mast_pi_zero mast_pi_zero.launch name:=`hostname`
