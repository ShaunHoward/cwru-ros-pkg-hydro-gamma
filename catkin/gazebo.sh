#!/bin/bash
#Runs Gazebo with roscore and cwrubot
#When Ctrl+C is pressed it will close all processes and killall gzserver 
trap clean_up SIGHUP SIGINT SIGTERM

function clean_up {
	kill 0
	killall gzserver
}
echo "Running roscore"
roscore &
sleep 1
echo "Running gazebo"
rosrun gazebo_ros gazebo &
sleep 3
echo "Running cwrubot"
roslaunch cwru_urdf cwruBot.launch &
wait




