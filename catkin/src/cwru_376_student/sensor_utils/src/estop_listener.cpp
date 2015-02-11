#include <ros/ros.h>
#include <cwru_base/Pose.h>
#include <iostream>
#include <fstream>
#include <cwru_base/cRIOSensors.h>
#include <std_msgs/Bool.h>

bool motorsEnabled;
ros::Publisher estopPublisher;

/**
 * Publishes an estop message as to whether the estop is enabled or not. If
 * the motors on the robot are not enabled, then estop is on according to this
 * message. 
 * In order to call the estop command from terminal, use the following line:
 * rostopic pub -r 10 /motors_enabled std_msgs/Bool False
 * @param motorsEnabled - a boolean message about whether the robot motors are enabled
 */
void estopCallback(const std_msgs::Bool::ConstPtr& motorsEnabled) {
    if (motorsEnabled->data == true)
        ROS_INFO("Estop is off."); // means motors are ENABLED
    else if (motorsEnabled->data == false)
        ROS_INFO("Estop is on."); // means motors are ENABLED

    std_msgs::Bool estopMessage;
    estopMessage.data = !(motorsEnabled->data);
    estopPublisher.publish(estopMessage);
}

/**
 * Initializes a new ROS Estop listener node and subscribes to the motors_enabled
 * message published by the robot. This subscription causes a callback call that 
 * will publish an estop message about whether the motors are enabled.
 * 
 * @param argc - arguments for initialization
 * @param argv - arguments for initialization
 * @return the exit code of the program
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "estop_listener");
    ros::NodeHandle nodeHandle;
    ros::Publisher publisher = nodeHandle.advertise<std_msgs::Bool>("estop_listener", 1);
    estopPublisher = publisher;
    ros::Subscriber subscriber = nodeHandle.subscribe("motors_enabled", 1, estopCallback);
    ros::spin();
    return 0;
}

