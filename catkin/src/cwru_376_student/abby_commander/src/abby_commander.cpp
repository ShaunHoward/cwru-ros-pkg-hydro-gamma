/* 
 * File:   abby_commander.cpp
 * Author: smh150
 *
 * Created on April 16, 2015, 2:46 PM
 */

#include <ros/ros.h>
#include <cstdlib>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
// this is a pre-defined service message, contained in shared "cwru_srv" package
#include <cwru_srv/simple_int_service_message.h> 

using namespace std;

// estimated height of cylinder
const double H_CYLINDER = 0.3; 
double can_z_coord = 0.0;
bool inGoalPose = false;
int g_fit_z = -1;
bool g_trigger = false;
double curr_arm_z = 0.0;

//define some processing modes; set these interactively via service
const int FINE = 0;
const int FINER = 1;
const int FINEST = 2;

/**
 * Use this service to set processing modes interactively.
 */
bool triggerService(cwru_srv::simple_int_service_messageRequest& request, cwru_srv::simple_int_service_messageResponse& response) {
    ROS_INFO("mode select service callback activated");
    
    // boring, but valid response info
    response.resp = true; 
    g_fit_z = request.req;
    
    //signal that we received a request; trigger a response
    g_trigger = true; 
    cout << "Mode set to: " << g_fit_z << endl;
    return true;
}

// static void alignFlangeWithCylinder(){
    
    
// }

void canZCB(const std_msgs::Float32::ConstPtr& can_z){
    //get z of top of can
    can_z_coord = can_z->data + H_CYLINDER;
}

void armZCB(const std_msgs::Float32::ConstPtr& arm_z){
    //get z of tool flange currently
    curr_arm_z = arm_z->data;
}

// void nextPoseCB(const geometry_msgs::Pose& feedback) {
//        ROS_INFO_STREAM(feedback->marker_name << " is now at x: "
//                << feedback->pose.position.x << ", y: " << feedback->pose.position.y
//                << ", z: " << feedback->pose.position.z << ", quatx: " << feedback->pose.orientation.x 
//                << ", quaty: " << feedback->pose.orientation.y << ", quatz: " << feedback->pose.orientation.z << ", quatw: " << feedback->pose.orientation.w);
//        //copy to global vars:
//        g_p[0] = feedback->pose.position.x;
//        g_p[1] = feedback->pose.position.y;
//        g_p[2] = feedback->pose.position.z;
//        g_quat.x() = feedback->pose.orientation.x;
//        g_quat.y() = feedback->pose.orientation.y;
//        g_quat.z() = feedback->pose.orientation.z;
//        g_quat.w() = feedback->pose.orientation.w;
//        g_R = g_quat.matrix();
// }

// void goalPoseCB(const std_msgs::Bool::ConstPtr& at_goal_pose){
//     if (at_goal_pose->data == true){
//         ROS_INFO("The arm is at its goal pose");
//         inGoalPose = true;
//     } else {
//         ROS_INFO("The arm is not at its goal pose");
//         inGoalPose = false;
//     }
// }

/*
 * 
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "abby_vertical_commander"); // this will be the node name;
    ros::NodeHandle nh;
    ros::Rate sleep_timer(10.0); //10Hz update rate 
    //subscribe to goal pose message from im interface
   // ros::Subscriber sub_goal = nh.subscribe("at_goal_pose", 1, goalPoseCB);
    ros::Subscriber sub_can_z = nh.subscribe("can_z", 1, canZCB);
    ros::Subscriber sub_arm_z = nh.subscribe("arm_z", 1, armZCB);
    ros::ServiceServer service = nh.advertiseService("lower_trigger", triggerService);
    ros::Publisher pub_z = nh.advertise<std_msgs::Float32>("new_arm_z", 1);
    std_msgs::Float32 new_arm_z;
    ROS_INFO("Initialized the vertical abby commander");
    //ros::Subscriber sub_goal_pose = nh.subscribe("")
    while(ros::ok()){
        ROS_INFO("In abby commander loop");
        ros::spinOnce();
        if (g_trigger) {
            ROS_INFO("g_trigger enabled");
            g_trigger = false; // reset the trigger
            
            //switch to fit according to closeness of flange to can
            //0 for fine, 1 for finer, 2 for finest
            switch (g_fit_z) {
                case FINE:
                    curr_arm_z -= .1;
                    break;
                case FINER:
                    curr_arm_z -= .01;
                    break;
                case FINEST:
                    curr_arm_z -= .001;
                    break;
            }
         }

         ROS_INFO("Modified arm z is: %f", curr_arm_z);
        
        //publish the new z to update the arm pose
        new_arm_z.data = curr_arm_z;
        pub_z.publish(new_arm_z);
        
//        //align arm with top of can by publishing the desired
//        if (!inGoalPose){
//            
//        }
        sleep_timer.sleep();
    }
    return 0;
}

