//example path sender
// wsn, Feb 2015
// test node compatible with example_des_state_generator.cpp;
// transmits a hard-coded path to desired-state generator node via service "appendPathService"
// the message must contain a nav_msgs/Path object

#include <stdlib.h>
#include <string>
#include <vector>
#include <queue>
#include <iostream>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <interactive_markers/interactive_marker_server.h>

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>

#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include <cwru_srv/path_service_message.h>

bool path_trigger = false;
bool first = true;
double marker_x,marker_y,marker_phi;
double last_phi = 0.0;
double d_phi = 0.0;

/**
 * Converts a quaternion to phi values for planar motion.
 * 
 * @param quaternion - the quaternion to convert to a phi for planar motion
 */
double convertPlanarQuat2Phi(double quat_z, double quat_w) {
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

void pathMarkerListenerCB(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
    marker_x = feedback->pose.position.x;
    marker_y = feedback->pose.position.y;
    marker_phi = convertPlanarQuat2Phi(feedback->pose.orientation.z, feedback->pose.orientation.w);
}

bool triggerService(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response) {
    ROS_INFO("path service callback activated");
    // boring, but valid response info
    response.resp = true;
    //inform "main" that we have a new goal!
    path_trigger = true;
    return true;
}


//utility to convert from heading to quaternion, planar motion
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return (quaternion);
}



//utility to fill a Pose object from planar x,y,phi info
geometry_msgs::Pose xyPhi2Pose(double x, double y, double phi) {
    geometry_msgs::Pose pose; // a pose object to populate
    pose.orientation = convertPlanarPhi2Quaternion(phi); // convert from heading to corresponding quaternion
    pose.position.x = x; // keep the robot on the ground!
    pose.position.y = y; // keep the robot on the ground!    
    pose.position.z = 0.0; // keep the robot on the ground!  
    return pose;
}

int main(int argc, char **argv) {
    double dt = 0.01;
    ros::init(argc, argv, "path_sender"); // name of this node 

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<cwru_srv::path_service_message>("appendPathService");
    ros::ServiceClient flushService = nh.serviceClient<cwru_srv::simple_bool_service_message>("flushPathService");
    ros::ServiceServer service = nh.advertiseService("path_trigger", triggerService);
    ros::Subscriber sub_im = nh.subscribe("path_marker/feedback", 1, pathMarkerListenerCB);

    cwru_srv::path_service_message path_message;
    cwru_srv::simple_bool_service_message flush_message;
    geometry_msgs::PoseStamped vertex; // this contains a header and a pose; the pose contains a point and a quaternion
    double x, y, phi;

                  //a timer for desired rate, e.g. 50Hz
    ros::Rate sleep_timer(50);

    vertex.header.stamp = ros::Time::now(); // look up the time and put it in the header; use same time stamp for all vertices in this path
    vertex.header.frame_id = "odom";
//vertex.header.frame_id = "map"; // specify this, so tf will know how to transform it

    // fill in the interesting data: (x,y) and phi = location and heading
    //vertex 1:
    //x=1.0;
    //y=2.0;
    //phi=0.123;
    //ROS_INFO("vertex: x,y,phi = %f, %f %f",x,y,phi);
    //vertex.pose = xyPhi2Pose(x,y,phi); //x,y,phi
    //path_message.request.path.poses.push_back(vertex);
    //vertex 2:
    //x=3.0;
    //y=4.0;
    //phi=0.567;
    //ROS_INFO("vertex: x,y,phi = %f, %f %f",x,y,phi);
    //vertex.pose = xyPhi2Pose(x,y,phi); //x,y,phi
    //path_message.request.path.poses.push_back(vertex);
    //vertex 3:
    //x=5.0;
    //y=6.0;
    //phi=2.345;
    //ROS_INFO("vertex: x,y,phi = %f, %f %f",x,y,phi);
    //vertex.pose = xyPhi2Pose(x,y,phi); //x,y,phi
    //path_message.request.path.poses.push_back(vertex);


    // fill in the interesting data: (x,y) and phi = location and heading
//    //vertex 1:
//    x = 23.547907;
//    y = 17.729401;
//    phi = -1.5; //this is just the end rotation we want
//    ROS_INFO("vertex: x,y,phi = %f, %f %f", x, y, phi);
//    vertex.pose = xyPhi2Pose(x, y, phi); //x,y,phi
//    path_message.request.path.poses.push_back(vertex);
//
//    //vertex 2:
//    x = 15;
//    y = 26;
//    phi = -1.5;
//    ROS_INFO("vertex: x,y,phi = %f, %f %f", x, y, phi);
//    vertex.pose = xyPhi2Pose(x, y, phi); //x,y,phi  
//    path_message.request.path.poses.push_back(vertex);
//
//    //vertex 3:
//    x = 21.034620;
//    y = 32.313902;
//    phi = 0.0;
//    ROS_INFO("vertex: x,y,phi = %f, %f %f", x, y, phi);
//    vertex.pose = xyPhi2Pose(x, y, phi); //x,y,phi  
//    path_message.request.path.poses.push_back(vertex);
    
    //vertex 1:
   // x = 3;
   // y = 3;
   // phi = 0; //this is just the end rotation we want
   // ROS_INFO("vertex: x,y,phi = %f, %f %f", x, y, phi);
   // vertex.pose = xyPhi2Pose(x, y, phi); //x,y,phi
   // path_message.request.path.poses.push_back(vertex);

//    //vertex 2:
//    x = 5;
//    y = 5;
//    phi = 0.0;
//    ROS_INFO("vertex: x,y,phi = %f, %f %f", x, y, phi);
//    vertex.pose = xyPhi2Pose(x, y, phi); //x,y,phi  
//    path_message.request.path.poses.push_back(vertex);
//
//    //vertex 3:
//    x = 0;
//    y = 0;
//    phi = 0.0;
//    ROS_INFO("vertex: x,y,phi = %f, %f %f", x, y, phi);
//    vertex.pose = xyPhi2Pose(x, y, phi); //x,y,phi  
//    path_message.request.path.poses.push_back(vertex);

    //path for gazebo since gazebo starts at odom x:0 y:0
    //vertex 1
    //x = -3.452093;
    //y = -3.270599;
//    x = 4.75539;
// 	//x = 5.5;    
// 	y = 0;
//     phi = -1.57; //this is just the end rotation we want
//     ROS_INFO("vertex: x,y,phi = %f, %f %f", x, y, phi);
//     vertex.pose = xyPhi2Pose(x, y, phi); //x,y,phi
//     path_message.request.path.poses.push_back(vertex);

//     //vertex 2
//     //x = -12;
//     //y = 5;
//     x = 4.75539;
// //	x = 5.5;    
// 	y = -12.069;
//     phi = -3.14; //this is just the end rotation we want
//     ROS_INFO("vertex: x,y,phi = %f, %f %f", x, y, phi);
//     vertex.pose = xyPhi2Pose(x, y, phi); //x,y,phi
//     path_message.request.path.poses.push_back(vertex);
//     //transmit this path message:

//     //vetex 3 
//     //x = -5.96538;
//     //y = 11.313902;
//     x = -3.97857;
// //	x=1;
//     y = -12.069;
//     phi = -3.14; //this is just the end rotation we want
//     ROS_INFO("vertex: x,y,phi = %f, %f %f", x, y, phi);
//     vertex.pose = xyPhi2Pose(x, y, phi); //x,y,phi
//     path_message.request.path.poses.push_back(vertex);



/* amcl points
    x = 7.65263438466;
    y = 14.5143860241;
    phi = -0.918061444575; 
    ROS_INFO("vertex: x,y,phi = %f, %f %f", x, y, phi);
    vertex.pose = xyPhi2Pose(x, y, phi); //x,y,phi
    path_message.request.path.poses.push_back(vertex);

    x = 5.09993113083;
    y = 11.9590104928;
    phi = 0.931375845707; 
    ROS_INFO("vertex: x,y,phi = %f, %f %f", x, y, phi);
    vertex.pose = xyPhi2Pose(x, y, phi); //x,y,phi
    path_message.request.path.poses.push_back(vertex);

    x = -3.33305079521;
    y = 20.7515996994;
    phi = 0.927774077288; 
    ROS_INFO("vertex: x,y,phi = %f, %f %f", x, y, phi);
    vertex.pose = xyPhi2Pose(x, y, phi); //x,y,phi
    path_message.request.path.poses.push_back(vertex);

    x = 0.652917432199;
    y = 24.4962222805;
    phi = -0.920845477744; 
    ROS_INFO("vertex: x,y,phi = %f, %f %f", x, y, phi);
    vertex.pose = xyPhi2Pose(x, y, phi); //x,y,phi
    path_message.request.path.poses.push_back(vertex);

    x = -3.68119336817;
    y = 20.442523328;
    phi = -0.355685201734; 
    ROS_INFO("vertex: x,y,phi = %f, %f %f", x, y, phi);
    vertex.pose = xyPhi2Pose(x, y, phi); //x,y,phi
    path_message.request.path.poses.push_back(vertex);

    x = 5.64358315338;
    y = 11.6388698361;
    phi = -0.923632539266; 
    ROS_INFO("vertex: x,y,phi = %f, %f %f", x, y, phi);
    vertex.pose = xyPhi2Pose(x, y, phi); //x,y,phi
    path_message.request.path.poses.push_back(vertex);
*/

// from now on, tfListener will keep track of transforms
//    path sender loop detects if service is triggered to add the path to marker or not
    while(ros::ok()){
        ros::spinOnce();
        if(path_trigger){
            //flush the path queue
            flush_message.request.req = true;
            if (flushService.call(flush_message)) {
                ROS_INFO("server successfully called for flushing path queue");
            }  else {
                ROS_ERROR("Failed to call service for flushing path queue");
                return 1;
            }
            path_message.request.path.poses.clear();

            if (first){
                last_phi = marker_phi;
                d_phi = marker_phi;
                first = false;
            } else {
                d_phi = marker_phi - last_phi;
            }
            //push point
            x = marker_x;
            y = marker_y;
            phi = 0; //marker_phi;//d_phi;
            vertex.pose = xyPhi2Pose(x, y, phi); //x,y,phi  
            ROS_INFO("vertex: x,y,phi = %f, %f %f", x, y, phi);
            path_message.request.path.poses.push_back(vertex);
            last_phi = marker_phi;

            path_trigger = false;
            if (client.call(path_message)) {
                ROS_INFO("server successfully called for pushing path points to queue");
            }  else {
                ROS_ERROR("Failed to call service for pushing path points to queue");
                return 1;
            }
         } 
        sleep_timer.sleep();
    }
    // if (client.call(path_message)) {
    //     ROS_INFO("got ack from server");
    // } else {
    //              ROS_ERROR("Failed to call service lookup_by_name");
    //             return 1;
    //         }
    return 0;
}
