// steering_algorithm.h header file //
// wsn; Feb, 2015
// include this file in "steering_algorithm.cpp"

#ifndef STEERING_ALGORITHM_H_
#define STEERING_ALGORITHM_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

//Eigen is useful for linear algebra
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>

//Include the steering vel profiler to control velocity
#include <SteerVelProfiler.h>

// define a class, including a constructor, member variables and member functions
class SteeringController
{
public:
    SteeringController(ros::NodeHandle* nodehandle, SteerVelProfiler* steerProfiler); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
    void my_clever_steering_algorithm(); // here is the heart of it...use odom state and desired state to compute twist command, and publish it
    //this may need to incorporate lateral error as well
    double compute_controller_speed(double trip_dist_err);
    double compute_controller_omega(double trip_dist_err,
        double heading_err, double lateral_err);
    double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);   
    double min_dang(double dang);  
    double sat(double x);
    // some utilities:
    //signum function: define this one in-line
    double sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
    }
private:
    
    // The steering velocity profiler to move the robot accordingly.
    SteerVelProfiler steeringProfiler_;
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    ros::Subscriber odom_subscriber_; //these will be set up within the class constructor, hiding these ugly details
    ros::Subscriber des_state_subscriber_;
    
    ros::Publisher cmd_publisher_; // = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
    ros::Publisher cmd_publisher2_; // = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped",1);
    ros::Publisher steering_errs_publisher_;
    
    ros::ServiceServer simple_service_; //a do-nothing service--but easily modified to be useful

    geometry_msgs::Twist twist_cmd_;
    geometry_msgs::TwistStamped twist_cmd2_;    
    double current_speed_des_;
    double current_omega_des_;
    double dt_;
   
    //Last time odom callback took place
    ros::Time lastCallbackTime;
    

    //state values from odometry; these will get filled in by odom callback
    nav_msgs::Odometry current_odom_; // fill in these objects from callbacks
    geometry_msgs::Pose odom_pose_;    
    double odom_vel_;
    double odom_omega_;
    double odom_x_;
    double odom_y_;
    double odom_phi_;
    geometry_msgs::Quaternion odom_quat_; 
    Eigen::Vector2d odom_xy_vec_;
    
    //state values from desired state; these will get filled in by desStateCallback
    nav_msgs::Odometry des_state_; 
    geometry_msgs::Pose des_state_pose_;    
    double des_state_vel_;
    double des_state_omega_;
    double des_state_x_;
    double des_state_y_;
    double des_state_phi_;
    geometry_msgs::Quaternion des_state_quat_;  
    Eigen::Vector2d des_xy_vec_;    
    
    // message to hold/publish steering performance data
    std_msgs::Float32MultiArray steering_errs_;
        
    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    void initializeServices();
    void initializeSteeringProfiler();
    void update_steering_profiler();
    double compute_omega_profile(double newPhi);
    void minimizeHeadingError(double heading_err, double lateral_err);
 
    void odomCallback(const nav_msgs::Odometry& odom_rcvd);
    void desStateCallback(const nav_msgs::Odometry& des_state_rcvd);    
        
    //prototype for callback for example service
    // might want this for software "halt" command--but rename it appropriately
    bool serviceCallback(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response);
}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef
