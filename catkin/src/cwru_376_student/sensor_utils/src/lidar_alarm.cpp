#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <float.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message time


const double MIN_SAFE_DISTANCE = 0.5; // set alarm if anything is within 0.5m of the front of robot

//Values that are utilized for laser callback
float ping_dist_in_front_ = 3.0; // global var to hold length of a SINGLE LIDAR ping--in front
float closest_ping = 3.0;
bool firstRun = true;
float angle_min_ = 0.0;
float angle_max_ = 0.0;
float angle_increment_ = 0.0;
float range_min_ = 0.0;
float range_max_ = 0.0;

int min_ping_index = 0;
int max_ping_index = 0;
float start_min_angle = -.52333333;
float end_max_angle = .52333333;
bool laser_alarm_=false;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;

/**
 * Determines the closest ping to the robot according to LIDAR ping ranges.
 * The smallest found ping distance value is returned.
 */
float getClosestPingDist(std::vector<float> ping_ranges, int minIndex, int maxIndex){
  float *pings = &ping_ranges[0];  
  float smallest = ping_ranges[0];
  for (int i = minIndex; i <= maxIndex; i++){
    if (ping_ranges[i] < smallest){
      smallest = ping_ranges[i];
    }
  }
  return smallest; 
}

/**
 * Receives the laser scan data from the last callback to the LIDAR.
 * Determines the smallest ping distance in a range between -30 degrees and 30 degrees
 * and publishes this as a lidar distance message. If the smallest ping distance from 
 * the LIDAR is less than .5 meters, then a lidar alarm message will be published that 
 * recognizes the lidar alarm has sounded.
 */
void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (firstRun)  {
      firstRun = false;
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;

        //find the indices to start and stop checking pings from lidar at
        min_ping_index = (int) start_min_angle / angle_increment_;
        max_ping_index = (int) end_max_angle / angle_increment_;
        ROS_INFO("LIDAR setup: min_ping_index = %i", min_ping_index);
        ROS_INFO("LIDAR setup: max_ping_index = %i", max_ping_index);
    }

   closest_ping = getClosestPingDist(laser_scan.ranges, min_ping_index, max_ping_index);
   ROS_INFO("The closest ping is: %f", closest_ping);

   if (closest_ping < MIN_SAFE_DISTANCE) {
       ROS_WARN("DANGER, WILL ROBINSON!!");
       laser_alarm_=true;
   }
   else {
       laser_alarm_=false;
   }
   std_msgs::Bool lidar_alarm_msg;
   lidar_alarm_msg.data = laser_alarm_;
   lidar_alarm_publisher_.publish(lidar_alarm_msg);
   std_msgs::Float32 lidar_dist_msg;
   lidar_dist_msg.data = closest_ping;
   lidar_dist_publisher_.publish(lidar_dist_msg);   
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh;
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("robot0/laser_0", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}

