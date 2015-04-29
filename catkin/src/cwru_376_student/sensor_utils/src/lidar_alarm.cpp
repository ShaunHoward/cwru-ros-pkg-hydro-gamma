#include <ros/ros.h>
#include <float.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

const double MIN_SAFE_DISTANCE = 0.5; // set alarm if anything is within 0.5m of the front of robot

//Values that are utilized for laser callback
float closest_ping = 0.0;
bool firstRun = true;
float angle_min_ = 0.0;
float angle_max_ = 0.0;
float angle_increment_ = 0.0;
float range_min_ = 0.0;
float range_max_ = 0.0;

//Min and max range values for lidar
int min_ping_index = 0;
int max_ping_index = 0;
float start_min_angle = -.523333333;
float end_max_angle = .52333333;
bool laser_alarm_ = false;

//Publishers for lidar alarm and closest ping distance
ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;

/**
 * Determines the closest ping to the robot according to LIDAR ping ranges.
 * The smallest found ping distance value is returned.
 * 
 * @param ping_ranges - the float vector of ping ranges from the lidar
 * @param minIndex - the index of the minimum (left) ping from the lidar
 * @param maxIndex - the index of the maximum (right) ping from the lidar
 */
float getClosestPingDist(std::vector<float> ping_ranges, int minIndex, int maxIndex) {
    float *pings = &ping_ranges[0];
    float smallest = pings[minIndex];
    for (int i = minIndex; i <= maxIndex; i++) {
        if (pings[i] < smallest) {
            smallest = pings[i];
        }
    }
    return smallest;
}

/**
 * Gets the absolute value of the given value.
 * 
 * @param value - the value to get the abs value of
 */
float absValue(float value) {
    if (value >= 0)
        return value;
    return -1 * value;
}

/**
 * Receives the laser scan data from the last callback to the LIDAR.
 * Determines the smallest ping distance in a range between -30 degrees and 30 degrees
 * and publishes this as a lidar distance message. If the smallest ping distance from 
 * the LIDAR is less than .5 meters, then a lidar alarm message will be published that 
 * recognizes the lidar alarm has sounded.
 * 
 * @param laser_scan - the laser scan message received from the lidar
 */
void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    //initialize range values for reading lidar pings
    if (firstRun) {
        firstRun = false;
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;

        //find the indices to start and stop checking pings from lidar at
        min_ping_index = (int) abs(start_min_angle - angle_min_) / angle_increment_; //159
        max_ping_index = (int) (end_max_angle + 1.5 * abs(angle_min_)) / angle_increment_; //477
        ROS_INFO("LIDAR setup: min_ping_index = %i", min_ping_index);
        ROS_INFO("LIDAR setup: max_ping_index = %i", max_ping_index);

        //changed this to get the ping straight in front of robot
        //min_ping_index = (int) abs(0.0 -angle_min_)/angle_increment_;
        ROS_INFO("LIDAR setup: ping_index = %d", min_ping_index);
    }

    //changed this to just get the ping directly in front of the robot
    //closest_ping = laser_scan.ranges[min_ping_index];
    //ROS_INFO("ping dist in front = %f", closest_ping);

     //gets the closest ping distance found in range -30 degrees to 30 degrees from front, center
     //of robot
     closest_ping = getClosestPingDist(laser_scan.ranges, min_ping_index, max_ping_index);
     ROS_INFO("The closest ping is: %f", closest_ping);

    //sound lidar alarm if object closer than min safe distance
    if (closest_ping < MIN_SAFE_DISTANCE) {
        ROS_WARN("DANGER, WILL ROBINSON!!");
        laser_alarm_ = true;
    } else {
        laser_alarm_ = false;
    }

    //publish lidar alarm and ping distance messages
    std_msgs::Bool lidar_alarm_msg;
    lidar_alarm_msg.data = laser_alarm_;
    lidar_alarm_publisher_.publish(lidar_alarm_msg);
    std_msgs::Float32 lidar_dist_msg;
    lidar_dist_msg.data = closest_ping;
    lidar_dist_publisher_.publish(lidar_dist_msg);
}

/**
 * Initializes a lidar alarm ROS node for the robot. Subscribes to the lidar of
 * the robot and publishes both a lidar alarm and ping distance message via callback
 * functions that determine these values.
 * 
 * @param argc - initialize ROS values
 * @param argv - initialize ROS values
 * @return the exit code of the program
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm");
    ros::NodeHandle nh;
	//Make new publishers for lidar alarm and ping distance
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub;
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);
    lidar_dist_publisher_ = pub2;
    //ros::Subscriber lidar_subscriber = nh.subscribe("base_laser1_scan", 1, laserCallback); //base_laser1_scan for Jinx
    ros::Subscriber lidar_subscriber = nh.subscribe("scan", 1, laserCallback);
    ros::spin();
    return 0;
}