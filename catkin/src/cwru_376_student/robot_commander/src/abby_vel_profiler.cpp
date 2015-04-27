#include <math.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <iostream>
#include <cwru_srv/simple_float_service_message.h>
#include <cwru_srv/simple_bool_service_message.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <SteerVelProfiler.h>



bool move_ = false;
float moveServiceLength_ = 0.0;
bool turn_ = false;
float rotatePhiService_ = 0.0;
float marker_x_ = 0.0;
float marker_y_ = 0.0;
float marker_phi_ = 0.0;
double dt_;
nav_msgs::Odometry current_odom_;    
geometry_msgs::Pose odom_pose_;    
geometry_msgs::PoseStamped odom_pose_stamped_;
double odom_vel_;
double odom_omega_;
double odom_x_;
double odom_y_;
double odom_phi_;
geometry_msgs::Quaternion odom_quat_;
SteerVelProfiler steeringProfiler_;
double ref_point_x;
double ref_point_y;
ros::Time lastCallbackTime;
geometry_msgs::Twist velocityCommand;


/**
 * Computes the minimum angle from the given angle, accounting for periodicity.
 * 
 * @param dang - the angle to convert to an angle between -PI and +PI
 */
double min_dang(double dang) {
    if (dang > M_PI) dang -= 2.0 * M_PI;
    if (dang<-M_PI) dang += 2.0 * M_PI;
    return dang;
}
/**
 * Converts a quaternion to phi values for planar motion.
 * 
 * @param quaternion - the quaternion to convert to a phi for planar motion
 */
double convertPlanarQuat2Phi(double quat_z, double quat_w) {
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}
/**
 * Converts a quaternion to phi values for planar motion.
 * 
 * @param quaternion - the quaternion to convert to a phi for planar motion
 */
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

void pathMarkerListenerCB(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
    marker_x_ = feedback->pose.position.x;
    marker_y_ = feedback->pose.position.y;
    marker_phi_ = convertPlanarQuat2Phi(feedback->pose.orientation.z, feedback->pose.orientation.w);
}

bool moveService(cwru_srv::simple_float_service_messageRequest& request, cwru_srv::simple_float_service_messageResponse& response){
	//check if the value is zero if it is then don't move
	if(request.req !=0){
		move_ = true;
	    response.resp = true;
		moveServiceLength_ = request.req;
		ROS_INFO("Move Service is called with segment length: %f",moveServiceLength_);
		return true;
	}
    response.resp = false;
	ROS_INFO("Move Service was called with value of:%f \n Invalid data or with zero was entered", request.req);
	return false;
}


bool turnService(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response){
	response.resp = true;
	turn_ = true;
	rotatePhiService_ = marker_phi_;
	ROS_INFO("Turn Service was called with heading phi: %f", rotatePhiService_);
	return true;
}

/**
 * Updates the steering velocity profiler instance with
 * fresh odometer readings.
 */
void update_vel_profiler() {
    steeringProfiler_.setOdomXYValues(odom_x_, odom_y_);
    steeringProfiler_.setOdomRotationValues(odom_phi_, odom_omega_);
    steeringProfiler_.setOdomForwardVel(odom_vel_);
    steeringProfiler_.setOdomDT(dt_);
    steeringProfiler_.current_seg_ref_point_0 = ref_point_x;
    steeringProfiler_.current_seg_ref_point_1 = ref_point_y;
    //steeringProfiler_.setSegLengthToGo(current_seg_length_to_go_);
    ROS_INFO("Steering profile: x: %f, y: %f, phi: %f, omega: %f, vel: %f, dt: %f, "
            "distance left: %f", steeringProfiler_.odomX, steeringProfiler_.odomY,
            steeringProfiler_.odomPhi, steeringProfiler_.odomOmega, steeringProfiler_.odomVel,
            steeringProfiler_.dt, steeringProfiler_.distanceLeft);
}


void odomCallback(const nav_msgs::Odometry& odom_rcvd) {

    //compute time since last callback
    dt_ = (ros::Time::now() - lastCallbackTime).toSec();

    // let's remember the current time, and use it next iteration
    lastCallbackTime = ros::Time::now();

    // on start-up, and with occasional hiccups, this delta-time can be unexpectedly large
    if (dt_ > 0.15) {
        // can choose to clamp a max value on this, if dt_callback is used for computations elsewhere
        dt_ = 0.1;
        // let's complain whenever this happens
        ROS_WARN("large dt; dt = %lf", dt_);
    }

    // copy some of the components of the received message into member vars
    // we care about speed and spin, as well as position estimates x,y and heading
    current_odom_ = odom_rcvd; // save the entire message
    // but also pick apart pieces, for ease of use
    odom_pose_stamped_.header = odom_rcvd.header;
    odom_pose_ = odom_rcvd.pose.pose;
    odom_vel_ = odom_rcvd.twist.twist.linear.x;
    odom_omega_ = odom_rcvd.twist.twist.angular.z;
    odom_x_ = odom_rcvd.pose.pose.position.x;
    odom_y_ = odom_rcvd.pose.pose.position.y;
    odom_quat_ = odom_rcvd.pose.pose.orientation;
    //odom publishes orientation as a quaternion.  Convert this to a simple heading
    odom_phi_ = convertPlanarQuat2Phi(odom_quat_); // cheap conversion from quaternion to heading for planar motion
}

void moveOnSegment(ros::Publisher velPublisher, float seglength, ros::Rate rTimer){
	steeringProfiler_.currSegLength = seglength;
	steeringProfiler_.distanceLeft = seglength;
	while(ros::ok()){
		ros::spinOnce();			
		update_vel_profiler();
        ROS_INFO("Distance to end of original path segment: %f", steeringProfiler_.distanceLeft);	
		double speedProfile = steeringProfiler_.trapezoidalSlowDown(steeringProfiler_.currSegLength);
	    double commandSpeed = steeringProfiler_.trapezoidalSpeedUp(speedProfile);
	    velocityCommand.linear.x = commandSpeed;
	    ROS_INFO("cmd vel: %f",commandSpeed);

	    if(steeringProfiler_.distanceLeft <= 0.0){
        	velocityCommand.linear.x = 0.0;
	    }
	   	velPublisher.publish(velocityCommand);
	   	//Put timer to sleep for rest of iteration
        rTimer.sleep();

        //halt when segment is complete
        if (steeringProfiler_.distanceLeft <= 0.0){ 
        	break;
        }
    }
}

void rotateToPhi(ros::Publisher velPublisher, float rotatePhi){
	update_vel_profiler();
	bool turnRight;
	if(min_dang(rotatePhi)-min_dang(odom_phi_) > 0){
		turnRight = false;
	}
	else{
		turnRight = true;
	}
	ROS_INFO("Rotating to Phi...");
	steeringProfiler_.rotateToPhi(velPublisher, velocityCommand, rotatePhi , turnRight);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "abby_vel_profiler");
	ros::NodeHandle nh;
	ros::Subscriber odomSub = nh.subscribe("/odom", 1, odomCallback); 
	ros::Subscriber sub_im = nh.subscribe("path_marker/feedback", 1, pathMarkerListenerCB); 
	ros::Publisher velocityPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::ServiceServer moveOnSegmentService = nh.advertiseService("moveOnSeg", moveService);
	ros::ServiceServer rotateToPhiService = nh.advertiseService("rotateToPhi", turnService);
	ros::Rate rTimer(1/0.05);
	//SteerVelProfiler* steerProfiler;
	//steeringProfiler_(*steerProfiler);
	bool firstcall = false;
	while(ros::ok()){
        ros::spinOnce();
        if(!firstcall){
        	ref_point_x = odom_x_;
        	ref_point_y = odom_y_;
        	firstcall = true;
        	ROS_INFO("Reference point initialized and waiting on service call");
        }
		if(move_){
			//move robot and set moveSegmentService to 0 
			moveOnSegment(velocityPublisher, moveServiceLength_, rTimer);
			ROS_INFO("Finished Movement on Segment");
			move_=false;
		}
		if(turn_){
			//turn robot and set rotatePhiService to 0
			rotateToPhi(velocityPublisher, rotatePhiService_);
			ROS_INFO("Finished Turning");
			turn_ = false;
		}        
		rTimer.sleep();
	}

	return 0;
}