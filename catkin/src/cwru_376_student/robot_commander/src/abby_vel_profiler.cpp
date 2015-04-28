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



bool move_forward = false;
bool move_back = false;
float moveForwardLength_ = 0.0;
float moveBackLength_ = 0.0;
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
double phiLeftTime;


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
    marker_x_ = feedback->pose.position.x;
    marker_y_ = feedback->pose.position.y;
    marker_phi_ = min_dang(convertPlanarQuat2Phi(feedback->pose.orientation.z, feedback->pose.orientation.w));
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << marker_phi_);
}

bool forwardService(cwru_srv::simple_float_service_messageRequest& request, cwru_srv::simple_float_service_messageResponse& response){
	//check if the value is zero if it is then don't move
	if(request.req !=0){
		move_forward = true;
	    response.resp = true;
		moveForwardLength_ = request.req;
		ROS_INFO("Move Service is called with segment length: %f",moveForwardLength_);
		return true;
	}
    response.resp = false;
	ROS_INFO("Move Service was called with value of:%f \n Invalid data or with zero was entered", request.req);
	return false;
}

bool backService(cwru_srv::simple_float_service_messageRequest& request, cwru_srv::simple_float_service_messageResponse& response){
    //check if the value is zero if it is then don't move
    if(request.req !=0){
        move_back = true;
        response.resp = true;
        moveBackLength_ = request.req;
        ROS_INFO("Move Service is called with segment length: %f",moveBackLength_);
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
    steeringProfiler_.setOdomForwardVel(fabs(odom_vel_));
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

void moveOnSegment(ros::Publisher velPublisher, float seglength, ros::Rate rTimer, int direction){
    steeringProfiler_.resetSegValues();
	steeringProfiler_.currSegLength = seglength;
	steeringProfiler_.distanceLeft = seglength;
	while(ros::ok()){
		ros::spinOnce();			
		update_vel_profiler();
        ROS_INFO("Distance to end of original path segment: %f", steeringProfiler_.distanceLeft);	
		double speedProfile = steeringProfiler_.trapezoidalSlowDown(steeringProfiler_.currSegLength);
	    double commandSpeed = steeringProfiler_.trapezoidalSpeedUp(speedProfile);
	    velocityCommand.linear.x = commandSpeed * direction;
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

bool isDoneRotating(bool turnRight){
    phiLeftTime = phiLeftTime + steeringProfiler_.getDeltaPhi(turnRight);
    return (phiLeftTime >= steeringProfiler_.desiredPhi);
}

void rotateToPhi(ros::Publisher velPublisher, float rotatePhi, ros::Rate rTimer){
    phiLeftTime = 0;
	steeringProfiler_.resetSegValues();
    steeringProfiler_.lastCallbackPhi = odom_phi_;
    int diff = min_dang(rotatePhi) - min_dang(steeringProfiler_.lastCallbackPhi);
    steeringProfiler_.desiredPhi = fabs(diff);
	bool turnRight;
    int turnDirection;
	if( diff > 0 && fabs(diff) > M_PI){
		turnRight = true;
        turnDirection = -1;
        steeringProfiler_.desiredPhi = 2*M_PI - steeringProfiler_.desiredPhi; 
	}
    else if(diff > 0){
        turnRight = false;
        turnDirection = 1;
    }
	else if( diff < 0 && fabs(diff) > M_PI){
		turnRight = false;
        turnDirection = 1;
        steeringProfiler_.desiredPhi = 2*M_PI + steeringProfiler_.desiredPhi;
	}
    else{
        turnRight = true;
        turnDirection = -1;
    }
	ROS_INFO("Rotating to Phi...");
    while(ros::ok()){
        ros::spinOnce();            
        update_vel_profiler();
    	double omegaProfile = steeringProfiler_.turnSlowDown(turnRight);
        double commandOmega = steeringProfiler_.turnSpeedUp(omegaProfile);  
        bool doneRotating = isDoneRotating(turnRight);

        velocityCommand.angular.z = commandOmega*turnDirection;
        //Set angular z velocity to 0 when done rotating
        if (doneRotating) {
            velocityCommand.angular.z = 0.0;
        }

        
        //Publish latest velocity command
        velPublisher.publish(velocityCommand);

        // sleep for remainder of timed iteration
        rTimer.sleep();

        //Break from rotation loop if done rotating
        if (doneRotating) {
            break;
        }
    }
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "abby_vel_profiler");
	ros::NodeHandle nh;
	ros::Subscriber odomSub = nh.subscribe("/odom", 1, odomCallback); 
	ros::Subscriber sub_im = nh.subscribe("path_marker/feedback", 1, pathMarkerListenerCB); 
	ros::Publisher velocityPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::ServiceServer moveForwardService = nh.advertiseService("moveforward", forwardService);
    ros::ServiceServer moveBackService = nh.advertiseService("moveback", backService);
	ros::ServiceServer rotateToPhiService = nh.advertiseService("rotateToPhi", turnService);
	ros::Rate rTimer(1/0.05);
	//SteerVelProfiler* steerProfiler;
	//steeringProfiler_(*steerProfiler);
	bool firstcall = false;
	while(ros::ok()){
        ros::spinOnce();
		if(move_forward){
            ref_point_x = odom_x_;
            ref_point_y = odom_y_;
			//move robot and set moveSegmentService to 0 
			moveOnSegment(velocityPublisher, moveForwardLength_, rTimer, 1);
			ROS_INFO("Finished Movement on Segment");
			move_forward=false;
		}
        if(move_back){
            ref_point_x = odom_x_;
            ref_point_y = odom_y_;
            //move robot and set moveSegmentService to 0 
            moveOnSegment(velocityPublisher, moveBackLength_, rTimer, -1);
            ROS_INFO("Finished Movement on Segment");
            move_back=false;
        }
		if(turn_){
            ref_point_x = odom_x_;
            ref_point_y = odom_y_;
			//turn robot and set rotatePhiService to 0
			rotateToPhi(velocityPublisher, rotatePhiService_, rTimer);
			ROS_INFO("Finished Turning");
			turn_ = false;
		}        
		rTimer.sleep();
	}

	return 0;
}