#include <vel_profiler.h>

// receive the pose and velocity estimates from the simulator (or the physical robot)
// copy the relevant values to global variables, for use by "main"
// Note: stdr updates odom only at 10Hz; Jinx is 50Hz (?)
void odomCallback(const nav_msgs::Odometry& odom_rcvd) {
    //here's a trick to compute the delta-time between successive callbacks:
    callback.dt = (ros::Time::now() - callback.lastCallbackTime).toSec();
    callback.lastCallbackTime = ros::Time::now(); // let's remember the current time, and use it next iteration

    if (callback.dt > 0.15) { // on start-up, and with occasional hiccups, this delta-time can be unexpectedly large
        callback.dt = 0.1; // can choose to clamp a max value on this, if dt_callback is used for computations elsewhere
        ROS_WARN("large dt; dt = %lf", callback.dt); // let's complain whenever this happens
    }

    // copy some of the components of the received message into global vars, for use by "main()"
    // we care about speed and spin, as well as position estimates x,y and heading
    callback.setOdomVelocity(odom_rcvd.twist.twist.linear.x);
    callback.setOdomOmega(odom_rcvd.twist.twist.angular.z);
    callback.setOdomX(odom_rcvd.pose.pose.position.x);
    callback.setOdomY(odom_rcvd.pose.pose.position.y);
    //odom publishes orientation as a quaternion.  Convert this to a simple heading
    // see notes above for conversion for simple planar motion
    callback.setQuaternionZ(odom_rcvd.pose.pose.orientation.z);
    callback.setQuaternionW(odom_rcvd.pose.pose.orientation.w);
    callback.setOdomPhi();

    callback.printValues();
}

bool rotate(float startTime, float currTime, float commandOmega, float currRotation, float endRotation) {
    //for degrees do: currRotation = currRotation + (new_cmd_omega) * (currTime - startTime);
    currRotation = (commandOmega) * (currTime - startTime);
    ROS_INFO("The current rotation in rads is: %f", currRotation);
    if (commandOmega < 0) {
        if (currRotation <= endRotation) {
            return true;
        }
    } else if (commandOmega > 0) {
        if (currRotation >= endRotation) {
            return true;
        }
    } else {
        return true;
    }
}

void resetVelocityCommand() {
    velocityCommand.linear.x = 0.0; // initialize these values to zero
    velocityCommand.linear.y = 0.0;
    velocityCommand.linear.z = 0.0;
    velocityCommand.angular.x = 0.0;
    velocityCommand.angular.y = 0.0;
    velocityCommand.angular.z = 0.0;
}

void eStop(){
    resetVelocityCommand();
    velocityPublisher.publish(velocityCommand); 
}

float trapezoidalSlowDown(float segmentLength){
    // compute distance traveled so far:
    float deltaX = callback.odomX - segment.startX;
    float deltaY = callback.odomY - segment.startY;
    float scheduledVelocity = 0.0f;
    segment.setLengthCompleted(sqrt(deltaX * deltaX + deltaY * deltaY));
    ROS_INFO("dist traveled: %f", segment.lengthCompleted);
    segment.setDistanceLeft(segmentLength - segment.lengthCompleted);

    //use segmentLengthCompleted to decide what vel should be, as per plan
    if (segment.distanceLeft <= 0.0) { // at goal, or overshot; stop!
        scheduledVelocity = 0.0;
    } else if (segment.distanceLeft <= decelerationDistance) { //possibly should be braking to a halt
        // dist = 0.5*a*t_halt^2; so t_halt = sqrt(2*dist/a);   v = a*t_halt
        // so v = a*sqrt(2*dist/a) = sqrt(2*dist*a)
        scheduledVelocity = sqrt(2 * segment.distanceLeft * maxAcceleration);
        ROS_INFO("braking zone: v_sched = %f", scheduledVelocity);
    } else { // not ready to decel, so target vel is v_max, either accel to it or hold it
        scheduledVelocity = maxVelocity;
    }
	ROS_INFO("Slow down scheduled velocity is: %f", scheduledVelocity);
    return scheduledVelocity;
}

void decideToStop(){
//    float slow_down_start = 2.0f;
//    float e_stop_distance = 0.5f;
//    float dist = closest_ping_dist;
//    //not sure about stop distance if obstacle before end of path
//    float calc_stop_dist = dist - e_stop_distance;
//    if (dist > e_stop_distance && dist <= slow_down_start){
//        stopping = true;
//        if (dist_to_go <= 0) {
//            stopping = false; 
//        } else if (calc_stop_dist >= dist_to_go){
//            ROS_INFO("Deciding to stop with trapezoidal slow down until goal.");
//            trapezoidal_slow_down(dist_to_go); //curr_seg_length - dist
//        } else {
//            ROS_INFO("Deciding to stop with trapezoidal slow down before goal.");
//            trapezoidal_slow_down(calc_stop_dist);
//        }
//    } else if (dist > slow_down_start) {
//        stopping = false;
//    }
}

float trapezoidalSpeedUp(float scheduledVelocity, float newVelocityCommand){
	//how does the current velocity compare to the scheduled vel?
    if (callback.odomVelocity < scheduledVelocity) { // maybe we halted, e.g. due to estop or obstacle;
        // may need to ramp up to v_max; do so within accel limits
        float testVelocity = callback.odomVelocity + maxAcceleration * callback.dt; // if callbacks are slow, this could be abrupt
        // operator:  c = (a>b) ? a : b;
        newVelocityCommand = (testVelocity < scheduledVelocity) ? testVelocity : scheduledVelocity; //choose lesser of two options
        // this prevents overshooting scheduled_vel
    } else if (callback.odomVelocity > scheduledVelocity) { //travelling too fast--this could be trouble
        // ramp down to the scheduled velocity.  However, scheduled velocity might already be ramping down at a_max.
        // need to catch up, so ramp down even faster than a_max.  Try 1.2*a_max.
        ROS_INFO("odom vel: %f; sched vel: %f", callback.odomVelocity, scheduledVelocity); //debug/analysis output; can comment this out

        float testVelocity = callback.odomVelocity - 1.2 * maxAcceleration * callback.dt; //moving too fast--try decelerating faster than nominal a_max

        newVelocityCommand = (testVelocity > scheduledVelocity) ? testVelocity : scheduledVelocity; // choose larger of two options...don't overshoot scheduled_vel
    } else {
        newVelocityCommand = scheduledVelocity; //silly third case: this is already true, if here.  Issue the scheduled velocity
    }
    ROS_INFO("New speedup command is: %f", newVelocityCommand);
    return newVelocityCommand;
}

//Moves the robot on a given segment length with the desired angular velocity z.
//Will stop rotating when the end rotation is met according to the timing of the method calls.
//Can use to just rotate robot if segment length is 0.
//Can use to just move robot forward if z and endRotation are both 0.
void moveOnSegment(ros::Publisher velocityPublisher, ros::Rate rTimer, float segmentLength) {
    segment.resetLengthCompleted(); // need to compute actual distance travelled within the current segment
	segment.setLength(segmentLength);
    float constantVelocityDistance = segmentLength - accelerationDistance - decelerationDistance; //if this is <0, never get to full spd
    float constantVelocityTime = constantVelocityDistance / maxVelocity; //will be <0 if don't get to full speed
    float duration = accelerationTime + decelerationTime + constantVelocityTime; // expected duration of this move

    // here is a crude description of one segment of a journey.  Will want to generalize this to handle multiple segments
    // define the desired path length of this segment
    float scheduledVelocity = 0.0; //desired vel, assuming all is per plan
    float newVelocityCommand = 0.1; // value of speed to be commanded; update each iteration
	float slowdownSegmentLength = 0.0;

    //dist_decel*= 2.0; // TEST TEST TEST
    while (ros::ok()) // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted (or ctl-C)
    {
        ros::spinOnce(); // allow callbacks to populate fresh data
		if(!(estop.on || lidar.alarm)) {
			ROS_INFO("Distance to end of path segment: %f", segment.distanceLeft);
			scheduledVelocity = trapezoidalSlowDown(segment.length);

			newVelocityCommand = trapezoidalSpeedUp(scheduledVelocity, newVelocityCommand);

			ROS_INFO("cmd vel: %f", newVelocityCommand); // debug output

			velocityCommand.linear.x = newVelocityCommand;
			
			if (segment.distanceLeft <= 0.0) { //uh-oh...went too far already!
				velocityCommand.linear.x = 0.0; //command vel=0
			}

			velocityPublisher.publish(velocityCommand);
			//rTimer.sleep();
			if (segment.distanceLeft <= 0.0) break; //halt when segment is complete
		}
    }
    ROS_INFO("completed move along segment with desired rotation");
}

void rotate(ros::Publisher velocityPublisher, ros::Rate rTimer, float z, float endRotation){
    bool firstCall = true;
    float startTime;
    float currTime = 0.0;
    float currRotation = 0.0;
    float newCommandOmega = z; //update spin rate
    while (ros::ok()){
        //Handle setting up timer for rotation since beginning of method call.
        if (firstCall) {
            firstCall = false;
            startTime = ros::Time::now().toSec();
            velocityCommand.angular.z = newCommandOmega; // spin command;
        }
	//Rotate and check if at desired rotation in rads.
        currTime = ros::Time::now().toSec();
        bool doneRotating = rotate(startTime, currTime, newCommandOmega, currRotation, endRotation);
	
        //Set angular z velocity to 0 when done rotating.
        if (doneRotating) {
            velocityCommand.angular.z = 0.0;
        }
        velocityPublisher.publish(velocityCommand); // publish the command to robot0/velocityCommand
        rTimer.sleep(); // sleep for remainder of timed iteration
    }
}

bool odomCallValidation(ros::Rate rTimer) {
    // let's wait for odom callback to start getting good values...
    callback.odomOmega = 1000000; // absurdly high
    ROS_INFO("waiting for valid odom callback...");
    callback.lastCallbackTime = ros::Time::now(); // initialize reference for computed update rate of callback
    while (callback.odomOmega > 1000) {
        rTimer.sleep();
        ros::spinOnce();
    }
    return true;
}

void initializePosition() {
    ROS_INFO("received odom message; proceeding");
    segment.setStartX(callback.odomX);
    segment.setStartY(callback.odomY);
    segment.setStartPhi(callback.odomPhi);
	segment.printValues();
}

void initializeNewMove(ros::Rate rTimer) {
    if (odomCallValidation(rTimer)) {
        initializePosition();
        resetVelocityCommand();
    }
}

void pingDistanceCallback(const std_msgs::Float32& pingDistance) {
    //assign the conversion float type from ROS Float32 type
    lidar.setClosestPing(pingDistance.data);
	float newSegmentLength = lidar.closestPing - minSafeRange;
    ROS_INFO("The ping distance from front of robot is: %f", lidar.closestPing);
	if (lidar.closestPing <= minSafeRange){
		lidar.setAlarm(true);
	} else if (lidar.closestPing <= maxSafeRange && !lidar.modifiedSegment && !estop.on){
		modifiedSegment.copy(segment);
		segment.setLength(newSegmentLength);
		segment.resetLengthCompleted();
		lidar.setStop(true);
		lidar.setModifiedSegment(true);
	} else if (lidar.closestPing > maxSafeRange && lidar.modifiedSegment){
		float currLengthCompleted = segment.lengthCompleted;
		float prevLengthCompleted = modifiedSegment.lengthCompleted;
		segment.copy(modifiedSegment);
		segment.setLength(segment.length - currLengthCompleted - prevLengthCompleted);
		segment.resetLengthCompleted();
		lidar.setStop(false);
		lidar.setModifiedSegment(false);
	}
}

void lidarAlarmCallback(const std_msgs::Bool& lidarAlarmMsg) {
    //assign conversion to bool type from ROS Bool type
    lidar.setAlarm(lidarAlarmMsg.data);
    if (lidar.alarm){
        ROS_INFO("The lidar alarm is on!");
        eStop();
    }
}

void estopCallback(const std_msgs::Bool& estopMsg) {
    //assign conversion to bool type from ROS Bool type
    estop.set(estopMsg.data);
    if (estop.on){
        ROS_INFO("Velocity profiler ESTOP enabled.");
        estop.set(true);
        eStop();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_profiler"); // name of this node will be "minimal_publisher1"
    ros::NodeHandle nodeHandle; // get a ros nodehandle; standard yadda-yadda
    //create a publisher object that can talk to ROS and issue twist messages on named topic;
    // note: this is customized for stdr robot; would need to change the topic to talk to jinx, etc.
    velocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>("jinx/cmd_vel", 1);
    ros::Subscriber sub = nodeHandle.subscribe("/jinx/odom", 1, odomCallback);
    
    ros::Subscriber ping_dist_subscriber = nodeHandle.subscribe("lidar_dist", 1, pingDistanceCallback);
    ros::Subscriber lidar_alarm_subscriber = nodeHandle.subscribe("lidar_alarm", 1, lidarAlarmCallback);
    ros::Subscriber estop_subscriber = nodeHandle.subscribe("estop_listener", 1, estopCallback);

    ros::Rate rTimer(1 / changeInTime); // frequency corresponding to chosen sample period DT; the main loop will run this fast

    initializeNewMove(rTimer);
    moveOnSegment(velocityPublisher, rTimer, 6); //4.75
//    initializeNewMove(rtimer);
//    moveOnSegment(velocityPublisher, rtimer, 0.0, -.314, -1.57);
//    initializeNewMove(rtimer);
//   moveOnSegment(velocityPublisher, rtimer, 12.3, 0.0, 0.0);
//   initializeNewMove(rtimer);
//    moveOnSegment(velocityPublisher, rtimer, 0.0, -.314, -1.57);
//    initializeNewMove(rtimer);
//    moveOnSegment(velocityPublisher, rtimer, 12.3, 0.0, 0.0);
}

