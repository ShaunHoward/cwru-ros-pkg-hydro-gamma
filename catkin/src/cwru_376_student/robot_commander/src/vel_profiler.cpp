#include <vel_profiler.h>

/**
 * Receives the pose and velocity estimates from the simulator (or the physical robot),
 * copies the relevant values to global variables, for use by "main"
 * Note: stdr updates odom only at 10Hz; Jinx is 50Hz (?).
 * 
 * @param odom_rcvd - the odom message from the robot
 */
void odomCallback(const nav_msgs::Odometry& odom_rcvd) {
    //compute time since last callback
    callback.dt = (ros::Time::now() - callback.lastCallbackTime).toSec();
    callback.lastCallbackTime = ros::Time::now(); // let's remember the current time, and use it next iteration

    if (callback.dt > 0.15) { // on start-up, and with occasional hiccups, this delta-time can be unexpectedly large
        callback.dt = 0.1; // can choose to clamp a max value on this, if dt_callback is used for computations elsewhere
        ROS_WARN("large dt; dt = %lf", callback.dt); // let's complain whenever this happens
    }

    //Copy some of the components of the received message into global vars, for use by "main()"
    //We care about speed and spin, as well as position estimates x,y and heading
    callback.setOdomVelocity(odom_rcvd.twist.twist.linear.x);
    callback.setOdomOmega(odom_rcvd.twist.twist.angular.z);
    callback.setOdomX(odom_rcvd.pose.pose.position.x);
    callback.setOdomY(odom_rcvd.pose.pose.position.y);
	
    //Odom publishes orientation as a quaternion. Must be converted to similar heading.
    callback.setQuaternionZ(odom_rcvd.pose.pose.orientation.z);
    callback.setQuaternionW(odom_rcvd.pose.pose.orientation.w);
    callback.setOdomPhi();

	//Print the callback values to console
    callback.printValues();
}

/**
 * Determines if the robot has rotated successfully to the desired end rotation.
 * 
 * @param startTime - the starting time of the rotation
 * @param currTime - the current time of the rotation
 * @param commandOmega - the omega desired
 * @param currRotation - the current rotation of the robot
 * @param endRotation - the desired end rotation of the robot
 * @return whether the robot rotated successfully
 */
bool isDoneRotating(float startTime, float currTime, float commandOmega, float currRotation, float endRotation) {
    //for degrees do: currRotation = currRotation + (new_cmd_omega) * (currTime - startTime);
    currRotation = (commandOmega) * (currTime - startTime);
    ROS_INFO("The current rotation in rads is: %f", currRotation);
	
	//When desired angular rotation is negative, check if the current rotation has met the desired end rotation.
    if (commandOmega < 0) {
        if (currRotation <= endRotation) {
            return true;
        }
    } else if (commandOmega > 0) {
		//When desired angular rotation is positive, check if the current rotation has met the desired end rotation.
        if (currRotation >= endRotation) {
            return true;
        }
    } else { 
		//Otherwise there is no rotation and rotate must be complete
        return true;
    }
}

/**
 * Resets all relevant velocities in the velocity command.
 */
void resetVelocityCommand() {
    velocityCommand.linear.x = 0.0; // initialize these values to zero
    velocityCommand.linear.y = 0.0;
    velocityCommand.linear.z = 0.0;
    velocityCommand.angular.x = 0.0;
    velocityCommand.angular.y = 0.0;
    velocityCommand.angular.z = 0.0;
}

/**
 * Resets the velocity of the robot in an effort to stop the robot immediately.
 */
void eStop() {
    velocityCommand.linear.x = 0.0; // initialize these values to zero
    velocityCommand.linear.y = 0.0;
    velocityCommand.linear.z = 0.0;
    velocityCommand.angular.x = 0.0;
    velocityCommand.angular.y = 0.0;
    velocityCommand.angular.z = 0.0;
    velocityPublisher.publish(velocityCommand);
    ROS_INFO("ESTOP function executed.");
}

/**
 * Slows down the robot trapezoidally.
 * 
 * @param segmentLength - the segment to slow the robot down along
 * @return the newly scheduled velocity determined by the algorithm
 */
float trapezoidalSlowDown(float segmentLength) {
    //Compute distance traveled thus far on the current segment
    float deltaX = callback.odomX - segment.startX;
    float deltaY = callback.odomY - segment.startY;
    float scheduledVelocity = 0.0f;
	
	//Set the length completed along the current segment thus far
    segment.setLengthCompleted(sqrt(deltaX * deltaX + deltaY * deltaY));
    ROS_INFO("dist traveled: %f", segment.lengthCompleted);
	//Set the distance left to travel on the current segment
    segment.setDistanceLeft(segmentLength - segment.lengthCompleted);

    //use segment.distanceLeft to decide what vel should be, as per plan
    if (segment.distanceLeft <= 0.0) { // at goal, or overshot; stop!
        scheduledVelocity = 0.0;
    } else if (segment.distanceLeft <= decelerationDistance) { //possibly should be braking to a halt
        // dist = 0.5*a*t_halt^2; so t_halt = sqrt(2*dist/a);   v = a*t_halt
        // so v = a*sqrt(2*dist/a) = sqrt(2*dist*a)
        scheduledVelocity = sqrt(2 * segment.distanceLeft * maxAcceleration);
        ROS_INFO("braking zone: v_sched = %f", scheduledVelocity);
    } else { 
	    //Not ready to decelerate robot so scheduled velocity will be the max velocity (need to accelerate 
	    //or hold the max velocity
        scheduledVelocity = maxVelocity;
    }
    ROS_INFO("Slow down scheduled velocity is: %f", scheduledVelocity);
    return scheduledVelocity;
}

/**
 * Speeds up the robot trapezoidally.
 * 
 * @param scheduledVelocity - the velocity provided via the trapezoidal slow down algorithm
 * @param newVelocityCommand - the velocity command to publish
 * @return the new velocity to command
 */
float trapezoidalSpeedUp(float scheduledVelocity, float newVelocityCommand) {
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

/**
 * Moves the robot on a given segment length.
 * When the lidar detector finds that an object is within a distance of 2.5m to the robot, it will
 * slow down to a distance of .5m before that distance. Once the object moves, it will resume and
 * finish its original path to the goal. 
 * When the estop is on or software halt is on, the robot will halt on its current segment. Once
 * both estop and halt are not on, the robot will resume to move along its original path to the goal.
 * 
 * @param velocityPublisher - the publisher to publish velocity commands to
 * @param rTimer - the timer for the movement
 * @param segmentLength - the length of the segment to move along
 */
void moveOnSegment(ros::Publisher velocityPublisher, ros::Rate rTimer, float segmentLength) {
	//Desired velocity, assuming all is per plan
    float scheduledVelocity = 0.0; 
	//Value of speed to be commanded; update each iteration
    float newVelocityCommand = 0.1; 
	
	//Reset the length completed on the current segment
	segment.resetLengthCompleted();
	//Set the length of the current segment
    segment.setLength(segmentLength);
	

    while (ros::ok()) // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted (or ctl-C)
    {
        ros::spinOnce(); // allow callbacks to populate fresh data
		
		//When the original segment is being followed and no stop commands are issued, move the robot
		//with a trapezoidal velocity profile until the original path is completed.
        if (!lidar.modifiedSegment && !(estop.on || lidar.alarm || halt)) {
            ROS_INFO("Distance to end of original path segment: %f", segment.distanceLeft);
			
			//Decide to slow down the robot
            scheduledVelocity = trapezoidalSlowDown(segment.length);

			//Decide to speed up the robot and issue the new velocity command
            newVelocityCommand = trapezoidalSpeedUp(scheduledVelocity, newVelocityCommand);

            ROS_INFO("cmd vel: %f", newVelocityCommand); // debug output
			
			//Set new velocity command value
            velocityCommand.linear.x = newVelocityCommand;

			//Issue a zero forward velocity if past segment length
            if (segment.distanceLeft <= 0.0) { //uh-oh...went too far already!
                velocityCommand.linear.x = 0.0; //command vel=0
            }

			//Publish new velocity command
            velocityPublisher.publish(velocityCommand);
			
			//Put timer to sleep for rest of iteration
            rTimer.sleep();
			
			//halt when segment is complete
            if (segment.distanceLeft <= 0.0) break; 
        } else if (lidar.modifiedSegment && !(estop.on || lidar.alarm || halt)) {
			//When a modified segment is being followed and no stop commands are issued, move the robot
			//with a trapezoidal velocity profile until the modified path is completed.
            ROS_INFO("Distance to end of modified path segment: %f", segment.distanceLeft);
			
			//Decide whether to slow down robot
            scheduledVelocity = trapezoidalSlowDown(segment.length);
			
			//Decide whether to speed up robot and issue new velocity command
            newVelocityCommand = trapezoidalSpeedUp(scheduledVelocity, newVelocityCommand);

            ROS_INFO("cmd vel: %f", newVelocityCommand); // debug output

			//Set new velocity command value
            velocityCommand.linear.x = newVelocityCommand;

			//Issue a zero forward velocity if past segment length
            if (segment.distanceLeft <= 0.0) { //uh-oh...went too far already!
                velocityCommand.linear.x = 0.0; //command vel=0
            }

			//Publish new velocity command
            velocityPublisher.publish(velocityCommand);
            
			//Put timer to sleep for rest of iteration
			rTimer.sleep();
			
			//When the modified segment is complete, we do not want to break from the loop
			//because we have not resumed on the original path yet.
			//Hence, the loop will continue until the original path is resumed.
            if (segment.distanceLeft <= 0.0) {
                ROS_INFO("Completed modified segment. Move out of way to finish original path.");
            }
        } else if (halt) {
			//We want to halt when software halt is enabled, so we constantly update velocity forward command to zero.
            velocityCommand.linear.x = 0.0;
            velocityPublisher.publish(velocityCommand);
            ROS_INFO("Software halt enabled. Robot is static.");
        }
    }
    ROS_INFO("Completed move along segment with desired rotation");
}

/**
 * Rotates the robot with a trapezoidal speed up and speed down profile.
 * 
 * @param velocityPublisher - the publisher to publish the new velocity to
 * @param rTimer - the timer of the rotation movement
 * @param z - the spin rate desired for the robot
 * @param endRotation - the desired end rotation of the robot
 */
void rotate(ros::Publisher velocityPublisher, ros::Rate rTimer, float z, float endRotation) {
    bool firstCall = true;
    float startTime;
    float currTime = 0.0;
    float currRotation = 0.0;
    float newCommandOmega = z; //update spin rate
    while (ros::ok()) {
        //Handle setting up timer for rotation since beginning of method call.
        if (firstCall) {
            firstCall = false;
            startTime = ros::Time::now().toSec();
            velocityCommand.angular.z = newCommandOmega; // spin command;
        }
        //Rotate and check if at desired rotation in rads.
        currTime = ros::Time::now().toSec();
        bool doneRotating = isDoneRotating(startTime, currTime, newCommandOmega, currRotation, endRotation);

        //Set angular z velocity to 0 when done rotating.
        if (doneRotating) {
            velocityCommand.angular.z = 0.0;
        }
        velocityPublisher.publish(velocityCommand); // publish the command to robot0/velocityCommand
        rTimer.sleep(); // sleep for remainder of timed iteration
    }
}

/**
 * Validates that the odometer is receiving feasible values to use.
 * 
 * @param rTimer - the timer of the odom callback call
 * @return true when validation is complete
 */
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

/**
 * Initializes the position of the robot according to the current segment.
 * These values are printed via ROS_INFO.
 */
void initializePosition() {
    ROS_INFO("received odom message; proceeding");
    segment.setStartX(callback.odomX);
    segment.setStartY(callback.odomY);
    segment.setStartPhi(callback.odomPhi);
    segment.printValues();
}

/**
 * Initializes moving along a new segment. Validates that the
 * odometer is receiving feasible values, initializes the position
 * of the robot according to the new segment, and resets all velocity
 * of the robot to 0.
 * 
 * @param rTimer - the timer for odometer validation
 */
void initializeNewMove(ros::Rate rTimer) {
    if (odomCallValidation(rTimer)) {
        initializePosition();
        resetVelocityCommand();
    }
}

/**
 * Determines whether to run the robot along a new, slow down segment, to sound a 
 * lidar alarm, or to keep moving.
 * 
 * @param pingDistance - the distance of the ping closest to the robot within a given angle range
 */
void pingDistanceCallback(const std_msgs::Float32& pingDistance) {
    lidar.setClosestPing(pingDistance.data);
    float newSegmentLength = lidar.closestPing - minSafeRange;
    ROS_INFO("The ping distance from front of robot is: %f", lidar.closestPing);
    //Determine to sound a lidar alarm
    if (lidar.closestPing <= minSafeRange) {
        lidar.setAlarm(true);
    } else if (lidar.closestPing <= maxSafeRange && !lidar.modifiedSegment && !estop.on) {
        //determine whether to slow down on a new segment to before the position of the object near robot
        modifiedSegment.copy(segment);
        segment.setLength(newSegmentLength);
        segment.resetLengthCompleted();
        lidar.setStop(true);
        lidar.setModifiedSegment(true);
    } else if (lidar.closestPing > maxSafeRange && lidar.modifiedSegment) {
        //run on the rest of the desired segment length
        float origPathLength = modifiedSegment.length;
        float currLengthCompleted = segment.lengthCompleted;
        float prevLengthCompleted = modifiedSegment.lengthCompleted;
        segment.copy(modifiedSegment);
        segment.setLength(origPathLength - currLengthCompleted - prevLengthCompleted);
        segment.resetLengthCompleted();
        lidar.setStop(false);
        lidar.setModifiedSegment(false);
        lidar.setAlarm(false);
    }
}

/**
 * Determines whether to stop the robot due to a lidar alarm.
 * 
 * @param lidarAlarmMsg - a boolean message that designates if the robot needs to
 * be stopped immediately
 */
void lidarAlarmCallback(const std_msgs::Bool& lidarAlarmMsg) {
    lidar.setAlarm(lidarAlarmMsg.data);
    if (lidar.alarm) {
        ROS_INFO("The lidar alarm is on!");
        eStop();
    }
}

/**
 * Determines whether to stop the robot immediately
 * because estop is on.
 * 
 * @param estopMsg - a boolean message that designates whether the robot
 * estop is on
 */
void estopCallback(const std_msgs::Bool& estopMsg) {
    //assign conversion to bool type from ROS Bool type
    estop.set(estopMsg.data);
    if (estop.on) {
        ROS_INFO("Velocity profiler ESTOP enabled.");
        estop.set(true);
        eStop();
    }
}

/**
 * Determines whether to stop the robot immediately
 * because software halt is enabled.
 * 
 * @param haltMsg - a boolean message that designates whether the robot
 * should halt
 */
void haltCallback(const std_msgs::Bool& haltMsg) {
    //assign conversion to bool type from ROS Bool type
    halt = haltMsg.data;
    if (halt) {
        ROS_INFO("Software halt enabled.");
        eStop();
    }
}

/**
 * Initializes a new velocity profiler node, subscribes to odometer, lidar, and estop
 * messages. Movement segments and rotations should be declared in this method after the
 * ROS timer is declared by initializing a new move and moving on a new segment.
 * 
 * @param argc - ROS initialization values
 * @param argv - ROS initialization values
 * @return the exit code of the program
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_profiler"); // name of this node will be "minimal_publisher1"
    ros::NodeHandle nodeHandle; // get a ros nodehandle; standard yadda-yadda
    //create a publisher object that can talk to ROS and issue twist messages on named topic;
    // note: this is customized for stdr robot; would need to change the topic to talk to jinx, etc.
    velocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber sub = nodeHandle.subscribe("odom", 1, odomCallback);

    ros::Subscriber ping_dist_subscriber = nodeHandle.subscribe("lidar_dist", 1, pingDistanceCallback);
    ros::Subscriber lidar_alarm_subscriber = nodeHandle.subscribe("lidar_alarm", 1, lidarAlarmCallback);
    ros::Subscriber estop_subscriber = nodeHandle.subscribe("estop_listener", 1, estopCallback);
    ros::Subscriber halt_subscriber = nodeHandle.subscribe("halt_cmd", 1, haltCallback);

    ros::Rate rTimer(1 / changeInTime); // frequency corresponding to chosen sample period DT; the main loop will run this fast

    initializeNewMove(rTimer);
    moveOnSegment(velocityPublisher, rTimer, 25); //4.75
    //    initializeNewMove(rtimer);
    //    moveOnSegment(velocityPublisher, rtimer, 0.0, -.314, -1.57);
    //    initializeNewMove(rtimer);
    //   moveOnSegment(velocityPublisher, rtimer, 12.3, 0.0, 0.0);
    //   initializeNewMove(rtimer);
    //    moveOnSegment(velocityPublisher, rtimer, 0.0, -.314, -1.57);
    //    initializeNewMove(rtimer);
    //    moveOnSegment(velocityPublisher, rtimer, 12.3, 0.0, 0.0);
    return 0;
}

