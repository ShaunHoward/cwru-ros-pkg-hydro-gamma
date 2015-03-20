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
    
    ROS_INFO("Odom x: %f y: %f", callback.odomX, callback.odomY);

    //Odom publishes orientation as a quaternion. Must be converted to similar heading.
    callback.setQuaternionZ(odom_rcvd.pose.pose.orientation.z);
    callback.setQuaternionW(odom_rcvd.pose.pose.orientation.w);
    callback.setOdomPhi();

    //Print the callback values to console
    callback.printValues();
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
 * Slows down the robot's forward velocity trapezoidally according to
 * the segment length distance traveled and distance left to travel as
 * well as deceleration constants.
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
 * Speeds up the robot's forward velocity trapezoidally according to the 
 * scheduled velocity provided from the slow down function and the odom 
 * callback velocity as well as acceleration constants.
 * 
 * @param scheduledVelocity - the velocity provided via the trapezoidal slow down algorithm
 * @return the new velocity to command
 */
float trapezoidalSpeedUp(float scheduledVelocity) {
    float newVelocityCommand;
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

        newVelocityCommand = (testVelocity < scheduledVelocity) ? testVelocity : scheduledVelocity; // choose larger of two options...don't overshoot scheduled_vel
    } else {
        newVelocityCommand = scheduledVelocity; //silly third case: this is already true, if here.  Issue the scheduled velocity
    }
    ROS_INFO("New speedup command is: %f", newVelocityCommand);
    return newVelocityCommand;
}

/**
 * Moves the robot on a given segment length.
 * First initializes the move timer, position and velocity of the robot.
 * Then the function will move the robot to the desired segment length with
 * trapezoidal speed up and slow down algorithms.
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
    //Initialize a new move.
    initializeNewMove(rTimer);
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
            newVelocityCommand = trapezoidalSpeedUp(scheduledVelocity);

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
            newVelocityCommand = trapezoidalSpeedUp(scheduledVelocity);

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
 * Slows down the robot's rotational velocity trapezoidally according to
 * the phi left to rotate on the current rotation segment as well as
 * rotational deceleration constants.
 * 
 * @param turnRight - whether the robot is currently turning right
 * @return the scheduled omega for slowing down the robot spin
 */
float turnSlowDown(bool turnRight) {
    float scheduledOmega = 0.0f;

    //Set the phi (angle) turned thus far in the current rotation segment
    rotate.setPhiCompleted(rotate.phiCompleted + getDeltaPhi(turnRight));
    ROS_INFO("Phi rotated: %f", rotate.phiCompleted);

    //Set the phi left to rotate on the current rotation segment
    rotate.setPhiLeft(fabs(rotate.phi) - rotate.phiCompleted);
    ROS_INFO("rads left: %f", rotate.phiLeft);

    //use rotate.phiLeft to decide what omega should be, as per plan
    if (rotate.phiLeft <= 0.0) { // at goal, or overshot; stop!
        scheduledOmega = 0.0;
    } else if (rotate.phiLeft <= rotationalDecelerationPhi) { //possibly should be braking to a halt
        // dist = 0.5*a*t_halt^2; so t_halt = sqrt(2*dist/a);   v = a*t_halt
        // so v = a*sqrt(2*dist/a) = sqrt(2*dist*a)
        scheduledOmega = sqrtf(2 * rotate.phiLeft * maxAlpha);
        ROS_INFO("braking zone: o_sched = %f", scheduledOmega);
    } else {
        //Not ready to decelerate robot so scheduled omega will be the max omega (need to accelerate 
        //or hold the max omega
        scheduledOmega = maxOmega;
    }
    ROS_INFO("Slow down scheduled omega is: %f", scheduledOmega);
    return scheduledOmega;
}

/**
 * Gets the change in phi since the last call to this method.
 * Tracks the last odom callback phi value and determines whether the
 * current odom callback phi has switched from + to - or vice versa. Then
 * it will calculate the delta phi based on the last odom callback phi and the
 * current odom callback phi.
 * 
 * @param turnRight - whether the robot is turning right currently
 * 
 * @return the delta phi since the last call to this method
 */
float getDeltaPhi(bool turnRight){
    float callbackPhi = callback.odomPhi;
    float dPhi = 0;
    
    //When turning from negative to positive phi
    if (lastCallbackPhi < 0 && callbackPhi >= 0) {
        //If the turn is to the right of the robot, normalize the rotation
        if (turnRight) {
            dPhi = (2 * M_PI) - callbackPhi + fabs((-2*M_PI) - lastCallbackPhi);
        } else {
            //Otherwise, just add the last and current callback phis together if turning right to left
            dPhi = callbackPhi + fabs(lastCallbackPhi);
        }
    } else if (lastCallbackPhi > 0 && callbackPhi <= 0) {
        //When turning from positive to negative phi
        //If the turn is to the right, just add the last and current callback phis together
        if (turnRight) {
            dPhi = lastCallbackPhi + fabs(callbackPhi);
        } else {
            //Otherwise, normalize the rotation if turning right to left
            dPhi = (2 * M_PI) - lastCallbackPhi + fabs((-2*M_PI) - callbackPhi);
        }
    } else if (callbackPhi < 0 && lastCallbackPhi > callbackPhi) {
        //When phi is decreasing and has gone from positive to negative,
        //we need the difference between the last and the current callback phi
        dPhi = fabs(lastCallbackPhi - callbackPhi); 
    } else if (callbackPhi < 0 && lastCallbackPhi < callbackPhi) {
        //When phi is decreasing and is negative, we need the 
        //difference between the current callback and last callback phi
        dPhi = fabs(callbackPhi - lastCallbackPhi);
    } else if (callbackPhi > 0 && lastCallbackPhi > callbackPhi) {
        //When phi is positive and increasing, we need the difference
        //between the last and current callback phis
        dPhi = lastCallbackPhi - callbackPhi;
    } else if (callbackPhi > 0 && lastCallbackPhi < callbackPhi) {
        //When phi is positive and decreasing, we need the difference
        //between the current and last callback phis
        dPhi = callbackPhi - lastCallbackPhi;
    }
    
    //We now have the last callback phi
    lastCallbackPhi = callbackPhi;
    return dPhi;

}

/**
 * Speeds up the robot's angular velocity according to the scheduled slow-down 
 * omega and the robot's odom omega as well as rotational accerlation constants.
 * 
 * @param scheduledOmega - the omega scheduled via the turn slow down function
 * @return the new omega spin command to publish to the robot's motors
 */
float turnSpeedUp(float scheduledOmega) {
    float newOmegaCommand;

    //how does the current omega compare to the scheduled omega?
    if (fabs(callback.odomOmega) < scheduledOmega) { // maybe we halted
        // may need to ramp up to maxOmega; do so within accel limits
        float testOmega = fabs(callback.odomOmega) + maxAlpha * callback.dt; // if callbacks are slow, this could be abrupt
        newOmegaCommand = (testOmega < scheduledOmega) ? testOmega : scheduledOmega; //choose lesser of two options
    } else if (fabs(callback.odomOmega) > scheduledOmega) { //travelling too fast--this could be trouble
        // ramp down to the scheduled omega.  However, scheduled omega might already be ramping down at maxAlpha.
        // need to catch up, so ramp down even faster than maxAlpha.  Try 1.2*maxAlpha.
        ROS_INFO("odom omega: %f; sched omega: %f", fabs(callback.odomOmega), scheduledOmega);

        //moving too fast decelerating faster than nominal maxAlpha
        float testOmega = fabs(callback.odomOmega) - 1.2 * maxAlpha * callback.dt;
        // choose larger of two..don't overshoot
        newOmegaCommand = (testOmega < scheduledOmega) ? testOmega : scheduledOmega;
    } else {
        //Just hold the scheduled omega
        newOmegaCommand = scheduledOmega;
    }

    ROS_INFO("New omega speedup command is: %f", newOmegaCommand);
    return newOmegaCommand;
}

/**
 * Determines if the robot has rotated successfully to the desired end rotation.
 *
 * @return whether the robot rotated to its desired phi 
 */
bool isDoneRotating() {
    //for degrees do: currRotation = currRotation + (new_cmd_omega) * (currTime - startTime);
    ROS_INFO("The current rotation in rads is: %f", rotate.phiCompleted);

    if (rotate.phiCompleted >= fabs(rotate.phi)){
        return true;
    }
    return false;
}

/**
 * Rotates the robot with a trapezoidal speed up and speed down profile in order to reach the
 * desired end phi. When lidar alarm or estop is on, the rotation will stop but the function will not return.
 * This means that the function will loop until either lidar alarm or estop is turned off. Once
 * these are both off, the rotation will resume and continue until it meets the desired end phi.
 * When software halt is enabled, the function will have the same behavior as in it will continually publish
 * a command omega of zero until it is disabled. Once disabled, the rotation will continue until the end phi.
 * 
 * @param velocityPublisher - the publisher to publish the new velocity to
 * @param rTimer - the timer of the rotation movement
 * @param z - the spin rate desired for the robot
 * @param endPhi - the desired end rotation of the robot
 */
void rotateToPhi(ros::Publisher velocityPublisher, ros::Rate rTimer, float endPhi) {

    //Initialize the rotation of the robot according to the given end phi
    bool turnRight = initializeRotation(endPhi, rTimer);
    bool firstCall = true;
    float startTime = 0.0;
    float currTime = 0.0;
    float scheduledOmega = 0.0;
    float newOmegaCommand = 0.1;
    lastCallbackPhi = rotate.startPhi;

    //Rotate to the given end phi while the robot is OK.
    //Hold rotation if any stop commands were given via estop, lidar, or halt
    while (ros::ok()) {
        ros::spinOnce();

        //Only rotation when estop, lidar alarm, and halt are off
        if (!(estop.on || lidar.alarm || halt)) {
            //Handle setting up timer for rotation since beginning of method call.
            if (firstCall && endPhi != 0.0) {
                firstCall = false;
                startTime = ros::Time::now().toSec();
            } else if (endPhi == 0.0) {
                return;
            }

            //calculates the new omega with trapezoidal velocity profiling
            scheduledOmega = turnSlowDown(turnRight);
            newOmegaCommand = turnSpeedUp(scheduledOmega);

            //assigns the sign to omega
            if (turnRight) {
                newOmegaCommand = -1.0 * (newOmegaCommand);
            }

            ROS_INFO("omega cmd vel: %f", newOmegaCommand); // debug output

            //Issue latest angular velocity command
            velocityCommand.angular.z = newOmegaCommand;

            currTime = ros::Time::now().toSec();

            //Determine if the robot has met it's end phi rotation
            bool doneRotating = isDoneRotating();

            //Set angular z velocity to 0 when done rotating
            if (doneRotating) {
                velocityCommand.angular.z = 0.0;
            }

            //Publish latest velocity command
            velocityPublisher.publish(velocityCommand);

            // sleep for remainder of timed iteration
            rTimer.sleep();

            //Break from rotation loop if done rotating
            if (doneRotating) {
                break;
            }
        } else if (halt) {
            //We want to halt when software halt is enabled, so we constantly update velocity rotation command to zero.
            velocityCommand.linear.z = 0.0;
            velocityPublisher.publish(velocityCommand);
            ROS_INFO("Software halt enabled. Robot is static.");
        }
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
 * Initializes the phi of the robot according to the odom callback.
 * Resets the phi completed during the current rotation thus far and
 * sets the end phi to reach during this current rotation. Also determines
 * what direction the robot is rotating in via end phi sign.
 * 
 * @param endPhi - the angle to rotate the robot to
 * @param rTimer - the timer for rotation movement
 * @return whether the robot is turning right
 */
bool initializeRotation(float endPhi, ros::Rate rTimer) {
    bool turnRight;
    
    //Validate odom callback and set up initial rotation values
    if (odomCallValidation(rTimer)) {
        resetVelocityCommand();
        rotate.setStartPhi(callback.odomPhi);
        rotate.resetPhiCompleted();
        rotate.setPhi(endPhi);

        //Turning right (clockwise) if negative end phi
        if (endPhi < 0) {
            turnRight = true;
            ROS_INFO("Turning to the right.");
        } else if (endPhi > 0) {
            //Turning left (counter-clockwise) if positive end phi
            turnRight = false;
            ROS_INFO("Turning to the left.");
        } else {
            //Not turning..
            turnRight = false;
            ROS_INFO("Rotation was called with zero phi... Returning.");
        }
    }

    return turnRight;
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
        modifiedSegment = segment;
        segment.setLength(newSegmentLength);
        segment.resetLengthCompleted();
        lidar.setStop(true);
        lidar.setModifiedSegment(true);
    } else if (lidar.closestPing > maxSafeRange && lidar.modifiedSegment) {
        //run on the rest of the desired segment length if the previous
        //segment length was a modified, slow-down segment
        float origPathLength = modifiedSegment.length;
        float currLengthCompleted = segment.lengthCompleted;
        float prevLengthCompleted = modifiedSegment.lengthCompleted;
        segment = modifiedSegment;
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
    velocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
    //velocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
    ros::Subscriber sub = nodeHandle.subscribe("/robot0/odom", 1, odomCallback);
    //ros::Subscriber sub = nodeHandle.subscribe("/robot0/odom", 4, odomCallback);

    ros::Subscriber ping_dist_subscriber = nodeHandle.subscribe("lidar_dist", 1, pingDistanceCallback);
    ros::Subscriber lidar_alarm_subscriber = nodeHandle.subscribe("lidar_alarm", 1, lidarAlarmCallback);
    ros::Subscriber estop_subscriber = nodeHandle.subscribe("estop_listener", 1, estopCallback);
    ros::Subscriber halt_subscriber = nodeHandle.subscribe("halt_cmd", 1, haltCallback);

    ros::Rate rTimer(1 / changeInTime); // frequency corresponding to chosen sample period DT; the main loop will run this fast
    
    //Move the robot forward 4.75 meters
    moveOnSegment(velocityPublisher, rTimer, 4.75); //4.75
    
    //turn the robot nearly 90 degrees
    //turn the robot nearly 180 degrees
    rotateToPhi(velocityPublisher, rTimer, -1.55);
    
    moveOnSegment(velocityPublisher, rTimer, 12.3);
    
    //turn the robot nearly 180 degrees
  //  rotateToPhi(velocityPublisher, rTimer, -3.13);
    
//    //Move the robot forward 12.3 meters
//    moveOnSegment(velocityPublisher, rTimer, 12.3);
   
    //turn the robot nearly 90 degrees
    rotateToPhi(velocityPublisher, rTimer, -1.55);
   
    //Move the robot forward 8 meters
    moveOnSegment(velocityPublisher, rTimer, 8);
    return 0;
}

