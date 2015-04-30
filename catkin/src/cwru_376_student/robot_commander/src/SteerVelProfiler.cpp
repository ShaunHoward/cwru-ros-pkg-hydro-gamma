/* 
 * File:   steer_vel_profiler.cpp
 * Author: shaun
 * 
 * Created on March 6, 2015, 5:08 PM
 */

#include "SteerVelProfiler.h"

// saturation function, values -1 to 1
double sat(double x) {
    if (x>1.0) {
        return 1.0;
    }
    if (x< -1.0) {
        return -1.0;
    }
    return x;
}

//SteerVelProfiler::SteerVelProfiler() {
//}
//

SteerVelProfiler::SteerVelProfiler(const SteerVelProfiler& orig) {
    this->odomX = orig.odomX;
    this->odomY = orig.odomY;
    this->odomPhi = orig.odomPhi;
    this->odomOmega = orig.odomOmega;
    this->odomVel = orig.odomVel;
    this->dt = 1 / UPDATE_RATE;
    this->distanceLeft = orig.distanceLeft;
    this->phiCompleted = orig.phiCompleted;
    this->phiLeft = orig.phiLeft;
    this->currSegLength = orig.currSegLength;
    this->turnRight = orig.turnRight;
}

SteerVelProfiler::SteerVelProfiler() {
    this->odomX = 0;
    this->odomY = 0;
    this->odomPhi = 0;
    this->odomOmega = 0;
    this->odomVel = 0;
    this->dt = 1 / UPDATE_RATE;
    this->distanceLeft = 0;
    this->phiCompleted = 0;
    this->phiLeft = 0;
    this->currSegLength = 0;
    this->turnRight = false;
}

void SteerVelProfiler::setOdomXYValues(double odomX, double odomY){
    this->odomX = odomX;
    this->odomY = odomY;
}

void SteerVelProfiler::setOdomRotationValues(double odomPhi, double odomOmega){
    this->odomPhi = odomPhi;
    this->odomOmega = odomOmega;
}

void SteerVelProfiler::setOdomForwardVel(double odomVel){
    this->odomVel = odomVel;
}

void SteerVelProfiler::setOdomDT(double dt){
    this->dt = dt;
}

void SteerVelProfiler::setDistanceLeft(double distanceLeft){
    this->distanceLeft = distanceLeft;
}

void SteerVelProfiler::setCurrSegLength(double currSegLength){
    this->currSegLength = currSegLength;
}

///**
// * Slows down the robot's forward velocity trapezoidally according to
// * the segment length distance traveled and distance left to travel as
// * well as deceleration constants.
// * 
// * @param segmentLength - the segment to slow the robot down along
// * @return the newly scheduled velocity determined by the algorithm
// */
//double SteerVelProfiler::reverseSlowDown(double segment_error) {
//    //Compute distance traveled thus far on the current segment
//    double deltaX = odomX - current_seg_ref_point_0;
//    double deltaY = odomY - current_seg_ref_point_1;
//    double scheduledVelocity = 0.0f;
//    //Calculate the length completed along the current segment thus far
//    double lengthCompleted = sqrt(deltaX * deltaX + deltaY * deltaY);
//    ROS_INFO("dist traveled: %f", lengthCompleted);
//    //Set the distance left to travel on the current segment
//    //currSegLengthToGo = segmentLength - lengthCompleted;
//
//    //use segment.distanceLeft to decide what vel should be, as per plan
//    if (currSegLengthToGo > -.05 && currSegLengthToGo <= 0.0) { // at goal, or overshot; stop!
//       //take this out and replace with slow down in reverse distance.
//        scheduledVelocity = 0.0;
//    } else if (currSegLengthToGo <= decelerationDistance) { //possibly should be braking to a halt
//        // dist = 0.5*a*t_halt^2; so t_halt = sqrt(2*dist/a);   v = a*t_halt
//        // so v = a*sqrt(2*dist/a) = sqrt(2*dist*a)
//        scheduledVelocity = sqrt(2 * currSegLengthToGo * maxAccel);
//        ROS_INFO("braking zone: v_sched = %f", scheduledVelocity);
//    } else {
//        //Not ready to decelerate robot so scheduled velocity will be the max velocity (need to accelerate 
//        //or hold the max velocity
//        scheduledVelocity = maxSpeed;
//    }
//    ROS_INFO("Slow down scheduled velocity is: %f", scheduledVelocity);
//    return scheduledVelocity;
//}

/**
 * Slows down the robot's forward velocity trapezoidally according to
 * the segment length distance traveled and distance left to travel as
 * well as deceleration constants.
 * 
 * @param segmentLength - the segment to slow the robot down along
 * @return the newly scheduled velocity determined by the algorithm
 */
double SteerVelProfiler::trapezoidalSlowDown(double segmentLength) {
    //Compute distance traveled thus far on the current segment
    double deltaX = odomX - current_seg_ref_point_0;
    double deltaY = odomY - current_seg_ref_point_1;
    double scheduledVelocity = 0.0f;
    //Calculate the length completed along the current segment thus far
    lengthCompleted = sqrt(deltaX * deltaX + deltaY * deltaY);
    ROS_INFO("dist traveled: %f", lengthCompleted);
    //Set the distance left to travel on the current segment
    distanceLeft = segmentLength - lengthCompleted;

    //use segment.distanceLeft to decide what vel should be, as per plan
    if (distanceLeft <= 0.0) { // at goal, or overshot; stop!
        scheduledVelocity = 0.0;
    } else if (distanceLeft <= DECEL_DIST) { //possibly should be braking to a halt
        // dist = 0.5*a*t_halt^2; so t_halt = sqrt(2*dist/a);   v = a*t_halt
        // so v = a*sqrt(2*dist/a) = sqrt(2*dist*a)
        scheduledVelocity = sqrt(2 * distanceLeft * MAX_ACCEL);
        ROS_INFO("braking zone: v_sched = %f", scheduledVelocity);
    } else {
        //Not ready to decelerate robot so scheduled velocity will be the max velocity (need to accelerate 
        //or hold the max velocity
        scheduledVelocity = MAX_SPEED;
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
double SteerVelProfiler::trapezoidalSpeedUp(double scheduledVelocity) {
    double newVelocityCommand;
    //how does the current velocity compare to the scheduled vel?
    if (odomVel < scheduledVelocity) { // maybe we halted, e.g. due to estop or obstacle;
        // may need to ramp up to v_max; do so within accel limits
        double testVelocity = odomVel + MAX_ACCEL * dt; // if callbacks are slow, this could be abrupt
        // operator:  c = (a>b) ? a : b;
        newVelocityCommand = (testVelocity < scheduledVelocity) ? testVelocity : scheduledVelocity; //choose lesser of two options
        // this prevents overshooting scheduled_vel
    } else if (odomVel > scheduledVelocity) { //travelling too fast--this could be trouble
        // ramp down to the scheduled velocity.  However, scheduled velocity might already be ramping down at a_max.
        // need to catch up, so ramp down even faster than a_max.  Try 1.2*a_max.
        ROS_INFO("odom vel: %f; sched vel: %f", odomVel, scheduledVelocity); //debug/analysis output; can comment this out

        double testVelocity = fabs(odomVel) - 1.2 * MAX_ACCEL * dt; //moving too fast--try decelerating faster than nominal a_max

        newVelocityCommand = (testVelocity < scheduledVelocity) ? testVelocity : scheduledVelocity; // choose larger of two options...don't overshoot scheduled_vel
    } else {
        newVelocityCommand = scheduledVelocity; //silly third case: this is already true, if here.  Issue the scheduled velocity
    }

    //give the robot some starting speed
    if (newVelocityCommand != 0 && fabs(newVelocityCommand) < MIN_SPEED){
	newVelocityCommand = sgn(newVelocityCommand) * MIN_SPEED;
    }

    ROS_INFO("New speedup command is: %f", newVelocityCommand);
    return newVelocityCommand;
}

double SteerVelProfiler::min_dang(double dang) {
    if (dang > M_PI) dang -= 2.0 * M_PI;
    if (dang<-M_PI) dang += 2.0 * M_PI;
    return dang;
}

/**
 * Slows down the robot's rotational velocity trapezoidally according to
 * the phi left to rotate on the current rotation segment as well as
 * rotational deceleration constants.
 * 
 * @param turnRight - whether the robot is currently turning right
 * @return the scheduled omega for slowing down the robot spin
 */
double SteerVelProfiler::turnSlowDown(bool turnRight_) {
    double scheduledOmega = 0.0f;
    turnRight = turnRight_;
    
    if(firstCall){
        firstCall = false;
        initOdomPhi = odomPhi;
    }

    //Set the phi (angle) turned thus far in the current rotation segment
    //phiCompleted = phiCompleted + getDeltaPhi(turnRight);
    phiCompleted = min_dang(odomPhi - initOdomPhi);
    ROS_INFO("Phi rotated: %f", phiCompleted);

    //Set the phi left to rotate on the current rotation segment
    phiLeft = desiredPhi - fabs(phiCompleted);
    //phiLeft = min_dang(desiredPhi - odomPhi);
    ROS_INFO("rads left: %f", phiLeft);
    //double phiLeft = desiredPhi;

    //use rotate.phiLeft to decide what omega should be, as per plan
    if (phiLeft <= 0.0) { // at goal, or overshot; stop!
        scheduledOmega = 0.0;
    } else if (phiLeft <= ROT_DECEL_PHI) { //possibly should be braking to a halt
        // dist = 0.5*a*t_halt^2; so t_halt = sqrt(2*dist/a);   v = a*t_halt
        // so v = a*sqrt(2*dist/a) = sqrt(2*dist*a)
        scheduledOmega = sqrtf(2 * phiLeft * MAX_ALPHA);
        ROS_INFO("braking zone: o_sched = %f", scheduledOmega);
    } else {
        //Not ready to decelerate robot so scheduled omega will be the max omega (need to accelerate 
        //or hold the max omega
        scheduledOmega = MAX_OMEGA;
    }
    ROS_INFO("Slow down scheduled omega is: %f", scheduledOmega);
    return scheduledOmega;
}

/**
 * Speeds up the robot's angular velocity according to the scheduled slow-down 
 * omega and the robot's odom omega as well as rotational accerlation constants.
 * 
 * @param scheduledOmega - the omega scheduled via the turn slow down function
 * @return the new omega spin command to publish to the robot's motors
 */
double SteerVelProfiler::turnSpeedUp(double scheduledOmega) {
    double newOmegaCommand;

    //how does the current omega compare to the scheduled omega?
    if (fabs(odomOmega) < scheduledOmega) { // maybe we halted
        // may need to ramp up to maxOmega; do so within accel limits
        double testOmega = fabs(odomOmega) + MAX_ALPHA * dt; // if callbacks are slow, this could be abrupt
        newOmegaCommand = (testOmega > scheduledOmega) ? testOmega : scheduledOmega; //choose lesser of two options
    } else if (fabs(odomOmega) > scheduledOmega) { //travelling too fast--this could be trouble
        // ramp down to the scheduled omega.  However, scheduled omega might already be ramping down at maxAlpha.
        // need to catch up, so ramp down even faster than maxAlpha.  Try 1.2*maxAlpha.
        ROS_INFO("odom omega: %f; sched omega: %f", fabs(odomOmega), scheduledOmega);

        //moving too fast decelerating faster than nominal maxAlpha
        double testOmega = fabs(odomOmega) - 1.2 * MAX_ALPHA * dt;
        // we want to decrease speed here
        newOmegaCommand = (testOmega < scheduledOmega) ? testOmega : scheduledOmega;
    } else {
        //Just hold the scheduled omega
        newOmegaCommand = scheduledOmega;
    }

    //give the robot some starting speed
    if(fabs(newOmegaCommand) < MIN_OMEGA && newOmegaCommand != 0.0){
        newOmegaCommand = sgn(newOmegaCommand) * MIN_OMEGA;
    }

    ROS_INFO("New omega speedup command is: %f", newOmegaCommand);
    return newOmegaCommand;
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
double SteerVelProfiler::getDeltaPhi(bool turnRight){
    double callbackPhi = odomPhi;
    double dPhi = 0;
    
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
    ROS_INFO("The delta phi is: %f", dPhi);
    return dPhi;
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
void SteerVelProfiler::rotateToPhi(ros::Publisher velocityPublisher, geometry_msgs::Twist velocityCommand,
        float endPhi, bool turnRight) {

    ros::Rate rTimer(UPDATE_RATE);
    //Initialize the rotation of the robot according to the given end phi
   // bool turnRight = initializeRotation(endPhi);
    bool firstCall = true;
    float scheduledOmega = 0.0;
    float newOmegaCommand = 0.0;
    resetSegValues();
    lastCallbackPhi = odomPhi;
    desiredPhi = endPhi;

    //Rotate to the given end phi while the robot is OK.
    //Hold rotation if any stop commands were given via estop, lidar, or halt
    while (ros::ok()) {
        ros::spinOnce();

        //Only rotation when estop, lidar alarm, and halt are off
 //       if (!(estop.on || lidar.alarm || halt)) {
            //Handle setting up timer for rotation since beginning of method call.
            if (endPhi == 0.0) {
                return;
            }

            //calculates the new omega with trapezoidal velocity profiling
            scheduledOmega = turnSlowDown(turnRight);
            newOmegaCommand = turnSpeedUp(scheduledOmega);

            //assigns the sign to omega
//            if (turnRight) {
//                newOmegaCommand = -1.0 * (newOmegaCommand);
//            }
            
            ROS_INFO("New steering controller omega: %f", newOmegaCommand);

            //ROS_INFO("omega cmd vel: %f", newOmegaCommand); // debug output

            //Freeze forward motion of robot
            velocityCommand.linear.x = 0.0;
            
            //Issue latest angular velocity command
            velocityCommand.angular.z = newOmegaCommand;

            //currTime = ros::Time::now().toSec();

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
//        } else if (halt) {
//            //We want to halt when software halt is enabled, so we constantly update velocity rotation command to zero.
//            velocityCommand.linear.z = 0.0;
//            velocityPublisher.publish(velocityCommand);
//            ROS_INFO("Software halt enabled. Robot is static.");
//        }
    }
}

/**
 * Determines if the robot has rotated successfully to the desired end rotation.
 *
 * @return whether the robot rotated to its desired phi 
 */
bool SteerVelProfiler::isDoneRotating() {
    //for degrees do: currRotation = currRotation + (new_cmd_omega) * (currTime - startTime);
    ROS_INFO("The current rotation in rads is: %f", phiCompleted);
    if (phiCompleted >= fabs(desiredPhi)){
        return true;
    }
//    if (-HEAD_ERR_TOL < headingError && headingError < HEAD_ERR_TOL){
//        return true;
//    }

    return false;
}

/**
 * Resets the segment values for this steering velocity profiler instance.
 * This will reset data necessary to use the trapezoidal forward and steering
 * velocity profiles.
 */
void SteerVelProfiler::resetSegValues(){
    phiLeft = 0;
    phiCompleted = 0;
    distanceLeft = 0;
    lengthCompleted = 0;
    firstCall = true;
    initOdomPhi = 0;
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
bool SteerVelProfiler::initializeRotation(double endPhi) {
    bool turnRight;
    desiredPhi = endPhi;
    resetSegValues();
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

    return turnRight;
}

SteerVelProfiler::~SteerVelProfiler() {
}

//int main(int argc, char** argv) {
//    while (ros::ok()){
//    }
//    return 0;
//}

