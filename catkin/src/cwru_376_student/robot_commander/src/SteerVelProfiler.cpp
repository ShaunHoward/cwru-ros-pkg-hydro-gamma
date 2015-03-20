/* 
 * File:   steer_vel_profiler.cpp
 * Author: shaun
 * 
 * Created on March 6, 2015, 5:08 PM
 */

#include "SteerVelProfiler.h"

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
    this->currSegLengthToGo = orig.currSegLengthToGo;
}

SteerVelProfiler::SteerVelProfiler() {
    this->odomX = 0;
    this->odomY = 0;
    this->odomPhi = 0;
    this->odomOmega = 0;
    this->odomVel = 0;
    this->dt = 1 / UPDATE_RATE;
    this->currSegLengthToGo = 0;
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

void SteerVelProfiler::setSegLengthToGo(double segToGo){
    this->currSegLengthToGo = segToGo;
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
    double lengthCompleted = sqrt(deltaX * deltaX + deltaY * deltaY);
    ROS_INFO("dist traveled: %f", lengthCompleted);
    //Set the distance left to travel on the current segment
    //currSegLengthToGo = segmentLength - lengthCompleted;

    //use segment.distanceLeft to decide what vel should be, as per plan
    if (currSegLengthToGo <= 0.0) { // at goal, or overshot; stop!
        scheduledVelocity = 0.0;
    } else if (currSegLengthToGo <= DECEL_DIST) { //possibly should be braking to a halt
        // dist = 0.5*a*t_halt^2; so t_halt = sqrt(2*dist/a);   v = a*t_halt
        // so v = a*sqrt(2*dist/a) = sqrt(2*dist*a)
        scheduledVelocity = sqrt(2 * currSegLengthToGo * MAX_ACCEL);
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

        double testVelocity = odomVel - 1.2 * MAX_ACCEL * dt; //moving too fast--try decelerating faster than nominal a_max

        newVelocityCommand = (testVelocity < scheduledVelocity) ? testVelocity : scheduledVelocity; // choose larger of two options...don't overshoot scheduled_vel
    } else {
        newVelocityCommand = scheduledVelocity; //silly third case: this is already true, if here.  Issue the scheduled velocity
    }
    ROS_INFO("New speedup command is: %f", newVelocityCommand);
    return newVelocityCommand;
}

/**
 * Slows down the robot's rotational velocity trapezoidally according to
 * the phi left to rotate on the current rotation segment as well as
 * rotational deceleration constants.
 * 
 * @param turnRight - whether the robot is currently turning right
 * @return the scheduled omega for slowing down the robot spin
 */
double SteerVelProfiler::turnSlowDown(bool turnRight, double desiredPhi) {
    double scheduledOmega = 0.0f;

//    //Set the phi (angle) turned thus far in the current rotation segment
//    rotate.setPhiCompleted(rotate.phiCompleted + getDeltaPhi(turnRight));
//    ROS_INFO("Phi rotated: %f", rotate.phiCompleted);
//
//    //Set the phi left to rotate on the current rotation segment
//    rotate.setPhiLeft(fabs(rotate.phi) - rotate.phiCompleted);
//    ROS_INFO("rads left: %f", rotate.phiLeft);
    double phiLeft = desiredPhi;

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
        newOmegaCommand = (testOmega < scheduledOmega) ? testOmega : scheduledOmega; //choose lesser of two options
    } else if (fabs(odomOmega) > scheduledOmega) { //travelling too fast--this could be trouble
        // ramp down to the scheduled omega.  However, scheduled omega might already be ramping down at maxAlpha.
        // need to catch up, so ramp down even faster than maxAlpha.  Try 1.2*maxAlpha.
        ROS_INFO("odom omega: %f; sched omega: %f", fabs(odomOmega), scheduledOmega);

        //moving too fast decelerating faster than nominal maxAlpha
        double testOmega = fabs(odomOmega) - 1.2 * MAX_ALPHA * dt;
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
    return dPhi;
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

