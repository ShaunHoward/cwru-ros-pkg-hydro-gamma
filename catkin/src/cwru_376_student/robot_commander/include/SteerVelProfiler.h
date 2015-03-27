/* 
 * File:   steer_vel_profiler.h
 * Author: shaun
 *
 * Created on March 6, 2015, 5:08 PM
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

#ifndef STEER_VEL_PROFILER_H
#define	STEER_VEL_PROFILER_H

// dynamic limitations
const double MAX_SPEED = 1.0; // m/sec; adjust this
const double MAX_OMEGA = 1.0; //1.0; // rad/sec; adjust this
const double MAX_ACCEL = 0.5; // m/sec^2; adjust this
const double MAX_ALPHA = 0.5; // rad/sec^2; adjust this

const double LENGTH_TOL = 0.05; // tolerance for path; adjust this
const double HEADING_TOL = 0.05; // heading tolerance; adjust this

const double UPDATE_RATE = 50.0; // choose the desired-state publication update rate
//The lateral error tolerance value.
const double LAT_ERR_TOL = 0.05;
const double HEAD_ERR_TOL = 0.1;
const double TRIP_ERR_TOL = 0.25;
const double OMEGA_GAIN = 5;

// compute some properties of trapezoidal velocity profile plan:
double ACCEL_TIME = MAX_SPEED / MAX_ACCEL; //...assumes start from rest
double DECEL_TIME = ACCEL_TIME; //(for same decel as accel); assumes brake to full halt
double ACCEL_DIST = 0.5 * MAX_ACCEL * (ACCEL_TIME * ACCEL_TIME); //distance rqd to ramp up to full speed
double DECEL_DIST = 0.5 * MAX_ACCEL * (DECEL_TIME * DECEL_TIME); //same as ramp-up distance

// compute properties of rotational trapezoidal velocity profile plan:
double TURN_ACCEL_TIME = MAX_OMEGA / MAX_ALPHA; //...assumes start from rest
double TURN_DECEL_TIME = TURN_ACCEL_TIME; //(for same decel as accel); assumes brake to full halt
//double turnAccelPhi = 0.5 * maxAlpha * (turnAccelTime * turnAccelTime); //same as ramp-up distance
double ROT_ACCEL_PHI = 0.5 * MAX_ALPHA * (TURN_ACCEL_TIME * TURN_ACCEL_TIME);
double ROT_DECEL_PHI = 0.5 * MAX_ALPHA * (TURN_DECEL_TIME * TURN_DECEL_TIME);

class SteerVelProfiler {
public:
    SteerVelProfiler(const SteerVelProfiler& orig);
    SteerVelProfiler();
    virtual ~SteerVelProfiler();
    
    // Compute the forward speed profile with trapezoidal speed profiling
    // These will recognize Estops, lidar alarm, and software halt.
    double trapezoidalSlowDown(double segmentLength);
    double trapezoidalSpeedUp(double scheduledVelocity);
    
    
    // Compute the rotational speed profile with trapezoidal speed profiling
    // These will recognize Estops, lidar alarm, and software halt.
    double turnSlowDown(bool turnRight);
    double turnSpeedUp(double scheduledOmega);
    void rotateToPhi(ros::Publisher velocityPublisher, geometry_msgs::Twist velocityCommand,
        float endPhi, bool turnRight);
    bool isDoneRotating();
    void setOdomXYValues(double odomX, double odomY);
    void setOdomRotationValues(double odomPhi, double odomOmega);
    void setOdomForwardVel(double odomVel);
    void setOdomDT(double dt);
    void setDistanceLeft(double distanceLeft);
    void setCurrSegLength(double currSegLength);
    double min_dang(double dang);
    void resetSegValues();
    double desiredPhi;
    double odomPhi;
    double phiCompleted;
    double phiLeft;
    double lastCallbackPhi;
    double initOdomPhi;
    double odomOmega;
    double dt;
    double odomX;
    double odomY;
    double current_seg_ref_point_0;
    double current_seg_ref_point_1;
    double odomVel;
    double distanceLeft;
    double lengthCompleted;
    double currSegLength;
    double headingError;
    double lateralError;
    double tripDistError;
    double getDeltaPhi(bool turnRight);
    bool initializeRotation(double endPhi);
    bool turnRight;
    double reverseSlowDown(double segment_error);
    bool firstCall;
};

#endif	/* STEER_VEL_PROFILER_H */

