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

class SteerVelProfiler {
public:
    SteerVelProfiler(const SteerVelProfiler& orig);
    SteerVelProfiler(const double& maxAlpha, double& rotationalDecelerationPhi,
        const double& maxAccel, double& decelerationDistance, const double& maxSpeed,
        const double& maxOmega);
    virtual ~SteerVelProfiler();
    
    // Compute the forward speed profile with trapezoidal speed profiling
    // These will recognize Estops, lidar alarm, and software halt.
    double trapezoidalSlowDown(double segmentLength);
    double trapezoidalSpeedUp(double scheduledVelocity);
    
    
    // Compute the rotational speed profile with trapezoidal speed profiling
    // These will recognize Estops, lidar alarm, and software halt.
    double turnSlowDown(bool turnRight, double desiredPhi);
    double turnSpeedUp(double scheduledOmega);
    void setOdomXYValues(double odomX, double odomY);
    void setOdomRotationValues(double odomPhi, double odomOmega);
    void setOdomForwardVel(double odomVel);
    void setOdomDT(double dt);
    void setSegLengthToGo(double segToGo);
    double odomPhi;
    double lastCallbackPhi;
    double odomOmega;
    double dt;
    double maxAlpha;
    double maxOmega;
    double rotationalDecelerationPhi;
    double maxAccel;
    double odomX;
    double odomY;
    double current_seg_ref_point_0;
    double current_seg_ref_point_1;
    double decelerationDistance;
    double maxSpeed;
    double odomVel;
    double currSegLengthToGo;
    double getDeltaPhi(bool turnRight);
    bool initializeRotation(double endPhi); 
    double reverseSlowDown(double segment_error);
};

#endif	/* STEER_VEL_PROFILER_H */

