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
#include <float.h>

#ifndef STEER_VEL_PROFILER_H
#define	STEER_VEL_PROFILER_H

class SteerVelProfiler {
public:
    SteerVelProfiler(const SteerVelProfiler& orig);
    SteerVelProfiler(const double& maxAlpha, double& rotationalDecelerationPhi,
        const double& MAX_ACCEL, double& decelerationDistance, const double& MAX_SPEED,
        const double& maxOmega);
    
    // Compute the forward speed profile with trapezoidal speed profiling
    // These will recognize Estops, lidar alarm, and software halt.
    double trapezoidalSlowDown(double segmentLength);
    double trapezoidalSpeedUp(double scheduledVelocity);
    
    
    // Compute the rotational speed profile with trapezoidal speed profiling
    // These will recognize Estops, lidar alarm, and software halt.
    float turnSlowDown(bool turnRight, float desiredPhi);
    float turnSpeedUp(float scheduledOmega);
    void setOdomXYValues(float odomX, float odomY);
    void setOdomRotationValues(float odomPhi, float odomOmega);
    void setOdomForwardVel(float odomVel);
    void setOdomDT(float dt);
    void setSegLengthToGo(float segToGo);
    float odomPhi;
    float lastCallbackPhi;
    float odomOmega;
    float dt_;
    float maxAlpha;
    float maxOmega;
    float rotationalDecelerationPhi;
    float MAX_ACCEL;
    float odom_x_;
    float odom_y_;
    float current_seg_ref_point_0;
    float current_seg_ref_point_1;
    float decelerationDistance;
    float MAX_SPEED;
    float odom_vel_;
    float current_seg_length_to_go_;
    float getDeltaPhi(bool turnRight);
    bool initializeRotation(float endPhi); 

};

#endif	/* STEER_VEL_PROFILER_H */

