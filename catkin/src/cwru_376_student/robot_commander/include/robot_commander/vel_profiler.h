#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <float.h>

struct callback{
	// globals for communication w/ callbacks:
	float odomVelocity = 0.0; // measured/published system speed
	float odomOmega = 0.0; // measured/published system yaw rate (spin)
	float odomX = 0.0;
	float odomY = 0.0;
	float odomPhi = 0.0;
	float odomDt = 0.0;
	ros::Time lastCallbackTime;
	float dt = 0.0;
	float quaternionZ = 0.0, quaternionW = 0.0;
	
	void setOdomPhi(){
		odomPhi = 2.0 * atan2(quaternionZ, quaternionW); // cheap conversion from quaternion to heading for planar motion		
	}
	void setOdomVelocity(float odomVelocity){
		this.odomVelocity = odomVelocity;
	}
	void setOdomOmega(float odomOmega){
		this.odomOmega = odomOmega;
	}
	void setOdomX(float odomX){
		this.odomX = odomX;
	}
	void setOdomY(float odomY){
		this.odomY = odomY;
	}
	void setOdomPhi(float odomPhi){
		this.odomPhi = odomPhi;
	}
	void setOdomDt(float odomDt){
		this.odomDt = odomDt;
	}
	void setDt(float dt){
		this.dt = dt;
	}
	void setOdomDt(float odomDt){
		this.odomDt = odomDt;
	}
	void setQuaternionZ(float z){
		this.quaternionZ = z;
	}
	void setQuaternionW(float w){
		this.quaternionW = w;
	}
	void printValues(){
		ROS_INFO("odom CB: x = %f, y= %f, phi = %f, v = %f, omega = %f", odomX, odomY, odomPhi, odomVel, odomOmega);		
	}
}

struct segment{
	//here's a subtlety:  might be tempted to measure distance to the goal, instead of distance from the start.
	// HOWEVER, will NEVER satisfy distance to goal = 0 precisely, but WILL eventually move far enought to satisfy distance travelled condition
	float lengthCompleted = 0.0; // need to compute actual distance travelled within the current segment
	float startX = 0.0; // fill these in with actual values once odom message is received
	float startY = 0.0; // subsequent segment start coordinates should be specified relative to end of previous segment
	float startPhi = 0.0;
	float distanceLeft = 0.0;
	
	void setStartX(float startX){
		this.startX = startX;
	}
	
	void setStartY(float startY){
		this.startY = startY;
	}
	
	void setStartPhi(float startPhi){
		this.startPhi = startPhi;
	}
	
	void setDistanceLeft(float distanceLeft){
		this.distanceLeft = distanceLeft;
	}
	
	void setLengthCompleted(float lengthCompleted){
		this.lengthCompleted = lengthCompleted;
	}
	
	void resetLengthCompleted(){
		this.lengthCompleted = 0.0;
	}
	
	void resetDistanceLeft(){
		this.distanceLeft = 0.0;
	}
	
	void printValues(){
		ROS_INFO("start pose: x %f, y= %f, phi = %f", startX, startY, startPhi);
	}
}

struct lidar{
	//Lidar variables
	float closestPing = 0.0f;
	bool alarm = false;
	bool stop = false;
	
	void setAlarm(bool alarm){
		this.alarm = alarm;
	}
	void setStop(bool stop){
		this.stop = stop;
	}
	void setClosestPing(float closestPing){
		this.closestPing = closestPing;
	}
}

struct estop{
	bool on = false;
	void set(bool on){
		this.on = on;
	}
}

struct velocityProfile{
	// set some dynamic limits...
	const float maxVelocity = 2.0; //1m/sec is a slow walk
	const float minVelocity = 0.1; // if command velocity too low, robot won't move
	const float maxAcceleration = 0.1; //1m/sec^2 is 0.1 g's
	const float maxOmega = 1.0; //1 rad/sec-> about 6 seconds to rotate 1 full rev
	const float maxAlpha = 0.5; //0.5 rad/sec^2-> takes 2 sec to get from rest to full omega
	const float changeInTime = 0.050; // choose an update rate of 20Hz; go faster with actual hardware
	
	// compute some properties of trapezoidal velocity profile plan:
	float accelerationTime = maxVelocity / maxAcceleration; //...assumes start from rest
	float decelerationTime = maxVelocity / maxAcceleration; //(for same decel as accel); assumes brake to full halt
	float accelerationDistance = 0.5 * maxAcceleration * (accelerationTime * accelerationTime); //distance rqd to ramp up to full speed
	float decelerationDistance = 0.5 * maxAcceleration * (decelerationTime * decelerationTime); //same as ramp-up distance
}

ros::Publisher velocityPublisher;

geometry_msgs::Twist velocityCommand; //create a variable of type "Twist" to publish speed/spin commands

void odomCallback(const nav_msgs::Odometry& odom_rcvd);

bool rotate(float startTime, float currTime, float commandOmega, float currRotation, float endRotation);

void resetVelocityCommand();

void eStop();

float trapezoidalSlowDown(float segmentLength);

void decideToStop();

float trapezoidalSpeedUp(float scheduledVelocity, float newVelocityCommand);

void moveOnSegment(ros::Publisher velocityPublisher, float segmentLength);

void rotate(ros::Publisher velocityPublisher, ros::Rate rTimer, float z, float endRotation);

bool odomCallValidation(ros::Rate rTimer);

void initializePosition();

void initializeNewMove(ros::Rate rTimer);

void pingDistanceCallback(const std_msgs::Float32& pingDistance);

void lidarAlarmCallback(const std_msgs::Bool& lidarAlarmMsg);

void estopCallback(const std_msgs::Bool& estopMsg);