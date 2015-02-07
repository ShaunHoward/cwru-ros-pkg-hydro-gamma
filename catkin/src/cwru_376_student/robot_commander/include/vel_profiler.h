#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <float.h>

struct Callback{
	// globals for communication w/ callbacks:
    public:
        float odomVelocity; // measured/published system speed
	float odomOmega; // measured/published system yaw rate (spin)
	float odomX;
	float odomY;
	float odomPhi;
	float odomDt;
	ros::Time lastCallbackTime;
	float dt;
	float quaternionZ, quaternionW;
	
	void setOdomPhi(){
		odomPhi = 2.0 * atan2(quaternionZ, quaternionW); // cheap conversion from quaternion to heading for planar motion		
	}
	void setOdomVelocity(float velocity){
		odomVelocity = velocity;
	}
	void setOdomOmega(float omega){
		odomOmega = omega;
	}
	void setOdomX(float x){
		odomX = x;
	}
	void setOdomY(float y){
		odomY = y;
	}
	void setOdomPhi(float phi){
		odomPhi = phi;
	}
	void setOdomDt(float oDt){
		odomDt = oDt;
	}
	void setDt(float Dt){
		dt = Dt;
	}
	void setQuaternionZ(float z){
		quaternionZ = z;
	}
	void setQuaternionW(float w){
		quaternionW = w;
	}
	void printValues(){
		ROS_INFO("odom CB: x = %f, y= %f, phi = %f, v = %f, omega = %f", odomX, odomY, odomPhi, odomVelocity, odomOmega);		
	}
};

struct Segment{
    public:
	//here's a subtlety:  might be tempted to measure distance to the goal, instead of distance from the start.
	// HOWEVER, will NEVER satisfy distance to goal = 0 precisely, but WILL eventually move far enought to satisfy distance travelled condition
	float lengthCompleted; // need to compute actual distance travelled within the current segment
	float startX; // fill these in with actual values once odom message is received
	float startY; // subsequent segment start coordinates should be specified relative to end of previous segment
	float startPhi;
	float distanceLeft;
	float length;
	
	void copy(const Segment& copyFrom){
		lengthCompleted = copyFrom.lengthCompleted;
		startX = copyFrom.startX;
		startY = copyFrom.startY;
		startPhi = copyFrom.startPhi;
		distanceLeft = copyFrom.distanceLeft;
		length = copyFrom.length;
	}
	
	void setLength(float l){
		length = l;
	}
	
	void setStartX(float x){
		startX = x;
	}
	
	void setStartY(float y){
		startY = y;
	}
	
	void setStartPhi(float phi){
		startPhi = phi;
	}
	
	void setDistanceLeft(float distLeft){
		distanceLeft = distLeft;
	}
	
	void setLengthCompleted(float length){
		lengthCompleted = length;
	}
	
	void resetLengthCompleted(){
		lengthCompleted = 0.0;
	}
	
	void resetDistanceLeft(){
		distanceLeft = 0.0;
	}
	
	void printValues(){
		ROS_INFO("start pose: x %f, y= %f, phi = %f", startX, startY, startPhi);
	}
};

struct Lidar{
    public:
	//Lidar variables
	float closestPing;
	bool alarm;
	bool stop;
	bool modifiedSegment;
	
	void setAlarm(bool alarm_bool){
		alarm = alarm_bool;
	}
	void setStop(bool stop_bool){
		stop = stop_bool;
	}
	void setClosestPing(float closest){
		closestPing = closest;
	}
	void setModifiedSegment(bool modified){
		modifiedSegment = modified;
	}
};

struct Estop{
    public:
	bool on;
	void set(bool on_){
		on = on_;
	}
};

Callback callback;
Segment segment, modifiedSegment;
Lidar lidar;
Estop estop;

// set some dynamic limits...
const float maxVelocity = 5.0; //1m/sec is a slow walk
const float minVelocity = 0.1; // if command velocity too low, robot won't move
const float maxAcceleration = 0.1; //1m/sec^2 is 0.1 g's
const float maxOmega = 1.0; //1 rad/sec-> about 6 seconds to rotate 1 full rev
const float maxAlpha = 0.5; //0.5 rad/sec^2-> takes 2 sec to get from rest to full omega
const float changeInTime = 0.050; // choose an update rate of 20Hz; go faster with actual hardware
const float maxSafeRange = 2.5; //start slowing down when object is within this range of robot
const float minSafeRange = 0.5; //stop the robot when at this distance from object

// compute some properties of trapezoidal velocity profile plan:
float accelerationTime = maxVelocity / maxAcceleration; //...assumes start from rest
float decelerationTime = maxVelocity / maxAcceleration; //(for same decel as accel); assumes brake to full halt
float accelerationDistance = 0.5 * maxAcceleration * (accelerationTime * accelerationTime); //distance rqd to ramp up to full speed
float decelerationDistance = 0.5 * maxAcceleration * (decelerationTime * decelerationTime); //same as ramp-up distance

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
