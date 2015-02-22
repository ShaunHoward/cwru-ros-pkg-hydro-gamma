#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <float.h>

/**
 * A callback struct for holding odometer values and change in time.
 */
struct Callback{
	//Globals for communication w/ callbacks
    public:

	//Odom callback values
        float odomVelocity; // measured/published system speed
	float odomOmega; // measured/published system yaw rate (spin)
	float odomX;
	float odomY;
	float odomPhi;
	float odomDt;

	//Last time odom callback took place
	ros::Time lastCallbackTime;

	//Change in time of the velocity profiler
	float dt;
	float quaternionZ, quaternionW;
	
	//Setters for callback values
	void setOdomPhi(){
		//Conversion from quaternion to heading for planar motion
		odomPhi = 2.0 * atan2(quaternionZ, quaternionW); 		
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

	/**
	 * Prints the callback values via ROS_INFO().
	 */
	void printValues(){
		ROS_INFO("odom CB: x = %f, y= %f, phi = %f, v = %f, omega = %f", odomX, odomY, odomPhi, odomVelocity, odomOmega);		
	}
};

/**
 * A segment struct for the current segment to move the robot on. It holds all
 * necessary values to run the robot on the given segment.
 */
struct Segment{
    public:
	//The length complete thus far on the current segment
	float lengthCompleted;

	//The initial x and y coordinates and phi of the robot when this segment was made
	float startX;
	float startY;
	float startPhi;

	//The distance left to travel along this path segment
	float distanceLeft;

	//The length of this path segment
	float length;
	
	/**
	 * Copies all important values from one segment to this segment.
	 * @param copyFrom - the segment to copy values from
	 */
	void copy(const Segment& copyFrom){
		lengthCompleted = copyFrom.lengthCompleted;
		startX = copyFrom.startX;
		startY = copyFrom.startY;
		startPhi = copyFrom.startPhi;
		distanceLeft = copyFrom.distanceLeft;
		length = copyFrom.length;
	}
	
	//Setters for values of this segment
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
	
	//Reset the length completed along this segment
	void resetLengthCompleted(){
		lengthCompleted = 0.0;
	}
	
	//Reset the distance left to travel along this segment
	void resetDistanceLeft(){
		distanceLeft = 0.0;
	}
	
	//Print x, y, and phi to console via ROS_INFO
	void printValues(){
		ROS_INFO("start pose: x %f, y= %f, phi = %f", startX, startY, startPhi);
	}
};


/**
  *A Rotate struct that holds the values needed for a rotational
  *command in degrees
  */
struct Rotate{
    public:
    float phi;
    float startPhi;
    float endPhi;
    float phiCompleted;
    float phiLeft;

    void setPhi(float phi_){
        phi = phi_;
    }
    void setStartPhi(float start_phi){
        startPhi = start_phi;
    }
    void setEndPhi(float end_phi){
        endPhi = end_phi;
    } 
    void setPhiCompleted(float phi_comp){
        phiCompleted = phi_comp;
    }
    void setPhiLeft(float phi_left){
        phiLeft = phi_left;
    }
    void resetPhiCompleted(){
        phiCompleted = 0.0;
    }
};

/**
 * A lidar struct that holds the values necessary to determine lidar values
 * in the velocity profiler.
 */
struct Lidar{
    public:
	//Lidar variables
	float closestPing;
	bool alarm;
	bool stop;
	bool modifiedSegment;
	
	//Setters for lidar values
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

/**
 * An estop struct that just allows estop to be set on or off.
 */
struct Estop{
    public:
	//Whether the estop is on
	bool on;

	//Set whether the estop is on
	void set(bool on_){
		on = on_;
	}
};

//Various structs for use within the vel_scheduler
Callback callback;
Segment segment, modifiedSegment;
Lidar lidar;
Estop estop;
Rotate rotate;

// set some dynamic limits...
const float maxVelocity = 0.5; //1m/sec is a slow walk
const float minVelocity = 0.1; // if command velocity too low, robot won't move
const float maxAcceleration = 0.5; //1m/sec^2 is 0.1 g's
const float maxOmega = 1.0; //1 rad/sec-> about 6 seconds to rotate 1 full rev
const float minOmenga = 0.1; //this might need to change if value is too small to move robot
const float maxAlpha = 1; //0.5 rad/sec^2-> takes 2 sec to get from rest to full omega
const float changeInTime = 0.05; // choose an update rate of 20Hz; go faster with actual hardware
const float maxSafeRange = 2.5; //start slowing down when object is within this range of robot
const float minSafeRange = 0.5; //stop the robot when at this distance from object

// compute some properties of trapezoidal velocity profile plan:
float accelerationTime = maxVelocity / maxAcceleration; //...assumes start from rest
float decelerationTime = maxVelocity / maxAcceleration; //(for same decel as accel); assumes brake to full halt
float accelerationDistance = 0.5 * maxAcceleration * (accelerationTime * accelerationTime); //distance rqd to ramp up to full speed
float decelerationDistance = 0.5 * maxAcceleration * (decelerationTime * decelerationTime); //same as ramp-up distance

// compute properties of rotational trapezoidal velocity profile plan:
float turnAccelTime = maxOmega / maxAlpha; //...assumes start from rest
float turnDecelTime = maxOmega / maxAlpha; //(for same decel as accel); assumes brake to full halt
//float turnAccelPhi = 0.5 * maxAlpha * (turnAccelTime * turnAccelTime); //same as ramp-up distance
float rotationalAccelerationPhi = 0.5 * maxAlpha * (turnAccelTime * turnAccelTime);
float rotationalDecelerationPhi = 0.5 * maxAlpha * (turnDecelTime * turnDecelTime);

//Tracks the last phi value for cases where rotation goes from + to - or vice versa
float lastCallbackPhi = 0.0f;

//Whether to halt the robot due to software halt command
bool halt = false;

ros::Publisher velocityPublisher;

geometry_msgs::Twist velocityCommand; //create a variable of type "Twist" to publish speed/spin commands

//See comments in actual cpp file.
void odomCallback(const nav_msgs::Odometry& odom_rcvd);

void resetVelocityCommand();

void eStop();

float trapezoidalSlowDown(float segmentLength);

void decideToStop();

float trapezoidalSpeedUp(float scheduledVelocity);

void moveOnSegment(ros::Publisher velocityPublisher, float segmentLength);

float turnSlowDown(bool turnRight);

float turnSpeedUp(float scheduledOmega);

void rotateToPhi(ros::Publisher velocityPublisher, ros::Rate rTimer, float endRotation);

bool odomCallValidation(ros::Rate rTimer);

void initializePosition();

bool initializeRotation(float endPhi, ros::Rate rTimer);

void initializeNewMove(ros::Rate rTimer);

void pingDistanceCallback(const std_msgs::Float32& pingDistance);

void lidarAlarmCallback(const std_msgs::Bool& lidarAlarmMsg);

void estopCallback(const std_msgs::Bool& estopMsg);

bool isDoneRotating();

float getDeltaPhi(bool turnRight);
