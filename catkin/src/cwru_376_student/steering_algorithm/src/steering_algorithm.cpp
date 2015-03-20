//steering_algorithm.cpp:
//wsn, Feb 2015
//should subscribe to desired state and to odom
// then invoke an algorithm to command speed and spin
// should update this computation adequately fast



// this header incorporates all the necessary #include files and defines the class "SteeringController"
#include "steering_algorithm.h"

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
SteeringController::SteeringController(ros::NodeHandle* nodehandle, SteerVelProfiler* steerProfiler):nh_(*nodehandle), steeringProfiler_(*steerProfiler) 
{ // constructor
    ROS_INFO("in class constructor of SteeringController");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    initializeServices();
    
    odom_phi_ = 1000.0; // put in impossible value for heading; test this value to make sure we have received a viable odom message
    ROS_INFO("waiting for valid odom message...");
    while (odom_phi_ > 500.0) {
        ros::Duration(0.5).sleep(); // sleep for half a second
        std::cout << ".";
        ros::spinOnce();
    }
    ROS_INFO("constructor: got an odom message");    
    
    dt_ = 1/UPDATE_RATE;
    
    //initialize desired state, in case this is not yet being published adequately
    des_state_ = current_odom_;  // use the current odom state
    // but make sure the speed/spin commands are set to zero
    current_speed_des_ = 0.0;  // 
    current_omega_des_ = 0.0;    
    des_state_.twist.twist.linear.x = current_speed_des_; // but specified desired twist = 0.0
    des_state_.twist.twist.angular.z = current_omega_des_;
    des_state_.header.stamp = ros::Time::now();   

    //initialize the twist command components, all to zero
    twist_cmd_.linear.x = 0.0;
    twist_cmd_.linear.y = 0.0;
    twist_cmd_.linear.z = 0.0;
    twist_cmd_.angular.x = 0.0;
    twist_cmd_.angular.y = 0.0;
    twist_cmd_.angular.z = 0.0;

    twist_cmd2_.twist = twist_cmd_; // copy the twist command into twist2 message
    twist_cmd2_.header.stamp = ros::Time::now(); // look up the time and put it in the header  
}

//member helper function to set up subscribers;
void SteeringController::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers: odom and desState");
    odom_subscriber_ = nh_.subscribe("odom", 1, &SteeringController::odomCallback, this); //subscribe to odom messages
    // add more subscribers here, as needed
    des_state_subscriber_ = nh_.subscribe("/desState", 1, &SteeringController::desStateCallback, this); // for desired state messages
}

//member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of "main()"
void SteeringController::initializeServices()
{
    ROS_INFO("Initializing Services: exampleMinimalService");
    simple_service_ = nh_.advertiseService("exampleMinimalService",
                                                   &SteeringController::serviceCallback,
                                                   this);  
    // add more services here, as needed
}

//member helper function to set up publishers;
void SteeringController::initializePublishers()
{
    ROS_INFO("Initializing Publishers: cmd_vel and cmd_vel_stamped");
    cmd_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true); // talks to the robot!
    cmd_publisher2_ = nh_.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped",1, true); //alt topic, includes time stamp
    steering_errs_publisher_ =  nh_.advertise<std_msgs::Float32MultiArray>("steering_errs",1, true);
}

void SteeringController::odomCallback(const nav_msgs::Odometry& odom_rcvd) {
    // copy some of the components of the received message into member vars
    // we care about speed and spin, as well as position estimates x,y and heading
    current_odom_ = odom_rcvd; // save the entire message
    // but also pick apart pieces, for ease of use
    odom_pose_ = odom_rcvd.pose.pose;
    odom_vel_ = odom_rcvd.twist.twist.linear.x;
    odom_omega_ = odom_rcvd.twist.twist.angular.z;
    odom_x_ = odom_rcvd.pose.pose.position.x;
    odom_y_ = odom_rcvd.pose.pose.position.y;
    odom_quat_ = odom_rcvd.pose.pose.orientation;
    //odom publishes orientation as a quaternion.  Convert this to a simple heading
    odom_phi_ = convertPlanarQuat2Phi(odom_quat_); // cheap conversion from quaternion to heading for planar motion
    // let's put odom x,y in an Eigen-style 2x1 vector; convenient for linear algebra operations
    odom_xy_vec_(0) = odom_x_;
    odom_xy_vec_(1) = odom_y_;   
}

void SteeringController::desStateCallback(const nav_msgs::Odometry& des_state_rcvd) {
    // copy some of the components of the received message into member vars
    // we care about speed and spin, as well as position estimates x,y and heading
    des_state_ = des_state_rcvd; // save the entire message
    // but also pick apart pieces, for ease of use
    des_state_pose_ = des_state_rcvd.pose.pose;
    des_state_vel_ = des_state_rcvd.twist.twist.linear.x;
    des_state_omega_ = des_state_rcvd.twist.twist.angular.z;
    des_state_x_ = des_state_rcvd.pose.pose.position.x;
    des_state_y_ = des_state_rcvd.pose.pose.position.y;
    des_state_quat_ = des_state_rcvd.pose.pose.orientation;
    //odom publishes orientation as a quaternion.  Convert this to a simple heading
    des_state_phi_ = convertPlanarQuat2Phi(des_state_quat_); // cheap conversion from quaternion to heading for planar motion
    // fill in an Eigen-style 2x1 vector as well--potentially convenient for linear algebra operations    
    des_xy_vec_(0) = des_state_x_;
    des_xy_vec_(1) = des_state_y_;      
}

//utility fnc to compute min dang, accounting for periodicity
double SteeringController::min_dang(double dang) {
    while (dang > M_PI) dang -= 2.0 * M_PI;
    while (dang < -M_PI) dang += 2.0 * M_PI;
    return dang;
}


// saturation function, values -1 to 1
double SteeringController::sat(double x) {
    if (x>1.0) {
        return 1.0;
    }
    if (x< -1.0) {
        return -1.0;
    }
    return x;
}

//some conversion utilities:
double SteeringController::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//member function implementation for a service callback function
// could do something useful with this
bool SteeringController::serviceCallback(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response) {
    ROS_INFO("service callback activated");
    response.resp = true; // boring, but valid response info
    return true;
}

// HERE IS THE BIG DEAL: USE DESIRED AND ACTUAL STATE TO COMPUTE AND PUBLISH CMD_VEL
void SteeringController::my_clever_steering_algorithm() {
    double controller_speed;
    double controller_omega;
    Eigen::Vector2d pos_err_xy_vec_;
    Eigen::Vector2d t_vec;    //tangent of desired path
    Eigen::Vector2d n_vec;    //normal to desired path, pointing to the "left" 
    t_vec(0) = cos(des_state_phi_);
    t_vec(1) = sin(des_state_phi_);
    n_vec(0) = -t_vec(1);
    n_vec(1) = t_vec(0);
    
    double heading_err;  
    double lateral_err;
    double trip_dist_err; // error is scheduling...are we ahead or behind?
    
    // have access to: des_state_vel_, des_state_omega_, des_state_x_, des_state_y_, des_state_phi_ and corresponding odom values    
    pos_err_xy_vec_ = des_xy_vec_ - odom_xy_vec_; // vector pointing from odom x-y to desired x-y
    lateral_err = n_vec.dot(pos_err_xy_vec_); //signed scalar lateral offset error; if positive, then desired state is to the left of odom
    trip_dist_err = t_vec.dot(pos_err_xy_vec_); // progress error: if positive, then we are behind schedule
    heading_err = min_dang(des_state_phi_ - odom_phi_); // if positive, should rotate +omega to align with desired heading
    
    
    // DEBUG OUTPUT...
    ROS_INFO("des_state_phi = %f, odom_phi = %f, heading err = %f", des_state_phi_,odom_phi_,heading_err);
    ROS_INFO("lateral err = %f, trip dist err = %f",lateral_err,trip_dist_err);
    // DEFINITELY COMMENT OUT ALL cout<< OPERATIONS FOR REAL-TIME CODE
    //std::cout<<des_xy_vec_<<std::endl;
    //std::cout<<odom_xy_vec_<<std::endl;
    // let's put these in a message to publish, for rqt_plot to display
    steering_errs_.data.clear();
    steering_errs_.data.push_back(lateral_err);
    steering_errs_.data.push_back(heading_err); 
    steering_errs_.data.push_back(trip_dist_err);

    steering_errs_publisher_.publish(steering_errs_); // suitable for plotting w/ rqt_plot
    //END OF DEBUG STUFF
    
     //currently, we are turning, so we must adjust turning
    if (des_state_omega_ != 0){
        //Correct errors in omega if there is heading error or lateral error
        controller_omega = compute_controller_omega(trip_dist_err, heading_err, lateral_err);
        controller_omega = MAX_OMEGA*sat(controller_omega/MAX_OMEGA); // saturate omega command at specified limits
        twist_cmd_.angular.z = controller_omega;
        twist_cmd_.linear.x = 0;
    } else if (des_state_vel_ != 0){
        //currently, we are moving forward, so we must adjust forward velocity
        //Correct errors in speed if there is a trip dist error
        controller_speed = compute_controller_speed(trip_dist_err);
            // send out our very clever speed/spin commands:
        twist_cmd_.linear.x = controller_speed;
        twist_cmd_.angular.z = 0;
    }
    //this is currently zero always..
    ROS_INFO("New steering controller speed: %f", controller_speed);
    ROS_INFO("New steering controller omega: %f", controller_omega);
 
    twist_cmd2_.twist = twist_cmd_; // copy the twist command into twist2 message
    twist_cmd2_.header.stamp = ros::Time::now(); // look up the time and put it in the header 
    cmd_publisher_.publish(twist_cmd_);  
    cmd_publisher2_.publish(twist_cmd2_);     
}

//this will compute the controller omega to accommodate the error
double SteeringController::compute_controller_omega(double trip_dist_err,
    double heading_err, double lateral_err){
    double controller_omega = des_state_omega_;
    double newPhi = (M_PI/2) - abs(atan2(trip_dist_err, lateral_err)) + heading_err;
    if (lateral_err > LAT_ERR_TOL){ //Rotate to the left 
        //heading_err > 0
        //use heading error to calculate +omega
        controller_omega = compute_omega_profile(newPhi);
    } else if (lateral_err < -LAT_ERR_TOL){
        //heading_err < 0
        //use heading error to calculate -omega
        controller_omega = compute_omega_profile(newPhi);
    }

    return controller_omega;
}

double SteeringController::compute_omega_profile(double newPhi) {
    //need to make object for steer vel profile then call these methods.
            //Turning right (clockwise) if negative end phi
   double turnDirection = sgn(newPhi);

    if (turnDirection != 0) {
        bool turnRight = turnDirection < 0;
        //Update the steering profiler with fresh odom readings.
        update_steering_profiler();
        
        //Compute the steering velocity profile via trapezoidal algorithms.
        double omegaProfile = steeringProfiler_.turnSlowDown(turnRight, newPhi);
        omegaProfile = steeringProfiler_.turnSpeedUp(omegaProfile);
        ROS_INFO("compute_omega_profile: modified_omega = %f", omegaProfile);
        return omegaProfile; // spin in direction of closest rotation to target heading
    }

    ROS_INFO("omega profile called with zero rotation, returning 0 omega.");
    return 0.0;
}

//this will compute the controller speed to accommodate the error but
//might need to take into account of lateral error
double SteeringController::compute_controller_speed(double trip_dist_err){
    double controller_speed = des_state_vel_;
    
    if(trip_dist_err<0){
        //ahead of schedule, slow down!
        //controller_speed = steeringProfiler_.reverseSlowDown(trip_dist_err);
        controller_speed = 0;
    }
    else if (trip_dist_err > 0){
        //behind schedule, speed up!
        //compute a trapezoidal speed
        controller_speed = steeringProfiler_.trapezoidalSlowDown(3 * trip_dist_err);
        controller_speed = steeringProfiler_.trapezoidalSpeedUp(controller_speed);
    }
    
    //Otherwise we are perfectly adjusted, stop!
    return controller_speed;
}

/**
 * Updates the steering velocity profiler instance with
 * fresh odometry readings.
 */
void SteeringController::update_steering_profiler(){
    steeringProfiler_.setOdomXYValues(odom_x_, odom_y_);
    steeringProfiler_.setOdomRotationValues(odom_phi_, odom_omega_);
    steeringProfiler_.setOdomForwardVel(odom_vel_);
    steeringProfiler_.setOdomDT(dt_);
    //This can be the trip distance error
    //steeringProfiler_.setSegLengthToGo(current_seg_length_to_go_);
    ROS_INFO("Steering profile: x: %f, y: %f, phi: %f, omega: %f, vel: %f, dt: %f, "
        "seg length to go: %f", steeringProfiler_.odomX, steeringProfiler_.odomY,
        steeringProfiler_.odomPhi, steeringProfiler_.odomOmega, steeringProfiler_.odomVel,
        steeringProfiler_.dt, steeringProfiler_.currSegLengthToGo);
}

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "steeringController"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    SteerVelProfiler steeringProfiler(MAX_ALPHA, rotationalDecelerationPhi,
        MAX_ACCEL, decelerationDistance, MAX_SPEED,
        MAX_OMEGA);

    ROS_INFO("main: instantiating an object of type SteeringController");
    SteeringController steeringController(&nh, &steeringProfiler);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use
    ros::Rate sleep_timer(UPDATE_RATE); //a timer for desired rate, e.g. 50Hz
   
    ROS_INFO:("starting steering algorithm");
    while (ros::ok()) {
        steeringController.my_clever_steering_algorithm(); // compute and publish twist commands and cmd_vel and cmd_vel_stamped

        ros::spinOnce();
        sleep_timer.sleep();
    }
    return 0;
} 

