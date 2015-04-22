//des_state_generator.cpp:
//implementation of class to generate a stream of desired states

// can test the flush-path service manually with 
// rosservice call flushPathService 1

// can test service appendPathCallback using test node: example_path_sender.cpp


// this header incorporates all the necessary #include files and defines the class "DesStateGenerator"
#include <geometry_msgs/Pose.h>
#include <cwru_msgs/PathSegment.h>

#include "des_state_generator.h"

int ans;

/**
 * Creates a desired state generator in order to generated proper poses for the robot to move to
 * at each time step based on its current pose and the pose necessary to reach the desired goal
 * pose which is fed to this node via a path sender file.
 * 
 * @param nodehandle - the ROS node handle for the des state generator
 * @param steerProfiler - the steering velocity profiler that handles generating proper forward
 * and turning velocities based on the current state of the robot and the desired state
 */
DesStateGenerator::DesStateGenerator(ros::NodeHandle* nodehandle, SteerVelProfiler* steerProfiler) : nh_(*nodehandle), steeringProfiler_(*steerProfiler) { // constructor
    ROS_INFO("in class constructor of DesStateGenerator");

    //not currently in a halt on the robot
    halt = false;

    // //transform listener is used for amcl navigation
    // tfListener_ = new tf::TransformListener;

    // // wait to start receiving valid tf transforms between map and odom:
    // bool tferr = true;
    // //ROS_INFO("waiting for tf between map and odom...");
    // ROS_INFO("waiting for tf between base_link and odom...");
    // while (tferr) {
    //     tferr = false;
    //     try {
    //         //try to lookup transform from target frame "odom" to source frame "map"
    //         //The direction of the transform returned will be from the target_frame to the source_frame. 
    //         //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
    //         tfListener_->lookupTransform("odom", "map", ros::Time(0), mapToOdom_);
    //         // tfListener_->lookupTransform("odom", "base_link", ros::Time(0), mapToOdom_);
    //     } catch (tf::TransformException &exception) {
    //         ROS_ERROR("%s", exception.what());
    //         tferr = true;
    //         ros::Duration(0.5).sleep(); // sleep for half a second
    //         ros::spinOnce();
    //     }
    // }
    // ROS_INFO("tf is good");
  //  from now on, tfListener will keep track of transforms

    //creates subscribers necessary to generate desired states
    initializeSubscribers();

    //creates the publishers necessary to publish desired states to the robot
    initializePublishers();

    //creates the services necessary to run the des state generator
    initializeServices();

    // put in impossible value for heading; test this value to make sure we have received a viable odom message
    odom_phi_ = 1000.0;
    ROS_INFO("waiting for valid odom message...");
    while (odom_phi_ > 500.0) {
        ros::Duration(0.5).sleep(); // sleep for half a second
        std::cout << ".";
        ros::spinOnce();
    }
    ROS_INFO("constructor: got an odom message");

    dt_ = 1.0 / UPDATE_RATE; // time step consistent with update frequency

    // construct a command state from current odom, + zero speed/spin
    des_state_ = update_des_state_halt();

    //for start-up, use the current odom values for the first vertex of the path
    //segment parameters:
    current_seg_type_ = HALT; // this should be enough...
    // nonetheless, let's fill in some dummy path segment parameters:    
    current_seg_length_ = 0.0;
    current_seg_phi_goal_ = odom_phi_;
    current_seg_ref_point_(0) = odom_x_;
    current_seg_ref_point_(1) = odom_y_;

    // these are dynamic variables, used to incrementally update the desired state:
    current_seg_phi_des_ = odom_phi_;
    current_seg_length_to_go_ = 0.0;
    current_seg_xy_des_ = current_seg_ref_point_;
    current_speed_des_ = 0.0;
    current_omega_des_ = 0.0;

    waiting_for_vertex_ = true;
    current_path_seg_done_ = true;
    lastCallbackTime = ros::Time::now();

    //track the segment length after stopping with lidar, estop, or halt
    modified_seg_length = 0.0;
    seg_length_left_after_stop = 0.0;

    last_map_pose_rcvd_ = odom_to_map_pose(odom_pose_stamped_); // treat the current odom pose as the first vertex--cast it into map coords to save
}

/**
 * Initializes the subscriber for odom feedback.
 */
void DesStateGenerator::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers");
    odom_subscriber_ = nh_.subscribe("odom", 1, &DesStateGenerator::odomCallback, this); //subscribe to odom messages
    //   odom_subscriber_ = nh_.subscribe("odom", 1, &DesStateGenerator::odomCallback, this); //subscribe to odom messages
    // add more subscribers here, as needed
}

/**
 * Initializes the services for flushing the path segments and appending path segments
 * necessary for generating desired states.
 */
void DesStateGenerator::initializeServices() {
    ROS_INFO("Initializing Services");
    flush_path_ = nh_.advertiseService("flushPathService",
            &DesStateGenerator::flushPathCallback,
            this);
    append_path_ = nh_.advertiseService("appendPathService",
            &DesStateGenerator::appendPathCallback,
            this);
    // add more services here, as needed
}

/**
 * Initializes the publisher for the desired state navigation msg.
 */
void DesStateGenerator::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    des_state_publisher_ = nh_.advertise<nav_msgs::Odometry>("desState", 1, true); // publish des state in same format as odometry messages
    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}

/**
 * Updates the steering velocity profiler instance with
 * fresh odometer readings.
 */
void DesStateGenerator::update_steering_profiler() {
    steeringProfiler_.setOdomXYValues(odom_x_, odom_y_);
    steeringProfiler_.setOdomRotationValues(odom_phi_, odom_omega_);
    steeringProfiler_.setOdomForwardVel(odom_vel_);
    steeringProfiler_.setOdomDT(dt_);
    steeringProfiler_.current_seg_ref_point_0 = current_seg_ref_point_(0);
    steeringProfiler_.current_seg_ref_point_1 = current_seg_ref_point_(1);
    //steeringProfiler_.setSegLengthToGo(current_seg_length_to_go_);
    ROS_INFO("Steering profile: x: %f, y: %f, phi: %f, omega: %f, vel: %f, dt: %f, "
            "distance left: %f", steeringProfiler_.odomX, steeringProfiler_.odomY,
            steeringProfiler_.odomPhi, steeringProfiler_.odomOmega, steeringProfiler_.odomVel,
            steeringProfiler_.dt, steeringProfiler_.distanceLeft);
}

/**
 * The callback for odometer from the robot.
 * This will calculate the change in time, dt, of this callback
 * since the last odom callback.
 * 
 * @param odom_rcvd - the odom nav msg received with updated odom data from the robot
 */
void DesStateGenerator::odomCallback(const nav_msgs::Odometry& odom_rcvd) {

    //compute time since last callback
    dt_ = (ros::Time::now() - lastCallbackTime).toSec();

    // let's remember the current time, and use it next iteration
    lastCallbackTime = ros::Time::now();

    // on start-up, and with occasional hiccups, this delta-time can be unexpectedly large
    if (dt_ > 0.15) {
        // can choose to clamp a max value on this, if dt_callback is used for computations elsewhere
        dt_ = 0.1;
        // let's complain whenever this happens
        ROS_WARN("large dt; dt = %lf", dt_);
    }

    // copy some of the components of the received message into member vars
    // we care about speed and spin, as well as position estimates x,y and heading
    current_odom_ = odom_rcvd; // save the entire message
    // but also pick apart pieces, for ease of use
    odom_pose_stamped_.header = odom_rcvd.header;
    odom_pose_ = odom_rcvd.pose.pose;
    odom_vel_ = odom_rcvd.twist.twist.linear.x;
    odom_omega_ = odom_rcvd.twist.twist.angular.z;
    odom_x_ = odom_rcvd.pose.pose.position.x;
    odom_y_ = odom_rcvd.pose.pose.position.y;
    odom_quat_ = odom_rcvd.pose.pose.orientation;
    //odom publishes orientation as a quaternion.  Convert this to a simple heading
    odom_phi_ = convertPlanarQuat2Phi(odom_quat_); // cheap conversion from quaternion to heading for planar motion
}

/**
 * A callback for the path flushing service.
 * 
 * @param request - the message request for the server
 * @param response - the message response for the server
 */
bool DesStateGenerator::flushPathCallback(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response) {
    ROS_INFO("service flush-Path callback activated");
    while (path_queue_.size() >0) {
        ROS_INFO("clearing the path queue...");
        std::cout << ' ' << path_queue_.front();
        path_queue_.pop();
    }
    response.resp = true; // boring, but valid response info
    return true;
}

/**
 * A callback for appending a path to the segment queue.
 * This service accepts a path service request (path message= vector of poses)
 * and it pushes received poses onto a local queue: path_queue_.
 * 
 * @param request - the message request for the server
 * @param response - the message response for the server
 */
bool DesStateGenerator::appendPathCallback(cwru_srv::path_service_messageRequest& request, cwru_srv::path_service_messageResponse& response) {
    geometry_msgs::PoseStamped pose;
    double x, y, phi;
    geometry_msgs::Quaternion quaternion;
    ROS_INFO("service append-Path callback activated");

    /* Path message:
     * #An array of poses that represents a Path for a robot to follow
     *   Header header
     *   geometry_msgs/PoseStamped[] poses
     */
    int nposes = request.path.poses.size();
    ROS_INFO("received %d vertices", nposes);

    //Adds all the given poses to the path queue
    for (int ipose = 0; ipose < nposes; ipose++) {
        ROS_INFO("pushing a pose onto queue");
        pose = request.path.poses[ipose];
        x = pose.pose.position.x;
        y = pose.pose.position.y;
        quaternion = pose.pose.orientation;
        phi = convertPlanarQuat2Phi(quaternion);
        std::cout << "x,y,phi = " << x << ", " << y << ", " << phi << std::endl;
        path_queue_.push(pose);
    }

    int nqueue = path_queue_.size();
    ROS_INFO("queue now contains %d vertices", nqueue);
    response.resp = true; // boring, but valid response info
    return true;
}

/**
 * Converts a quaternion to phi values for planar motion.
 * 
 * @param quaternion - the quaternion to convert to a phi for planar motion
 */
double DesStateGenerator::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

/**
 * Converts a planar phi value to a quaternion.
 * 
 * @param phi - the phi value to convert to a quaternion
 */
geometry_msgs::Quaternion DesStateGenerator::convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

/**
 * Computes the minimum angle from the given angle, accounting for periodicity.
 * 
 * @param dang - the angle to convert to an angle between -PI and +PI
 */
double DesStateGenerator::min_dang(double dang) {
    if (dang > M_PI) dang -= 2.0 * M_PI;
    if (dang<-M_PI) dang += 2.0 * M_PI;
    return dang;
}

/**
 * Computes the corresponding heading from v1 to v2 given points v1 and v2 in a plane.
 * 
 * @param v1 - the initial starting point vector
 * @param v2 - the goal point vector
 */
double DesStateGenerator::compute_heading_from_v1_v2(Eigen::Vector2d v1, Eigen::Vector2d v2) {
    //vector from v1 to v2 
    Eigen::Vector2d dv = v2 - v1;
    
    //heading from v1 to v2= target heading; head here incrementally  
    double heading_v1_to_v2 = atan2(dv(1), dv(0));
    return (heading_v1_to_v2);
}

/**
 * Converts a map pose to an odom pose using AMCL and the transform listener.
 * 
 * @param map_pose - the time-stamped map pose to convert to an odom pose
 */
geometry_msgs::PoseStamped DesStateGenerator::map_to_odom_pose(geometry_msgs::PoseStamped map_pose) {
 //   to use tf, need to convert coords from a geometry_msgs::Pose into a tf::Point
   //  tf::Point tf_map_goal;
    
   //  //fill in the data members of this tf::Point
   //  tf_map_goal.setX(map_pose.pose.position.x); 
   //  tf_map_goal.setY(map_pose.pose.position.y);
   //  tf_map_goal.setZ(map_pose.pose.position.z);
    
   //  //another tf::Point for result
   //  tf::Point tf_odom_goal;
    
   //  // and we'll convert back to a geometry_msgs::Pose to return our result
   //  geometry_msgs::PoseStamped odom_pose; 
   //  const geometry_msgs::PoseStamped c_map_pose = map_pose;
   //  ROS_INFO("new subgoal: goal in map pose is (x,y) = (%f, %f)", map_pose.pose.position.x, map_pose.pose.position.y);

   //  //now, use the tf listener to find the transform from map coords to odom coords:
   //  tfListener_->lookupTransform("odom", "map", ros::Time(0), mapToOdom_);
   // // tfListener_->lookupTransform("odom", "base_link", ros::Time(0), mapToOdom_);
    
   //  //here's one way to transform: operator "*" defined for class tf::Transform
   //  tf_odom_goal = mapToOdom_*tf_map_goal; 
   //  ROS_INFO("new subgoal: goal in odom pose is (x,y) = (%f, %f)", tf_odom_goal.x(), tf_odom_goal.y());

   //  //must update the time stamp of the next map pose
   //  map_pose.header.stamp = ros::Time::now();

   //  //let's transform the map_pose goal point into the odom frame:
   //  tfListener_->transformPose("odom", map_pose, odom_pose);
   //  //tf::TransformListener tfl;
   //  //tfl.transformPoint("odom",c_map_pose,odom_pose);
   //  //tfl.transformPose()
   //  ROS_INFO("new subgoal: goal in odom pose is (x,y) = (%f, %f)", odom_pose.pose.position.x, odom_pose.pose.position.y);
   //  ROS_INFO("odom_pose frame id: ");

   //  std::cout << odom_pose.header.frame_id << std::endl;
   //  if (DEBUG_MODE) {
   //      std::cout << "DEBUG: enter 1: ";
   //      std::cin>>ans;
   //  }
   //  return odom_pose;
      return map_pose;
}

/**
 * Returns the odom pose given as arguments.
 */
geometry_msgs::PoseStamped DesStateGenerator::odom_to_map_pose(geometry_msgs::PoseStamped odom_pose) {
    return odom_pose;
}

/**
 * NEED TO CONVERT FROM POLYLINE PATH TO DYNAMICALLY FEASIBLE PATH SEGMENTS.
 * PUT THE NEWLY GENERATED PATH SEGMENTS INTO A PATH-SEGMENT QUEUE.
 * This version is a special case--always assumes spin-in-place followed by line segment.
 * Get a new path vertex (subgoal pose) and compute corresponding dynamically-feasible path segments;
 * Put these path segments in a queue.
 * For this version, each new path subgoal generates exactly 2 path segments: a spin-in-place and a line-segment
 * should extend this to include blended circular arc path segments.
 */
void DesStateGenerator::process_new_vertex() {
    if (path_queue_.empty()) { // do nothing
        waiting_for_vertex_ = true;
        //current_seg_type_ = HALT;
        return;
    }

    ROS_INFO("process_new_vertex: ");
    
    // here if we can process a new path subgoal
    waiting_for_vertex_ = false; 
    int npts = path_queue_.size();
    ROS_INFO("there are %d vertices in the queue", npts);
    //if here, get the next vertex from the queue, convert to odom coords, and set up path segment params
    //will build new path segments from most recent path vertex
    waiting_for_vertex_ = false; 
    
    // note: we have a copy of front of queue, but we have not popped it from the queue yet
    geometry_msgs::PoseStamped map_pose_stamped = path_queue_.front(); 
    
    // remove this subgoal from the queue
    path_queue_.pop(); 

    // we want to build path segments to take us from the current pose to the new goal pose
    // the goal pose is transformed to odom coordinates at the last moment, to minimize odom drift issues
    //strip off the header to simplify notation
    geometry_msgs::Pose map_pose = map_pose_stamped.pose; 
    
    // convert new subgoal pose from map to odom coords    
    geometry_msgs::PoseStamped goal_pose_wrt_odom = map_to_odom_pose(map_pose_stamped); 
    
    // this should be the starting point for our next journey segment
    geometry_msgs::Pose start_pose_wrt_odom; 

    // save a copy of this subgoal in memory, in case we need it later
    last_map_pose_rcvd_ = map_pose_stamped; 

    // we get a choice here: for starting pose, use the previous desired state, or use the current odometry feedback pose
    // ideally, these are identical, if the robot successfully achieves the desired state precisely
    // odometry should be better for live machine--but previous command is suitable for testing w/o actual robot

    // USE THIS for init w/rt odometry feedback:
    // value refreshed in member var by odom callback
    start_pose_wrt_odom = odom_pose_; 

    // or USE THIS  for init w/rt most recently computed desired state
    //start_pose_wrt_odom =  des_state_.pose.pose;   

    // container for path segments to be built
    std::vector<cwru_msgs::PathSegment> vec_of_path_segs; 

    // the following will construct two path segments: spin to reorient, then lineseg to reach goal point
    vec_of_path_segs = build_spin_then_line_path_segments(start_pose_wrt_odom, goal_pose_wrt_odom.pose);

    // more generally, could replace the above with a segment builder that included circular arcs, etc,
    // potentially generating more path segments in the list.  
    // Or more simply, could add a segpath builder that ONLY re-orients, yielding a single path seg
    // regardless, take however many resulting path segments and push them into a pathseg queue:
    for (int i = 0; i < vec_of_path_segs.size(); i++) {
        segment_queue_.push(vec_of_path_segs[i]);
    }
    // we have now updated the segment queue; these segments should get processed before they get "stale"
}

/**
 * build_spin_then_line_path_segments: given two poses, p1 and p2 (in consistent reference frame),
 * Construct a vector of path segments consistent with those poses;
 * For just two poses, command spin segment to reorient along path from p1 to p2, 
 * then a second segment to move along line from p1 to p2
 * NOTE: pose1 should contain realistic starting heading value, but target heading will be derived
 * from vector from pose1 to pose2
 *
 * BETTER: if successive line segments in path are nearly colinear, don't need to stop and spin;
 * needs more logic
 */
std::vector<cwru_msgs::PathSegment> DesStateGenerator::build_spin_then_line_path_segments(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2) {
    // a container for new path segment, spin
    cwru_msgs::PathSegment spin_path_segment; 
    
    // a container for new path segment, line  
    cwru_msgs::PathSegment line_path_segment; 
    
    //container to hold results
    std::vector<cwru_msgs::PathSegment> vec_of_path_segs; 
    Eigen::Vector2d v1, v2;

    //unpack the x,y coordinates, and put these in a vector of type Eigen
    v1(0) = pose1.position.x;
    v1(1) = pose1.position.y;
    v2(0) = pose2.position.x;
    v2(1) = pose2.position.y;

    // populate a PathSegment object corresponding to a line segment from v1 to v2
    line_path_segment = build_line_segment(v1, v2);

    // get presumed initial heading from pose1:
    double init_heading = convertPlanarQuat2Phi(pose1.orientation);
    
    // goal heading will be derived from above line-segment orientation, as computed above
    double des_heading = convertPlanarQuat2Phi(line_path_segment.init_tan_angle);

    // populate a PathSegment object corresponding to spin-in-place from initial heading to lineseg heading:
    spin_path_segment = build_spin_in_place_segment(v1, init_heading, des_heading);

    //put these path segments in a vector: first spin, then move along lineseg:
    vec_of_path_segs.push_back(spin_path_segment);
    vec_of_path_segs.push_back(line_path_segment);
    std::cout << "vec of pathsegs[0] =" << vec_of_path_segs[0] << std::endl;
    std::cout << "vec of pathsegs[1] =" << vec_of_path_segs[1] << std::endl;
    return vec_of_path_segs;
}

/**
 * Return a spin-in-place segment object given an x-y point in space and initial and desired heading.
 * 
 * @param v1 - the vector to use as a reference point for the spin path segment
 * @param init_heading - the initial heading that the robot will start at
 * @param des_heading - the desired heading that the robot will end at
 */
cwru_msgs::PathSegment DesStateGenerator::build_spin_in_place_segment(Eigen::Vector2d v1, double init_heading, double des_heading) {
    //orient towards desired heading
    ROS_INFO("build_spin_in_place_segment");
    // unpack spin_dir_, current_segment_length_, current_segment_type_, init length to go; 
    // a container for new path segment
    cwru_msgs::PathSegment spin_path_segment;        
    double delta_phi = min_dang(des_heading - init_heading);

    //start from this heading
    spin_path_segment.init_tan_angle = convertPlanarPhi2Quaternion(init_heading); 
    
    // rotate in this direction: +1 or -1 
    spin_path_segment.curvature = sgn(delta_phi);  
    
    // rotate this much (magnitude)
    spin_path_segment.seg_length = fabs(delta_phi);        
    spin_path_segment.seg_type = cwru_msgs::PathSegment::SPIN_IN_PLACE;
    spin_path_segment.ref_point.x = v1(0);
    spin_path_segment.ref_point.y = v1(1);

    ROS_INFO("seg_length = %f", spin_path_segment.seg_length);
    if (DEBUG_MODE) {
        std::cout << "enter 1: ";
        std::cin>>ans;
    }
    return spin_path_segment;
}

// INTENDED FOR EXTENSION TO INCLUDE CIRCULAR ARCS...
// not ready for prime time
// This method has not been altered whatsoever.
cwru_msgs::PathSegment DesStateGenerator::build_arc_segment(Eigen::Vector2d arc_center, double init_heading, double final_heading, double curvature) {
    cwru_msgs::PathSegment arc_path_segment; // a container for new path segment    
    double delta_phi;
    delta_phi = min_dang(final_heading - init_heading);
    //account for +/- rotation
    if (curvature > 0) { // want to spin in + dir; make sure delta_phi is positive
        if (delta_phi < 0) {
            delta_phi += 2.0 * M_PI;
        }
    }
    if (curvature < 0) {// want to spin in - dir; make sure delta_phi is negative
        if (delta_phi > 0) {
            delta_phi -= 2.0 * M_PI;
        }
    }
    arc_path_segment.seg_length = fabs(delta_phi); // rotate this much (magnitude)        
    arc_path_segment.init_tan_angle = convertPlanarPhi2Quaternion(init_heading); //start from this heading
    arc_path_segment.curvature = curvature; // rotate in this direction: +1 or -1         
    arc_path_segment.seg_type = cwru_msgs::PathSegment::ARC;
    arc_path_segment.ref_point.x = arc_center(0);
    arc_path_segment.ref_point.y = arc_center(1);
    return arc_path_segment;
}

/**
 * Given two x-y vertices, define and return a line path segment object.
 * 
 * @param v1 - the vector point to start at
 * @param v2 - the vector point to end at
 */
cwru_msgs::PathSegment DesStateGenerator::build_line_segment(Eigen::Vector2d v1, Eigen::Vector2d v2) {
    ROS_INFO("build_line_segment");

    // a container for new path segment
    cwru_msgs::PathSegment line_path_segment; 
    double des_heading;
    
    //vector from v1 to v2
    Eigen::Vector2d dv = v2 - v1;  

    //heading from v1 to v2= target heading; head here incrementally
    des_heading = compute_heading_from_v1_v2(v1, v2); 
    line_path_segment.init_tan_angle = convertPlanarPhi2Quaternion(des_heading);
    line_path_segment.curvature = 0.0;
    line_path_segment.seg_length = dv.norm();
    line_path_segment.seg_type = cwru_msgs::PathSegment::LINE;
    line_path_segment.ref_point.x = v1(0);
    line_path_segment.ref_point.y = v1(1);

    ROS_INFO("new line seg starts from x,y = %f, %f", v1(0), v1(1));
    ROS_INFO("new line seg_length = %f", line_path_segment.seg_length);
    ROS_INFO("heading: %f", des_heading);

    if (DEBUG_MODE) {
        std::cout << "enter 1: ";
        std::cin>>ans;
    }

    return line_path_segment;
}

/**
 * This function takes a path_segment object and fills in member variables, for
 * iterative re-use by "update_des_state"
 */
void DesStateGenerator::unpack_next_path_segment() {
    cwru_msgs::PathSegment path_segment;
    //ROS_INFO("queue now contains %d vertices", (int)path_queue_.size());

    ROS_INFO("unpack_next_path_segment: ");
    if (segment_queue_.empty()) {
        ROS_INFO("no more segments in the path-segment queue");
        
        //build and enqueue more path segments, if possible
        process_new_vertex(); 
    }
    if (waiting_for_vertex_) {
        //we need more path segments.  Do we have another path vertex available?
        ROS_INFO("no more vertices in the path queue either...");
        
        // nothing more we can do until get more subgoals
        current_seg_type_ = HALT; 
        return;
    }

    ROS_INFO("processed a new path vertex; should have more path segments now");

    //if here, then we should have a new path segment or two in the queue--perhaps newly generated
    //let's make sure:
    if (segment_queue_.empty()) {
        ROS_WARN("this should not happen--just processed a vertex, but have no resulting path segs!");
        
        // nothing more we can do until get more subgoals 
        current_seg_type_ = HALT;   
        current_path_seg_done_ = true;
        return;
    }

    // finally, if we have survived to here, we have a new path segment;
    // let's pop it from the queue:
    int npts = segment_queue_.size();
    ROS_INFO("there are %d segments in the path-segment queue", npts);
    path_segment = segment_queue_.front(); // grab the next one;
    std::cout << ' ' << path_segment; // nice...this works
    segment_queue_.pop(); //remove this segment from the queue
    // unpack the new segment:

    // given a path segment; populate member vars for current segment
    // the following are segment parameter values, unchanging while traversing the segment:
    current_seg_type_ = path_segment.seg_type;
    current_seg_curvature_ = path_segment.curvature;
    current_seg_length_ = path_segment.seg_length;
    current_seg_ref_point_(0) = path_segment.ref_point.x;
    current_seg_ref_point_(1) = path_segment.ref_point.y;
    
    // path segments store heading as a quaternion...convert to scalar heading:
    current_seg_init_tan_angle_ = convertPlanarQuat2Phi(path_segment.init_tan_angle);
    current_seg_tangent_vec_(0) = cos(current_seg_init_tan_angle_);
    current_seg_tangent_vec_(1) = sin(current_seg_init_tan_angle_);

    //initialize these values, which will evolve while traveling the segment
    current_seg_length_to_go_ = current_seg_length_;

    //Reset the values for the steering profiler to prepare for the next path 
    //segment.
    steeringProfiler_.resetSegValues();
    current_seg_phi_des_ = current_seg_init_tan_angle_;

    Eigen::Vector2d current_seg_xy_des_ = current_seg_ref_point_;

    // interpretation of goal heading depends on segment type:
    switch (current_seg_type_) {
        case LINE:
            ROS_INFO("unpacking a lineseg segment");
            
            // this will remain constant over lineseg     
            current_seg_phi_goal_ = current_seg_init_tan_angle_; 
            
            //Track the distance left and the current segment length in the
            //steering profiler.
            steeringProfiler_.distanceLeft = current_seg_length_;
            steeringProfiler_.currSegLength = current_seg_length_;
            break;
        case SPIN_IN_PLACE:
            
            //compute goal heading:
            ROS_INFO("unpacking a spin-in-place segment");
            current_seg_phi_goal_ = current_seg_init_tan_angle_ + sgn(current_seg_curvature_) * current_seg_length_;
            
            //Track the desired phi, the phi left to turn, and the last callback phi from odom
            //in the steering profiler.
            steeringProfiler_.desiredPhi = current_seg_length_;
            steeringProfiler_.phiLeft = current_seg_length_;
            steeringProfiler_.lastCallbackPhi = odom_phi_;
            break;
        case ARC: // not implemented; set segment type to HALT
        default:
            ROS_WARN("segment type not defined");
            current_seg_type_ = HALT;
    }

    if (DEBUG_MODE) {
        std::cout << "enter 1: ";
        std::cin>>ans;
    }

    // we are ready to execute this new segment, so enable it:
    current_path_seg_done_ = false;
}

/**
 * Update the desired state and publish it: 
 * watch out--this function uses values stored in member variables
 * need to update these values:
 *    current_seg_length_to_go_ 
 *    current_seg_phi_des_ 
 *    current_seg_xy_des_ 
 *    current_speed_des_
 *    current_omega_des_
 * these will get used to populate des_state_, which will get published on topic "desState"
 * 
 *  no arguments--uses values in member variables 
 */
void DesStateGenerator::update_des_state() {
    switch (current_seg_type_) {
        case LINE:
            des_state_ = update_des_state_lineseg();
            ROS_INFO("Segment type is a line");
            break;
        case SPIN_IN_PLACE:
            des_state_ = update_des_state_spin();
            ROS_INFO("Segment type is a spin");
            break;
        case ARC: // not implemented; set segment type to HALT
        default:
            des_state_ = update_des_state_halt();
    }
    //ROS_INFO("queue now contains %d vertices", (int)path_queue_.size());

    des_state_publisher_.publish(des_state_); //send out our message
}

/**
 * Updates the desired forward movement state based on odom values and whether the trip
 * distance is within a tolerance value to the desired state.
 * This uses the trapezoidal steering velocity profiler to calculate the necessary
 * velocity for moving forward.
 */
nav_msgs::Odometry DesStateGenerator::update_des_state_lineseg() {
    nav_msgs::Odometry desired_state; // fill in this message and return it
    // but we will also update member variables:
    // need to update these values:
    //    current_seg_length_to_go_, current_seg_phi_des_, current_seg_xy_des_ 
    //    current_speed_des_, current_omega_des_

    //calls the steering velocity profiler class to find the current desired speed
    current_speed_des_ = compute_speed_profile();

    //omega is fixed at zero for a straight line segment
    current_omega_des_ = 0.0;

    // this value will not change during lineseg motion
    current_seg_phi_des_ = current_seg_init_tan_angle_;

    //track the current segment length to go
    //use the value calculated via odom in the steering profiler
    //this value is set when compute_speed_profile() is called    
    current_seg_length_to_go_ = steeringProfiler_.distanceLeft;

    ROS_INFO("update_des_state_lineseg: current_segment_length_to_go_ = %f", current_seg_length_to_go_);
    if (current_seg_length_to_go_ < LENGTH_TOL) { // check if done with this move
        // done with line segment;
        current_seg_type_ = HALT;
        current_seg_xy_des_ = current_seg_ref_point_ + current_seg_tangent_vec_*current_seg_length_; // specify destination vertex as exact, current goal
        current_seg_length_to_go_ = 0.0;
        current_speed_des_ = 0.0;
        current_path_seg_done_ = true;
        ROS_INFO("update_des_state_lineseg: done with translational motion commands");
    } else { // not done with translational move yet--step forward
        // based on distance covered, compute current desired x,y; use scaled vector from v1 to v2 
        current_seg_xy_des_ = current_seg_ref_point_ + current_seg_tangent_vec_ * (current_seg_length_ - current_seg_length_to_go_);
    }

    // fill in components of desired-state message:
    desired_state.twist.twist.linear.x = current_speed_des_;
    desired_state.twist.twist.angular.z = current_omega_des_;
    desired_state.pose.pose.position.x = current_seg_xy_des_(0);
    desired_state.pose.pose.position.y = current_seg_xy_des_(1);
    desired_state.pose.pose.orientation = convertPlanarPhi2Quaternion(current_seg_phi_des_);
    desired_state.header.stamp = ros::Time::now();
    return desired_state;
}

/**
 * Updates the desired spin state based on odom values and whether the spin
 * heading is within a tolerance value to the desired state.
 * This uses the trapezoidal steering velocity profiler to calculate the necessary
 * omega for spinning.
 */
nav_msgs::Odometry DesStateGenerator::update_des_state_spin() {
    nav_msgs::Odometry desired_state; // fill in this message and return it
    // need to update these values:
    //    current_seg_length_to_go_, current_seg_phi_des_, current_seg_xy_des_ 
    //    current_speed_des_, current_omega_des_
    current_seg_xy_des_ = current_seg_ref_point_; // this value will not change during spin-in-place
    current_speed_des_ = 0.0; // also unchanging

    current_omega_des_ = compute_omega_profile(); //USE VEL PROFILING 

    //track the current segment length to go (phi)
    //use the value calculated via odom in the steering profiler
    //this value is set when compute_omega_profile() is called  
    current_seg_length_to_go_ = steeringProfiler_.phiLeft;
    ROS_INFO("update_des_state_spin: current_segment_length_to_go_ = %f", current_seg_length_to_go_);

    if (current_seg_length_to_go_ < HEADING_TOL) { // check if done with this move
        current_seg_type_ = HALT;
        current_seg_xy_des_ = current_seg_ref_point_; // specify destination vertex as exact, current goal
        current_seg_length_to_go_ = 0.0;
        current_speed_des_ = 0.0;
        current_omega_des_ = 0.0;
        current_seg_phi_des_ = current_seg_init_tan_angle_ + sgn(current_seg_curvature_) * current_seg_length_;
        current_path_seg_done_ = true;
        ROS_INFO("update_des_state_spin: done with spin");
    } else { // not done yet--rotate some more
        // based on angular distance covered, compute current desired heading
        // consider specified curvature ==> rotation direction to goal
        current_seg_phi_des_ = current_seg_init_tan_angle_ + sgn(current_seg_curvature_)*(current_seg_length_ - current_seg_length_to_go_);
    }

    // fill in components of desired-state message:
    desired_state.twist.twist.linear.x = current_speed_des_;
    desired_state.twist.twist.angular.z = current_omega_des_;
    desired_state.pose.pose.position.x = current_seg_xy_des_(0);
    desired_state.pose.pose.position.y = current_seg_xy_des_(1);
    desired_state.pose.pose.orientation = convertPlanarPhi2Quaternion(current_seg_phi_des_);
    desired_state.header.stamp = ros::Time::now();
    return desired_state;
}

/**
 * Updates the robot to halt. This just sets the linear x and angular z to 0.
 */
nav_msgs::Odometry DesStateGenerator::update_des_state_halt() {
    nav_msgs::Odometry desired_state; // fill in this message and return it
    // fill in components of desired-state message from most recent odom message
    //desired_state = current_odom_; //OPTIONAL: CAN SIMPLY RETAIN LAST COMPUTED DESIRED STATE
    desired_state = des_state_; // OPTION:  NOT USING ODOMETRY

    current_speed_des_ = 0.0;
    current_omega_des_ = 0.0;
    desired_state.twist.twist.linear.x = current_speed_des_; // but specified desired twist = 0.0
    desired_state.twist.twist.angular.z = current_omega_des_;
    desired_state.header.stamp = ros::Time::now();

    current_path_seg_done_ = true; // let the system know we are anxious for another segment to process...
    return desired_state;
}

/**
 * Computes the desired speed profile based on the current segment length (in meters)
 * in order to reach the desired goal coordinate. This uses trapezoidal forward
 * slow down and speed up algorithms from the steering velocity profiler class.
 * This method will produce a state where the robot will speed up or slow down
 * to approach the target heading as close as possible.
 * 
 * @return the desired speed for the next state in order to reach the goal coordinate 
 */
double DesStateGenerator::compute_speed_profile() {
    //Update the steering profiler with fresh odom readings.
    update_steering_profiler();
    //put check in here for segment type
    if (!lidar.alarm && !estop.on) {
        //Compute the speed profile from the steering velocity profiler.
        double speedProfile = steeringProfiler_.trapezoidalSlowDown(steeringProfiler_.currSegLength);
        double commandSpeed = steeringProfiler_.trapezoidalSpeedUp(speedProfile);
        ROS_INFO("compute_speed_profile: cmd_speed = %f", commandSpeed);
        return commandSpeed;
    } else {
        return 0.0;
    }
}

/**
 * Computes the desired omega profile based on the current segment length left to go (as phi)
 * in order to reach the desired goal coordinate. This uses trapezoidal turn
 * slow down and speed up algorithms from the steering velocity profiler class.
 * This method will produce a state where the robot will spin in the direction
 * of closest rotation to the target heading.
 * 
 * @return the desired omega for the next state in order to reach the goal coordinate 
 */
double DesStateGenerator::compute_omega_profile() {
    //determine the direction of the turn
    double turnDirection = sgn(current_seg_curvature_);

    //if turning, run trapezoidal omega profiler based on the turn direction and
    //the current segment length left to travel
    if (turnDirection != 0 && !lidar.alarm && !estop.on) {
        bool turnRight = false;
        if (turnDirection < 0) {
            turnRight = true;
        }

        //Update the steering profiler with fresh odom readings.
        update_steering_profiler();

        //Compute the steering omega velocity profile via trapezoidal algorithms.
        double omegaProfile = steeringProfiler_.turnSlowDown(turnRight);
        double commandOmega = steeringProfiler_.turnSpeedUp(omegaProfile);
        ROS_INFO("compute_omega_profile: cmd_omega = %f", commandOmega);
        return turnDirection * commandOmega; // spin in direction of closest rotation to target heading
    }

    //otherwise, omega will be zero because the robot is not turning
    ROS_INFO("omega profile called with zero rotation, returning 0 omega.");
    return 0.0;
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
        //   eStop();
    } else {
        ROS_INFO("The lidar alarm is off!");
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
    ROS_INFO("The ping distance from front of robot is: %f", lidar.closestPing);

    //    //Slow down to min safe range if on a line segment
    //    if (lidar.closestPing <= MAX_SAFE_RANGE && current_seg_type_ == LINE && !lidar.modifiedSegment){
    //
    //        modified_seg_length = MAX_SAFE_RANGE - MIN_SAFE_RANGE;
    //        if (modified_seg_length < steeringProfiler.distLeft){
    //            lidar.setModifiedSegment(true);
    //            seg_length_left_after_stop = steeringProfiler_.distLeft - modified_seg_length;
    //            steeringProfiler.currSegLength = modified_seg_length;
    //            steeringProfiler.distLeft = modified_seg_length;
    //      
    //        }
    //    } else if (lidar.closestPing > MAX_SAFE_RANGE && current_seg_type_ == LINE
    //            && lidar.modifiedSegment){
    //        steeringProfiler.currSegLength = modified_seg_length;
    //    }
    //    
    //Sound an alarm if something is too close
    if (lidar.closestPing <= MIN_SAFE_RANGE) {
        lidar.setAlarm(true);
    } else {
        lidar.setAlarm(false);
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
        //  eStop();
    } else {
        estop.set(false);
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
        ROS_INFO("Software halt enabled. ESTOP on.");
        estop.set(true);
    } else {
        estop.set(false);
    }
}

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "desStateGenerator"); //node name
    
    // create a node handle; need to pass this to the class constructor
    ros::NodeHandle nh; 
    SteerVelProfiler steeringProfiler;
    ROS_INFO("main: instantiating a DesStateGenerator");
    //instantiate a DesStateGenerator object and pass in pointer to nodehandle and steering profiler
    //for constructor to use
    DesStateGenerator desStateGenerator(&nh, &steeringProfiler); 
    
     //a timer for desired rate, e.g. 50Hz
    ros::Rate sleep_timer(UPDATE_RATE);

    ros::Subscriber ping_dist_subscriber = nh.subscribe("lidar_dist", 1, pingDistanceCallback);
    ros::Subscriber lidar_alarm_subscriber = nh.subscribe("lidar_alarm", 1, lidarAlarmCallback);
    ros::Subscriber estop_subscriber = nh.subscribe("estop_listener", 1, estopCallback);
    ros::Subscriber halt_subscriber = nh.subscribe("halt_cmd", 1, haltCallback);

    //constructor will wait for a valid odom message; let's use this for our first vertex;
    ROS_INFO("main: going into main loop");

    while (ros::ok()) {

        if (desStateGenerator.get_current_path_seg_done()) {
            //here if we have completed a path segment, so try to get another one
            // if necessary, construct new path segments from new polyline path subgoal
            desStateGenerator.unpack_next_path_segment();
        }

        // update the desired state and publish it; 
        desStateGenerator.update_des_state(); 
        // when segment is traversed, set: current_path_seg_done_ = true

        ros::spinOnce();
        sleep_timer.sleep();
    }
    return 0;
}

