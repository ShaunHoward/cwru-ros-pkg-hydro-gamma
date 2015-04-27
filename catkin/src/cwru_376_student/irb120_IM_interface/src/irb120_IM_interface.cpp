// irb_IM_interface.cpp
// Wyatt Newman, Team Gamma
// node that listens on topic "marker_listener" and prints pose received

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <interactive_markers/interactive_marker_server.h>
#include <irb120_kinematics.h>
#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

// this is a pre-defined service message, contained in shared "cwru_srv" package
#include <cwru_srv/simple_int_service_message.h> 

//callback to subscribe to marker state
Eigen::Vector3d g_p;
Vectorq6x1 g_q_state;
double g_x, g_y, g_z;
//geometry_msgs::Quaternion g_quat; // global var for quaternion
Eigen::Quaterniond g_quat;
Eigen::Matrix3d g_R;
Eigen::Affine3d g_A_flange_desired;
bool g_trigger = false;

int g_fit_z = -1;
bool lower_trigger = false;
double curr_arm_z = 0.0;

//define some processing modes; set these interactively via service
const int FINE = 0;
const int FINER = 1;
const int FINEST = 2;

//have a tolerance on the goal pose position values
const double JOINT_ERR_TOL = 0.1f;

//account for the gripper height
const double GRIPPER_HEIGHT = 0.15f;

tf::TransformListener* g_tfListener;
tf::StampedTransform g_armlink1_wrt_baseLink;
geometry_msgs::PoseStamped g_marker_pose_in;
geometry_msgs::PoseStamped g_marker_pose_wrt_arm_base;

using namespace std;

void markerListenerCB(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);

    ROS_INFO_STREAM("marker frame_id is " << feedback->header.frame_id);
    g_marker_pose_in.header = feedback->header;
    g_marker_pose_in.pose = feedback->pose;
    g_tfListener->transformPose("link1", g_marker_pose_in, g_marker_pose_wrt_arm_base);

    g_p[0] = g_marker_pose_wrt_arm_base.pose.position.x;
    g_p[1] = g_marker_pose_wrt_arm_base.pose.position.y;
    g_p[2] = g_marker_pose_wrt_arm_base.pose.position.z;
    g_quat.x() = g_marker_pose_wrt_arm_base.pose.orientation.x;
    g_quat.y() = g_marker_pose_wrt_arm_base.pose.orientation.y;
    g_quat.z() = g_marker_pose_wrt_arm_base.pose.orientation.z;
    g_quat.w() = g_marker_pose_wrt_arm_base.pose.orientation.w;
    g_R = g_quat.matrix();
    ROS_INFO_STREAM(feedback->marker_name << " is now at x: "
            << g_p[0] << ", y: " << g_p[1]
            << ", z: " << g_p[2] << ", quatx: " << g_quat.x()
            << ", quaty: " << g_quat.y() << ", quatz: " << g_quat.z() << ", quatw: " << g_quat.w());
    //        //copy to global vars:
    //        g_p[0] = feedback->pose.position.x;
    //        g_p[1] = feedback->pose.position.y;
    //        g_p[2] = feedback->pose.position.z;
    //        g_quat.x() = feedback->pose.orientation.x;
    //        g_quat.y() = feedback->pose.orientation.y;
    //        g_quat.z() = feedback->pose.orientation.z;
    //        g_quat.w() = feedback->pose.orientation.w;
    //        g_R = g_quat.matrix();
}

void alignWithCanCB(const geometry_msgs::Vector3 feedback) {
    //    ROS_INFO_STREAM(feedback->marker_name << " is now at "
    //            << feedback->pose.position.x << ", " << feedback->pose.position.y
    //            << ", " << feedback->pose.position.z);
    //
    //    ROS_INFO_STREAM("marker frame_id is " << feedback->header.frame_id);
    //    g_marker_pose_in.header = feedback->header;
    //    g_marker_pose_in.pose = feedback->pose;
    //    g_tfListener->transformPose("link1", g_marker_pose_in, g_marker_pose_wrt_arm_base);
    //
    //    g_p[0] = g_marker_pose_wrt_arm_base.pose.position.x;
    //    g_p[1] = g_marker_pose_wrt_arm_base.pose.position.y;
    //    g_p[2] = g_marker_pose_wrt_arm_base.pose.position.z;
    //    g_quat.x() = g_marker_pose_wrt_arm_base.pose.orientation.x;
    //    g_quat.y() = g_marker_pose_wrt_arm_base.pose.orientation.y;
    //    g_quat.z() = g_marker_pose_wrt_arm_base.pose.orientation.z;
    //    g_quat.w() = g_marker_pose_wrt_arm_base.pose.orientation.w;
    //    g_R = g_quat.matrix();
    //        ROS_INFO_STREAM("can pose is now at x: "
    //                << feedback->pose.position.x << ", y: " << feedback->pose.position.y
    //                << ", z: " << feedback->pose.position.z << ", quatx: " << feedback->pose.orientation.x 
    //                << ", quaty: " << feedback->pose.orientation.y << ", quatz: " << feedback->pose.orientation.z << ", quatw: " << feedback->pose.orientation.w);
    
    //copy to global vector and adjust for gripper height
    g_p[0] = feedback.x;
    g_p[1] = feedback.y;
    g_p[2] = feedback.z + GRIPPER_HEIGHT;
    ROS_INFO("The flange is set to go to: x: %f, y: %f, z: %f", g_p[0], g_p[1], g_p[2]);

    g_A_flange_desired.translation() = g_p;
    g_A_flange_desired.linear() = g_R;
    cout << "g_p: " << g_p.transpose() << endl;
    cout << "R: " << endl;
    cout << g_R << endl;
}

void jointStateCB(const sensor_msgs::JointStatePtr &js_msg) {
    ROS_INFO("Got joint states from callback");
    for (int i = 0; i < 6; i++) {
        g_q_state[i] = js_msg->position[i];
    }
    cout << "g_q_state: " << g_q_state.transpose() << endl;
}

bool triggerService(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response) {
    ROS_INFO("service callback activated");
    // boring, but valid response info
    response.resp = true;

    // grab the most recent IM data and repackage it as an Affine3 matrix to set a target hand pose;
    g_A_flange_desired.translation() = g_p;
    g_A_flange_desired.linear() = g_R;
    cout << "g_p: " << g_p.transpose() << endl;
    cout << "R: " << endl;
    cout << g_R << endl;

    //inform "main" that we have a new goal!
    g_trigger = true;
    return true;
}

/**
 * Use this service to set processing modes interactively.
 */
bool lowerService(cwru_srv::simple_int_service_messageRequest& request, cwru_srv::simple_int_service_messageResponse& response) {
    ROS_INFO("mode select service callback activated");
    
    // boring, but valid response info
    response.resp = true; 
    g_fit_z = request.req;
    
    //signal that we received a request; trigger a response
    lower_trigger = true; 
    cout << "Lower mode set to: " << g_fit_z << endl;
    return true;
}

//command robot to move to "qvec" using a trajectory message, sent via ROS-I

void stuff_trajectory(Vectorq6x1 qvec, trajectory_msgs::JointTrajectory &new_trajectory) {

    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    trajectory_msgs::JointTrajectoryPoint trajectory_point2;

    new_trajectory.points.clear();
    new_trajectory.joint_names.clear();
    new_trajectory.joint_names.push_back("joint_1");
    new_trajectory.joint_names.push_back("joint_2");
    new_trajectory.joint_names.push_back("joint_3");
    new_trajectory.joint_names.push_back("joint_4");
    new_trajectory.joint_names.push_back("joint_5");
    new_trajectory.joint_names.push_back("joint_6");

    new_trajectory.header.stamp = ros::Time::now();

    trajectory_point1.positions.clear();
    trajectory_point2.positions.clear();
    //fill in the points of the trajectory: initially, all home angles
    for (int ijnt = 0; ijnt < 6; ijnt++) {
        trajectory_point1.positions.push_back(g_q_state[ijnt]); // stuff in position commands for 6 joints
        //should also fill in trajectory_point.time_from_start
        trajectory_point2.positions.push_back(0.0); // stuff in position commands for 6 joints        
    }
    trajectory_point1.time_from_start = ros::Duration(0);
    trajectory_point2.time_from_start = ros::Duration(2.0);

    // start from home pose... really, should should start from current pose!
    new_trajectory.points.push_back(trajectory_point1); // add this single trajectory point to the trajectory vector   
    // new_trajectory.points.push_back(trajectory_point2); // quick hack--return to home pose

    // fill in the target pose: really should fill in a sequence of poses leading to this goal
    trajectory_point2.time_from_start = ros::Duration(4.0);
    for (int ijnt = 0; ijnt < 6; ijnt++) {
        trajectory_point2.positions[ijnt] = qvec[ijnt];
    }

    new_trajectory.points.push_back(trajectory_point2); // append this point to trajectory
}

void initialize_arm_position(ros::Publisher pub, Eigen::Matrix3d R_urdf_wrt_DH, Irb120_IK_solver ik_solver) {
    trajectory_msgs::JointTrajectory home_trajectory;
    std::vector<Vectorq6x1> q6dof_solns;
    Vectorq6x1 qvec;
    //old home pose
    //    g_p[0] = -0.540994;
    //    g_p[1] = -0.00188585;
    //    g_p[2] = 0.571356;
    //    g_quat.x() = 0.0128913;
    //    g_quat.y() = -0.710416;
    //    g_quat.z() = -0.0152647;
    //    g_quat.w() = 0.703499;
    //    g_R = g_quat.matrix();

    //home pose:  x: 0.0704939, y: -0.505254, z: 0.446664, quatx: -0.544637, quaty: 0, quatz: 0, quatw: 0.838672
    //
    //    //home pose in link_1 frame
    //    //put arm to the right of robot
    //    g_p[0] = 0.0704939;
    //    g_p[1] = -0.505254;
    //    g_p[2] = 0.446664;
    //    g_quat.x() = -0.544637;
    //    g_quat.y() = 0;
    //    g_quat.z() = 0;
    //    g_quat.w() = 0.838672;
    //    g_R = g_quat.matrix();

    //home pose in base_link frame
    g_p[0] = -0.110573;
    g_p[1] = -0.336946;
    g_p[2] = 0.503469;
    g_quat.x() = 3.56917e-08;
    g_quat.y() = -0.0260768;
    g_quat.z() = 1.15523e-08;
    g_quat.w() = 0.99966;
    g_R = g_quat.matrix();

    ROS_INFO_STREAM("Home pose is at x: "
            << g_p[0] << ", y: " << g_p[1]
            << ", z: " << g_p[2] << ", quatx: " << g_quat.x()
            << ", quaty: " << g_quat.y() << ", quatz: " << g_quat.z() << ", quatw: " << g_quat.w());

    g_A_flange_desired.translation() = g_p;
    g_A_flange_desired.linear() = g_R;
    cout << "g_p: " << g_p.transpose() << endl;
    cout << "R: " << endl;
    cout << g_R << endl;

 //   Eigen::Affine3d A_flange_des_DH;
  //  A_flange_des_DH = g_A_flange_desired;
   // A_flange_des_DH.linear() = g_A_flange_desired.linear() * R_urdf_wrt_DH.transpose();
}

/**
 * Determine if the arm is currently at the goal pose.
 */
bool isAtGoal(Vectorq6x1 qvec, std_msgs::Bool goalMessage, ros::Publisher goalPub) {

    cout << "g_q_state: " << g_q_state.transpose() << endl;
    cout << "qvec: " << qvec.transpose() << endl;

    //Calculate the error between the current joint states and the desired joint states
    Vectorq6x1 error_vector = g_q_state - qvec;
    double errorFromGoal = error_vector.norm();

    //check if the current position is within a tolerance from the goal position
    if (errorFromGoal > JOINT_ERR_TOL) {
        //the arm has not reached the goal pose
        ROS_INFO("Goal joint states have NOT been met.");
        goalMessage.data = false;
        goalPub.publish(goalMessage);
        return false;
    }

    //the arm has reached the goal pose
    goalMessage.data = true;
    goalPub.publish(goalMessage);
    ROS_INFO("Goal joint states have been met.");
    return true;
}

// //Set the goal pose z to the new calculated arm z coordinate 

// void armZCB(const std_msgs::Float32::ConstPtr& arm_z) {
//     g_p[2] = arm_z->data;
// }

void lower_arm(){

    //the current arm z is the 3rd value in the position vector
    curr_arm_z = g_p[0];

    ROS_INFO("lower_trigger enabled");
    lower_trigger = false; // reset the trigger
    
    //switch to fit according to closeness of flange to can
    //0 for fine, 1 for finer, 2 for finest
    switch (g_fit_z) {
        case FINE:
            curr_arm_z += .05;
            break;
        case FINER:
            curr_arm_z += .01;
            break;
        case FINEST:
            curr_arm_z += .005;
            break;
    }
    ROS_INFO("Modified arm z is: %f", curr_arm_z);

    //z-adjusted pose in base_link frame
    g_p[0] = curr_arm_z;
    g_R = g_quat.matrix();

    ROS_INFO_STREAM("Home pose is at x: "
            << g_p[0] << ", y: " << g_p[1]
            << ", z: " << g_p[2] << ", quatx: " << g_quat.x()
            << ", quaty: " << g_quat.y() << ", quatz: " << g_quat.z() << ", quatw: " << g_quat.w());

    g_A_flange_desired.translation() = g_p;
    g_A_flange_desired.linear() = g_R;
    cout << "g_p: " << g_p.transpose() << endl;
    cout << "R: " << endl;
    cout << g_R << endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_marker_listener"); // this will be the node name;
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);

    //publish whether we are at the goal pose of the last given marker
    ros::Publisher goalPub = nh.advertise<std_msgs::Bool>("at_goal_pose", 1);

    //Make goal message
    std_msgs::Bool goalMessage;
    goalMessage.data = false;
    goalPub.publish(goalMessage);

    ROS_INFO("setting up subscribers ");
    ros::Subscriber sub_js = nh.subscribe("/abby/joint_states", 1, jointStateCB);
    ros::Subscriber sub_im = nh.subscribe("example_marker/feedback", 1, markerListenerCB);

    //must determine can coordinates in order to move to above can
    ros::Subscriber sub_can_coords = nh.subscribe("can_coords", 1, alignWithCanCB);


    ros::ServiceServer service = nh.advertiseService("move_trigger", triggerService);
    
    //service for lowering arm in step heights, from big to small
    ros::ServiceServer lower_service = nh.advertiseService("lower_trigger", lowerService);
   // ros::Publisher pub_z = nh.advertise<std_msgs::Float32>("arm_z", 1);

    Eigen::Vector3d p;
    Eigen::Vector3d n_des, t_des, b_des;
    std::vector<Vectorq6x1> q6dof_solns;
    Vectorq6x1 qvec;
    ros::Rate sleep_timer(10.0); //10Hz update rate    
    Irb120_fwd_solver irb120_fwd_solver; //instantiate forward and IK solvers
    Irb120_IK_solver ik_solver;
    Eigen::Vector3d n_urdf_wrt_DH, t_urdf_wrt_DH, b_urdf_wrt_DH;
    bool first = true;
    //bool nextPose = true;
    // in home pose, R_urdf = I
    //DH-defined tool-flange axes point as:
    // z = 1,0,0
    // x = 0,0,-1
    // y = 0,1,0
    // but URDF frame is R = I
    // so, x_urdf_wrt_DH = z_DH = [0;0;1]
    // y_urdf_wrt_DH = y_DH = [0;1;0]
    // z_urdf_wrt_DH = -x_DH = [-1; 0; 0]
    // so, express R_urdf_wrt_DH as:
    n_urdf_wrt_DH << 0, 0, 1;
    t_urdf_wrt_DH << 0, 1, 0;
    b_urdf_wrt_DH << -1, 0, 0;
    Eigen::Matrix3d R_urdf_wrt_DH;
    R_urdf_wrt_DH.col(0) = n_urdf_wrt_DH;
    R_urdf_wrt_DH.col(1) = t_urdf_wrt_DH;
    R_urdf_wrt_DH.col(2) = b_urdf_wrt_DH;

    trajectory_msgs::JointTrajectory new_trajectory; // an empty trajectory


    //qvec<<0,0,0,0,0,0;
    Eigen::Affine3d A_flange_des_DH;

    //   A_fwd_DH = irb120_fwd_solver.fwd_kin_solve(qvec); //fwd_kin_solve

    //std::cout << "A rot: " << std::endl;
    //std::cout << A_fwd_DH.linear() << std::endl;
    //std::cout << "A origin: " << A_fwd_DH.translation().transpose() << std::endl;   

    g_tfListener = new tf::TransformListener; //create a transform listener
    // wait to start receiving valid tf transforms between map and odom:
    bool tferr = true;
    ROS_INFO("waiting for tf between base_link and link1 of arm...");
    while (tferr) {
        tferr = false;
        try {
            //try to lookup transform from target frame "odom" to source frame "map"
            //The direction of the transform returned will be from the target_frame to the source_frame.
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            g_tfListener->lookupTransform("base_link", "link1", ros::Time(0), g_armlink1_wrt_baseLink);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good");
    // from now on, tfListener will keep track of transforms 

    int nsolns;

    while (ros::ok()) {
        ros::spinOnce();

        if (first) {
            //      first = false;
            initialize_arm_position(pub, R_urdf_wrt_DH, ik_solver);
            //  g_trigger = true;
        }

        if (first || g_trigger || lower_trigger){ //|| !isAtGoal(qvec, goalMessage, goalPub)) {
            //if (g_trigger) {
            //no longer on the first call
            first = false;
            // reset the triggers
            g_trigger = false;

            if (lower_trigger){
                lower_arm();
            }

            //is this point reachable?
            A_flange_des_DH = g_A_flange_desired;
            A_flange_des_DH.linear() = g_A_flange_desired.linear() * R_urdf_wrt_DH.transpose();
            cout << "R des DH: " << endl;
            // cout << A_flange_des_DH.linear() << endl;
            nsolns = ik_solver.ik_solve(A_flange_des_DH);
            ROS_INFO("there are %d solutions", nsolns);

            if (nsolns > 0) {
                ik_solver.get_solns(q6dof_solns);

                //qvec = q6dof_solns[0]; // arbitrarily choose first soln

                // See how many results we get.
                int amount = q6dof_solns.size();

                // counter for comparing results, choice for the best result index.
                int counter = 0, choice = 0;

                // Result matrix consists of members under double type.
                // Assign a REALLY large value.
                double minSum = 10000;
                double moveSum;

                // Go through all results...
                for (; counter < amount; ++counter) {
                    moveSum = 0;

                    // Add up absolute values of all members of a single matrix.
                    for (int i = 0; i < q6dof_solns[0].size(); ++i) {

                        // Every matrix is 6 * 1, so the second parameter is always 0.
                        // Type is double, so use abs instead of fabs.
                        moveSum += abs(q6dof_solns[counter](i, 0));
                    }

                    if (moveSum < minSum) {
                        minSum = moveSum;
                        choice = counter;
                    }
                }

                // Print debug info in terminal.
                ROS_INFO("Choose result No.%d.", choice);

                // Choose the best result.
                qvec = q6dof_solns[choice];

                stuff_trajectory(qvec, new_trajectory);

                pub.publish(new_trajectory);
            }
        }
        sleep_timer.sleep();
    }

    return 0;
}


