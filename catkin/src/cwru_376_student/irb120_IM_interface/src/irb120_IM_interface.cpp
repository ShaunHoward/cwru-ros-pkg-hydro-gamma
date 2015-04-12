// simple_marker_listener.cpp
// Wyatt Newman
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

//callback to subscribe to marker state
Eigen::Vector3d g_p;
Vectorq6x1 g_q_state;
double g_x, g_y, g_z;
//geometry_msgs::Quaternion g_quat; // global var for quaternion
Eigen::Quaterniond g_quat;
Eigen::Matrix3d g_R;
Eigen::Affine3d g_A_flange_desired;
bool g_trigger = false;

//have a tolerance on the goal pose position values
const double POS_TOL = 0.1f;

using namespace std;

void markerListenerCB(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at x: "
            << feedback->pose.position.x << ", y: " << feedback->pose.position.y
            << ", z: " << feedback->pose.position.z << ", quatx: " << feedback->pose.orientation.x 
            << ", quaty: " << feedback->pose.orientation.y << ", quatz: " << feedback->pose.orientation.z << ", quatw: " << feedback->pose.orientation.w);
    //copy to global vars:
    g_p[0] = feedback->pose.position.x;
    g_p[1] = feedback->pose.position.y;
    g_p[2] = feedback->pose.position.z;
    g_quat.x() = feedback->pose.orientation.x;
    g_quat.y() = feedback->pose.orientation.y;
    g_quat.z() = feedback->pose.orientation.z;
    g_quat.w() = feedback->pose.orientation.w;
    g_R = g_quat.matrix();
}

void jointStateCB(const sensor_msgs::JointStatePtr &js_msg) {
    ROS_INFO("Got joint states from callback");
    for (int i = 0; i < 6; i++) {
        g_q_state[i] = js_msg->position[i];
    }
    cout<<"g_q_state: "<<g_q_state.transpose()<<endl;
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

//command robot to move to "qvec" using a trajectory message, sent via ROS-I

void stuff_trajectory(Vectorq6x1 qvec, trajectory_msgs::JointTrajectory &new_trajectory) {

    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    trajectory_msgs::JointTrajectoryPoint trajectory_point2;

    new_trajectory.points.clear();
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

void initialize_arm_position(ros::Publisher pub, Eigen::Matrix3d R_urdf_wrt_DH, Irb120_IK_solver ik_solver){
    trajectory_msgs::JointTrajectory home_trajectory;
    std::vector<Vectorq6x1> q6dof_solns;
    Vectorq6x1 qvec;
    g_p[0] = -0.540994;
    g_p[1] = -0.00188585;
    g_p[2] = 0.571356;
    g_quat.x() =  0.0128913;
    g_quat.y() = -0.710416;
    g_quat.z() = -0.0152647;
    g_quat.w() = 0.703499;
    g_R = g_quat.matrix();

    g_A_flange_desired.translation() = g_p;
    g_A_flange_desired.linear() = g_R;
    cout << "g_p: " << g_p.transpose() << endl;
    cout << "R: " << endl;
    cout << g_R << endl;

    Eigen::Affine3d A_flange_des_DH;
    A_flange_des_DH = g_A_flange_desired;
    A_flange_des_DH.linear() = g_A_flange_desired.linear() * R_urdf_wrt_DH.transpose();

    // int nsolns = ik_solver.ik_solve(A_flange_des_DH);
    // ROS_INFO("there are %d solutions", nsolns);
    // if(nsolns > 0){
    //     ik_solver.get_solns(q6dof_solns);
    
    //     // See how many results we get.
    //     int amount = q6dof_solns.size();

    //     // counter for comparing results, choice for the best result index.
    //     int counter = 0, choice = 0;

    //     // Result matrix consists of members under double type.
    //     // Assign a REALLY large value.
    //     double minSum = 10000;
    //     double moveSum;

    //     // Go through all results...
    //     for (; counter < amount; ++counter) {
    //         moveSum = 0;

    //         // Add up absolute values of all members of a single matrix.
    //         for (int i = 0; i < q6dof_solns[0].size(); ++i) {
    //             // Every matrix is 6 * 1, so the second parameter is always 0.
    //             // Type is double, so use abs instead of fabs.
    //             moveSum += abs(q6dof_solns[counter](i, 0));
    //         }

    //         if (moveSum < minSum) {
    //             minSum = moveSum;
    //             choice = counter;
    //         }
    //     }
    //     // Print debug info in terminal.
    //     ROS_INFO("Choose result No.%d.", choice);
    //     // Choose the best result.
    //     qvec = q6dof_solns[choice];
    //     stuff_trajectory(qvec, home_trajectory);
    //     ROS_INFO("publishng initial trajectory");
    //     //pub.publish(home_trajectory);   
    // }

}

/**
 * Determine if the arm is currently at the goal pose.
 */
bool isAtGoal(){

    cout << "g_p: " << g_p.transpose() << endl;
    cout << "R: " << endl;
    cout << g_R << endl;

    //gather desired position values
    Vectorq6x1 q_des(6);
    //double q_des[7];
    q_des[0] = g_p[0];
    q_des[1] = g_p[1];
    q_des[2] = g_p[2];
    q_des[3] = g_quat.x();
    q_des[4] = g_quat.y();
    q_des[5] = g_quat.z();
  //  q_des[6] = g_quat.w();

  //  g_A_flange_desired.translation();
  //  g_A_flange_desired.linear();

    cout<<"g_q_state: "<<g_q_state.transpose()<<endl;
    cout<<"q_des: "<<q_des.transpose()<<endl;

    // ROS_INFO("The state values are: x: %f, y: %f, z: %f, quatx: %f, quaty: %f, quatz: %f, quatw: %f",
     //    g_q_state[0],g_q_state[1],g_q_state[2],g_q_state[3],g_q_state[4],g_q_state[5],g_q_state[6]);

    // ROS_INFO("The desired values are: x: %f, y: %f, z: %f, quatx: %f, quaty: %f, quatz: %f, quatw: %f",
    //     q_des[0],q_des[1],q_des[2],q_des[3],q_des[4],q_des[5],q_des[6]);

    //check if each current position value is within the tolerance
    //of the desired positiion values.
    for (int i = 0; i < 6; i++) {
        //check if the current position is within a tolerance from the goal position
        if (!(g_q_state[i] > q_des[i] - POS_TOL && g_q_state[i] < q_des[i] + POS_TOL)){
            ROS_INFO("The arm is not currently at the goal");
            //when it is outside of this range, it is not at the goal position
            return false;
        }
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_marker_listener"); // this will be the node name;
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);
    ROS_INFO("setting up subscribers ");
    ros::Subscriber sub_js = nh.subscribe("/abby/joint_states", 1, jointStateCB);
    ros::Subscriber sub_im = nh.subscribe("example_marker/feedback", 1, markerListenerCB);
    ros::ServiceServer service = nh.advertiseService("move_trigger", triggerService);

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


    int nsolns;

    while (ros::ok()) {
        if(first){
      //      first = false;
            initialize_arm_position(pub, R_urdf_wrt_DH,ik_solver);
          //  g_trigger = true;
        }
        ros::spinOnce();

        if (first || g_trigger || !isAtGoal()) {
            //no longer on the first call
            first = false;
             // reset the trigger
            g_trigger = false;

            //is this point reachable?
            A_flange_des_DH = g_A_flange_desired;
            A_flange_des_DH.linear() = g_A_flange_desired.linear() * R_urdf_wrt_DH.transpose();
            cout << "R des DH: " << endl;
            cout << A_flange_des_DH.linear() << endl;
            nsolns = ik_solver.ik_solve(A_flange_des_DH);
            ROS_INFO("there are %d solutions", nsolns);

            if (nsolns > 0){
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
        ros::spinOnce();
        sleep_timer.sleep();
    }

    return 0;
}


