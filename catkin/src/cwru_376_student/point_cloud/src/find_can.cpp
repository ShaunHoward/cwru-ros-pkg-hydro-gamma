/**
 * Team Gamma
 * find_can.cpp
 * 
 * A program to acquire a pointcloud from disk, then perform various processing steps interactively.
 * Processing is invoked by point-cloud selections in rviz, as well as "mode" settings via a service
 * e.g.:  rosservice call process_mode 0 induces processing in mode zero (plane fitting), then finds 
 * a cloud of points above the given plane. This cloud of points should be a cylinder shape resembling
 * a can. A model is produced to fit the point cloud can above the table with minimal error based on
 * the initial patch of points given. Then the cylinder coordinates are published via Vector3 to a program to 
 * move the arm above the origin in order to pick the can up on topic: "can_coords"
 */

#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> 
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/features/normal_3d.h>

// this is a pre-defined service message, contained in shared "cwru_srv" package
#include <cwru_srv/simple_int_service_message.h> 

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>

#include <std_msgs/Float32.h>

//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud; // can use this for short-hand

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace pcl::io;

Eigen::Vector3f computeCentroid(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud);
void computeRsqd(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, Eigen::Vector3f centroid, std::vector<float> &rsqd_vec);
Eigen::Vector3f computeCentroid(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, std::vector<int>iselect);
void transform_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::Matrix3f R_xform, PointCloud<pcl::PointXYZ>::Ptr outputCloud);
void transform_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::Matrix3f R_xform, Eigen::Vector3f offset, PointCloud<pcl::PointXYZ>::Ptr outputCloud);
void transform_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::Affine3f A,  PointCloud<pcl::PointXYZ>::Ptr outputCloud);
void filter_cloud_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_nom, double z_eps, vector<int> &indices);
void filter_cloud_above_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_threshold, vector<int> &indices);

void copy_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, PointCloud<pcl::PointXYZ>::Ptr outputCloud);
void copy_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, vector<int> &indices, PointCloud<pcl::PointXYZ>::Ptr outputCloud);

void find_plane(Eigen::Vector4f plane_params, std::vector<int> &indices_z_eps);

void process_patch(std::vector<int> &iselect_filtered, Eigen::Vector3f &centroidEvec3f, Eigen::Vector4f &plane_params);
void compute_radial_error(PointCloud<pcl::PointXYZ>::Ptr inputCloud, std::vector<int> indices, double r, Eigen::Vector3f center, double &E, double &dEdCx, double &dEdCy);
void make_can_cloud(PointCloud<pcl::PointXYZ>::Ptr canCloud, double r_can,double h_can);

// a bunch of pointcloud holders, all global

//pcl::PointCloud<pcl::PointXYZ>::Ptr g_pclKinect(new PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud_from_disk(new pcl::PointCloud<pcl::PointXYZ>); //this one is read from PCD file on disk
pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud_out(new pcl::PointCloud<pcl::PointXYZ>); // holder for processed point clouds
pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>); // holder for processed point clouds
pcl::PointCloud<pcl::PointXYZ>::Ptr g_display_cloud(new pcl::PointCloud<pcl::PointXYZ>); // this cloud gets published--viewable in rviz
pcl::PointCloud<pcl::PointXYZ>::Ptr g_pclSelect(new pcl::PointCloud<pcl::PointXYZ>); // holds published points, per Rviz tool
pcl::PointCloud<pcl::PointXYZ>::Ptr g_canCloud(new pcl::PointCloud<pcl::PointXYZ>); // holds model for a can
pcl::PointCloud<pcl::PointXYZ>::Ptr g_canEstimate(new pcl::PointCloud<pcl::PointXYZ>); // holds model for a can
// PointXYZRGB would be colorized

// a matrix useful for rotating the data
Eigen::Matrix3f g_R_transform; 

//define some processing modes; set these interactively via service
const int FIND_ON_TABLE = 0;
const int IDENTIFY_PLANE = 1;
const int FIND_PNTS_ABOVE_PLANE = 2;
const int COMPUTE_CYLINDRICAL_FIT_ERR_INIT = 3;
const int COMPUTE_CYLINDRICAL_FIT_ERR_ITERATE = 4;
const int PUBLISH_TO_ROBOT = 5;

// choose a tolerance for plane fitting, e.g. 1cm
const double Z_EPS = 0.01; 

// tolerance for finding can in point cloud above selected plane
const double Z_CAN_THRESHOLD = 0.03;

//choose tolerance for min distance
//this is the min depth of the point cloud selection that we want
const double Y_EPS = -0.1;

//choose threshold for max distance
//this is the max depth of the point cloud selection that we want
const double Y_CAN_THRESHOLD = 0.4;

// choose a tolerance for cylinder-fit outliers
const double R_EPS = 0.05; 

// estimated from ruler tool...example to fit a cylinder of this radius to data
const double R_CYLINDER = 0.03; 

// estimated height of cylinder
const double H_CYLINDER = 0.12; 

// the E fit value should be less than this
const double FIT_TOL = 0.000001;

// origin of model for cylinder registration
Eigen::Vector3f g_cylinder_origin; 

// mode--set by service
int g_pcl_process_mode = 0; 

// a trigger, to tell "main" to process points in the currently selected mode
bool g_trigger = false;

// a state--to let us know if a default set of plane_parameters exists
bool g_processed_patch = false; 

//more globals--to share info on planes and patches
Eigen::Vector4f g_plane_params;
Eigen::Vector3f g_patch_centroid;
Eigen::Vector3f g_plane_normal;
Eigen::Vector3f g_plane_origin;
Eigen::Vector3f g_can_origin_robot;
Eigen::Affine3f g_A_plane;
double g_z_plane_nom;
std::vector<int> g_indices_of_plane; //indices of patch that do not contain outliers 

// tf::TransformListener* g_tfListener;
// tf::StampedTransform g_baseLink_wrt_kinect;

const tf::TransformListener* tfListener_;
tf::StampedTransform kinectToRobot; 

void transform_to_robot(){//const PointCloud<pcl::PointXYZ>::Ptr cloud_rcvd) {
    //pcl::PointCloud<pcl::PointXYZ>::Ptr pclKinect(new PointCloud<pcl::PointXYZ>);
    //pcl::fromROSMsg(*cloud, *g_pclKinect);
   // pcl::PointCloud<pcl::PointXYZRGB> cloud_rcvd;
    //pcl::PointCloud<pcl::PointXYZ> cloud_points;
    
    //pcl::fromROSMsg(*cloud, cloud_rcvd);
    
    // try {
    //     tfListener_->lookupTransform("base_link", cloud_rcvd->header.frame_id, ros::Time(0), kinectToRobot);
    // }
    // catch (tf::TransformException e) {
    //     ROS_ERROR("Transform Exception caught: %s",e.what());
    // }

    // pcl_ros::transformPointCloud (*cloud_rcvd, cloud_points, kinectToRobot);
    // cloud_points.header.frame_id="base_link";
    // sensor_msgs::PointCloud2 temp_cloud;
    // pcl::toROSMsg(cloud_points, temp_cloud);
    // pcl::fromROSMsg(temp_cloud, *g_canEstimate);
    // ROS_INFO("Transforming kinect model into base_link frame");

    // const tf::Vector3 kinect_vector(g_cylinder_origin[0], g_cylinder_origin[1], g_cylinder_origin[2]);
    // const tf::Stamped<tf::Vector3> stamped_kinect(kinect_vector, ros::Time::now(), "kinect_pc_frame");

    // // geometry_msgs::Vector3Stamped kinect_vector;
    // // kinect_vector.vector = v;
    // // kinect_vector.vector.x = g_cylinder_origin[0];
    // // kinect_vector.vector.y = g_cylinder_origin[1];
    // // kinect_vector.vector.z = g_cylinder_origin[2];
    // //kinect_vector.header.stamp = ros::Time::now();

    // //const geometry_msgs::Vector3Stamped robot_vector;
    // tf::Stamped<tf::Vector3> robot_vector;

    // g_tfListener->transformVector("base_link", stamped_kinect, robot_vector);

    // g_can_origin_robot[0] = robot_vector.getX();
    // g_can_origin_robot[1] = robot_vector.getY();
    // g_can_origin_robot[2] = robot_vector.getZ();

     try {
        tfListener_->lookupTransform("base_link", "/camera_depth_optical_frame", ros::Time(0), kinectToRobot);
        }
        catch (tf::TransformException e) {
        ROS_ERROR("Transform Exception caught: %s",e.what());
        }
        const tf::Vector3 kinect_vector(g_cylinder_origin[0], g_cylinder_origin[1], g_cylinder_origin[2]);
        const tf::Stamped<tf::Vector3> stamped_kinect(kinect_vector, ros::Time::now(), "/camera_depth_optical_frame");
        // geometry_msgs::Vector3Stamped kinect_vector;
        // kinect_vector.vector = v;
        // kinect_vector.vector.x = g_cylinder_origin[0];
        // kinect_vector.vector.y = g_cylinder_origin[1];
        // kinect_vector.vector.z = g_cylinder_origin[2];
        //kinect_vector.header.stamp = ros::Time::now();
        //const geometry_msgs::Vector3Stamped robot_vector;
        tf::Stamped<tf::Vector3> robot_vector;
        tfListener_->transformVector(string("base_link"), stamped_kinect, robot_vector);
        g_cylinder_origin[0] = robot_vector.getX();
        g_cylinder_origin[1] = robot_vector.getY();
        g_cylinder_origin[2] = robot_vector.getZ();
}

/**
 * Use this service to set processing modes interactively.
 */
bool modeService(cwru_srv::simple_int_service_messageRequest& request, cwru_srv::simple_int_service_messageResponse& response) {
    ROS_INFO("mode select service callback activated");
    
    // boring, but valid response info
    response.resp = true; 
    g_pcl_process_mode = request.req;
    
    //signal that we received a request; trigger a response
    g_trigger = true; 
    cout << "Mode set to: " << g_pcl_process_mode << endl;
    return true;
}

/**
 * This callback wakes up when a new "selected Points" message arrives.
 */
void selectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {

    pcl::fromROSMsg(*cloud, *g_pclSelect);
    ROS_INFO("RECEIVED NEW PATCH w/  %d * %d points", g_pclSelect->width, g_pclSelect->height);
    //ROS_INFO("frame id is: %s",cloud->header.frame_id);
    cout << "header frame: " << cloud->header.frame_id << endl;
    int npts = g_pclSelect->width * g_pclSelect->height;
    
    //indices of patch that do not contain outliers
    std::vector<int> iselect_filtered; 

    //operate on selected points to remove outliers and
    //find centroid and plane params
    process_patch(iselect_filtered, g_patch_centroid, g_plane_params); 
    
    std::cout << "latest filtered global patch: " << g_plane_params.transpose() << std::endl;
    
    // update our states to note that we have process a patch, and thus have valid plane info
    g_processed_patch = true; 
}

/**
 * Process patch: filter selected points to remove outliers;
 * then compute the centroid and the plane parameters of the filtered points
 * return these values in centroidEvec3f and plane_params.
 */
void process_patch(std::vector<int> &iselect_filtered, Eigen::Vector3f &centroidEvec3f, Eigen::Vector4f &plane_params) {
    ROS_INFO("PROCESSING THE PATCH: ");
    int npts = g_pclSelect->width * g_pclSelect->height;
    
    // compute the centroid of this point cloud (selected patch)
    centroidEvec3f = computeCentroid(g_pclSelect); 
    std::vector<float> rsqd_vec;
    computeRsqd(g_pclSelect, centroidEvec3f, rsqd_vec);
    //ROS_INFO("computing rsqd vec: ");
    float variance = 0.0;
    for (int i = 0; i < rsqd_vec.size(); i++) {
        variance += rsqd_vec[i];
        //cout<<rsqd_vec[i]<<", ";      
    }
    cout << endl;
    variance /= ((float) npts);

    // now, eliminate any outliers; actually, keep only points withing 1 std;
    for (int i = 0; i < npts; i++) {
        if (rsqd_vec[i] < variance) {
            // choosey: retain only points within 1 std dev
            iselect_filtered.push_back(i); 
        }
    }
    cout << "npts = " << npts << endl;
    cout << "npts of filtered patch: " << iselect_filtered.size() << endl;
    cout << "variance = " << variance << endl;
    cout << "std_dev: " << sqrt(variance) << endl;
    centroidEvec3f = computeCentroid(g_pclSelect, iselect_filtered);
    cout << "refined centroid:    " << centroidEvec3f.transpose() << endl;

    // object to compute the normal to a set of points
    NormalEstimation<PointXYZ, Normal> n; 

    float curvature;

    // find plane params for filtered patch
    n.computePointNormal(*g_pclSelect, iselect_filtered, plane_params, curvature); 
    
    // any surface viewed w/ z_optical pointing "out" from camera must have a surface normal with z-component that is negative w/rt camera
    if (plane_params[2]>0.0) {
        
        //need to negate the surface normal
        for (int i=0;i<3;i++)
            plane_params[i]*= -1.0;
    }
    std::cout << "plane_params, filtered patch: " << plane_params.transpose() << std::endl;
}

/**
 * This function operates on the global cloud pointer g_cloud_from_disk;
 * g_cloud_transformed contains a cloud rotated s.t. identified plane has normal (0,0,1), 
 * indices_z_eps contain the indices of the points on the identified plane;
 * g_display_cloud is a reduced version of g_cloud_from_disk, w/ only the planar points (expressed in original frame).
 */
void find_plane(Eigen::Vector4f plane_params, std::vector<int> &indices_z_eps) {
    float curvature;
    std::vector<int> iselect_all;
    
    // object to compute the normal to a set of points
    NormalEstimation<PointXYZ, Normal> n;  

   //Eigen::Vector3f plane_normal;
    Eigen::Vector3f x_dir;
    
    //for selected patch, get plane normal from point-cloud processing result
    for (int i = 0; i < 3; i++) g_plane_normal[i] = plane_params[i]; 

    // keep x-axis the same...nominally
    x_dir << 1, 0, 0; 
    
     // force x-dir to be orthogonal to z-dir
    x_dir = x_dir - g_plane_normal * (g_plane_normal.dot(x_dir));
    
    // want this to be unit length as well
    x_dir /= x_dir.norm(); 
    
    //populate g_R_transform with the direction vectors of the plane frame, with respect to the kinect frame
    g_R_transform.col(0) = x_dir;
    
    // want the z-axis to be the plane normal 
    g_R_transform.col(2) = g_plane_normal;    
    
    //make y-axis consistent right-hand triad
    g_R_transform.col(1) = g_R_transform.col(2).cross(g_R_transform.col(0)); 
    
    // let's define an origin on this plane as well.  The patch centroid should do
    g_plane_origin = g_patch_centroid;
    
    // define the transform s.t. A*pt_wrt_plane coords = pt_wrt_sensor_coords
    g_A_plane.linear()= g_R_transform;
    g_A_plane.translation() = g_plane_origin;
    
    // use the following to transform kinect points into the plane frame; could do translation as well, but not done here
    Eigen::Matrix3f R_transpose = g_R_transform.transpose();

    // distance of plane from sensor origin--same as distance measured along plane normal
    g_z_plane_nom = plane_params[3]; 
    
    // after rotating the points to align with the plane of the selected patch, all z-values should be approximately the same,
    // = z_plane_nom
    double z_eps = Z_EPS; // choose a tolerance for plane inclusion +/- z; 1cm??

    //OK...let's try transforming the ENTIRE point cloud:
    //transform_cloud(g_cloud_from_disk, R_transpose, g_cloud_transformed); // rotate the entire point cloud
    // transform the entire point cloud 
    transform_cloud(g_cloud_from_disk, g_A_plane.inverse(), g_cloud_transformed);   
    
    // g_cloud_transformed is now expressed in the frame of the selected plane;
    // let's extract all of the points (i.e., name the indices of these points) for which the z value corresponds to the chosen plane,
    // within tolerance z_eps
    //filter_cloud_z(g_cloud_transformed, g_z_plane_nom, z_eps, indices_z_eps);
    //transform --> z-values of points on plane should be 0.0
    filter_cloud_z(g_cloud_transformed, 0.0, z_eps, indices_z_eps); 
    
    // point indices of interest are in indices_z_eps; use this to extract this subset from the parent cloud to create a new cloud
    copy_cloud(g_cloud_from_disk, indices_z_eps, g_display_cloud); //g_display_cloud is being published regularly by "main"
}


/**
 * Given a point cloud, compute the centroid. Mostly useful for small patches, 
 * since centroid of the whole cloud is not too useful, no selection vector provided,
 *  --> use all the points.
 */
Eigen::Vector3f computeCentroid(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud) {
    Eigen::Vector3f centroid;
    centroid << 0, 0, 0;

    int size = pcl_cloud->width * pcl_cloud->height;
    std::cout << "frame: " << pcl_cloud->header.frame_id << std::endl;
    for (size_t i = 0; i != size; ++i) {
        centroid += pcl_cloud->points[i].getVector3fMap();
    }
    if (size > 0) {
        centroid /= ((float) size);
    }
    return centroid;
}

/**
 * Second version: 
 * will operate only on the points listed by index in iselect
 */
Eigen::Vector3f computeCentroid(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, std::vector<int>iselect) {
    Eigen::Vector3f centroid;
    centroid << 0, 0, 0;
    int nselect = iselect.size();
    for (int i = 0; i < nselect; ++i) {
        centroid += pcl_cloud->points[iselect[i]].getVector3fMap();
    }
    if (nselect > 0) {
        centroid /= ((float) nselect);
    }
    return centroid;
}

/**
 * Compute the distance-squared of each point from the provided centroid
 * presumably useful for outlier removal filtering.
 */
void computeRsqd(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, Eigen::Vector3f centroid, std::vector<float> &rsqd_vec) {
    Eigen::Vector3f evec3f;
    int npts = pcl_cloud->points.size();
    rsqd_vec.clear();
    rsqd_vec.resize(npts);
    for (int i = 0; i < npts; i++) {
        evec3f = pcl_cloud->points[i].getVector3fMap();
        evec3f -= centroid;
        rsqd_vec[i] = evec3f.dot(evec3f);
    }
}

/**
 * Given an input cloud, rotate ALL points using matrix R_xform, and put the result in outputCloud.
 * This should be generalized for an affine transform (translation plus rotation).
 */
void transform_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::Matrix3f R_xform, PointCloud<pcl::PointXYZ>::Ptr outputCloud) {
    // copy over the header info from the inputCloud.
    outputCloud->header = inputCloud->header;
    outputCloud->is_dense = inputCloud->is_dense;
    outputCloud->width = inputCloud->width;
    outputCloud->height = inputCloud->height;
    int npts = inputCloud->points.size();
    cout << "transforming npts = " << npts << endl;
    outputCloud->points.resize(npts);
    Eigen::Vector3f pt;
    for (int i = 0; i < npts; ++i) {
        pt = R_xform * inputCloud->points[i].getVector3fMap();
        //cout<<"transformed pt: "<<pt.transpose()<<endl;
        outputCloud->points[i].getVector3fMap() = pt; //R_xform * inputCloud->points[i].getVector3fMap ();
    }
}

void transform_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::Matrix3f R_xform, Eigen::Vector3f offset, PointCloud<pcl::PointXYZ>::Ptr outputCloud) {
    // copy over the header info from the inputCloud.
    outputCloud->header = inputCloud->header;
    outputCloud->is_dense = inputCloud->is_dense;
    outputCloud->width = inputCloud->width;
    outputCloud->height = inputCloud->height;
    int npts = inputCloud->points.size();
    cout << "transforming npts = " << npts << endl;
    outputCloud->points.resize(npts);
    Eigen::Vector3f pt;
    for (int i = 0; i < npts; ++i) {
        pt = R_xform * inputCloud->points[i].getVector3fMap() + offset; // + 
        //cout<<"transformed pt: "<<pt.transpose()<<endl;
        outputCloud->points[i].getVector3fMap() = pt; //R_xform * inputCloud->points[i].getVector3fMap ();
    }
}

/**
 * For this version, provide an affine transform:
 */
void transform_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::Affine3f A,  PointCloud<pcl::PointXYZ>::Ptr outputCloud) {
    outputCloud->header = inputCloud->header;
    outputCloud->is_dense = inputCloud->is_dense;
    outputCloud->width = inputCloud->width;
    outputCloud->height = inputCloud->height;
    int npts = inputCloud->points.size();
    cout << "transforming npts = " << npts << endl;
    outputCloud->points.resize(npts);
    Eigen::Vector3f pt;
    for (int i = 0; i < npts; ++i) {
        outputCloud->points[i].getVector3fMap() = A * inputCloud->points[i].getVector3fMap(); // + 
        //cout<<"transformed pt: "<<pt.transpose()<<endl;
        //outputCloud->points[i].getVector3fMap() = pt; //R_xform * inputCloud->points[i].getVector3fMap ();
    }    
}

/**
 * Function to copy ALL of the cloud points.
 */
void copy_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, PointCloud<pcl::PointXYZ>::Ptr outputCloud) {
    int npts = inputCloud->points.size(); //how many points to extract?
    outputCloud->header = inputCloud->header;
    outputCloud->is_dense = inputCloud->is_dense;
    outputCloud->width = npts;
    outputCloud->height = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud->points.resize(npts);
    for (int i = 0; i < npts; ++i) {
        outputCloud->points[i].getVector3fMap() = inputCloud->points[i].getVector3fMap();
    }
}

/**
 * Make a new point cloud, extracted from inputCloud using only points listed  in "indices".
 */
void copy_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, vector<int> &indices, PointCloud<pcl::PointXYZ>::Ptr outputCloud) {
    int npts = indices.size(); //how many points to extract?
    outputCloud->header = inputCloud->header;
    outputCloud->is_dense = inputCloud->is_dense;
    outputCloud->width = npts;
    outputCloud->height = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud->points.resize(npts);
    for (int i = 0; i < npts; ++i) {
        outputCloud->points[i].getVector3fMap() = inputCloud->points[indices[i]].getVector3fMap();
    }
}



/**
 * Given a cloud, identify which points are within z_eps of z_nom; put these point indices in "indices".
 * If the cloud has been rotated such that we expect a plane with normals (0,0,1) and offset z_nom,
 * this function will find the points within tolerance z_eps of such a plane.
 */
void filter_cloud_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_nom, double z_eps, vector<int> &indices) {
    int npts = inputCloud->points.size();
    Eigen::Vector3f pt;
    indices.clear();
    double dz;
    int ans;
    for (int i = 0; i < npts; ++i) {
        pt = inputCloud->points[i].getVector3fMap();
        //cout<<"pt: "<<pt.transpose()<<endl;
        dz = pt[2] - z_nom;
        if (fabs(dz) < z_eps) {
            indices.push_back(i);
            //cout<<"dz = "<<dz<<"; saving this point...enter 1 to continue: ";
            //cin>>ans;
        }
    }
    int n_extracted = indices.size();
    cout << " number of points in range = " << n_extracted << endl;
}

/**
 * Given a cloud, identify which points are ABOVE z_threshold; put these point indices in "indices".
 * This can be useful, e.g., for finding objects "on" a table.
 */
void filter_cloud_above_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_threshold, vector<int> &indices) {
    int npts = inputCloud->points.size();
    Eigen::Vector3f pt;
    indices.clear();
    //double dz;
    int ans;
    for (int i = 0; i < npts; ++i) {
        pt = inputCloud->points[i].getVector3fMap();
        //cout<<"pt: "<<pt.transpose()<<endl;
        //dz = pt[2] - z_threshold;
        if (pt[2] < H_CYLINDER && pt[2] > z_threshold) {
            indices.push_back(i);
            //cout<<"dz = "<<dz<<"; saving this point...enter 1 to continue: ";
            //cin>>ans;
        }
    }
    int n_extracted = indices.size();
    cout << " number of points extracted = " << n_extracted << endl;
}

/**
 * Given a cloud, identify which points are ABOVE z_threshold; put these point indices in "indices".
 * This can be useful, e.g., for finding objects "on" a table.
 */
void filter_cloud_in_range(PointCloud<pcl::PointXYZ>::Ptr inputCloud, vector<int> &indices_above, vector<int> &indices_in_range) {
    int npts = indices_above.size();
    
    cout << " number of initial points = " << npts << endl;     

    Eigen::Vector3f pt;
    indices_in_range.clear();
    int index = 0;
    for (int i = 0; i < npts; ++i) {
	index = indices_above[i];
        pt = inputCloud->points[index].getVector3fMap();
        if (pt[1] > Y_EPS && pt[1] < Y_CAN_THRESHOLD) {
            indices_in_range.push_back(index);
        }
    }
    int n_extracted = indices_in_range.size();
    cout << " number of points extracted = " << n_extracted << endl;
}

/**
 * Given a cloud, identify which points are ABOVE z_threshold; put these point indices in "indices".
 * This can be useful, e.g., for finding objects "on" a table.
 */
void filter_cloud_above_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud, PointCloud<pcl::PointXYZ>::Ptr outputCloud, double z_threshold) {
    int npts = inputCloud->points.size();
    Eigen::Vector3f pt;
    //double dz;
    int ans;
    int curr_index = 0;

    //takes approx 10 points from center of the cloud above z
    int mid_lower = (npts / 2) - 5;
    int mid_upper = (npts / 2) + 5;
    for (int i = 0; i < npts; ++i) {
        pt = inputCloud->points[i].getVector3fMap();
        //cout<<"pt: "<<pt.transpose()<<endl;
        //dz = pt[2] - z_threshold;
        //only take points from near the middle of the point cloud
        if (pt[2] > z_threshold && mid_lower <= i && i <= mid_upper) {
            outputCloud->points.push_back(inputCloud->points[i]);
            //cout<<"dz = "<<dz<<"; saving this point...enter 1 to continue: ";
            //cin>>ans;
        }
    }
    int n_extracted = outputCloud->points.size();
    cout << " number of points extracted = " << n_extracted << endl;
}

/**
 * This is a pretty specific function, but useful for illustrating how to create a point cloud that
 * can be visualized in rviz.
 * Create a cloud to visualize a can of radius r, height h.
 * Assume origin of bottom of can is 0,0,0, and z-axis points up;
 */
void make_can_cloud(PointCloud<pcl::PointXYZ>::Ptr canCloud, double r_can, double h_can) {
    double theta,h;
    Eigen::Vector3f pt;
    int npts=0;
    //count the points:
    for (theta=0;theta<2.0*M_PI;theta+=0.3)
        for (h=0;h<h_can;h+= 0.01)  
            npts++;
    canCloud->points.resize(npts);
    int i=0;
    for (theta=0;theta<2.0*M_PI;theta+=0.3)
        for (h=0;h<h_can;h+= 0.01) {
            
            pt[0] = r_can*cos(theta);
            pt[1] = r_can*sin(theta);
            pt[2] = h;
            canCloud->points[i].getVector3fMap() = pt;
            i++;
        }
    //canCloud->header = inputCloud->header;
    canCloud->header.frame_id = "/camera_depth_optical_frame"; 
    //canCloud->header.stamp = ros::Time::now();
    canCloud->is_dense = true;
    canCloud->width = npts;
    canCloud->height = 1;

    copy_cloud(canCloud,g_canCloud);
    //optionally, rotate the cylinder to make its axis parallel to the most recently defined normal axis
    //transform_cloud(g_canCloud, g_R_transform, canCloud);    
}

/**
 * COMPUTE_RADIAL_ERROR:
 * Try to fit a cylinder to points in inputCloud, assuming cylinder axis is vertical (0,0,1).  
 * Given radius of model and center coords, cx, cy, return the fit error and derivatives dE/dCx and dE/dCy
 * Eliminate points that are too far from expectation
 * Define the error as: r_i = sqrt(c_vec - p_i); err_i = (r_i - r)^2; E = sum_i err_i
 * inputCloud should be rotated appropriately, so expected cylinder has major axis along z
 */
void compute_radial_error(PointCloud<pcl::PointXYZ>::Ptr inputCloud, std::vector<int> indices, double r, Eigen::Vector3f center, double &E, double &dEdCx, double &dEdCy) {
    std::vector<int> copy_indices;
    int npts = indices.size();
    copy_indices.resize(npts);
    
    cout << " number of initial points = " << npts << endl;     
    for (int i=0;i<npts;i++) copy_indices[i]=indices[i];
    
    // we will re-populate this with inliers
    indices.clear(); 
    double sum_sqd_err=0.0;
    double r_i,err_i,r_sqd_err;
    dEdCx=0.0;
    dEdCy=0.0;
    E=0.0;
    double dx,dy;
    Eigen::Vector3f pt;
    //indices.clear();
    //double dz;
    int ans;
    for (int i = 0; i < npts; ++i) {
        pt = inputCloud->points[copy_indices[i]].getVector3fMap();
        dx = center[0] - pt[0];
        dy = center[1] - pt[1];
        r_i = sqrt(dx*dx+dy*dy);
        r_sqd_err = (r_i - r)*(r_i-r);
        err_i = sqrt(r_sqd_err);
        if (err_i<R_EPS) {
            // include this point as an inlier:
            indices.push_back(copy_indices[i]);
            sum_sqd_err += r_sqd_err;
            dEdCx += (r_i-r)*(dx)/r_i;
            dEdCy += (r_i-r)*(dy)/r_i;            
        }       
    }
    int n_extracted = indices.size();
    cout << " number of inliers = " << n_extracted << endl;   
    E = 0.5*sum_sqd_err/n_extracted;
    dEdCx/= n_extracted;
    dEdCy/= n_extracted;

    cout<<"rms radial error: "<<sqrt(2.0*E)<<endl;
    cout<<"E = "<<E<<endl;    
    cout<<"dE/dCx = "<<dEdCx<<"; dEdCy = "<<dEdCy<<endl;
}

int main(int argc, char** argv) {
    // Do some initialization here
    ros::init(argc, argv, "process_pcl");
    ros::NodeHandle nh;
    ros::Rate rate(2);
    // Subscribers
    // use the following, if have "live" streaming from a Kinect
    //ros::Subscriber getPCLPoints = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect/depth/points", 1, kinectCB);
    
    // subscribe to "selected_points", which is published by Rviz tool
    ros::Subscriber selectedPoints = nh.subscribe<sensor_msgs::PointCloud2> ("/selected_points", 1, selectCB);

    // have rviz display both of these topics
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/plane_model", 1);
    ros::Publisher pubPcdCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcd_from_disk", 1);
    ros::Publisher pubCanCoords = nh.advertise<geometry_msgs::Vector3>("can_coords", 1);

    // service used to interactively change processing modes
    ros::ServiceServer service = nh.advertiseService("process_mode", modeService);

    std::vector<int> indices_pts_above_plane;
    std::vector<int> indices_pts_in_range;

    //load a pointcloud from file: 
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *g_cloud_from_disk) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
            << g_cloud_from_disk->width * g_cloud_from_disk->height
            << " data points from test_pcd.pcd  " << std::endl;

    g_cloud_from_disk->header.frame_id = "/camera_depth_optical_frame"; //looks like PCD does not encode the reference frame id
    double z_threshold=Z_CAN_THRESHOLD;
    double E;
    double dEdCx=0.0;
    double dEdCy=0.0;
    bool tried_model_fit = false;

    ROS_INFO("Waiting on a tf from kinect to robot frame");
    tf::TransformListener tf_listener_;
    tfListener_ = &tf_listener_;
    ROS_INFO("Received a good tf");
 
    int ans;
    Eigen::Vector3f can_center_wrt_plane;
    Eigen::Affine3f A_plane_to_sensor;
    
    //use these for fixing errors in registration of can cloud
    Eigen::Vector3f dEdC;
    Eigen::Vector3f dEdC_norm;

    //number of error fit revision iterations
    int numIter = 0;

    //track the origin of the can
    geometry_msgs::Vector3 can_origin;

    while (ros::ok()) {
        if (g_trigger) {
            ROS_INFO("g_trigger enabled");
            numIter = 0;

            /**
             * Identify the plane from a selected patch of points.
             * Find the points above the plane and keep track of them.
             * Compute the cylindrical fit error and make a can cloud to fit the point cloud
             * can representation.
             * Iterate over the model fit error to minimize it using gradient descent optimization.
             * Publish the can coordinates to the topic "can_coords".
             *
             * This method of processing is essentially autonomous. All that has to be done is to select a patch from the table,
             * call service "process_mode 0", and make sure the can model lines up with the point cloud representation. 
             * Then the arm can be run to go pickup the can.
             */
            switch (g_pcl_process_mode) { 
                case FIND_ON_TABLE:
                    ROS_INFO("Executing all modes.");

                case IDENTIFY_PLANE:
                    ROS_INFO("MODE 1: identifying plane based on patch selection...");
                    
                    // results in g_display_cloud (in orig frame), as well as 
                    find_plane(g_plane_params, g_indices_of_plane); 
                    //g_cloud_transformed (rotated version of original cloud); g_indices_of_plane indicate points on the plane
                    
                case FIND_PNTS_ABOVE_PLANE:
                    ROS_INFO("MODE 2: filtering for points above identified plane");

                    // w/ affine transform, z-coord of points on plane (in plane frame) should be ~0
                    z_threshold = 0.0 + Z_EPS; //g_plane_params[3] + Z_EPS;

                    ROS_INFO("filtering for points above %f ", z_threshold);

                    filter_cloud_above_z(g_cloud_transformed, z_threshold, indices_pts_above_plane);

		    ROS_INFO("filtering for points between y: %f and y: %f", Y_EPS, Y_CAN_THRESHOLD);
		    filter_cloud_in_range(g_cloud_transformed, indices_pts_above_plane, indices_pts_in_range);

                    //extract these points--but in original, non-rotated frame; useful for display
                    copy_cloud(g_cloud_from_disk, indices_pts_in_range, g_display_cloud);

		    //and also display whatever we choose to put in here
		    pubCloud.publish(g_display_cloud); 
                    
                case COMPUTE_CYLINDRICAL_FIT_ERR_INIT:
                    ROS_INFO("MODE 3: Creating centroid and a can model for the cloud");

                    //the cylinder origin will temporarily be the centroid of the points about the plane
                    g_cylinder_origin = computeCentroid(g_cloud_from_disk, indices_pts_in_range);
                    
                    make_can_cloud(g_display_cloud, R_CYLINDER, H_CYLINDER);
                    
                    //walk back from the normal of the surface by one radius
                    //to get near the center of the object
                    g_cylinder_origin[1] -= R_CYLINDER;
                    
                    //the initial guess for the origin of the cylinder, expressed in sensor coords,
                    //has z-height fixed based on plane height
                    g_cylinder_origin = g_cylinder_origin + (g_z_plane_nom - g_plane_normal.dot(g_cylinder_origin))*g_plane_normal;
		    ROS_INFO("Cylinder origin guess is: x: %f, y: %f, z: %f",g_cylinder_origin[0], g_cylinder_origin[1], g_cylinder_origin[2]);

                    //now, cast this into the rotated coordinate frame:
                    can_center_wrt_plane = g_A_plane.inverse()*g_cylinder_origin; 

                    cout<<"initial guess for cylinder fit: "<<endl;
                    cout<<" attempting fit at c = "<<can_center_wrt_plane.transpose()<<endl;                

                    compute_radial_error(g_cloud_transformed,indices_pts_in_range,R_CYLINDER,can_center_wrt_plane,E,dEdCx,dEdCy);
                    cout<<"E: "<<E<<"; dEdCx: "<<dEdCx<<"; dEdCy: "<<dEdCy<<endl;
                    cout<<"R_xform: "<<g_R_transform<<endl;
                    A_plane_to_sensor.linear() = g_R_transform;
                    A_plane_to_sensor.translation() = g_cylinder_origin;
                    transform_cloud(g_canCloud, A_plane_to_sensor, g_display_cloud);
                    
                case COMPUTE_CYLINDRICAL_FIT_ERR_ITERATE:    
                    
                    //Minimize the error between the model and the point cloud can origin    
                    while (E > FIT_TOL && numIter < 100){
                        cout<<"current cx,cy = "<<can_center_wrt_plane[0]<<", "<<can_center_wrt_plane[1]<<endl;
                          
                        dEdC[0] = dEdCx;
                        dEdC[1] = dEdCy;
                        dEdC[2] = can_center_wrt_plane[2];

                        //get normalized error vector
                        dEdC_norm = dEdC.normalized();

                        //use gradient descent to improve registration values
                        //from normalized error vector
                        can_center_wrt_plane[0] -= 0.001 * dEdC_norm[0];
                        can_center_wrt_plane[1] -= 0.001 * dEdC_norm[1];

                        ROS_INFO(" MODE 4: attempting to fit points to cylinder, radius %f, cx = %f, cy = %f",R_CYLINDER,can_center_wrt_plane[0],can_center_wrt_plane[1]);
                        compute_radial_error(g_cloud_transformed,indices_pts_in_range,R_CYLINDER,can_center_wrt_plane,E,dEdCx,dEdCy);
                        cout<<"E: "<<E<<"; dEdCx: "<<dEdCx<<"; dEdCy: "<<dEdCy<<endl;

                        //set the cylinder origin to the latest optimized guess
                        g_cylinder_origin = g_A_plane*can_center_wrt_plane; 
                        A_plane_to_sensor.translation() = g_cylinder_origin;
                        
                        tried_model_fit = true;
                        numIter++;
                    }
                    transform_cloud(g_canCloud, A_plane_to_sensor, g_display_cloud);
                    
                case PUBLISH_TO_ROBOT:

                    //publish the origin of the can for the arm to subscribe to
                    can_origin.x = g_cylinder_origin[0];
                    can_origin.y = g_cylinder_origin[1];
                    can_origin.z = g_cylinder_origin[2];

                    ROS_INFO("Publishing the can origin: x: %f, y: %f, z %f", can_origin.x, can_origin.y, can_origin.z);
                    pubCanCoords.publish(can_origin);

                    g_trigger = false;
                    break;

                default:
                    ROS_WARN("this mode is not implemented");
                    break;
            }
        }

        //keep displaying the original scene
        pubPcdCloud.publish(g_cloud_from_disk); 

        //and also display whatever we choose to put in here
        pubCloud.publish(g_display_cloud); 

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
