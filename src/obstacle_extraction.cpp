#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());

//get object position (x, y and z)
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& depth_msg){
    //locate_object(depth_msg, parameters);
    pcl::fromROSMsg(*depth_msg, *cloud);
}

void convert_object_position_to_robot_base(Eigen::Vector3d& object_pose_in_camera_frame, Eigen::Vector3d& object_pose_in_robot_frame){
    tf::TransformListener listener;
    tf::StampedTransform stamped_transform;
    //std::string child_frame = "/camera_depth_optical_frame";
    std::string child_frame = "/camera_rgb_optical_frame";
    std::string parent_frame = "/world";
    try{
        listener.lookupTransform(child_frame, parent_frame,
                                 ros::Time::now(), stamped_transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    geometry_msgs::PointStamped camera_point;
    geometry_msgs::PointStamped base_point;
    camera_point.header.frame_id = child_frame;

    //we'll just use the most recent transform available for our simple example
    camera_point.header.stamp = ros::Time();

    //just an arbitrary point in space
    camera_point.point.x = object_pose_in_camera_frame(0);
    camera_point.point.y = object_pose_in_camera_frame(1);
    camera_point.point.z = object_pose_in_camera_frame(2);

    ros::Duration my_duration(0.03);
    try{

        //listener.waitForTransform(parent_frame, child_frame, ros::Time::now(), my_duration);
        listener.transformPoint(parent_frame, camera_point, base_point);

        ROS_INFO("camera_depth_optical_frame: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                 camera_point.point.x, camera_point.point.y, camera_point.point.z,
                 base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from \"camera_depth_optical_frame\" to \"world\": %s", ex.what());
    }
    object_pose_in_robot_frame << base_point.point.x,
            base_point.point.y,
            base_point.point.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_extraction");
    ros::NodeHandle n;

    ros::Subscriber cloud_sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, cloud_cb);

    ros::Publisher first_corner = n.advertise<std_msgs::Float64MultiArray>("obstacle_first_corner", 100);
    ros::Publisher second_corner = n.advertise<std_msgs::Float64MultiArray>("obstacle_second_corner", 100);

    ros::AsyncSpinner my_spinner(1);
    my_spinner.start();

    usleep(2e6);

    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute ();

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEccentricity (eccentricity);
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter (mass_center);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");

    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Vector3d position_camera_frame, position_robot_frame;

    position_camera_frame << position(0), position(1), position(2);
    convert_object_position_to_robot_base(position_camera_frame, position_robot_frame);
    Eigen::Quaternionf quat (rotational_matrix_OBB);
    viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");   

    pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
    pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
    pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
    pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
    viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
    viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
    viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
    ROS_INFO_STREAM("position in rgb_optical frame is: " << position_camera_frame);
    ROS_INFO_STREAM("position in robot frame is: " << position_robot_frame);
    Eigen::Vector3d min_coord, max_coord, min_robot_frame, max_robot_frame;
    min_coord << position_OBB.x - fabs(min_point_OBB.x), position_OBB.y - fabs(min_point_OBB.y), position_OBB.z - fabs(min_point_OBB.z);
    max_coord << position_OBB.x + fabs(max_point_OBB.x), position_OBB.y + fabs(max_point_OBB.y), position_OBB.z + fabs(max_point_OBB.z);
    convert_object_position_to_robot_base(min_coord, min_robot_frame);
    convert_object_position_to_robot_base(max_coord, max_robot_frame);
    ROS_INFO_STREAM("minimum point in camera frame is: " << min_coord);
    ROS_INFO_STREAM("maximum point in camera frame is: " << max_coord);
    ROS_INFO_STREAM("minimum point in robot frame is: " << min_robot_frame);
    ROS_INFO_STREAM("maximum point in robot frame is: " << max_robot_frame);
    //ROS_INFO_STREAM("position in robot frame is: " << position_robot_frame);
    //ROS_INFO_STREAM("orientation is: " << quat.);

    std_msgs::Float64MultiArray first_corner_data, second_corner_data;
    first_corner_data.data.push_back(min_robot_frame(0));
    first_corner_data.data.push_back(min_robot_frame(1));
    first_corner_data.data.push_back(min_robot_frame(2));

    second_corner_data.data.push_back(max_robot_frame(0));
    second_corner_data.data.push_back(max_robot_frame(1));
    second_corner_data.data.push_back(max_robot_frame(2));

    ros::Rate my_rate(1);
    while(!viewer->wasStopped())
    {
        first_corner.publish(first_corner_data);
        second_corner.publish(second_corner_data);
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        ros::spinOnce();
    }
    return 0;
}
