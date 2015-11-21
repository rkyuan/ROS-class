// subscribe to rviz selected points 
// create plane off of that
// subscribe to kinect
// for all points that fit the plane add to new point cloud
// publish to rviz selected points

#include <ros/ros.h> 
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <vector>

#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <sensor_msgs/PointCloud2.h> //useful ROS message types
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <cwru_msgs/PatchParams.h>

#include <tf/transform_listener.h>  // transform listener headers
#include <tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>  //point-cloud library headers; likely don't need all these
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>

#include <cwru_pcl_utils/cwru_pcl_utils.h>

sensor_msgs::PointCloud2 g_lastCloud;

void selectPointCloudCB(const sensor_msgs::PointCloud2 & message_holder){
	pcl::PointCloud<pcl::PointXYZ> sample;
	pcl::fromROSMsg(message_holder,sample);
	Eigen::Vector3f plane_normal;
	double planeDist;
	cwru_pcl_utils::cwru_pcl_utils helper;
	helper.fit_points_to_plane(sample,plane_normal,planeDist);
}

void updateKinectCB(const sensor_msgs::PointCloud2 & message_holder){
	g_lastCloud = message_holder;
}





int main(int argc, char **argv){
	ros::NodeHandle nh;
	ros::Subscriber my_subscriber_object= nh.subscribe("rviz_selected_points",1,selectPointCloudCB);
	ros::Subscriber my_subscriber_object2= nh.subscribe("kinect?",1,updateKinectCB);
}
