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
ros::NodeHandle *g_nh;
ros::Publisher *g_pub;


double distPointToPlane(Eigen::Vector3f point, Eigen::Vector3f plane_normal, double planeDist ){
	double dist;
	Eigen::Vector3f plane_point = plane_normal*planeDist;
	dist = plane_normal.dot(point-plane_point);
	return dist;
}

void selectPointCloudCB(const sensor_msgs::PointCloud2 & message_holder){
	//ROS_INFO("1");
	pcl::PointCloud<pcl::PointXYZ> sample;
	pcl::fromROSMsg(message_holder,sample);
	Eigen::Vector3f plane_normal;
	double planeDist;
	CwruPclUtils helper(g_nh);
	


	Eigen::MatrixXf points_mat;
    Eigen::Vector3f cloud_pt;
    //populate points_mat from cloud data;

    int npts = sample.points.size();
    points_mat.resize(3, npts);

    //somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud, with conversions to/from Eigen
    for (int i = 0; i < npts; ++i) {
        cloud_pt = sample.points[i].getVector3fMap();
        points_mat.col(i) = cloud_pt;
    }

    //points_mat = sample.getMatrixXfMap();
    helper.fit_points_to_plane(points_mat,plane_normal,planeDist);

    pcl::PointCloud<pcl::PointXYZ> output;
    pcl::PointCloud<pcl::PointXYZ> lastCloud;
    pcl::fromROSMsg(g_lastCloud,lastCloud);

    for (int i =0; i<lastCloud.points.size(); i++){
    	double dist = distPointToPlane(lastCloud.points[i].getVector3fMap(),plane_normal,planeDist);
    	if (abs(dist)<0.01){
    		output.push_back(lastCloud.points[i]);
    	}
    }

    sensor_msgs::PointCloud2 msgOut;

    pcl::toROSMsg(output,msgOut);

    msgOut.header.frame_id = g_lastCloud.header.frame_id;
    
    g_pub->publish(msgOut);
    //ROS_INFO("2");

}

void updateKinectCB(const sensor_msgs::PointCloud2 & message_holder){
	g_lastCloud = message_holder;
}






int main(int argc, char **argv){
	ros::init(argc,argv,"coplanar_points");
	ros::NodeHandle nh;
	g_nh = &nh;
	ros::Publisher planepub = nh.advertise<sensor_msgs::PointCloud2>("coplanar_points", 1);
	g_pub = &planepub;
	ros::Subscriber my_subscriber_object= nh.subscribe("rviz_selected_points",1,selectPointCloudCB);
	ros::Subscriber my_subscriber_object2= nh.subscribe("kinect/depth/points",1,updateKinectCB);
	ros::spin();
}
