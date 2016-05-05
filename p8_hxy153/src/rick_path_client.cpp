#include <ros/ros.h>
#include <mobot_pub_des_state/path.h> // this message type is defined in the current package


int main(int argc, char **argv) {
	ros::init(argc, argv, "rick_path_client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<mobot_pub_des_state::path>("append_path_queue_service");
    mobot_pub_des_state::path des_path;
    nav_msgs::Path p;
    p.header.frame_id = "odom";
    p.header.stamp = ros::Time::now();
    geometry_msgs::Pose point;
    point.position.x = 4.5;
    geometry_msgs::PoseStamped temp;
    temp.header = p.header;
    temp.pose = point;
    p.poses.push_back(temp);

    point.position.y = 8;
    temp.pose = point;
    p.poses.push_back(temp);

    point.position.x = -10;
    //point.position.y=0;
    temp.pose = point;
    p.poses.push_back(temp);

    des_path.request.path = p;

    client.call(des_path);
}