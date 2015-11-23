#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <math.h>

class gripper_control{
public:
	gripper_control(ros::NodeHandle* nodehandle);
private:
	ros::NodeHandle nh_;
	ros::Publisher  gripPub_;
	void open();
	void close();
}