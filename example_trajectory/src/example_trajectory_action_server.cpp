// example_trajectory_action_server: complementary server for trajectory messages

#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include<example_trajectory/TrajActionAction.h>

int g_count = 0;
bool g_count_failure = false;

class TrajectoryActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    actionlib::SimpleActionServer<example_trajectory::TrajActionAction> as_;
    
    // here are some message types to communicate with our client(s)
    example_trajectory::TrajActionGoal goal_; // goal message, received from client
    example_trajectory::TrajActionResult result_; // put results here, to be sent back to the client when done w/ goal
    example_trajectory::TrajActionFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client



public:
    TrajectoryActionServer(); //define the body of the constructor outside of class definition

    ~TrajectoryActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<example_trajectory::TrajActionAction>::GoalConstPtr& goal);
};


TrajectoryActionServer::TrajectoryActionServer() :
   as_(nh_, "example_traj_action_server", boost::bind(&TrajectoryActionServer::executeCB, this, _1),false) 
{
    ROS_INFO("in constructor of TrajectoryActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}

void TrajectoryActionServer::executeCB(const actionlib::SimpleActionServer<example_trajectory::TrajActionAction>::GoalConstPtr& goal) {
    // the class owns the action server, so we can use its member methods here
    int npts = goal->trajectory.points.size();
    ROS_INFO("received trajectory with %d points",npts);
    //should populate result with something useful...
         as_.setSucceeded(result_); // tell the client that we were successful acting on the request, and return the "result" message
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_trajectory_action_node"); // name this node 

    ROS_INFO("instantiating the trajectory action server: ");

    TrajectoryActionServer as_object; // create an instance of the class "TrajectoryActionServer"
    
    ROS_INFO("going into spin");

    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        ros::Duration(0.1).sleep();
    }

    return 0;
}

