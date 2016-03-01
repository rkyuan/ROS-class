
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <p4_hxy153/robotPathAction.h> //reference action message in this package
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <vector>


using namespace std;

bool g_lidar_alarm=false;
bool g_done = false;

void alarmCallback(const std_msgs::Bool& alarm_msg) 
{ 
  g_lidar_alarm = alarm_msg.data; //make the alarm status global, so main() can use it
  if (g_lidar_alarm) {
     ROS_INFO("LIDAR alarm received!"); 
  }
} 

void doneCb(const actionlib::SimpleClientGoalState& state,
        const p4_hxy153::robotPathResultConstPtr& result) {
    g_done = true;
}


void feedbackCb(const p4_hxy153::robotPathFeedbackConstPtr& fdbk_msg) {
    
}

// Called once when the goal becomes active; not necessary, but could be useful diagnostic
void activeCb()
{
  ROS_INFO("Goal just went active");
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "robot_path_client"); // name this node 
        ros::NodeHandle n;
        ros::Rate main_timer(1.0);
        // here is a "goal" object compatible with the server, as defined in example_action_server/action
        
        
        // use the name of our server, which is: timer_action (named in example_action_server_w_fdbk.cpp)
        // the "true" argument says that we want our new client to run as a separate thread (a good idea)
        ros::Subscriber alarm_subscriber = n.subscribe("lidar_alarm",1,alarmCallback);
        actionlib::SimpleActionClient<p4_hxy153::robotPathAction>action_client("path_follower", true);
        
        // attempt to connect to the server: need to put a test here, since client might launch before server
        ROS_INFO("attempting to connect to server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(1.0)); // wait for up to 1 second
        // something odd in above: sometimes does not wait for specified seconds, 
        //  but returns rapidly if server not running; so we'll do our own version
        while (!server_exists && ros::ok()) { // keep trying until connected
            ROS_WARN("could not connect to server; retrying...");
            server_exists = action_client.waitForServer(ros::Duration(1.0)); // retry every 1 second
            ros::spinOnce();
        }
        ROS_INFO("connected to action server");  // if here, then we connected to the server;


        p4_hxy153::robotPathGoal goal;
        std::vector<geometry_msgs::PoseStamped> waypoints;
        geometry_msgs::PoseStamped p;
        //waypoints.push_back(p);
        for(int i = 0; i < 3; i++){
            p.pose.position.x = 5;
            waypoints.push_back(p);
            p.pose.position.y = 5;
            waypoints.push_back(p);
            p.pose.position.x = 0;
            waypoints.push_back(p);
            p.pose.position.y = 0;
            waypoints.push_back(p);
        }
        goal.input.poses = waypoints;
        


        action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

        while(ros::ok){
            ros::spinOnce();
            if (g_done){
                break;
            }
            if(g_lidar_alarm){
                //g_lidar_alarm=false;
                action_client.cancelGoal();
                ROS_WARN("ABORT!");
                //waypoints.push_back(p);
                waypoints.clear();
                for(int i = 0; i < 1; i++){
                    p.pose.position.x = -3;
                    waypoints.push_back(p);
                    p.pose.position.y = -3;
                    waypoints.push_back(p);
                    p.pose.position.x = 0;
                    waypoints.push_back(p);
                    p.pose.position.y = 0;
                    waypoints.push_back(p);
                }
                goal.input.poses = waypoints;
                action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
                break;
                
            }

        }
        //check lidar loop
        
        // int countdown_goal = 1; //user will specify a timer value
        
        //    cout<<"enter a desired timer value, in seconds (0 to abort, <0 to quit): ";
        //    cin>>countdown_goal;
        //    if (countdown_goal==0) { //see if user wants to cancel current goal
        //      ROS_INFO("cancelling goal");
        //      action_client.cancelGoal(); //this is how one can cancel a goal in process
        //    }
        //    if (countdown_goal<0) { //option for user to shut down this client
        //       ROS_INFO("this client is quitting");
        //       return 0;
        //    }
        //    //if here, then we want to send a new timer goal to the action server
        //    ROS_INFO("sending timer goal= %d seconds to timer action server",countdown_goal);
        //    goal.input = countdown_goal; //populate a goal message
        //    //here are some options:
        //    //action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
        //    //action_client.sendGoal(goal,&doneCb); // send goal and specify a callback function
        //    //or, send goal and specify callbacks for "done", "active" and "feedback"
        //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); 
           
        //    //this example will loop back to the the prompt for user input.  The main function will be
        //    // suspended while waiting on user input, but the callbacks will still be alive
        //    //if user enters a new goal value before the prior request is completed, the prior goal will
        //    // be aborted and the new goal will be installed
        
       
    return 0;
}

