#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
//this #include refers to the new "action" message defined for this package
// the action message can be found in: .../baxter_traj_streamer/action/traj.action
// automated header generation creates multiple headers for message I/O
// these are referred to by the root name (traj) and appended name (Action)
// If you write a new client of the server in this package, you will need to include baxter_traj_streamer in your package.xml,
// and include the header file below
#include <cwru_action/trajAction.h>
#include <interesting_moves/interesting_moves.h>


#define VECTOR_DIM 7 // e.g., a 7-dof vector

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
// void doneCb(const actionlib::SimpleClientGoalState& state,
//         const baxter_traj_streamer::trajResultConstPtr& result) {
//     ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
//     ROS_INFO("got return val = %d; traj_id = %d",result->return_val,result->traj_id);
// }


int main(int argc, char** argv) {
        ros::init(argc, argv, "traj_action_client_node"); // name this node 
        ros::NodeHandle nh; //standard ros node handle        
        int g_count = 0;
                int ans;

    cout<<"warming up callbacks..."<<endl;
    for (int i=0;i<100;i++) {
        ros::spinOnce();
        //cout<<"spin "<<i<<endl;
        ros::Duration(0.01).sleep();
    }


    actionlib::SimpleActionClient<cwru_action::trajAction> action_client("trajActionServer", true);
        
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running


    if (!server_exists) {
        ROS_WARN("could not connect to server; will wait forever");
        return 0; // bail out; optionally, could print a warning message and retry
    }
    server_exists = action_client.waitForServer(); //wait forever 
    
   
    ROS_INFO("connected to action server");  // if here, then we connected to the server;

    cwru_action::trajGoal goal; 
    trajectory_msgs::JointTrajectory interestingTraj;
    interestingMoves mover(&nh);
    mover.populateMove1(interestingTraj);
    goal.trajectory = interestingTraj;
    //cout<<"ready to connect to action server; enter 1: ";
    //cin>>ans;
    // use the name of our server, which is: trajActionServer (named in traj_interpolator_as.cpp)
    

    
    action_client.sendGoal(goal); 
    


    
    
    bool finished_before_timeout = action_client.waitForResult(ros::Duration(8.5));
    mover.populateMove2(interestingTraj);
    goal.trajectory = interestingTraj;
    action_client.sendGoal(goal);
    finished_before_timeout = action_client.waitForResult(ros::Duration(8.5));
     mover.populateMove3(interestingTraj);
    goal.trajectory = interestingTraj;
    action_client.sendGoal(goal);
    finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
    finished_before_timeout = action_client.waitForResult(); // wait forever...
    
    
    //}

    return 0;
}

