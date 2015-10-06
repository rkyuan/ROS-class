// example_trajectory_action_client: 

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include<example_trajectory/TrajActionAction.h>


// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
        const example_trajectory::TrajActionResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "demo_trajectory_client_node"); // name this node 
        example_trajectory::TrajActionGoal goal; 
	double omega = 1.0; //rad/sec
        double amp = 0.5; //radians
	double final_time; //seconds
        double phase = 0.0; //radians
	double start_angle= amp;
	double final_phase = 4*3.1415927; // radians--two periods
	double dt = 0.1; // break up trajectory into incremental commands this far apart in time
	double time_from_start = 0.0;
	double q_des,qdot_des;
        
        actionlib::SimpleActionClient<example_trajectory::TrajActionAction> action_client("example_traj_action_server", true);
        
        // attempt to connect to the server:
        ROS_INFO("waiting for server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
        //bool server_exists = action_client.waitForServer(); //wait forever

        if (!server_exists) {
            ROS_WARN("could not connect to server; halting");
            return 0; // bail out; optionally, could print a warning message and retry
        }
        
       
        ROS_INFO("connected to action server");  // if here, then we connected to the server;


        // stuff a goal message:
	trajectory_msgs::JointTrajectory trajectory;
	trajectory_msgs::JointTrajectoryPoint trajectory_point;
	//list the joint names:
	trajectory.joint_names.push_back("joint1");
	// repeat the above command for every joint of the robot, in some preferred order
	// joint position commands below must be specified in the same order
	int njnts = trajectory.joint_names.size(); // need same size for position and velocity vectors
	trajectory_point.positions.resize(njnts);
	trajectory_point.velocities.resize(njnts);

	ROS_INFO("populating trajectory...");
	for (phase=0.0;phase<final_phase;phase+=omega*dt) {
		ROS_INFO("phase = %f",phase);
		q_des = start_angle + amp*sin(phase);
		qdot_des = amp*omega*cos(phase); // this is the time derivative of q_des
		trajectory_point.positions[0] = q_des; // do this for every joint, from 0 through njnts-1
		trajectory_point.velocities[0] = qdot_des; // and all velocities
		time_from_start+= dt;
		//specify arrival time for this point--in ROS "duration" format
		trajectory_point.time_from_start = ros::Duration(time_from_start);
		//append this trajectory point to the vector of points in trajectory:
		trajectory.points.push_back(trajectory_point);		
	}
	final_time = time_from_start; // the last assigned time
	int npts = trajectory.points.size();
	ROS_INFO("populated trajectory with %d points",npts);
	//copy this trajectory into our action goal:	
	goal.trajectory = trajectory;

	//and send out the goal:
        action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired

        // wait for expected duration--plus some tolerance
        bool finished_before_timeout = action_client.waitForResult(ros::Duration(final_time+2.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result");
            return 0;
        }
        else {
          //if here, then server returned a result to us
        }
        

    return 0;
}

