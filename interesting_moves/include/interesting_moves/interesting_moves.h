/// a set of interesting moves for baxter
#ifndef INTERESTING_MOVES_H
#define INTERESTING_MOVES_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <baxter_traj_streamer/trajAction.h>

class interestingMoves{
	public:
		interestingMoves();
		void populateMove1(trajectory_msgs::JointTrajectory &des_trajectory);
		void populateMove2(trajectory_msgs::JointTrajectory &des_trajectory);
		void populateMove3(trajectory_msgs::JointTrajectory &des_trajectory);

};


#endif