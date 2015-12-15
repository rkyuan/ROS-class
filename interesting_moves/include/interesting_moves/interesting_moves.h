
#ifndef INTERESTING_MOVES_H
#define INTERESTING_MOVES_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <cwru_action/trajAction.h>

///a set of interesting moves designed to show off baxter's glorious muscles

class interestingMoves{
	private:
		//Baxter_traj_streamer *baxter_traj_streamer_;
		ros::NodeHandle *nh_;
	public:
		/**
		*constructor requires nodehandle to be passed in
		*/
		interestingMoves(ros::NodeHandle *nh);
		/**
		*the arm in front of chest flex
		*@param[out] des_trajectory the joint trajectory to be populated
		*/
		void populateMove1(trajectory_msgs::JointTrajectory &des_trajectory);
		/**
		*flexes the arm to the side
		*@param[out] des_trajectory the joint trajectory to be populated
		*/
		void populateMove2(trajectory_msgs::JointTrajectory &des_trajectory);
		/**
		*extends the arm to the side
		*@param[out] des_trajectory the joint trajectory to be populated
		*/
		void populateMove3(trajectory_msgs::JointTrajectory &des_trajectory);
	
};


#endif
