/// a set of interesting moves for baxter
#ifndef INTERESTING_MOVES_H
#define INTERESTING_MOVES_H

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

class interestingMoves{
	public:
		void populateMove1(trajectory_msgs::JointTrajectory &des_trajectory);
		void populateMove2(trajectory_msgs::JointTrajectory &des_trajectory);
		void populateMove3(trajectory_msgs::JointTrajectory &des_trajectory);

};


#endif