/// a set of interesting moves for baxter
#ifndef INTERESTING_MOVES_H
#define INTERESTING_MOVES_H

#include <ros/ros.h>

class interestingMoves{
public:
	interestingMoves();
	populateMove1(trajectory_msgs::JointTrajectory &des_trajectory);
	populateMove2(trajectory_msgs::JointTrajectory &des_trajectory);
	populateMove3(trajectory_msgs::JointTrajectory &des_trajectory);

};


#endif