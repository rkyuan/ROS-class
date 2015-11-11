#include <interesting_moves/interesting_moves.h>


using namespace Eigen;

interestingMoves::interestingMoves(ros::NodeHandle *nh){
	Baxter_traj_streamer  b(nh);
	baxter_traj_streamer_ = &b;
}

void interestingMoves::populateMove1(trajectory_msgs::JointTrajectory &des_trajectory){
	Vectorq7x1 q_pre_pose;
	q_pre_pose<< -0.907528, -0.111813,   2.06622,    1.8737,    -1.295,   2.00164,  -2.87179;
	std::vector<Eigen::VectorXd> des_path;
	Vectorq7x1 q_vec_right_arm;
	q_vec_right_arm =  baxter_traj_streamer_->get_qvec_right_arm();
	des_path.push_back(q_vec_right_arm);
	des_path.push_back(q_pre_pose);
	baxter_traj_streamer_->stuff_trajectory(des_path, des_trajectory);

}
void populateMove2(trajectory_msgs::JointTrajectory &des_trajectory){
	
}
void populateMove3(trajectory_msgs::JointTrajectory &des_trajectory){
	
}