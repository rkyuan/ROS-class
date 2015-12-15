#include <interesting_moves/interesting_moves.h>


using namespace Eigen;

interestingMoves::interestingMoves(ros::NodeHandle *nh)
{
	Baxter_traj_streamer  b(nh);
	nh_ = nh;
}

void interestingMoves::populateMove1(trajectory_msgs::JointTrajectory &des_trajectory)
{
	// flex1
	Baxter_traj_streamer  b(nh_);
	for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
	Vectorq7x1 q_pre_pose;
	Eigen::VectorXd q_in_vecxd;
	q_pre_pose<< 1.2, 0.6,   2.5,   1.2,    0.5,   1.5,  0;
	std::vector<Eigen::VectorXd> des_path;
	Vectorq7x1 q_vec_right_arm;
	q_vec_right_arm =  b.get_qvec_right_arm();
	q_in_vecxd = q_vec_right_arm;
	des_path.push_back(q_in_vecxd);
	q_in_vecxd = q_pre_pose;
	des_path.push_back(q_in_vecxd);
	b.stuff_trajectory(des_path, des_trajectory);
}
void interestingMoves::populateMove2(trajectory_msgs::JointTrajectory &des_trajectory)
{
	// flex 2
	Baxter_traj_streamer  b(nh_);
	for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
	Vectorq7x1 q_pre_pose;
	Eigen::VectorXd q_in_vecxd;
	q_pre_pose<< -0.5, -0.2,   3.14,   1.5,    -0.4,   1.5,  0;
	std::vector<Eigen::VectorXd> des_path;
	Vectorq7x1 q_vec_right_arm;
	q_vec_right_arm =  b.get_qvec_right_arm();
	q_in_vecxd = q_vec_right_arm;
	des_path.push_back(q_in_vecxd);
	q_in_vecxd = q_pre_pose;
	des_path.push_back(q_in_vecxd);
	b.stuff_trajectory(des_path, des_trajectory);
}
void interestingMoves::populateMove3(trajectory_msgs::JointTrajectory &des_trajectory)
{
	// flex 3
	Baxter_traj_streamer  b(nh_);
	for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
	Vectorq7x1 q_pre_pose;
	Eigen::VectorXd q_in_vecxd;
	q_pre_pose<< -0.8, -0.8,   0,   0,    0,   0,  0;
	std::vector<Eigen::VectorXd> des_path;
	Vectorq7x1 q_vec_right_arm;
	q_vec_right_arm =  b.get_qvec_right_arm();
	q_in_vecxd = q_vec_right_arm;
	des_path.push_back(q_in_vecxd);
	q_in_vecxd = q_pre_pose;
	des_path.push_back(q_in_vecxd);
	b.stuff_trajectory(des_path, des_trajectory);	
}
