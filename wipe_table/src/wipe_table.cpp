// example_baxter_cart_move_action_client: 
// wsn, Nov, 2015
// illustrates use of baxter_cart_move_as, action server called "cartMoveActionServer"

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cwru_action/cwru_baxter_cart_moveAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>  // transform listener headers
#include <tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>  //point-cloud library headers; likely don't need all these
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
//#include <tf/transform_listener.h>

#include <cwru_pcl_utils/cwru_pcl_utils.h>
//#include <tf2_sensor_msgs.h>

bool g_start;
Eigen::Vector3f g_centroid;

//define a class to encapsulate some of the tedium of populating and sending goals,
// and interpreting responses
class ArmMotionCommander {
private:
    ros::NodeHandle nh_;

    //messages to send/receive cartesian goals / results:
    cwru_action::cwru_baxter_cart_moveGoal cart_goal_;
    cwru_action::cwru_baxter_cart_moveResult cart_result_;    
    std::vector <double> q_vec_; //holder for right-arm angles
    geometry_msgs::PoseStamped tool_pose_stamped_;
    //an action client to send goals to cartesian-move action server
    actionlib::SimpleActionClient<cwru_action::cwru_baxter_cart_moveAction> cart_move_action_client_; //("cartMoveActionServer", true);
    double computed_arrival_time_;
    bool finished_before_timeout_;
    //callback fnc for cartesian action server to return result to this node:
    void doneCb_(const actionlib::SimpleClientGoalState& state,
    const cwru_action::cwru_baxter_cart_moveResultConstPtr& result);
public:
        ArmMotionCommander(ros::NodeHandle* nodehandle); //define the body of the constructor outside of class definition

    ~ArmMotionCommander(void) {
    }
    void send_test_goal(void);
    int plan_move_to_pre_pose(void);
    int rt_arm_execute_planned_path(void);
    int rt_arm_request_q_data(void);
    int rt_arm_request_tool_pose_wrt_torso(void);
    geometry_msgs::PoseStamped get_rt_tool_pose_stamped(void) { return tool_pose_stamped_;};
    
    Eigen::VectorXd get_right_arm_joint_angles(void); 
    int rt_arm_plan_jspace_path_current_to_qgoal(Eigen::VectorXd q_des_vec);  
    int rt_arm_plan_path_current_to_goal_pose(geometry_msgs::PoseStamped des_pose);
    int rt_arm_plan_path_current_to_goal_dp_xyz(Eigen::Vector3d dp_displacement);

    //utilities to convert between affine and pose
    Eigen::Affine3d transformPoseToEigenAffine3d(geometry_msgs::Pose pose); 
    geometry_msgs::Pose transformEigenAffine3dToPose(Eigen::Affine3d e);

};

ArmMotionCommander::ArmMotionCommander(ros::NodeHandle* nodehandle): nh_(*nodehandle),
cart_move_action_client_("cartMoveActionServer", true) { // constructor
    ROS_INFO("in constructor of ArmMotionInterface");

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = cart_move_action_client_.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to action server"); // if here, then we connected to the server; 
    
}
// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
//int g_return_code=0;
void ArmMotionCommander::doneCb_(const actionlib::SimpleClientGoalState& state,
        const cwru_action::cwru_baxter_cart_moveResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return value= %d", result->return_code);
    cart_result_=*result;
}

Eigen::Affine3d ArmMotionCommander::transformPoseToEigenAffine3d(geometry_msgs::Pose pose) {
    Eigen::Affine3d affine;

    Eigen::Vector3d Oe;

    Oe(0) = pose.position.x;
    Oe(1) = pose.position.y;
    Oe(2) = pose.position.z;
    affine.translation() = Oe;

    Eigen::Quaterniond q;
    q.x() = pose.orientation.x;
    q.y() = pose.orientation.y;
    q.z() = pose.orientation.z;
    q.w() = pose.orientation.w;
    Eigen::Matrix3d Re(q);

    affine.linear() = Re;

    return affine;
}

geometry_msgs::Pose ArmMotionCommander::transformEigenAffine3dToPose(Eigen::Affine3d e) {
    Eigen::Vector3d Oe;
    Eigen::Matrix3d Re;
    geometry_msgs::Pose pose;
    Oe = e.translation();
    Re = e.linear();

    Eigen::Quaterniond q(Re); // convert rotation matrix Re to a quaternion, q
    pose.position.x = Oe(0);
    pose.position.y = Oe(1);
    pose.position.z = Oe(2);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}


void ArmMotionCommander::send_test_goal(void) {
    ROS_INFO("sending a test goal");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::ARM_TEST_MODE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired

    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
    if (!finished_before_timeout_) {
            ROS_WARN("giving up waiting on result");
        } else {
            ROS_INFO("finished before timeout");
            ROS_INFO("return code: %d",cart_result_.return_code);
        }        
}

int ArmMotionCommander::plan_move_to_pre_pose(void) {
    ROS_INFO("requesting a joint-space motion plan");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_JSPACE_PATH_CURRENT_TO_PRE_POSE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
            ROS_WARN("giving up waiting on result");
            return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;
}

int ArmMotionCommander::rt_arm_plan_jspace_path_current_to_qgoal(Eigen::VectorXd q_des_vec) {    
    ROS_INFO("requesting a joint-space motion plan");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_JSPACE_PATH_CURRENT_TO_QGOAL;
    cart_goal_.q_goal_right.resize(7);
    for (int i=0;i<7;i++) cart_goal_.q_goal_right[i] = q_des_vec[i]; //specify the goal js pose
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
            ROS_WARN("giving up waiting on result");
            return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;    
    
}

int ArmMotionCommander::rt_arm_plan_path_current_to_goal_pose(geometry_msgs::PoseStamped des_pose) {
    
    ROS_INFO("requesting a cartesian-space motion plan");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_PATH_CURRENT_TO_GOAL_POSE;
    cart_goal_.des_pose_gripper_right = des_pose;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
            ROS_WARN("giving up waiting on result");
            return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;        
}

int ArmMotionCommander::rt_arm_plan_path_current_to_goal_dp_xyz(Eigen::Vector3d dp_displacement) {
    
    ROS_INFO("requesting a cartesian-space motion plan along vector");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_PATH_CURRENT_TO_GOAL_DP_XYZ;
    //must fill in desired vector displacement
    cart_goal_.arm_dp_right.resize(3);
    for (int i=0;i<3;i++) cart_goal_.arm_dp_right[i] = dp_displacement[i];
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
            ROS_WARN("giving up waiting on result");
            return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;      
}
    

int ArmMotionCommander::rt_arm_execute_planned_path(void) {
    ROS_INFO("requesting execution of planned path");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_EXECUTE_PLANNED_PATH;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(computed_arrival_time_+2.0));
    if (!finished_before_timeout_) {
        ROS_WARN("did not complete move in expected time");
        return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;  
    }
    if (cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d",cart_result_.return_code);
        return (int) cart_result_.return_code;
    }

    ROS_INFO("move returned success");
    return (int) cart_result_.return_code;
}

//send goal command to request right-arm joint angles; these will be stored in internal variable
int ArmMotionCommander::rt_arm_request_q_data(void) {
   ROS_INFO("requesting right-arm joint angles");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_GET_Q_DATA;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(computed_arrival_time_+2.0));
   if (!finished_before_timeout_) {
        ROS_WARN("did not respond within timeout");
        return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;  
    }
    if (cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d",cart_result_.return_code);
        return (int) cart_result_.return_code;
    }
    
    q_vec_ = cart_result_.q_arm_right;
    ROS_INFO("move returned success; right arm angles: ");
    ROS_INFO("%f; %f; %f; %f; %f; %f; %f",q_vec_[0],q_vec_[1],q_vec_[2],q_vec_[3],q_vec_[4],q_vec_[5],q_vec_[6]);
    return (int) cart_result_.return_code;
}

Eigen::VectorXd ArmMotionCommander::get_right_arm_joint_angles(void) {
    rt_arm_request_q_data();
    Eigen::VectorXd rt_arm_angs_vecXd;
    rt_arm_angs_vecXd.resize(7);
    for (int i=0;i<7;i++) {
        rt_arm_angs_vecXd[i] = q_vec_[i];
    }
    return rt_arm_angs_vecXd;
}

int ArmMotionCommander::rt_arm_request_tool_pose_wrt_torso(void) {
    // debug: compare this to output of:
    //rosrun tf tf_echo torso yale_gripper_frame
    ROS_INFO("requesting right-arm tool pose");    
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_GET_TOOL_POSE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
   if (!finished_before_timeout_) {
        ROS_WARN("did not respond within timeout");
        return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;  
    }
    if (cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d",cart_result_.return_code);
        return (int) cart_result_.return_code;
    }    
    
        tool_pose_stamped_ = cart_result_.current_pose_gripper_right;
        ROS_INFO("move returned success; right arm tool pose: ");
        ROS_INFO("origin w/rt torso = %f, %f, %f ",tool_pose_stamped_.pose.position.x,
                tool_pose_stamped_.pose.position.y,tool_pose_stamped_.pose.position.z);
        ROS_INFO("quaternion x,y,z,w: %f, %f, %f, %f",tool_pose_stamped_.pose.orientation.x,
                tool_pose_stamped_.pose.orientation.y,tool_pose_stamped_.pose.orientation.z,
                tool_pose_stamped_.pose.orientation.w);
  return (int) cart_result_.return_code;
}

void selectPointCloudCB(const sensor_msgs::PointCloud2 & message_holder){
    // geometry_msgs::TransformStamped transform;
    // transform.transform.rotation.x = -0.5;
    // transform.transform.rotation.x = 0.5;
    // transform.transform.rotation.x = -0.5;
    // transform.transform.rotation.x = 0.5;
    // sensor_msgs::PointCloud2 copy;
    // tf2::doTransform (message_holder,copy, transform);
    // pcl::PointCloud<pcl::PointXYZ> sample;
    // pcl::fromROSMsg(message_holder,sample);

    // Eigen::Vector3f centroid;

    // for (int i = 0; i < sample.points.size(); i++){
    //     centroid[0]+=sample.points[i].x;
    //     centroid[1]+=sample.points[i].y;
    //     centroid[2]+=sample.points[i].z;
    // }

    // centroid /= sample.points.size();
    // //centroid[2]= centroid[2]*-1 +1.5;
    // g_centroid = centroid;
    g_start = true;

    // ROS_INFO("centroid x: %f, y: %f, z: %f",centroid[0],centroid[1],centroid[2]);
    
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "wipe_table"); // name this node 
    ros::NodeHandle nh; //standard ros node handle     
    ArmMotionCommander arm_motion_commander(&nh);
    Eigen::VectorXd right_arm_joint_angles;
    Eigen::Vector3d dp_displacement;
    int rtn_val;
    geometry_msgs::PoseStamped rt_tool_pose;

    CwruPclUtils helper(&nh);

    ros::Subscriber my_subscriber_object= nh.subscribe("selected_points",1,selectPointCloudCB);
    g_start = false;

    
    arm_motion_commander.send_test_goal(); // send a test command
    
    //send a command to plan a joint-space move to pre-defined pose:
    rtn_val=arm_motion_commander.plan_move_to_pre_pose();
    
    //send command to execute planned motion
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    
    //inquire re/ right-arm joint angles:
    rtn_val=arm_motion_commander.rt_arm_request_q_data();
    
    //inquire re/ right-arm tool pose w/rt torso:    
    rtn_val=arm_motion_commander.rt_arm_request_tool_pose_wrt_torso();
    
    //do a joint-space move; get the start angles:
    right_arm_joint_angles = arm_motion_commander.get_right_arm_joint_angles();
    
    // //increment all of the joint angles by 0.1:
    // for (int i=0;i<7;i++) right_arm_joint_angles[i]+=0.2;
    
    // //try planning a joint-space motion to this new joint-space pose:
    // rtn_val=arm_motion_commander.rt_arm_plan_jspace_path_current_to_qgoal(right_arm_joint_angles);

    // //send command to execute planned motion
    // rtn_val=arm_motion_commander.rt_arm_execute_planned_path();   
    
    // //let's see where we ended up...should match goal request
    // rtn_val=arm_motion_commander.rt_arm_request_q_data();
    
    // //return to pre-defined pose:
    // rtn_val=arm_motion_commander.plan_move_to_pre_pose();
    // rtn_val=arm_motion_commander.rt_arm_execute_planned_path();  

    ROS_INFO("waiting for selection__________");
    while (g_start ==false){
        ros::spinOnce();
        //ros::Duration(0.5).sleep();
        //ROS_INFO("/b",g_start);
    }
    for (int times = 0; times < 100; times++){
        ros::spinOnce();
        ros::Duration(0.01).sleep();

    } 
    sensor_msgs::PointCloud2 message_holder;
    

    pcl::PointCloud<pcl::PointXYZ> sample;
    //pcl::fromROSMsg(message_holder,sample);




    tf::StampedTransform tf_sensor_frame_to_torso_frame; //use this to transform sensor frame to torso frame
    tf::TransformListener tf_listener; //start a transform listener

    //let's warm up the tf_listener, to make sure it get's all the transforms it needs to avoid crashing:
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and torso...");
    while (tferr) {
        tferr = false;
        try {

            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tf_listener.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good"); //tf-listener found a complete chain from sensor to world; ready to roll
    //convert the tf to an Eigen::Affine:
    Eigen::Affine3f A_sensor_wrt_torso;
    A_sensor_wrt_torso = helper.transformTFToEigen(tf_sensor_frame_to_torso_frame);


    helper.transform_selected_points_cloud(A_sensor_wrt_torso);



    helper.get_transformed_selected_points(sample);

    Eigen::Vector3f centroid;

    for (int i = 0; i < sample.points.size(); i++){
        centroid[0]+=sample.points[i].x;
        centroid[1]+=sample.points[i].y;
        centroid[2]+=sample.points[i].z;
    }

    centroid /= sample.points.size();
    //centroid[2]= centroid[2]*-1 +1.5;
    g_centroid = centroid;
    g_start = true;

    ROS_INFO("centroid x: %f, y: %f, z: %f",centroid[0],centroid[1],centroid[2]);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //get tool pose
    rtn_val = arm_motion_commander.rt_arm_request_tool_pose_wrt_torso();
    rt_tool_pose = arm_motion_commander.get_rt_tool_pose_stamped();

    //alter the tool pose:
    rt_tool_pose.pose.position.x = g_centroid[0];
    rt_tool_pose.pose.position.y = g_centroid[1];
    rt_tool_pose.pose.position.z = g_centroid[2]; // descend 20cm, along z in torso frame
    // send move plan request:
    rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
    //send command to execute planned motion
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();

    rt_tool_pose = arm_motion_commander.get_rt_tool_pose_stamped();
    dp_displacement<<0,-0.05,0;
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_dp_xyz(dp_displacement);
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    for (int times = 0; times < 5; times++){
        dp_displacement<<0,0.1,0;
        rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_dp_xyz(dp_displacement);
        rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
        dp_displacement<<0,-0.1,0;
        rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_dp_xyz(dp_displacement);
        rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    }
    
    //try vector cartesian displacement at fixed orientation:
    // dp_displacement<<0,0,-0.25;
    // rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_dp_xyz(dp_displacement);
    // if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS)  { 
    //         //send command to execute planned motion
    //        rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    // }
    return 0;
}

