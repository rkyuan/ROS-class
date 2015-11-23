#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <math.h>
#include <gripper_control.h>

gripper_control::gripper_control(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    gripPub_ = nh_.advertise<std_msgs::Int16>("dynamixel_motor1_cmd", 1);
}

void gripper_control::open(){
    std_msgs::Int16 int_angle;
    int_angle.data = 3100;
    gripPub_.publish(int_angle);
}
void gripper_control::open(){
    std_msgs::Int16 int_angle;
    int_angle.data = 3700;
    gripPub_.publish(int_angle);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gripper_control"); 
    ros::NodeHandle n; 


}


//     //std::cout<<"enter motor_id to test: ";
//     int motor_id=1; // hard coded for Yale hand
//     //std::cin>>motor_id;
//     char cmd_topic_name[50];
//     sprintf(cmd_topic_name,"dynamixel_motor%d_cmd",motor_id);
//     ROS_INFO("using command topic: %s",cmd_topic_name);

//     ros::Publisher dyn_pub = n.advertise<std_msgs::Int16>(cmd_topic_name, 1);
    
//     std_msgs::Int16 int_angle; 
//    double dt = 0.02; // repeat at freq 1/dt
//    ros::Rate naptime(1/dt); //create a ros object from the ros “Rate” class; 

//     int_angle.data = 0.0;

//     double mid_angle = 3500;
//     double amp = 500;
//     double omega = 0.1*2*M_PI; // in Hz 
//     double phase = 0;
//     double dbl_ang=0.0;
//     short int int_ang=0;
    
    
//     // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
//     while (ros::ok()) 
//     {
//         phase += omega*dt;
//         dbl_ang = amp*sin(phase)+mid_angle;
// 	int_ang = (short int) dbl_ang;
//         int_angle.data = int_ang;
//         ROS_INFO("sending: %d",int_ang);
//         dyn_pub.publish(int_angle); // publish the value--of type Float64-- 
// 	naptime.sleep(); 
//     }
// }

