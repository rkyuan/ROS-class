// example_action_server: 2nd version, includes "cancel" and "feedback"
// expects client to give an integer corresponding to a timer count, in seconds
// server counts up to this value, provides feedback, and can be cancelled any time
// re-use the existing action message, although not all fields are needed
// use request "input" field for timer setting input, 
// value of "fdbk" will be set to the current time (count-down value)
// "output" field will contain the final value when the server completes the goal request

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../example_action_server/action/demo.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (demo) and appended name (Action)
#include <p4_hxy153/robotPathAction.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdlib.h>

class PathActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<p4_hxy153::robotPathAction> as_;
    
    // here are some message types to communicate with our client(s)
    p4_hxy153::robotPathGoal goal_; // goal message, received from client
    p4_hxy153::robotPathResult result_; // put results here, to be sent back to the client when done w/ goal
    p4_hxy153::robotPathFeedback feedback_; // for feedback 
    //  use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    //int countdown_val_;
    

	float dangle(geometry_msgs::Pose,float x, float y, float angle);  
	float dlin(geometry_msgs::Pose, float x, float y);  


public:
    PathActionServer(); //define the body of the constructor outside of class definition

    ~PathActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<p4_hxy153::robotPathAction>::GoalConstPtr& goal);
};

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, 
// which is a member method of our class exampleActionServer.  
// Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB function takes one argument
// The final argument,  "false", says don't start the server yet.  (We'll do this in the constructor)

PathActionServer::PathActionServer() :
   as_(nh_, "path_follower", boost::bind(&PathActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of ActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <example_action_server::demoAction> customizes the simple action server to use our own "action" message 
// defined in our package, "example_action_server", in the subdirectory "action", called "demo.action"
// The name "demo" is prepended to other message types created automatically during compilation.
// e.g.,  "demoAction" is auto-generated from (our) base name "demo" and generic name "Action"
void PathActionServer::executeCB(const actionlib::SimpleActionServer<p4_hxy153::robotPathAction>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB");

    float angle;
    float x;
    float y;
    float dt;

    angle = 0;
    x = 0;
    y = 0;

    dt = 0.05;
    double speed = 1.0;
    double yaw_rate = 0.5;
	ros::Rate loop_timer(1/dt);
    ros::Publisher commander = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    

    for (int i = 0; i < goal->input.poses.size();i++){
    //first turn towards goal
      geometry_msgs::Twist command;
      geometry_msgs::Twist stop;
      ROS_INFO("turning");
      float turn = dangle(goal->input.poses[i].pose,x,y,angle);
      ROS_INFO("current pos: %f, %f",x,y);
      ROS_INFO("goal pos: %f, %f",goal->input.poses[i].pose.position.x,goal->input.poses[i].pose.position.y );
      ROS_INFO("turn angle: %f",turn);
      int yaw_mult =1;
      if (turn>0) yaw_mult =  1;
      if (turn<0) yaw_mult = -1;
      command.angular.z = yaw_rate*yaw_mult;


      double timer = 0.0;
      while(timer<std::abs(turn)/yaw_rate) {
          commander.publish(command);
          timer+=dt;
          
          loop_timer.sleep();

          
          }
      angle += turn;
      	if (angle>3.14159) angle -= 2*3.14159;
		if (angle<-3.14159) angle += 2*3.14159;

      //then advance
      ROS_INFO("moving");
      command.angular.z = 0;
      command.linear.x = speed;
      timer = 0.0;
      float dist = dlin(goal->input.poses[i].pose,x,y);
      while(timer<dist/speed) {
	      commander.publish(command);
	      timer+=dt;
	      loop_timer.sleep();

	      if (as_.isPreemptRequested()){	
	      ROS_WARN("goal cancelled!");
	      commander.publish(stop);
	      //turn around
	      // command.angular.z = yaw_rate;
   	 		// command.linear.x = 0;
     		// timer = 0.0;
     		// while (timer<3.14159/yaw_rate){
     		// 	commander.publish(command);
	      // 		timer+=dt;
	      // 		loop_timer.sleep();
     		// }
	      as_.setAborted(result_); 
	      return; 
		  }
      }
      x = goal->input.poses[i].pose.position.x;
      y = goal->input.poses[i].pose.position.y;

    }
    geometry_msgs::Twist command;
    command.linear.x = 0;
    commander.publish(command);
    ROS_INFO("done");
    result_.output=0;
    as_.setSucceeded(result_);
    



   //  //ROS_INFO("goal input is: %d", goal->input);
   //  //do work here: this is where your interesting code goes
   //  ros::Rate timer(1.0); // 1Hz timer
   //  countdown_val_ = goal->input;
   //  //implement a simple timer, which counts down from provided countdown_val to 0, in seconds
   //  while (countdown_val_>0) {
   //     ROS_INFO("countdown = %d",countdown_val_);
       
   //     // each iteration, check if cancellation has been ordered
   //     if (as_.isPreemptRequested()){	
   //        ROS_WARN("goal cancelled!");
   //        result_.output = countdown_val_;
   //        as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
   //        return; // done with callback
 		// }
 	
 	 //   //if here, then goal is still valid; provide some feedback
 	 //   feedback_.fdbk = countdown_val_; // populate feedback message with current countdown value
 	 //   as_.publishFeedback(feedback_); // send feedback to the action client that requested this goal
   //     countdown_val_--; //decrement the timer countdown
   //     timer.sleep(); //wait 1 sec between loop iterations of this timer
   //  }
   //  //if we survive to here, then the goal was successfully accomplished; inform the client
   //  result_.output = countdown_val_; //value should be zero, if completed countdown
   //  as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_action_server_node"); // name this node 

    ROS_INFO("instantiating the timer_action_server: ");

    PathActionServer as_object; // create an instance of the class "ExampleActionServer"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    ros::spin();

    return 0;
}

float PathActionServer::dangle(geometry_msgs::Pose p,float x, float y, float angle){

	float dy = p.position.y - y;
	float dx = p.position.x - x;
	float goal_angle = atan2(dy,dx);
	//take min angle;
	float angleDiff = goal_angle - angle;
	ROS_INFO("angleDiff: %f", angleDiff);
	if (angleDiff>3.14159) angleDiff -= 2*3.14159;
	if (angleDiff<-3.14159) angleDiff += 2*3.14159;
	//angle = goal_angle;
	return angleDiff;
}


float PathActionServer::dlin(geometry_msgs::Pose p, float x, float y){
	float dx = p.position.x - x;
	float dy = p.position.y - y;

	float dist = sqrt(pow(dx,2)+pow(dy,2));
	return dist;
}
