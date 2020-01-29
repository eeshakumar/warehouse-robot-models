#include <ros/ros.h>
//#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
//#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/Twist.h>
//#include <gazebo_msgs/SetModelState.h>
//#include <gazebo/gazebo.hh>
//#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <std_msgs/String.h>

/*
process final position x, final position y, and robot id 
*/

void move_robot_sub()

int main(int argc, char ** argv) {
	ros::init(argc, argv, "move_robot");
	ros::NodeHandle n("~");
	//Subscribe to APIs instructions -/api/move_to needs to be created
	ros::Subscriber sub = n.subscribe("/api/move_to", 10, publish_to_gazebo);
	
	ros::Rate loop_rate(10);
	while(ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();	
	}
	
}

