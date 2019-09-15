#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>

#include <sstream>

/*
This node publishes on the gazebo topic to move robots to x and y.
*/
int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_move_publisher");
	ros::NodeHandle n("~");
	ros::Publisher 	pub = n.advertise<geometry_msgs::Vector3>("/kiva/mov", 10);
	ros::Rate loop_rate(10);
	while(ros::ok()) {
		std_msgs::String msg;
		std::stringstream ss;
		ss << "3 3 1";
		msg.data = ss.str();

		geometry_msgs::Vector3 move_cmd; //= new ignition::math::Vector3d(3, 3, 1);
		move_cmd.x = 3;
		move_cmd.y = 3;
		move_cmd.z = 1;
		
		pub.publish(move_cmd);
		ROS_INFO("Published on /kiva/mov %s", ss.str().c_str());
		ros::spinOnce();
		loop_rate.sleep();
		ros::Duration(10).sleep();
	} 
}

