#include <ros/ros.h>
#include <warehouse_functions/create_warehouse_msg_request.h>
#include <unistd.h>


/*
Sould create be a service call instead? 
*/

int main(int argc, char **argv) {
	ros::init(argc, argv, "create_topics");
	ros::NodeHandle n("~");
	ros::Publisher create_warehouse_pub = n.advertise<warehouse_functions::create_warehouse_msg_request>("/warehouses/create_warehouse_models", 10);
	ros::Rate loop_rate(1000);
	//ros::spinOnce();
	ROS_INFO("Sleep to init publisher");
	loop_rate.sleep();
	//bool is_first_message = true;
	//int connections = create_warehouse_pub.getNumConnections();	
	while(ros::ok()) {
		//connections = create_warehouse_pub.getNumConnections();
		//if (is_first_messaage && connections != 0) {
			warehouse_functions::create_warehouse_msg_request msg;

			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = "/world";
			msg.warehouse_name = "warehouse_new";
			msg.warehouse_id = "wh123";
			msg.no_of_robots = 1;
			msg.no_of_storage_units = 2;
			msg.no_of_items = 2;

			ROS_INFO("Publishing message ");
			create_warehouse_pub.publish(msg);
		//	is_first_message = false;
		//}
		ros::spinOnce();
		ros::Duration(5).sleep();
		loop_rate.sleep();	
	}
}
