#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <gazebo_msgs/SpawnModel.h>
#include <warehouse_functions/create_warehouse_msg_request.h>
#include <iostream>
#include <fstream>
#include <sstream>

#define ROBOT_MODEL_SDF "/home/eeshakumar/SoSe19/robot-se/models/models/kiva/model.sdf"
#define STORAGE_UNIT_MODEL_SDF "/home/eeshakumar/SoSe19/robot-se/models/models/storage_unit/model.sdf"
#define MODEL_ITEM_SDF "/home/eeshakumar/SoSe19/robot-se/models/models/item_box/model.sdf"

#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <gazebo_msgs/SpawnModel.h>
#include <warehouse_functions/create_warehouse_msg_request.h>
#include <iostream>
#include <fstream>
#include <sstream>

/*
This node subscribes to topic to pick up only create warehouse requests.
The create request is transformed to spawn models.
The spawn objects are published on gazebo_msgs/SpawnModel.
*/

void spawn_model(ros::NodeHandle n, std::string model_name, std::string model_sdf, int pos_x, int pos_y, int pos_z=0) {
	//ros::NodeHandle n;
	std::ifstream ifs(model_sdf);
	ROS_INFO("Init complete");	
	ROS_INFO("Calling Service");
	//ros::Rate loop_rate(1000);
	//loop_rate.sleep();
	ros::ServiceClient client = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
	gazebo_msgs::SpawnModel robot;
	robot.request.model_name = model_name;
	std::string model_xml;
	model_xml.assign( (std::istreambuf_iterator<char>(ifs) ),
                (std::istreambuf_iterator<char>()    ) );
	robot.request.model_xml = model_xml;
	//ROS_INFO("%c",robot.request.model_xml);
        geometry_msgs::Pose pose;
        pose.position.x = pos_x;
        pose.position.y = pos_y;
        pose.position.z = pos_z;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 0;
	robot.request.initial_pose = pose;
	robot.request.reference_frame = "";

	if (!client.waitForExistence(ros::Duration(2.0)))
	{
		ROS_FATAL_STREAM("Unable to locate service '" << client.getService() << "'");
	}

	if (client.call(robot)) {
		ROS_INFO("Successful!!");
	} else {
		ROS_ERROR("Failed");
		//ROS_ERROR(client.response.c_str());	
	}
}

void createWarehouseCallback(warehouse_functions::create_warehouse_msg_request create_request) {
	ros::NodeHandle n;
	int no_of_robots = create_request.no_of_robots;
	int no_of_storage_units = create_request.no_of_storage_units;
	int no_of_items = create_request.no_of_items;
	std::string warehouse_name = create_request.warehouse_name;
	
	int pos_x = 0, pos_y = 0;
	while(no_of_robots) {
		ROS_INFO("Spawning kiva_%d", no_of_robots);
		std::string model_name = "kiva_" + std::to_string(no_of_robots);
		spawn_model(n, model_name, ROBOT_MODEL_SDF, pos_x, pos_y);	
		no_of_robots--;
		pos_y++;
	}
	pos_x = 0, pos_y = 0;
	bool alt = true;
	while(no_of_storage_units) {
		ROS_INFO("Spawning storage_unit_%d", no_of_storage_units);
		std::string model_name = "storage_unit_" + std::to_string(no_of_storage_units);
		spawn_model(n, model_name, STORAGE_UNIT_MODEL_SDF, pos_x, pos_y);	
		no_of_storage_units--;
		if (alt) 
			pos_x++;		
		else
			pos_y++;
		alt=!alt;
	}
	pos_x = 0, pos_y = 0;
	int pos_z=1;
	alt = true;
	while(no_of_items) {
		ROS_INFO("Spawning item_box_%d", no_of_items);
		std::string model_name = "item_box_" + std::to_string(no_of_items);
		spawn_model(n, model_name, MODEL_ITEM_SDF, pos_x, pos_y, pos_z);	
		no_of_items--;
		if (alt) 
			pos_x++;		
		else
			pos_y++;
		alt=!alt;
	}
	ROS_INFO("%s", warehouse_name.c_str());
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "create_warehouse_service_node");
	ros::NodeHandle n;
	ros::Subscriber create_warehouse_sub = n.subscribe("/warehouses/create_warehouse_models", 10, createWarehouseCallback);

	ros::spin();
	return 0; 	
}
