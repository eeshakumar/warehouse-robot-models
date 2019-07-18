#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetModelState.h>
#include <warehouse_functions/track_items_pub_msg.h>

ros::ServiceClient client;
ros::Publisher pub;

void getAllModelsCallBack(const gazebo_msgs::ModelStatesPtr &model_states) {
	std::vector<std::string> model_names = model_states->name;
	std::vector<std::string> items;
	for(std::string model_name: model_names) {
		if(model_name.find("item_") == 0) {
			items.push_back(model_name);
			ROS_INFO("Found items %s", model_name.c_str());		
		}		
	}

	ros::NodeHandle n;
	for (std::string item: items) {
		gazebo_msgs::GetModelState getModelState;
		//gazebo_msgs::GetModelState::Response response;
		getModelState.request.model_name = item;
		if (!client.waitForExistence(ros::Duration(2.0)))
			{
				ROS_FATAL_STREAM("Unable to locate service '" << client.getService() << "'");
			}

			if (client.call(getModelState)) {
				ros::Rate loop_rate(1000);
				ROS_INFO("Response %s", getModelState.response.status_message.c_str());
				pub = n.advertise<warehouse_functions::track_items_pub_msg>("/items/" + item + "/pose", 10);
				geometry_msgs::Pose pose = getModelState.response.pose;
				warehouse_functions::track_items_pub_msg msg;
				msg.x = pose.position.x;
				msg.y = pose.position.y;
				msg.z = pose.position.z;
				msg.roll = pose.orientation.x;
				msg.pitch = pose.orientation.y;
				msg.yaw = pose.orientation.z;
				
				pub.publish(msg);
				ros::spinOnce();
				loop_rate.sleep();	
			} else {
				ROS_ERROR("Failed");
				//ROS_ERROR(client.response.c_str());	
			}	
	}
		

	
	
	/*for()
		{
		     ROS_INFO("Models %s\n", i->c_str());
		}
	*/
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "track_items_node");
	ros::NodeHandle n("~");
	
	ros::Subscriber sub = n.subscribe("/gazebo/model_states", 10, getAllModelsCallBack);
	client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	ros::spin();
	ros::Duration(5).sleep();
}
