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

//ros::Publisher pose_pub;
int i=0;

void printModelNames(const gazebo_msgs::ModelStatesPtr &model_states) {
    /*std::vector<geometry_msgs::Pose> model_poses = model_states->pose;
    //for(geometry_msgs::Pose model_pose: model_poses) {	
    //for(std::string model_name = model_names.begin(); model_name != model_names.end(); model_name++) {
		std::cout<<"Model Name: "<<model_name<<std::endl;	
	}*/
	std::vector<std::string> model_names = model_states->name;
	for(std::string model_name: model_names) {
		std::cout<<"Model Name: "<<model_name<<std::endl;		
	}		
	i++;
	/*std_msgs::UInt8 brightness_value;
   	brightness_value.data = i;
   	pose_pub.publish(brightness_value);*/
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kiva_rotate");
    ros::NodeHandle n("~");
    //ros::Subscriber sub = n.subscribe("/gazebo/model_states", 10, printModelNames);
    //pose_pub = n.advertise<std_msgs::UInt8>("brightness", 1);
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    //ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");


	/*geometry_msgs::Pose start_pose;
	start_pose.position.x = 0.0;
	start_pose.position.y = 0.0;
	start_pose.position.z = 2.0;
	start_pose.orientation.x = 0.0;
	start_pose.orientation.y = 0.0;
	start_pose.orientation.z = 0.0;
	start_pose.orientation.w = 0.0;
	
	geometry_msgs::Twist start_twist;
	start_twist.linear.x = 0.0;
	start_twist.linear.y = 0.0;
	start_twist.linear.z = 0.0;
	start_twist.angular.x = 0.0;
	start_twist.angular.y = 0.0;
	start_twist.angular.z = 0.0;

	gazebo_msgs::ModelState modelstate;
	modelstate.model_name = (std::string) "kiva_2";
	modelstate.reference_frame = (std::string) "world";
	modelstate.pose = start_pose;
	modelstate.twist = start_twist;

	gazebo_msgs::SetModelState set_model_state;
	set_model_state.request.model_state = modelstate;*/

	/*pose_pub.publish(set_model_state); */

/*if(client.call(set_model_state))
    {
        ROS_INFO("Robot Moved");
    }
    else
    {
        ROS_ERROR("Failed to mode! Error msg:%s", (std::string) set_model_state.response.status_message);
    } */



    ros::Rate loop_rate(10);
	int count = 0;
    while (ros::ok())
    {
	std_msgs::String msg;
	std::stringstream ss;

	ss<< " { \"x\": 5, \"y\":5 }";
	msg.data = ss.str();
	ROS_INFO("%s", msg.data.c_str());

	chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
	++count;
    }
}

