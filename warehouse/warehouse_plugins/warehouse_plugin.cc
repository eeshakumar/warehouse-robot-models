#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"

namespace gazebo {
	class WarehousePlugin: public WorldPlugin {
 		public: WarehousePlugin(): WorldPlugin() {}
		public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
			this->world = _world;
			this->sdf = _sdf;
			int no_of_shelves = 0;
			if (sdf->HasElement("shelves")) {
				no_of_shelves = this->sdf->Get<int>("shelves");			
			}
			std::cout<<"Initializing world for warehouse"<<"\n";
			//Insert cannot be used with a plugin. Why?
			//world->InsertModelFile("model://./models/shelf");
			//world->InsertModelFile("model://./models/kiva");
			this->init_world("model://./models/shelf", ignition::math::Vector3d(0, 0, 0), "1");
			//this->init_world("model://./models/shelf", ignition::math::Vector3d(2, 2, 0), "2");
		}
		private: void init_world(std::string model_path, ignition::math::Vector3d pos, std::string model_no) {
		      //std::cout<<"Adding Shelves to the world\n";
		      // Create a new transport node
	      	      transport::NodePtr node(new transport::Node());

		      // Initialize the node with the world name
		      node->Init(this->world->Name());

		      // Create a publisher on the ~/factory topic
		      transport::PublisherPtr factoryPub =
		      	node->Advertise<msgs::Factory>("~/shelves");

		      // Create the message
		      msgs::Factory msg;

		      // Model file to load
		      msg.set_sdf_filename(model_path);

		      // Pose to initialize the model to
		      msgs::Set(msg.mutable_pose(),
	    		    ignition::math::Pose3d(
			    pos,
			    ignition::math::Quaterniond(0, 0, 0)));

		      // Send the message
		      factoryPub->Publish(msg);	
		      std::cout<<"Added Shelf to the world\n";	
		}
		private: physics::WorldPtr world;
		private: sdf::ElementPtr sdf;	
	};
	GZ_REGISTER_WORLD_PLUGIN(WarehousePlugin)
}
