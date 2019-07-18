#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

namespace gazebo
{

	class WarehouseWorldPlugin : public WorldPlugin {
		  public:WarehouseWorldPlugin() : WorldPlugin()
	  	  {
	  	  }

		  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
		  {
		    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
		    if (!ros::isInitialized())
		    {
		      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
			<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
		      return;
		    }

		    ROS_INFO("PLUGIN_LOAD_MESSAGE");

		    this->kiva_loaded = false;			    
		    this->world = _world;
		    this->sdf = _sdf;		
		    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&WarehouseWorldPlugin::OnUpdate, this));
		  
		  }
			
		public: void OnUpdate() {
			//std::vector<gazebo::physics::ModelPtr> models = world->Models();
			gazebo::physics::ModelPtr kiva_1 = world->ModelByName("kiva_1");
    			if (!kiva_loaded && kiva_1 != NULL) {
				this->model = kiva_1;	
				this->model->LoadPlugins();
				kiva_loaded = true;
				ROS_INFO("kiva_1 plugin loaded");
				
			}		
			//OnUpdateModel();
		}

        private: physics::ModelPtr model;
	private: bool kiva_loaded = false;
	private: event::ConnectionPtr updateConnection;
	private: physics::WorldPtr world;
	private: sdf::ElementPtr sdf;
	};
	GZ_REGISTER_WORLD_PLUGIN(WarehouseWorldPlugin)
}
