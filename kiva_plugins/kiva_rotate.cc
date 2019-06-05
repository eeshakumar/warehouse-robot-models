#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>


namespace gazebo {
	class KivaRotate: public ModelPlugin {
		public: KivaRotate() {}
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
			std::cout<<"Init complete";
			this->model = _parent;
			ignition::math::Pose3d pose = this->model->WorldPose();
			//Get Roll.
			double x_roll = pose.Rot().Roll();
			//Get Pitch.
			double y_pitch = pose.Rot().Pitch();
			//Get Yaw.
			double z_yaw = pose.Rot().Yaw();
			//print to console
			std::cout<<"printng rpy";
			std::cout<<"x = "<<x_roll;
			std::cout<<"y = "<<y_pitch;
			std::cout<<"z = "<<z_yaw;
			//Apply angular velocity to kiva robot
			this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 2.85));	

			//Listen to update event.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&KivaRotate::OnUpdate, this));	
		}	
		public: void OnUpdate() {	
			ignition::math::Pose3d pose = this->model->WorldPose();
			//Get Roll.
			double x_roll = pose.Rot().Roll();
			//Get Pitch.
			double y_pitch = pose.Rot().Pitch();
			//Get Yaw.
			double z_yaw = pose.Rot().Yaw();
			//print to console
			std::cout<<"printng rpy";
			std::cout<<"x = "<<x_roll;
			std::cout<<"y = "<<y_pitch;
			std::cout<<"z = "<<z_yaw;	
		}
		// Define ptr to physics model
		private: physics::ModelPtr model;
		// Pointer to update event connection
		private: event::ConnectionPtr updateConnection;
	};
	//Register Kiva Rotate for simulator
	GZ_REGISTER_MODEL_PLUGIN(KivaRotate)
}
