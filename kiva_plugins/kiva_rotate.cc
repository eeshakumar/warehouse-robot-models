#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#define degreesToRadians(angleDegrees) ((angleDegrees) * M_PI / 180.0)

namespace gazebo {
	class KivaRotate: public ModelPlugin {
		public: KivaRotate() {}
		public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
			std::cout<<"Init complete\n";
			this->model = _parent;
			this->sdf = _sdf;

			int direction = -1;
			//Apply angular velocity to kiva robot
			if(this->sdf->HasElement("direction")) {
				direction = this->sdf->Get<int>("direction");
			}
			if(direction==0)
				this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 2.895));	
			else
				this->model->SetAngularVel(ignition::math::Vector3d(0, 0, -2.895));
			//Listen to update event.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&KivaRotate::OnUpdate, this));	
		}	
		public: void OnUpdate() {	
			ignition::math::Pose3d pose = this->model->WorldPose();
			//Get Roll.
			float roll = pose.Rot().Roll();
			//Get Pitch.
			float pitch = pose.Rot().Pitch();
			//Get Yaw.
			float yaw = pose.Rot().Yaw();
			//print to console
			std::cout<<"printng rpy";
			std::cout<<"x = "<<roll<<"\n";
			std::cout<<"y = "<<pitch<<"\n";
			std::cout<<"z = "<<yaw<<"\n";
		}
		// Define ptr to physics model
		private: physics::ModelPtr model;
		// Pointer to update event connection
		private: event::ConnectionPtr updateConnection;
		//Define ptr to sdf object world file
		private: sdf::ElementPtr sdf;
		//Define 90 degrees for rotation
		private: float rotation_in_rads;
	};
	//Register Kiva Rotate for simulator
	GZ_REGISTER_MODEL_PLUGIN(KivaRotate)
}
