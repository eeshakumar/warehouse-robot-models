#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#define degreesToRadians(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define INITIAL_ANGULAR_VELOCITY_LEFT 2
#define INITIAL_ANGULAR_VELOCITY_RIGHT -2
#define CONTINUED_ANGULAR_VELOCITY_LEFT 0.25
#define CONTINUED_ANGULAR_VELOCITY_RIGHT -0.25
#define NUMBER_OF_ITERATIONS 50
#define WORLD_STR "warehouse_rotate.world"

namespace gazebo {
	int iterations = 0;
	class KivaRotateCenter: public ModelPlugin {
		public: KivaRotateCenter() {}
		public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
			this->model = _parent;
			this->sdf = _sdf;
			this->world = this->model->GetWorld();
			std::cout<<"World: "<<this->world->Name()<<"\n";
			this->shelf = this->world->ModelByName("shelf");
			this->continued_angl_vel_left = CONTINUED_ANGULAR_VELOCITY_LEFT;
			this->continued_angl_vel_right = CONTINUED_ANGULAR_VELOCITY_RIGHT;
			this->direction = extract_direction();
			if(direction==0) {
				//left	
				std::cout<<"Rotating left\n";
				this->model->SetAngularVel(ignition::math::Vector3d(0, 0, INITIAL_ANGULAR_VELOCITY_LEFT));			
				ignition::math::Pose3d pose = this->get_world_pose();
				float current_yaw = this->get_yaw(pose);
				this->rotation_in_rads = degreesToRadians(90) + current_yaw;
				std::cout<<"Current Yaw: "<<current_yaw<<"\n";
				this->set_shelf_pose(this->get_world_pose());
			} else if(direction==1) {
				//right	
				std::cout<<"Rotating right\n";		
				this->model->SetAngularVel(ignition::math::Vector3d(0, 0, INITIAL_ANGULAR_VELOCITY_RIGHT));
				ignition::math::Pose3d pose = this->get_world_pose();
				float current_yaw = this->get_yaw(pose);
				this->rotation_in_rads = degreesToRadians(-90) + current_yaw;
				std::cout<<"Current Yaw: "<<current_yaw<<"\n";
				this->set_shelf_pose(this->get_world_pose());
			} else {
				std::cout<<"Invalid Direction Code "<<direction<<"\n";
				this->rotation_in_rads = 0;
			}
			std::cout<<"Necessary Yaw: "<<this->rotation_in_rads<<"\n";
			//Listen to update event.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&KivaRotateCenter::OnUpdate, this));	
			
		}
		public: void OnUpdate() {
			if (iterations<=NUMBER_OF_ITERATIONS) {
				ignition::math::Vector3d angular_vel = this->get_angular_vel();
				if (angular_vel[2]<0.01 && angular_vel[2]>-0.01) {
					ignition::math::Pose3d pose = this->get_world_pose();
					float current_yaw = this->get_yaw(pose);
					std::cout<<"Current Yaw: "<<current_yaw<<"\n";				
					std::cout<<"Achieved near rest angular vel :"<<angular_vel<<"\n";
					if((current_yaw-rotation_in_rads<0.01)){
						std::cout<<"Rotating left again\n";					
						this->model->SetAngularVel(ignition::math::Vector3d(0, 0, continued_angl_vel_left));
						//this->continued_angl_vel_left/=2;
						this->set_shelf_pose(this->get_world_pose());	
						iterations++;
					} else if((current_yaw-rotation_in_rads>0.01)) {
						std::cout<<"Rotating right again\n";	
						this->model->SetAngularVel(ignition::math::Vector3d(0, 0, continued_angl_vel_right));
						//this->continued_angl_vel_left/=2;
						this->set_shelf_pose(this->get_world_pose());
						iterations++;
					} else {
						std::cout<<"Achieved required yaw: "<<current_yaw<<"\n";
					}	
				}
			} else {
				std::cout<<"Reached maximum iterations!!\n";
ignition::math::Pose3d pose = this->get_world_pose();
				float final_yaw = this->get_yaw(pose);
				std::cout<<"Final Yaw: "<<final_yaw<<"\n";				
			}		
		}
		private: int extract_direction() {
			int direction = -1;
			if(this->sdf->HasElement("direction")) {
				direction = this->sdf->Get<int>("direction");
			}
			return direction;		
		}
		private: float get_yaw(ignition::math::Pose3d pose) {
			return pose.Rot().Yaw();		 
		}
		private: ignition::math::Vector3d get_angular_vel() {
			return this->model->RelativeAngularVel();
		}
		private: ignition::math::Pose3d get_world_pose() {
			return this->model->WorldPose();
		}
		private: void set_shelf_pose(ignition::math::Pose3d pose_robot) {
			ignition::math::Pose3d pose_shelf = this->shelf->WorldPose();
			ignition::math::Pose3d pose_shelf_new = ignition::math::Pose3d(pose_shelf.Pos().X(), pose_shelf.Pos().Y(), pose_shelf.Pos().Z(), pose_shelf.Rot().Roll(), pose_shelf.Rot().Pitch(), pose_robot.Rot().Yaw());
			this->shelf->SetWorldPose(pose_shelf_new);
			std::cout<<"Current Shelf Yaw: "<<this->get_yaw(this->shelf->WorldPose())<<"\n";		
		} 
		// Define ptr to physics model
		private: physics::ModelPtr model, shelf;
		//Define world ptr for model
		private: physics::WorldPtr world;
		// Pointer to update event connection
		private: event::ConnectionPtr updateConnection;
		//Define ptr to sdf object world file
		private: sdf::ElementPtr sdf;
		//Define 90 degrees for rotation and velocities
		private: float rotation_in_rads, continued_angl_vel_left, continued_angl_vel_right;
		//Define direction
		private: int direction;
	};
	//Register Kiva Rotate Center for simulator
	GZ_REGISTER_MODEL_PLUGIN(KivaRotateCenter)
}
