#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#define degreesToRadians(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define INITIAL_ANGULAR_VELOCITY_LEFT 2
#define INITIAL_ANGULAR_VELOCITY_RIGHT -2
#define CONTINUED_ANGULAR_VELOCITY_LEFT 0.25
#define CONTINUED_ANGULAR_VELOCITY_RIGHT -0.25
#define NUMBER_OF_ITERATIONS 50

namespace gazebo {
        double init_x, init_y;
	double dist = 1;
	int iterations = 0;
	class KivaRotateCenter: public ModelPlugin {
		public: KivaRotateCenter() {}
		public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {




		    if (!ros::isInitialized())
		    {
		      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
			<< "Load the Gazebo system plugin 'libkiva_rotate.so' in the gazebo_ros package)");
		      return;
		    }


			this->model = _parent;
			this->sdf = _sdf;
			ignition::math::Pose3d pose = this->get_world_pose();
			init_y = pose.Pos().Y();
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
			} else if(direction==1) {
				//right	
				std::cout<<"Rotating right\n";		
				this->model->SetAngularVel(ignition::math::Vector3d(0, 0, INITIAL_ANGULAR_VELOCITY_RIGHT));
				ignition::math::Pose3d pose = this->get_world_pose();
				float current_yaw = this->get_yaw(pose);
				this->rotation_in_rads = degreesToRadians(-90) + current_yaw;
				std::cout<<"Current Yaw: "<<current_yaw<<"\n";
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
						iterations++;
					} else if((current_yaw-rotation_in_rads>0.01)) {
						std::cout<<"Rotating right again\n";	
						this->model->SetAngularVel(ignition::math::Vector3d(0, 0, continued_angl_vel_right));
						//this->continued_angl_vel_left/=2;
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
		        	this->model->GetJoint("left_front_wheel_hinge")->SetVelocity(0, 2.0);
        this->model->GetJoint("right_front_wheel_hinge")->SetVelocity(0, 2.0);
        this->model->GetJoint("left_back_wheel_hinge")->SetVelocity(0, 2.0);
        this->model->GetJoint("right_back_wheel_hinge")->SetVelocity(0, 2.0);		

pose = this->model->WorldPose();
        double y = pose.Pos().Y();
      
        if(y - init_y >= 1 * dist){
            this->model->GetJoint("left_front_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("right_front_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("left_back_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("right_back_wheel_hinge")->SetVelocity(0, 0.0);
        }		
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
		// Define ptr to physics model
		private: physics::ModelPtr model;
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
