#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#define degreesToRadians(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define INITIAL_ANGULAR_VELOCITY_LEFT 2
#define INITIAL_ANGULAR_VELOCITY_RIGHT -2
#define CONTINUED_ANGULAR_VELOCITY_LEFT 0.25
#define CONTINUED_ANGULAR_VELOCITY_RIGHT -0.25
#define NUMBER_OF_ITERATIONS 100
#define X_AXIS 0
#define Y_AXIS 1

namespace gazebo {
    double init_x, init_y;
	double distX = 0;
    double distY = 0;
    bool started = false;
	int iterations = 0;
    int numRotations = 0;
    bool isRotating = false;
    bool isMovingOnX = false;
    bool isMovingOnY = false;
    bool isBusy = false;
    int movingOn = -1;
    gazebo::transport::PublisherPtr pub;
    
	class KivaRotateCenter: public ModelPlugin {
		public: KivaRotateCenter() {}
		public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
            
			this->model = _parent;
			this->sdf = _sdf;
            
            std::cout << "Plugin Loaded for: "<< model->GetName()<< "\n";
            
            this->node = transport::NodePtr(new transport::Node());
            #if GAZEBO_MAJOR_VERSION < 8
            this->node->Init(this->model->GetWorld()->GetName());
            #else
            this->node->Init(this->model->GetWorld()->Name());
            #endif
    
            this->sub = this->node->Subscribe("~/kiva/mov", &KivaRotateCenter::OnMsg, this);
            
            pub = node->Advertise<gazebo::msgs::Vector3d>("~/"+model->GetName()+"/status");
          //  pub->WaitForConnection();
            
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&KivaRotateCenter::OnUpdate, this));
		}
		public: void OnUpdate() {
           
            gazebo::msgs::Vector3d publisherMsg;
            
            if(isMovingOnX || isMovingOnY || isRotating) { //Check if kiva is moving -> set status to 1 (busy)
                #if GAZEBO_MAJOR_VERSION < 6
                gazebo::msgs::Set(&publisherMsg, gazebo::math::Vector3(1, 0, 0));
                #else
                gazebo::msgs::Set(&publisherMsg, ignition::math::Vector3d(1, 0, 0));
                #endif
            } else { //Kiva is idle -> set status to 0 (idle)
                #if GAZEBO_MAJOR_VERSION < 6
                gazebo::msgs::Set(&publisherMsg, gazebo::math::Vector3(0, 0, 0));
                #else
                gazebo::msgs::Set(&publisherMsg, ignition::math::Vector3d(0, 0, 0));
                #endif
            }
            
            pub->Publish(publisherMsg);
            
            int dir = -1;
            bool isMoving = false;
            if(isMovingOnX && !isBusy){
                std::cout << "Orienting on x \n";
                dir = orientX(distX);
                isBusy = true;
                movingOn = X_AXIS;
            } else if(isMovingOnY && !isBusy){
                std::cout << "Orienting on y \n";
                dir = orientY(distY);
                isBusy = true;
                movingOn = Y_AXIS;
            }
            if(isRotating){
                if(!started){
                    iterations = 0;
                    start_rotate(dir, numRotations);
                    started = true;
                }
			if (iterations<=(NUMBER_OF_ITERATIONS*numRotations)) {
                ignition::math::Vector3d angular_vel = this->get_angular_vel();
				if (angular_vel[2]<0.01 && angular_vel[2]>-0.01) {
					ignition::math::Pose3d pose = this->get_world_pose();
					float current_yaw = this->get_yaw(pose);
                    if(isCloseTo(current_yaw, -3.14)){
                        current_yaw += degreesToRadians(360);
                    }
			//		std::cout<<"Current Yaw: " << current_yaw <<"\n";
			//		std::cout<<"Achieved near rest angular vel :" << angular_vel<<"\n";
					if(current_yaw-rotation_in_rads<0.01 || current_yaw-rotation_in_rads > 1){
			//			std::cout<<"Rotating left again\n";
						this->model->SetAngularVel(ignition::math::Vector3d(0, 0, continued_angl_vel_left));
						//this->continued_angl_vel_left/=2;	
						iterations++;
					} else if(current_yaw-rotation_in_rads>0.01 || current_yaw-rotation_in_rads<-1) {
				//		std::cout<<"Rotating right again\n";
						this->model->SetAngularVel(ignition::math::Vector3d(0, 0, continued_angl_vel_right));
						this->continued_angl_vel_left/=2;
						iterations++;
					} else {
				//		std::cout<<"Achieved required yaw: "<<current_yaw<<"\n";
                        started = false;
                        isRotating = false;
                        isMoving = true;
                        ignition::math::Pose3d pose = this->get_world_pose();
                        init_x = pose.Pos().X();
                        init_y = pose.Pos().Y();
					}	
				}
			} else {
				std::cout<<"Reached maximum iterations!!\n";
                ignition::math::Pose3d pose = this->get_world_pose();
				float final_yaw = this->get_yaw(pose);
				std::cout<<"Final Yaw: "<<final_yaw<<"\n";
                started = false;
                isRotating = false;
                isMoving = true;
                init_x = pose.Pos().X();
                init_y = pose.Pos().Y();
                }
            }
            if(!isRotating){
                if (movingOn==X_AXIS){
                    moveOnX(distX);
                }
                else if (movingOn==Y_AXIS){
                    moveOnY(distY);
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
        
    private: void start_rotate(int dir, int numRotations){
        ignition::math::Pose3d pose = this->get_world_pose();
        init_y = pose.Pos().Y();
        this->continued_angl_vel_left = CONTINUED_ANGULAR_VELOCITY_LEFT;
        this->continued_angl_vel_right = CONTINUED_ANGULAR_VELOCITY_RIGHT;
        std::cout<< "kiva will rotate: " << dir << " " << numRotations << "times\n";
        if(isRotating) this->direction = dir;
        if(direction==0) {
            //left
            std::cout<<"Rotating left\n";
            this->model->SetAngularVel(ignition::math::Vector3d(0, 0, INITIAL_ANGULAR_VELOCITY_LEFT));
            ignition::math::Pose3d pose = this->get_world_pose();
            float current_yaw = this->get_yaw(pose);
            this->rotation_in_rads = degreesToRadians(90*numRotations) + current_yaw;
            if(this->rotation_in_rads < -3.14) {
                this->rotation_in_rads = this->rotation_in_rads + 6.28;
            }
            std::cout<<"Current Yaw: "<<current_yaw<<"\n";
        } else if(direction==1) {
            //right
            std::cout<<"Rotating right\n";
            this->model->SetAngularVel(ignition::math::Vector3d(0, 0, INITIAL_ANGULAR_VELOCITY_RIGHT));
            ignition::math::Pose3d pose = this->get_world_pose();
            float current_yaw = this->get_yaw(pose);
            this->rotation_in_rads = degreesToRadians(-90*numRotations) + current_yaw;
            if(this->rotation_in_rads < -3.14) {
                this->rotation_in_rads = this->rotation_in_rads + 6.28;
            }
            std::cout<<"Current Yaw: "<<current_yaw<<"\n";
        } else {
            std::cout<<"Invalid Direction Code "<<direction<<"\n";
            this->rotation_in_rads = 0;
        }
        std::cout<<"Necessary Yaw: "<<this->rotation_in_rads<<"\n";
    }
        
    public: int orientX(int dest) {
        ignition::math::Pose3d pose = this->get_world_pose();
        double x = pose.Pos().X();
        double yaw = get_yaw(pose);
        std::cout<<"x: "<<x << "yaw: "<< yaw<< "\n";
        if(abs(x - dest) < 0.01){
            isRotating = false;
            isMovingOnX = false;
            return -1;
        }
        if(isCloseTo(yaw, 0) && dest < 0){
            numRotations = 2;
            isRotating = true;
            return 0;
        }
        else if(isCloseTo(yaw, 1.571) && dest < 0){
            isRotating = true;
            numRotations = 1;
            return 0;
        }
        else if(isCloseTo(yaw, 1.571) && dest > 0){
            isRotating = true;
            numRotations = 1;
            return 1;
        }
        else if((isCloseTo(yaw, 3.141) || isCloseTo(yaw, -3.141)) && dest > 0){
            isRotating = true;
            numRotations = 2;
            return 0;
        }
        else if(isCloseTo(yaw, -1.571) && dest < 0){
            isRotating = true;
            numRotations = 1;
            return 1;
        }
        else if(isCloseTo(yaw, -1.571) && dest > 0){
            isRotating = true;
            numRotations = 1;
            return 0;
        }
        isMovingOnX = true;
        movingOn = X_AXIS;
        isRotating = false;
        return -1;
    }
        
    public: int orientY(int dest) {
        ignition::math::Pose3d pose = this->get_world_pose();
        double y = pose.Pos().Y();
        if(abs(y - dest) < 0.01){
            isRotating = false;
            isMovingOnY = false;
            return -1;
        }
        double yaw = get_yaw(pose);
        std::cout<<"dest: "<<dest <<" y: "<<y << "yaw: "<< yaw<< "\n";
        if(isCloseTo(yaw, 0) && dest < 0){
            isRotating = true;
            numRotations = 1;
            return 1;
        }
        else if(isCloseTo(yaw, 0) && dest > 0){
            isRotating = true;
            numRotations = 1;
            return 0;
        }
        else if(isCloseTo(yaw, 1.571) && dest < 0){
            isRotating = true;
            numRotations = 2;
            return 1;
        }
        else if((isCloseTo(yaw, 3.141) || isCloseTo(yaw, -3.141)) && dest < 0){
            isRotating = true;
            numRotations = 1;
            return 0;
        }
        else if((isCloseTo(yaw, 3.141) || isCloseTo(yaw, -3.141)) && dest > 0){
            isRotating = true;
            numRotations = 1;
            return 1;
        }
        else if(isCloseTo(yaw, -1.571) && dest > 0){
            isRotating = true;
            numRotations = 2;
            return 0;
        }
        isMovingOnY = true;
        movingOn = Y_AXIS;
        return -1;
    }
        
    public: void moveOnX(int dist) {
        ignition::math::Pose3d pose = this->get_world_pose();
    
        this->model->GetJoint("left_front_wheel_hinge")->SetVelocity(0, 5.0);
        this->model->GetJoint("right_front_wheel_hinge")->SetVelocity(0, 5.0);
        this->model->GetJoint("left_back_wheel_hinge")->SetVelocity(0, 5.0);
        this->model->GetJoint("right_back_wheel_hinge")->SetVelocity(0, 5.0);
    
        double x = pose.Pos().X();
        if(abs(x - init_x) >= 1 * abs(dist)){
        
            this->model->GetJoint("left_front_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("right_front_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("left_back_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("right_back_wheel_hinge")->SetVelocity(0, 0.0);
            isMovingOnX = false;
            isBusy = false;
            movingOn = -1;
            init_x = pose.Pos().X();
            init_y = pose.Pos().Y();
        }
    }
        
    public: void moveOnY(int dist) {
        ignition::math::Pose3d pose = this->get_world_pose();
        double yaw = get_yaw(pose);
        this->model->GetJoint("left_front_wheel_hinge")->SetVelocity(0, 5.0);
        this->model->GetJoint("right_front_wheel_hinge")->SetVelocity(0, 5.0);
        this->model->GetJoint("left_back_wheel_hinge")->SetVelocity(0, 5.0);
        this->model->GetJoint("right_back_wheel_hinge")->SetVelocity(0, 5.0);
        
        double y = pose.Pos().Y();
        
        if(abs(y - init_y) >= 1 * abs(dist)){
            this->model->GetJoint("left_front_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("right_front_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("left_back_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("right_back_wheel_hinge")->SetVelocity(0, 0.0);
            isMovingOnY = false;
            isBusy = false;
            movingOn = -1;
            init_x = pose.Pos().X();
            init_y = pose.Pos().Y();
        }
    }
        
    public: bool isCloseTo(double yaw, double val){
        if(yaw <= val+0.05 && yaw >= val-0.05)
            return true;
        return false;
    }
        
    private: void OnMsg(ConstVector3dPtr &_msg){
        int kivaId = _msg->z();
        if(model->GetName().compare("kiva_"+ std::to_string(kivaId))==0){
            std::cout << "moving " << "kiva_"+ std::to_string(kivaId) << "\n";
            
            if(!isMovingOnX && !isMovingOnY){
                distX = 0;
                distY = 0;
                int xCoord = _msg->x();
                int yCoord = _msg->y();
                ignition::math::Pose3d pose = this->get_world_pose();
                double current_x = round(pose.Pos().X());
                double current_y = round(pose.Pos().Y());
                std::cout << "x coord of kiva " << current_x <<" y coord of kiva " << current_y << "\n";
                if(!isCloseTo(xCoord, current_x)){
                    isMovingOnX = true;
                    distX = xCoord-current_x;
                }
                
                if(!isCloseTo(yCoord, current_y)){
                    isMovingOnY = true;
                    distY = yCoord-current_y;
                }
                
                std::cout << "x val " << xCoord <<" y val " << yCoord << "\n";
                std::cout << "dist x " << distX <<" dist y " << distY << "\n";
            }
        }
    }
        
	
		private: physics::ModelPtr model;
		
		private: event::ConnectionPtr updateConnection;
		
		private: sdf::ElementPtr sdf;
		
		private: float rotation_in_rads, continued_angl_vel_left, continued_angl_vel_right;
		
		private: int direction;

        private: transport::NodePtr node;
        
        private: transport::SubscriberPtr sub;
	};

	GZ_REGISTER_MODEL_PLUGIN(KivaRotateCenter)
}
