#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <iterator> 
#include <map> 
#define degreesToRadians(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define INITIAL_ANGULAR_VELOCITY_LEFT 2
#define INITIAL_ANGULAR_VELOCITY_RIGHT -2
#define CONTINUED_ANGULAR_VELOCITY_LEFT 0.25
#define CONTINUED_ANGULAR_VELOCITY_RIGHT -0.25
#define NUMBER_OF_ITERATIONS 100
#define X_AXIS 0
#define Y_AXIS 1

namespace gazebo {
    std::map<int, double> init_x;
    std::map<int, double> init_y;
    std::map<int, double> distX;
    std::map<int, double> distY;
    std::map<int, bool> started;
    std::map<int, int> iterations;
    std::map<int, int> numRotations;
    std::map<int, int> rotation;
    std::map<int, bool> isBusy;
    std::map<int, bool> isRotating;
    std::map<int, bool> isMovingOnX;
    std::map<int, bool> isMovingOnY;
    std::map<int, int> movingOn;
    std::map<int, bool> isBlocked;
    std::map<int, int> direction;
    std::map<int, float> continued_angl_vel_left;
    std::map<int, float> continued_angl_vel_right;
    gazebo::transport::PublisherPtr pub;
    int dir = -1;

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
            this->collisionSub = this->node->Subscribe("~/" +model->GetName() +"/chassis/laser/scan", &KivaRotateCenter::OnCollision, this);
            this->xSub = this->node->Subscribe("~/kiva/movx", &KivaRotateCenter::OnMsgX, this);
            this->ySub = this->node->Subscribe("~/kiva/movy", &KivaRotateCenter::OnMsgY, this);
            this->rotSub = this->node->Subscribe("~/kiva/rot", &KivaRotateCenter::OnMsgRot, this);
            

            pub = node->Advertise<gazebo::msgs::Vector3d>("~/"+model->GetName()+"/status");
            
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&KivaRotateCenter::OnUpdate, this));
		}
		public: void OnUpdate() {
            std::map<int, bool>::iterator itr;
            for (itr = isMovingOnX.begin(); itr != isMovingOnX.end(); ++itr) {
                int kivaId = itr->first;
                if(model->GetName().compare("kiva_"+ std::to_string(kivaId))==0){
                    gazebo::msgs::Vector3d publisherMsg;
                    ignition::math::Pose3d pose = this->get_world_pose();
                            
                    if(isMovingOnX[kivaId]  || isMovingOnY[kivaId]  || isRotating[kivaId]) { //Check if kiva is moving -> set status to 1 (busy)
                        #if GAZEBO_MAJOR_VERSION < 6
                        gazebo::msgs::Set(&publisherMsg, gazebo::math::Vector3(1, pose.Pos().X(), pose.Pos().Y()));
                        #else
                        gazebo::msgs::Set(&publisherMsg, ignition::math::Vector3d(1, pose.Pos().X(), pose.Pos().Y()));
                        #endif
                    } else if(isBlocked[kivaId]){ //kiva couldn't move to previous location because it was blocked -> set status to 2 (error) 
                        #if GAZEBO_MAJOR_VERSION < 6
                        gazebo::msgs::Set(&publisherMsg, gazebo::math::Vector3(2, pose.Pos().X(), pose.Pos().Y()));
                        #else
                        gazebo::msgs::Set(&publisherMsg, ignition::math::Vector3d(2, pose.Pos().X(), pose.Pos().Y()));
                        #endif
                    } 
                    else { //Kiva is idle -> set status to 0 (idle)
                        #if GAZEBO_MAJOR_VERSION < 6
                        gazebo::msgs::Set(&publisherMsg, gazebo::math::Vector3(0, pose.Pos().X(), pose.Pos().Y()));
                        #else
                        gazebo::msgs::Set(&publisherMsg, ignition::math::Vector3d(0, pose.Pos().X(), pose.Pos().Y()));
                        #endif
                    }

                    pub->Publish(publisherMsg);
                    
                    bool isMoving = false;
                    if(isMovingOnX[kivaId] && !isBusy[kivaId]){
                        std::cout << "Orienting on x \n";
                        dir = orientX(distX[kivaId], kivaId);
                        isBusy[kivaId] = true;
                        movingOn[kivaId]= X_AXIS;
                    } else if(isMovingOnY[kivaId] && !isBusy[kivaId]){
                        std::cout << "Orienting on y \n";
                        dir = orientY(distY[kivaId], kivaId);
                        std::cout << dir << std::endl;
                        isBusy[kivaId] = true;
                        movingOn[kivaId] = Y_AXIS;
                    }
                    if(isRotating[kivaId]){
                        if(!started[kivaId]){
                            iterations[kivaId] = 0;
                            start_rotate(dir, numRotations[kivaId], kivaId);
                            started[kivaId] = true;
                             std::cout<<"this Yaw: " << rotation[kivaId] << "\n";
                        }
                    if (iterations[kivaId]<=(NUMBER_OF_ITERATIONS*numRotations[kivaId])) {
                        ignition::math::Vector3d angular_vel = this->get_angular_vel();
                        if (angular_vel[2]<0.01 && angular_vel[2]>-0.01) {
                            float current_yaw = radiansToDegrees(this->get_yaw(pose));
                            
                            std::cout<<"Current Yaw: " << current_yaw <<"\n";
                            std::cout<<"Final Yaw: " << rotation[kivaId] << "\n";
                            std::cout<<"Achieved near rest angular vel :" << angular_vel <<"\n";
                            if(current_yaw-rotation[kivaId]<0.1){
                        		std::cout<<"Rotating left again\n";
                                this->model->SetAngularVel(ignition::math::Vector3d(0, 0, continued_angl_vel_left[kivaId]));
                                iterations[kivaId]++;
                            } else if(current_yaw-rotation[kivaId]>0.1) {
                        		std::cout<<"Rotating right again\n";
                                this->model->SetAngularVel(ignition::math::Vector3d(0, 0, continued_angl_vel_right[kivaId]));
                                iterations[kivaId]++;
                            } else {
                        		std::cout<<"Achieved required yaw: "<<current_yaw<<"\n";
                                started[kivaId] = false;
                                isRotating[kivaId] = false;
                                isMoving = true;
                                ignition::math::Pose3d pose = this->get_world_pose();
                                init_x[kivaId] = pose.Pos().X();
                                init_y[kivaId] = pose.Pos().Y();
                            }	
                        }
                    } else {
                        std::cout<<"Reached maximum iterations!!\n";
                        ignition::math::Pose3d pose = this->get_world_pose();
                        float final_yaw = radiansToDegrees(this->get_yaw(pose));
                        std::cout<<"Final Yaw: "<<final_yaw<<"\n";
                        started[kivaId] = false;
                        isRotating[kivaId] = false;
                        isMoving = true;
                        init_x[kivaId] = pose.Pos().X();
                        init_y[kivaId] = pose.Pos().Y();
                        }
                    }
                    if(!isRotating[kivaId] ){
                        if (movingOn[kivaId]==X_AXIS){
                            moveOnX(distX[kivaId], kivaId);
                        }
                        else if (movingOn[kivaId]==Y_AXIS){
                            moveOnY(distY[kivaId], kivaId);
                        }
                    }
                }
            }
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
        
    private: void start_rotate(int dir, int numRotations, int kivaId){
        ignition::math::Pose3d pose = this->get_world_pose();
        continued_angl_vel_left[kivaId] = CONTINUED_ANGULAR_VELOCITY_LEFT;
        continued_angl_vel_right[kivaId] = CONTINUED_ANGULAR_VELOCITY_RIGHT;
        std::cout<< "kiva will rotate: " << dir << " " << numRotations << "times\n";
        if(isRotating[kivaId]) direction[kivaId] = dir;
        if(direction[kivaId]==0) {
            //left
            std::cout<<"Rotating left\n";
            this->model->SetAngularVel(ignition::math::Vector3d(0, 0, INITIAL_ANGULAR_VELOCITY_LEFT));
            ignition::math::Pose3d pose = this->get_world_pose();
            float current_yaw = radiansToDegrees(this->get_yaw(pose));
            rotation[kivaId] = closestDegree((int)round((90*numRotations) + current_yaw) % 360);
            if(isCloseToDegrees(current_yaw, 270) && (rotation[kivaId] == 0 || rotation[kivaId] == 360)){
                rotation[kivaId] = 359;
            }

            std::cout<<"Current Yaw: "<<current_yaw<<"\n";
        } else if(direction[kivaId]==1) {
            //right
            std::cout<<"Rotating right\n";
            this->model->SetAngularVel(ignition::math::Vector3d(0, 0, INITIAL_ANGULAR_VELOCITY_RIGHT));
            ignition::math::Pose3d pose = this->get_world_pose();
            float current_yaw = radiansToDegrees(this->get_yaw(pose));
            rotation[kivaId] = closestDegree((int)round(current_yaw - (90*numRotations)) % 360);

            if(isCloseToDegrees(current_yaw, 90) && (rotation[kivaId] == 0 || rotation[kivaId] == 360)){
                rotation[kivaId] = 1;
            } 
            std::cout<<"Current Yaw: "<<current_yaw<<"\n";
        } else {
            std::cout<<"Invalid Direction Code "<<direction[kivaId]<<"\n";
            rotation[kivaId] = 0;
        }
        std::cout<<"Necessary Yaw: "<<rotation[kivaId]<<"\n";
    }
        
    public: int orientX(int dest, int kivaId) {
        ignition::math::Pose3d pose = this->get_world_pose();
        double x = pose.Pos().X();
        double yaw = radiansToDegrees(get_yaw(pose));
        std::cout<<"x: "<<x << "yaw: "<< yaw<< "\n";

        if((isCloseToDegrees(yaw, 0) || isCloseToDegrees(yaw, 359)) && dest < 0){
            numRotations[kivaId] = 2;
            isRotating[kivaId]  = true;
            return 0;
        }
        else if(isCloseToDegrees(yaw, 90) && dest < 0){
            isRotating[kivaId]  = true;
            numRotations[kivaId] = 1;
            return 0;
        }
        else if(isCloseToDegrees(yaw, 90) && dest > 0){
            isRotating[kivaId]  = true;
            numRotations[kivaId] = 1;
            return 1;
        }
        else if(isCloseToDegrees(yaw, 180) && dest > 0){
            isRotating[kivaId]  = true;
            numRotations[kivaId] = 2;
            return 1;
        }
        else if(isCloseToDegrees(yaw, 270) && dest < 0){
            isRotating[kivaId] = true;
            numRotations[kivaId] = 1;
            return 1;
        }
        else if(isCloseToDegrees(yaw, 270) && dest > 0){
            isRotating[kivaId] = true;
            numRotations[kivaId] = 1;
            return 0;
        }
        isMovingOnX[kivaId] = true;
        movingOn[kivaId] = X_AXIS;
        isRotating[kivaId] = false;
        return -1;
    }
        
    public: int orientY(int dest, int kivaId) {
        ignition::math::Pose3d pose = this->get_world_pose();
        double y = pose.Pos().Y();
        double yaw = radiansToDegrees(get_yaw(pose));
        std::cout<<"dest: "<<dest <<" y: "<<y << "yaw: "<< yaw << "\n";
        if((isCloseToDegrees(yaw, 0) || isCloseToDegrees(yaw, 359)) && dest < 0){
            isRotating[kivaId] = true;
            numRotations[kivaId] = 1;
            return 1;
        }
        else if((isCloseToDegrees(yaw, 0) || isCloseToDegrees(yaw, 359)) && dest > 0){
            isRotating[kivaId] = true;
            numRotations[kivaId] = 1;
            return 0;
        }
        else if(isCloseToDegrees(yaw, 90) && dest < 0){
            isRotating[kivaId] = true;
            numRotations[kivaId] = 2;
            return 0;
        }
        else if(isCloseToDegrees(yaw, 180) && dest < 0){
            isRotating[kivaId] = true;
            numRotations[kivaId] = 1;
            return 0;
        }
        else if(isCloseToDegrees(yaw, 180) && dest > 0){
            isRotating[kivaId] = true;
            numRotations[kivaId] = 1;
            return 1;
        }
        else if(isCloseToDegrees(yaw, 270) && dest > 0){
            isRotating[kivaId] = true;
            numRotations[kivaId] = 2;
            return 1;
        }
        isMovingOnY[kivaId] = true;
        movingOn[kivaId] = Y_AXIS;
        return -1;
    }
        
    public: void moveOnX(int dist, int kivaId) {
        ignition::math::Pose3d pose = this->get_world_pose();
       this->model->GetJoint("left_front_wheel_hinge")->SetVelocity(0, 5.0);
        this->model->GetJoint("right_front_wheel_hinge")->SetVelocity(0, 5.0);
        this->model->GetJoint("left_back_wheel_hinge")->SetVelocity(0, 5.0);
        this->model->GetJoint("right_back_wheel_hinge")->SetVelocity(0, 5.0);
    
        double x = pose.Pos().X();

        if(abs(x - init_x[kivaId]) >= 1 * abs(dist) || isBlocked[kivaId]){
            if(isBlocked[kivaId]){
                std::cout << "kiva " << kivaId << "is blocked"<< std::endl;
                isMovingOnY[kivaId] = false;
                isBusy[kivaId] = false;
            }
            this->model->GetJoint("left_front_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("right_front_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("left_back_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("right_back_wheel_hinge")->SetVelocity(0, 0.0);
            
            isMovingOnX[kivaId] = false;
            isBusy[kivaId] = false;
            movingOn[kivaId] = -1;
            init_x[kivaId] = pose.Pos().X();
            init_y[kivaId] = pose.Pos().Y();
        }
    }
        
    public: void moveOnY(int dist, int kivaId) {
        ignition::math::Pose3d pose = this->get_world_pose();
        double yaw = get_yaw(pose);
        this->model->GetJoint("left_front_wheel_hinge")->SetVelocity(0, 5.0);
        this->model->GetJoint("right_front_wheel_hinge")->SetVelocity(0, 5.0);
        this->model->GetJoint("left_back_wheel_hinge")->SetVelocity(0, 5.0);
        this->model->GetJoint("right_back_wheel_hinge")->SetVelocity(0, 5.0);
        
        double y = pose.Pos().Y();
        
        // std::cout << isBlocked[kivaId] << std::endl;
        if(abs(y - init_y[kivaId]) >= 1 * abs(dist) || isBlocked[kivaId]){
            if(isBlocked[kivaId]){
                isBusy[kivaId] = false;
                std::cout << "kiva " << kivaId << "is blocked"<< std::endl;
            }
            this->model->GetJoint("left_front_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("right_front_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("left_back_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("right_back_wheel_hinge")->SetVelocity(0, 0.0);
            isMovingOnY[kivaId] = false;
            isBusy[kivaId] = false;
            movingOn[kivaId] = -1;
            init_x[kivaId] = pose.Pos().X();
            init_y[kivaId] = pose.Pos().Y();
        }
    }
        
    public: bool isCloseToDegrees(double degrees, double val){
        if(degrees <= val+2 && degrees >= val-2)
            return true;
        return false;
    }
    
    public: bool isCloseTo(double dist, double val){
        if(dist <= val+0.01 && dist >= val-0.01)
            return true;
        return false;
    }
        
    public: double radiansToDegrees(double rad){
        if(rad >= 0){
            return (rad*180)/ M_PI;
        }
         else {
            return 360-((abs(rad)*180)/M_PI);
        }
    }

    public: int closestDegree(double deg){
        std::cout << "closest degree to calculate: " << deg << "\n";
        if(isCloseToDegrees(deg, 0)){
            return 0;
        } 
        if(isCloseToDegrees(deg, -90)){
            return 270;
        } 
        if(isCloseToDegrees(deg, -180)){
            return 180;
        } 
        if(isCloseToDegrees(deg, 90)){
            return 90;
        } 
        if(isCloseToDegrees(deg, 180)){
            return 180;
        } 
        if(isCloseToDegrees(deg, 270)){
            return 270;
        } 
        if(isCloseToDegrees(deg, 360)){
            return 360;
        } 
        return 0;  
    }
        
    private: void OnMsg(ConstVector3dPtr &_msg){
        int kivaId = _msg->z();
        if(model->GetName().compare("kiva_"+ std::to_string(kivaId))==0){
            if(!isMovingOnX[kivaId] && !isMovingOnY[kivaId]){
                isMovingOnX[kivaId] = false;
                isMovingOnY[kivaId] = false;
                distX[kivaId] = 0;
                distY[kivaId] = 0;
                int xCoord = _msg->x();
                int yCoord = _msg->y();
                ignition::math::Pose3d pose = this->get_world_pose();
                double current_x = round(pose.Pos().X());
                double current_y = round(pose.Pos().Y());
                std::cout << "x coord of kiva " << current_x <<" y coord of kiva " << current_y << "\n";
                if(!isCloseTo(xCoord, current_x)){
                    isMovingOnX[kivaId] = true;
                    distX[kivaId] = xCoord-current_x; 
                }
                
                if(!isCloseTo(yCoord, current_y)){
                    isMovingOnY[kivaId] = true;
                    distY[kivaId] = yCoord-current_y;
                }
                
                std::cout << "x val " << xCoord <<" y val " << yCoord << "\n";
                std::cout << "dist x " << distX[kivaId] <<" dist y " << distY[kivaId] << "\n";
            }
        }
    }

    private: void OnMsgX(ConstVector3dPtr &_msg){
        int kivaId = _msg->x();
        if(model->GetName().compare("kiva_"+ std::to_string(kivaId))==0){
            if(!isMovingOnX[kivaId] && !isMovingOnY[kivaId]){
                isMovingOnX[kivaId] = false;
                isMovingOnY[kivaId] = false;
                distX[kivaId] = 0;
                distY[kivaId] = 0;
                int xCoord = _msg->y();
                ignition::math::Pose3d pose = this->get_world_pose();
                double current_x = round(pose.Pos().X());
                if(!isCloseTo(xCoord, current_x)){
                    isMovingOnX[kivaId] = true;
                    distX[kivaId] = xCoord; 
                }
            }
        }
    }

    private: void OnMsgY(ConstVector3dPtr &_msg){
        int kivaId = _msg->x();
        if(model->GetName().compare("kiva_"+ std::to_string(kivaId))==0){
            if(!isBusy[kivaId]){
                isMovingOnX[kivaId] = false;
                isMovingOnY[kivaId] = false;
                distX[kivaId] = 0;
                distY[kivaId] = 0;
                int yCoord = _msg->y();
                ignition::math::Pose3d pose = this->get_world_pose();
                double current_y = round(pose.Pos().Y());
                if(!isCloseTo(yCoord, current_y)){
                    isMovingOnY[kivaId] = true;
                    distY[kivaId] = yCoord;
                }
            }
        }
    }

    private: void OnMsgRot(ConstVector3dPtr &_msg){
        int kivaId = _msg->x();
        if(model->GetName().compare("kiva_"+ std::to_string(kivaId))==0){
            if(!isBusy[kivaId]){
                dir = _msg->y();
                isRotating[kivaId] = true;
            }
        }
    }

    void OnCollision(ConstLaserScanStampedPtr &_msg){
        ::google::protobuf::int32 sec = _msg->time().sec();
        ::google::protobuf::int32 nsec = _msg->time().nsec();
        // std::cout << "Read time: sec: " << sec << " nsec: " << nsec << std::endl;
        
        ::gazebo::msgs::LaserScan scan = _msg->scan();
        int kivaId = std::stoi(scan.frame().substr(5,6));
        for(int i=0; i < scan.count(); i++){
            if(scan.ranges(i)<0.7){
               // std::cout << kivaId << " Scan: " << scan.ranges(i) << std::endl;
                isBlocked[kivaId] = true;
            } else {
                isBlocked[kivaId] = false;
            }
        }   
    }
    
		private: physics::ModelPtr model;
		
		private: event::ConnectionPtr updateConnection;
		
		private: sdf::ElementPtr sdf;

        private: transport::NodePtr node;
        private: transport::SubscriberPtr sub;
        private: transport::SubscriberPtr xSub;
        private: transport::SubscriberPtr ySub;
        private: transport::SubscriberPtr rotSub;
        private: transport::SubscriberPtr collisionSub;
	};

	GZ_REGISTER_MODEL_PLUGIN(KivaRotateCenter)
}
