#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>



#define degreesToRadians(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define INITIAL_ANGULAR_VELOCITY_LEFT 2
#define INITIAL_ANGULAR_VELOCITY_RIGHT -2
#define CONTINUED_ANGULAR_VELOCITY_LEFT 0.25
#define CONTINUED_ANGULAR_VELOCITY_RIGHT -0.25
#define NUMBER_OF_ITERATIONS 100
#define X_AXIS 0
#define Y_AXIS 1


namespace gazebo
{


    std::map<int, double> init_x;
    std::map<int, double> init_y;
	//double distX = 0;
    //double distY = 0;
    std::map<int, double> distX;
    std::map<int, double> distY;
    //bool started = false;
    std::map<int, bool> started;
	//int iterations = 0;
    //int numRotations = 0;
    std::map<int, int> iterations;
    std::map<int, int> numRotations;
    //bool isRotating = false;
    //bool isMovingOnX = false;
    //bool isMovingOnY = false;
    //bool isBusy = false;
    std::map<int, bool> isBusy;
    std::map<int, bool> isRotating;
    std::map<int, bool> isMovingOnX;
    std::map<int, bool> isMovingOnY;
    //int movingOn = -1;
    std::map<int, int> movingOn;





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
			moveOnX(2, 1);
			//moveOnY(2, 1);
		}




    public: int orientX(int dest, int kivaId) {
        ignition::math::Pose3d pose = this->model->WorldPose();
        double x = pose.Pos().X();
        double yaw = radiansToDegrees(get_yaw(pose));
        std::cout<<"x: "<<x << "yaw: "<< yaw<< "\n";
        if(abs(x - dest) < 0.01){
            isRotating[kivaId]  = false;
            isMovingOnX[kivaId]  = false;
            return -1;
        }
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
        ignition::math::Pose3d pose = this->model->WorldPose();
        double y = pose.Pos().Y();
        if(abs(y - dest) < 0.01){
            isRotating[kivaId] = false;
            isMovingOnY[kivaId] = false;
            return -1;
        }
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
	if (kiva_loaded) {

	
        ignition::math::Pose3d pose = this->model->WorldPose();
       this->model->GetJoint("left_front_wheel_hinge")->SetVelocity(0, 5.0);
        this->model->GetJoint("right_front_wheel_hinge")->SetVelocity(0, 5.0);
        this->model->GetJoint("left_back_wheel_hinge")->SetVelocity(0, 5.0);
        this->model->GetJoint("right_back_wheel_hinge")->SetVelocity(0, 5.0);
    
        double x = pose.Pos().X();
	ROS_INFO("Robot at: %f", x);
        
        if(abs(x - init_x[kivaId]) >= 1 * abs(dist)){
        
            this->model->GetJoint("left_front_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("right_front_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("left_back_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("right_back_wheel_hinge")->SetVelocity(0, 0.0);
            isMovingOnX[kivaId] = false;
            isBusy[kivaId] = false;
            movingOn[kivaId] = -1;
            //init_x[kivaId] = pose.Pos().X();
            //init_y[kivaId] = pose.Pos().Y();
        }
	}
    }




public: void moveOnY(int dist, int kivaId) {
if (kiva_loaded) {
        ignition::math::Pose3d pose = this->model->WorldPose();
        double yaw = get_yaw(pose);
        this->model->GetJoint("left_front_wheel_hinge")->SetVelocity(0, 5.0);
        this->model->GetJoint("right_front_wheel_hinge")->SetVelocity(0, 5.0);
        this->model->GetJoint("left_back_wheel_hinge")->SetVelocity(0, 5.0);
        this->model->GetJoint("right_back_wheel_hinge")->SetVelocity(0, 5.0);
        
        double y = pose.Pos().Y();
        
        if(abs(y - init_y[kivaId]) >= 1 * abs(dist)){
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
    }
        
    public: bool isCloseToDegrees(double degrees, double val){
        if(degrees <= val+1 && degrees >= val-1)
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



private: void start_rotate(int dir, int numRotations, int kivaId){
        ignition::math::Pose3d pose = this->model->WorldPose();
        this->continued_angl_vel_left = CONTINUED_ANGULAR_VELOCITY_LEFT;
        this->continued_angl_vel_right = CONTINUED_ANGULAR_VELOCITY_RIGHT;
        std::cout<< "kiva will rotate: " << dir << " " << numRotations << "times\n";
        if(isRotating[kivaId] ) this->direction = dir;
        if(direction==0) {
            //left
            std::cout<<"Rotating left\n";
            this->model->SetAngularVel(ignition::math::Vector3d(0, 0, INITIAL_ANGULAR_VELOCITY_LEFT));
            ignition::math::Pose3d pose = this->model->WorldPose();
            float current_yaw = radiansToDegrees(this->get_yaw(pose));
            this->rotation = closestDegree((int)round((90*numRotations) + current_yaw) % 360);
            if(isCloseToDegrees(current_yaw, 270) && (this->rotation == 0 || this->rotation == 360)){
                this->rotation = 359;
            }
            std::cout<<"Current Yaw: "<<current_yaw<<"\n";
        } else if(direction==1) {
            //right
            std::cout<<"Rotating right\n";
            this->model->SetAngularVel(ignition::math::Vector3d(0, 0, INITIAL_ANGULAR_VELOCITY_RIGHT));
            ignition::math::Pose3d pose = this->model->WorldPose();
            float current_yaw = radiansToDegrees(this->get_yaw(pose));
            this->rotation = closestDegree((int)round(current_yaw - (90*numRotations)) % 360);
            if(isCloseToDegrees(current_yaw, 270) && (this->rotation == 0 || this->rotation == 360)){
                this->rotation = 359;
            }
            std::cout<<"Current Yaw: "<<current_yaw<<"\n";
        } else {
            std::cout<<"Invalid Direction Code "<<direction<<"\n";
            this->rotation = 0;
        }
        std::cout<<"Necessary Yaw: "<<this->rotation<<"\n";
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




private: float rotation, continued_angl_vel_left, continued_angl_vel_right;
		
		private: int direction;










        private: physics::ModelPtr model;
	private: bool kiva_loaded = false;
	private: event::ConnectionPtr updateConnection;
	private: physics::WorldPtr world;
	private: sdf::ElementPtr sdf;
	};
	GZ_REGISTER_WORLD_PLUGIN(WarehouseWorldPlugin)
}
