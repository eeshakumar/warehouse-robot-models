#ifndef _KIVA_PLUGIN_HH_
#define _KIVA_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
    double init_x, init_y;
    double dist = 0;
    class KivaPlugin : public ModelPlugin{
   
        public: KivaPlugin() {}

        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  
        if (_model->GetJointCount() == 0){
            std::cerr << "Invalid joint count, plugin not loaded\n";
            return;
        }
        std::cout << "Plugin loaded\n";
        
        this->model = _model;
        if (_sdf->HasElement("dist"))
            dist = _sdf->Get<double>("dist");
  
        ignition::math::Pose3d pose = this->model->WorldPose();
        init_x = pose.Pos().X();
        init_y = pose.Pos().Y();
        
        this->model->GetJoint("left_front_wheel_hinge")->SetVelocity(0, 2.0);
        this->model->GetJoint("right_front_wheel_hinge")->SetVelocity(0, 2.0);
        this->model->GetJoint("left_back_wheel_hinge")->SetVelocity(0, 2.0);
        this->model->GetJoint("right_back_wheel_hinge")->SetVelocity(0, 2.0);
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                                std::bind(&KivaPlugin::OnUpdate, this));
}
  public: void OnUpdate(){
        ignition::math::Pose3d pose = this->model->WorldPose();
        double x = pose.Pos().X();
      
        if(x - init_x >= 1 * dist){
            this->model->GetJoint("left_front_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("right_front_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("left_back_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("right_back_wheel_hinge")->SetVelocity(0, 0.0);
        }  
      }
      
private: physics::ModelPtr model;
private: event::ConnectionPtr updateConnection;
      
};

GZ_REGISTER_MODEL_PLUGIN(KivaPlugin)
}
#endif
