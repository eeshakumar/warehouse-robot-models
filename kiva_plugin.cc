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
    /// \brief Constructor
        public: KivaPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    // Safety check
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
        //TODO: set velocity as an input
        this->model->GetJoint("left_front_wheel_hinge")->SetVelocity(0, 2.0);
        this->model->GetJoint("right_front_wheel_hinge")->SetVelocity(0, 2.0);
        this->model->GetJoint("left_back_wheel_hinge")->SetVelocity(0, 2.0);
        this->model->GetJoint("right_back_wheel_hinge")->SetVelocity(0, 2.0);
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                                std::bind(&KivaPlugin::OnUpdate, this));
}
  public: void OnUpdate(){
          // TODO: it should be able to move towards both x and y coordinates -/+ values
          // make if clause generic
        ignition::math::Pose3d pose = this->model->WorldPose();
        double x = pose.Pos().X();
        std::cout << x << "\n";
        // Apply a small linear velocity to the model.
        if(x - init_x >= 1 * dist){ // TODO: take x as an input
            this->model->GetJoint("left_front_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("right_front_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("left_back_wheel_hinge")->SetVelocity(0, 0.0);
            this->model->GetJoint("right_back_wheel_hinge")->SetVelocity(0, 0.0);
        }  
      }
      
/// \brief Pointer to the model.
private: physics::ModelPtr model;

/// \brief A PID controller for the joint.
private: common::PID pid;
private: event::ConnectionPtr updateConnection;
      
};


  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(KivaPlugin)
}
#endif
