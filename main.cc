#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <iostream>

gazebo::physics::WorldPtr world;

void liftObject(double x, double y)
{
    gazebo::physics::ModelPtr shelf = world->ModelByName("shelf");
    ignition::math::Pose3d pose = shelf->WorldPose();
    pose = ignition::math::Pose3d(x, y, 0, 0, 0, 0);
    shelf->SetWorldPose(pose);
}


void posesStampedCallback(ConstPosesStampedPtr &posesStamped)
{
 // std::cout << posesStamped->DebugString();

  ::google::protobuf::int32 sec = posesStamped->time().sec();
  ::google::protobuf::int32 nsec = posesStamped->time().nsec();
 // std::cout << "Read time: sec: " << sec << " nsec: " << nsec << std::endl;

  for (int i = 0; i < posesStamped->pose_size(); ++i)
  {
    const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
    std::string name = pose.name();
      double kiva_x, kiva_y, kiva_z, shelf_x, shelf_y, shelf_z;
    if (name == std::string("kiva"))
    {
      const ::gazebo::msgs::Vector3d &position = pose.position();

        kiva_x = position.x();
        kiva_y = position.y();
        kiva_z = position.z();

      std::cout << "Read kiva position: x: " << kiva_x
          << " y: " << kiva_y << " z: " << kiva_z << std::endl;
    }
      
    if (name == std::string("shelf"))
    {
        const ::gazebo::msgs::Vector3d &position = pose.position();
          
         shelf_x = position.x();
         shelf_y = position.y();
         shelf_z = position.z();
          
        std::cout << "Read shelf position: x: " << shelf_x
          << " y: " << shelf_y << " z: " << shelf_z << std::endl;
    }
      gazebo::physics::ModelPtr shelf = world->ModelByName("shelf");
      ignition::math::Pose3d newPose = shelf->WorldPose();
      newPose = ignition::math::Pose3d(kiva_x, kiva_y, 0, 0, 0, 0);
      shelf->SetWorldPose(newPose);
    /*if((kiva_x - shelf_x) < 0.00001){
          liftObject(kiva_x, kiva_y);
          std::cout << "kiva can move object" << (kiva_x - shelf_x) << "\n";
    }*/
  }
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  std::string str = "warehouse.world";
  if (_argc > 1)
  {
    str = _argv[1];
  }

  try
  {
    // load gazebo server
    gazebo::setupServer(_argc, _argv);

    // Load a world
    world = gazebo::loadWorld(str);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    
    std::vector<gazebo::physics::ModelPtr> models = world->Models();

    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose/info", posesStampedCallback);
    gazebo::physics::ModelPtr shelf = world->ModelByName("shelf");
    gazebo::physics::ModelPtr kiva = world->ModelByName("kiva");
    kiva->LoadPlugins();
    
    while (true)
    {
        gazebo::runWorld(world, 100);
    }

    gazebo::shutdown();
  }
    
  catch(gazebo::common::Exception &e)
  {
    std::cerr << "ERROR: " << e << std::endl;
    return 1;
  }

  return 0;
}
