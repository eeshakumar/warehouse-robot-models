#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>


#include <iostream>

gazebo::physics::WorldPtr world;
bool isLifted = false;
int liftedShelf = 0;

void liftObject(double x, double y)
{
    gazebo::physics::ModelPtr storage_unit = world->ModelByName("storage_unit");
    ignition::math::Pose3d pose = storage_unit->WorldPose();
    pose = ignition::math::Pose3d(x, y, 0, 0, 0, 0);
    storage_unit->SetWorldPose(pose);
}

bool isCloseTo(double yaw, double val){
    if(yaw <= val+0.05 && yaw >= val-0.05)
        return true;
    return false;
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
    if (name == std::string("kiva_1"))
    {
      const ::gazebo::msgs::Vector3d &position = pose.position();

        kiva_x = position.x();
        kiva_y = position.y();
        kiva_z = position.z();

    //  std::cout << "Read kiva position: x: " << kiva_x << " y: "
     //   << kiva_y << " z: " << kiva_z << std::endl;
    }
      
    if (name == std::string("storage_unit"))
    {
        const ::gazebo::msgs::Vector3d &position = pose.position();
          
         shelf_x = position.x();
         shelf_y = position.y();
         shelf_z = position.z();
        
  //      std::cout << "Read shelf position: x: " << shelf_x
   //       << " y: " << shelf_y << " z: " << shelf_z << std::endl;
    }
    if(isLifted){
        if(isCloseTo(shelf_x, kiva_x) && isCloseTo(shelf_y, kiva_y) && isLifted ){
              liftObject(kiva_x, kiva_y);
            //  std::cout << "kiva is moving shelf" << "\n";
        }
    }
  }
}
void OnMsg(ConstVector3dPtr &_msg){
    int msg = _msg->x();
    if (msg==0 && isLifted == 1){
        std::cout << "not lifting anymore \n";
        isLifted = 0;
    } else if (msg==1 && isLifted == 0) {
        std::cout << "lifting shelf \n";
        isLifted = 1;
    } else {
        std::cout << "cannot lift shelf \n";
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

     
      // Subscribe to the topic, and register a callback
    gazebo::transport::SubscriberPtr subLift = node->Subscribe("~/lift", OnMsg);
    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose/info", posesStampedCallback);
    gazebo::physics::ModelPtr shelf = world->ModelByName("storage_unit");
    gazebo::physics::ModelPtr kiva_1 = world->ModelByName("kiva_1");
    kiva_1->LoadPlugins();
    /*gazebo::physics::ModelPtr kiva_2 = world->ModelByName("kiva_2");
    kiva_2->LoadPlugins();*/
      
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

