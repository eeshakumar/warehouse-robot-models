#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <iostream>
#include <iterator> 
#include <map> 

gazebo::physics::WorldPtr world;
bool* isLifted; // boolean array to see if a certain shelf is being carried or not
std::map<int, int> liftingMap; // maps kiva ids to shelf ids to see which kiva is carrying which shelf
bool kivaFlag, shelfFlag = false; // used to check if the read value is from the kiva that is being searched by the iterator

void liftObject(double x, double y, int kivaId)
{
    gazebo::physics::ModelPtr storage_unit = world->ModelByName("storage_unit_"+std::to_string(liftingMap[kivaId]));
    ignition::math::Pose3d pose = storage_unit->WorldPose();
    pose = ignition::math::Pose3d(x, y, 0, 0, 0, 0);
    storage_unit->SetWorldPose(pose);
}

bool isCloseTo(double loc, double val){
    if(loc <= val+0.5 && loc >= val-0.5)
        return true;
    return false;
}

void posesStampedCallback(ConstPosesStampedPtr &posesStamped)
{
  ::google::protobuf::int32 sec = posesStamped->time().sec();
  ::google::protobuf::int32 nsec = posesStamped->time().nsec();
 // std::cout << "Read time: sec: " << sec << " nsec: " << nsec << std::endl;

  for (int i = 0; i < posesStamped->pose_size(); ++i)
  {
    const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
    std::string name = pose.name();
   // std::cout << "name: "  << name << std::endl; 
    double kiva_x, kiva_y, kiva_z, shelf_x, shelf_y, shelf_z;
    std::map<int, int>::iterator itr;
   // std::cout << "iterator position:" << liftingMap.begin() << std::endl; 
    for (itr = liftingMap.begin(); itr != liftingMap.end(); ++itr) {  
      // std::cout << "mapping " << itr->first << "to " << itr->second << std::endl; 
      if (name.compare(std::string("kiva_"+std::to_string(itr->first)))==0)
      {
        const ::gazebo::msgs::Vector3d &position = pose.position();
          kiva_x = position.x();
          kiva_y = position.y();
          kiva_z = position.z();
    
          kivaFlag = true;
      //  std::cout << "Read kiva position: x: " << kiva_x << " y: "
      //   << kiva_y << " z: " << kiva_z << std::endl;
      }
    
      if (name.compare(std::string("storage_unit_"+std::to_string(itr->second)))==0)
      {
       const ::gazebo::msgs::Vector3d &position = pose.position();
          shelf_x = position.x();
          shelf_y = position.y();
          shelf_z = position.z();
          shelfFlag = true;
      //   std::cout << "Read shelf position: x: " << shelf_x
      //     << " y: " << shelf_y << " z: " << shelf_z << std::endl;
      }
      if(kivaFlag && shelfFlag){
        if(isLifted[(itr->second)-1]==1){
          if(isCloseTo(shelf_x, kiva_x) && isCloseTo(shelf_y, kiva_y)){
              liftObject(kiva_x, kiva_y, itr->first);
              // std::cout << "kiva is moving shelf" << "\n";
            }
        }
        kivaFlag = false;
        shelfFlag = false;
      }
    }
  }
}
void OnMsg(ConstVector3dPtr &_msg){
    int msg = _msg->x();
    int kivaId = _msg->y();
    int shelfId = _msg->z();
    std::cout << "shelf id "<< shelfId <<"\n";
    std::cout << "shelf is lifted "<< isLifted[shelfId-1] <<"\n";
    if (msg==0 && isLifted[shelfId-1]){
        std::cout << "not lifting anymore \n";
        isLifted[shelfId-1] = 0;
        liftingMap.erase(kivaId);
    } else if (msg==1 && isLifted[shelfId-1]==0) {
        std::cout << "lifting shelf \n";
        isLifted[shelfId-1] = 1;
        liftingMap.insert(std::pair<int, int>(kivaId, shelfId));
    } else {
        std::cout << "cannot lift shelf \n";
    }
}

void OnCollision(ConstLaserScanStampedPtr &_msg){
  ::google::protobuf::int32 sec = _msg->time().sec();
  ::google::protobuf::int32 nsec = _msg->time().nsec();
  // std::cout << "Read time: sec: " << sec << " nsec: " << nsec << std::endl;

    ::gazebo::msgs::LaserScan scan = _msg->scan();
    for(int i=0; i< scan.count(); i++){
      if(scan.ranges(i)<0.4)
        std::cout << "Scan: " << scan.ranges(i) << std::endl;
    } 
}

int main(int _argc, char **_argv)
{
  std::string str = "warehouse.world";
  if (_argc == 3)
  {
    str = _argv[1];
    int numShelves = atoi(_argv[2]);
    isLifted = new bool[numShelves];  
    for (int i=0; i<numShelves; i++){
      isLifted[i] = false;
    }
    for (int i=0; i<numShelves; i++){
       std::cout << "isLifted " << isLifted[i] << "\n";
    }
  } else {
    isLifted = new bool[1];
    isLifted[0] = false;
  }

  try
  {
    gazebo::setupServer(_argc, _argv);
    world = gazebo::loadWorld(str);
    gazebo::sensors::run_once(true);
    gazebo::sensors::run_threads();

    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    
    std::vector<gazebo::physics::ModelPtr> models = world->Models();
    gazebo::transport::SubscriberPtr subLift = node->Subscribe("~/lift", OnMsg);
    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose/info", posesStampedCallback);
    gazebo::physics::ModelPtr kiva_1 = world->ModelByName("kiva_1");
    kiva_1->LoadPlugins();
    gazebo::physics::ModelPtr kiva_2 = world->ModelByName("kiva_2");
    kiva_2->LoadPlugins(); 
    
    while (true)
    {
        gazebo::runWorld(world, 100);
        gazebo::sensors::run_once();
        gazebo::common::Time::MSleep(1);
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

