#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <iostream>

#define WAREHOUSE_SMALL "../../warehouse_sm.world"
#define WAREHOUSE_MEDIUM "../../warehouse_md.world"
#define WAREHOUSE_LARGE "../../warehouse_lg.world"
#define WAREHOUSE_DEFAULT "../../warehouse_sm.world"

#define SIZE_CODE_SMALL 0
#define SIZE_CODE_MEDIUM 1
#define SIZE_CODE_LARGE 2

std::string get_world_by_size(int size) {
	if (size == SIZE_CODE_SMALL) {
		return WAREHOUSE_SMALL;	
	} else if (size == SIZE_CODE_MEDIUM) {
		return WAREHOUSE_MEDIUM;	
	} else if (size == SIZE_CODE_LARGE) {
		return WAREHOUSE_LARGE;
	} else {
		return WAREHOUSE_DEFAULT;	
	}
}

gazebo::physics::WorldPtr world;
int main(int _argc, char **_argv)
{
  	//Abort if no input is provided.
	int size = -1;
	if (_argc >= 1) {
		size = atoi(_argv[1]);
	} else {
		std::cout<<"Sumtin went wong!!"<<"\n";
		return 1;
	}
	
	std::cout<<"Size is :"<<size<<"\n";

	try {
		// load gazebo server
		std::cout<<"Setting up server\n";
		gazebo::setupServer(_argc, _argv);
		std::cout<<"Server setup complete\n";

		// Load a world
		std::cout<<"Loading the requried world\n";
		world = gazebo::loadWorld(get_world_by_size(size));
		std::cout<<"World loaded\n";

		while (true) {
			gazebo::runWorld(world, 100);
		}

		gazebo::shutdown();
	} catch(gazebo::common::Exception &e) {
		std::cout << "ERROR: " << e << std::endl;
		return 1;
	}

	return 0;
}
