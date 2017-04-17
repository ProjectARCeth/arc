#include <arc/CarModel.hpp>
#include <arc/GridAnalyser.hpp>
#include <arc/Guard.hpp>
#include <arc/Information_and_Tools.hpp>
#include <arc/ObstacleDetection.hpp>
#include <arc/PurePursuit.hpp>
#include <arc/ROSInterface.hpp>
#include <arc/StateEstimation.hpp>
#include <arc/VCUInterface.hpp>

#include <iostream>

#include <ros/ros.h>

int main(int argc, char** argv) {
	//Init ROS.
	ros::init(argc, argv, "arc");
	ros::NodeHandle node;
	//Declaring classes.
	CarModel *car_model = new CarModel();
	GridAnalyser* grid_analyser = new GridAnalyser();
	Guard* guard = new Guard();
	Information* info = new Information();
	ObstacleDetection* obstacle_detection = new ObstacleDetection();
	PurePursuit* pure_pursuit = new PurePursuit();
	ROSInterface* ros_interface = new ROSInterface();
	StateEstimation* state_estimation = new StateEstimation();
	Tracker* tracker = new Tracker();
	VCUInterface* vcu_interface = new VCUInterface();
	//Getting mode and name.
	std::string name = *(argv + 1);
	bool mode = (strlen(*(argv + 2)) == 5) ? false : true;
	//Getting launchable programms.
	Programm to_launch; 
	to_launch.fill();
	//Mode.
	if(!mode){
		to_launch.obstacle_detection = false;
		to_launch.pure_pursuit = false;
		to_launch.velodyne = false;
	}
	//Obstacle Detection.
	if(strlen(*(argv + 3)) == 5){
		to_launch.obstacle_detection = false;
		to_launch.velodyne = false;
	}
	//Sensors.
	if(strlen(*(argv + 4)) == 5){
		to_launch.gps = false;
		to_launch.vi = false;
		to_launch.velodyne = false;
	}
	//VCU Interface.
	if(strlen(*(argv + 5)) == 5) to_launch.vcu = false;
	//GPS.
	if(strlen(*(argv + 6)) == 5) to_launch.gps = false;
	//Initialising classes.
	info->init(&node, to_launch, mode, name);
	state_estimation->init(info, guard, pure_pursuit);
	vcu_interface->init(info, car_model, guard);
	car_model->init(info, ros_interface, state_estimation);
	if(mode) pure_pursuit->init(info, guard);
	if(mode) grid_analyser->init(info, guard, pure_pursuit);
	if(mode) obstacle_detection->init(info, grid_analyser);
	if(mode) tracker->init(name);
	guard->init(info, grid_analyser, ros_interface, state_estimation, vcu_interface);
	ros_interface->init(&node, info, tracker, car_model, guard, obstacle_detection, 
						pure_pursuit, state_estimation, vcu_interface);
	std::cout << "ARC: Everything initialised !" << std::endl;
	//Looping.
	ros_interface->spinning();
	std::cout << "ARC: Shutting down !" << std::endl;
	//Closing classes.
	delete info;
	delete grid_analyser;
	delete guard;
	delete car_model;
	delete obstacle_detection;
	delete pure_pursuit;
	delete state_estimation;
	delete vcu_interface;
	delete ros_interface;
	delete tracker;
	//Finishing.
   	return 0;
}


	
