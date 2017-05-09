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
	//Getting Sensors.
	bool use_obstacle_detection = (strlen(*(argv + 3)) == 5) ? false : true;
	bool use_sensors = (strlen(*(argv + 4)) == 5) ? false : true;
	bool use_vcu = (strlen(*(argv + 5)) == 5) ? false : true;
	bool use_gps = (strlen(*(argv + 6)) == 5) ? false : true;
	//Getting launchable programms.
	Programm to_launch; 
	to_launch.fill();
	to_launch.obstacle_detection = (mode && use_obstacle_detection);
	to_launch.pure_pursuit = (mode);
	to_launch.gps = (use_sensors && use_gps);
	to_launch.vcu = (use_vcu);
	to_launch.velodyne = (use_obstacle_detection && use_sensors);
	to_launch.vi = (use_sensors);
	//Rosbags.
	bool rosbag_record = (strlen(*(argv + 7)) == 5) ? false : true;
	bool rosbag_play = (strlen(*(argv + 8)) == 5) ? false : true;
	//Initialising classes.
	info->init(&node, to_launch, mode, name);
	state_estimation->init(info, guard, pure_pursuit);
	vcu_interface->init(info, car_model, guard, ros_interface, rosbag_record);
	car_model->init(info, ros_interface, state_estimation);
	if(mode) pure_pursuit->init(info, guard);
	if(mode) grid_analyser->init(info, guard, pure_pursuit);
	if(mode) obstacle_detection->init(info, grid_analyser);
	if(mode) tracker->init(name);
	guard->init(info, grid_analyser, ros_interface, state_estimation, vcu_interface);
	ros_interface->init(&node, info, tracker, car_model, guard, obstacle_detection, 
						pure_pursuit, state_estimation, vcu_interface, rosbag_play, rosbag_record);
	std::cout << "ARC: Everything initialised !" << std::endl;
	//Looping.
	ros_interface->spinning(rosbag_play);
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


	
