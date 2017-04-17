#ifndef GUARD_ARC_HPP
#define GUARD_ARC_HPP

#include <arc/GridAnalyser.hpp>
#include <arc/Information_and_Tools.hpp>
#include <arc/ROSInterface.hpp>
#include <arc/StateEstimation.hpp>
#include <arc/VCUInterface.hpp>

#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>

class GridAnalyser;
class Information;
class ROSInterface;
class StateEstimation;
class VCUInterface;

class Guard{
public:
	Guard();
	~Guard();
	void init(Information* infos, GridAnalyser* grid_analyser, ROSInterface* ros_interface,
			  StateEstimation* state_estimation, VCUInterface* vcu_interface);
	bool initialProgrammCheck();
	void checkAndSendControlling(AckermannControl should_control);
	bool checkObstacles();
	bool checkRunningPrograms();
	bool checkState(State state);		
	void emergencyStop(std::string reason);
	bool watchdog();
	Programm getRunningProgramms();
	float getOrientationError();
	double getTrackingError();
	void setRunningStates(bool value, std::string name);
private:
	//Emergency stop state.
	bool emergency_stop_;
	//Tracking error.
	float orientation_error_;
	double tracking_error_;
	//Init and guarding programmes.
	Programm to_run_;
	Programm running_programmes_;
	//Class elements.
	arc_tools::Clock clock_;
	Information* infos_;
	GridAnalyser* grid_analyser_;
	ROSInterface* ros_interface_;
	StateEstimation* state_estimation_;
	VCUInterface* vcu_interface_;
	//Other information.
	Erod car_;
	Safety safety_;
	std::vector<Eigen::Vector3d> teach_positions_;
	std::vector<Eigen::Vector4d> teach_orientations_;
};

#endif