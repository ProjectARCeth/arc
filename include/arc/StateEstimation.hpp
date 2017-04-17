#ifndef STATE_ESTIMATION_ARC_HPP
#define STATE_ESTIMATION_ARC_HPP

#include <arc/Guard.hpp>
#include <arc/Information_and_Tools.hpp>
#include <arc/PurePursuit.hpp>

#include <eigen3/Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <string>

class Guard;
class Information;
class PurePursuit;

class StateEstimation{
public:
	StateEstimation();
	~StateEstimation();
	void init(Information* infos, Guard* guard, PurePursuit* pure_pursuit);
	void updatePose(Eigen::Vector3d position, Eigen::Vector4d quat);
	void updatePose(Eigen::Vector3d position, Eigen::Vector4d quat, Eigen::Vector4d quat_soll);
	void updateVelocity(double velocity);
	int searchCurrentArrayIndex();
	double getDistanceFull();
	double getDistanceStart();
	std::vector<Eigen::Vector3d> getPath();
	State getState();
private:
	State state_;
	//Name and mode.
	bool mode_;
	std::string file_name_;
	//Teach path.
	std::vector<Eigen::Vector3d> path_;
	std::vector<Eigen::Vector3d> teach_positions_;
	double distance_full_, distance_start_;
	//Parameter.
	Control control_;
	Eigen::Vector4d init_quat_;
	Eigen::Vector3d trans_vi_rear_axis_;
	//Class elements.
	Information* infos_;
	Guard* guard_;
	PurePursuit* pure_pursuit_;
};

#endif