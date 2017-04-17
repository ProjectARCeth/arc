#ifndef PURE_PURSUIT_ARC_HPP
#define PURE_PURSUIT_ARC_HPP

#include <arc/Information_and_Tools.hpp>
#include <arc/Guard.hpp>

#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <vector>

class Guard;
class Information;

class PurePursuit{
public:
	PurePursuit();
	~PurePursuit();
	void init(Information* infos, Guard* guard);
	void calculateControls(State state);
	double calculateSteering(State state);
	double calculateVel(State state);
	AckermannControl getControls();
	double getObstacleDistance();
	double getTeachVelocity(int index);
	void setObstacleDistance(double distance);
	void setShutDown(bool shut_down);
private:
	//Current controls.
	AckermannControl should_controls_;
	//Safety controls.
	double obstacle_distance_;
	bool shut_down_;
	//Time
	arc_tools::Clock BigBen_;
	//Parameter.
	Control control_;
	Erod erod_;
	Safety safety_;
	//Teach path.
	std::vector<Eigen::Vector3d> teachs_;
	std::vector<double> teach_velocities_;
	int slow_down_index_;
	//Class elements.
	Guard* guard_;
	Information* infos_;
	//Helper functions.
	double curveRadius(int index);
};

#endif