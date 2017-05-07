#ifndef CAR_MODEL_ARC_HPP
#define CAR_MODEL_ARC_HPP

#include <arc/Guard.hpp>
#include <arc/Information_and_Tools.hpp>
#include <arc/ROSInterface.hpp>
#include <arc/StateEstimation.hpp>

#include "Eigen/Dense"
#include <iostream>
#include <math.h>

#include <ros/ros.h>

class Guard;
class Information;
class ROSInterface;
class StateEstimation;

class CarModel{
public:
	CarModel();
	~CarModel();
	void init(Information* infos, ROSInterface* ros_interface, 
			  StateEstimation* state_estimation);	
	void updateModel();
	double getSteeringAngle();
	Eigen::Vector2d getVelocity();
	void setSteeringAngle(double steering_angle);
	void setRearLeftWheelVel(double vel);
	void setRearRightWheelVel(double vel);
	void setTimeStamp(ros::Time timestamp);
private:
	Eigen::Vector2d local_velocity_;
	//Current measurements.
	double steering_angle_;
	double velocity_rear_left_;
	double velocity_rear_right_;
	ros::Time timestamp_;
	//Parameter.
	float distance_rear_front_axis_;
	float width_axis_;
	//Class elements.
	Information* infos_;
	ROSInterface* ros_interface_;
	StateEstimation* state_estimation_;
	//Init bool.
	bool _first_;
};

#endif