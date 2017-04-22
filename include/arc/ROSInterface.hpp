#ifndef ROS_INTERFACE_ARC_HPP
#define ROS_INTERFACE_ARC_HPP

#include <arc/CarModel.hpp>
#include <arc/Guard.hpp>
#include <arc/Information_and_Tools.hpp>
#include <arc/ObstacleDetection.hpp>
#include <arc/PurePursuit.hpp>
#include <arc/StateEstimation.hpp>
#include <arc/VCUInterface.hpp>

#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <map>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

class CarModel;
class Guard;
class Information;
class ObstacleDetection;
class PurePursuit;
class StateEstimation;
class Tracker;
class VCUInterface;

struct Pubs{
	ros::Publisher car_model_velocity;
	ros::Publisher gui_info;
	ros::Publisher programms;
	ros::Publisher repeat_path;
	ros::Publisher steering_angle;
	ros::Publisher teach_path;
	ros::Publisher wheel_rear_left;
	ros::Publisher wheel_rear_right;
};

struct Subs{
	ros::Subscriber cam;
	ros::Subscriber gui;
	ros::Subscriber laser;
	ros::Subscriber rovio;
	ros::Subscriber rslam;
	ros::Subscriber steering_angle;
	ros::Subscriber wheel_rear_left;
	ros::Subscriber wheel_rear_right;
};

class ROSInterface{
public:
	ROSInterface();
	~ROSInterface();
	void init(ros::NodeHandle* node, Information* infos, Tracker* tracker, CarModel* car_model,
			  Guard* guard, ObstacleDetection* obstacle_detection, PurePursuit* pure_pursuit, 
			  StateEstimation* state_estimation,VCUInterface* vcu_interface, 
			  bool rosbag_play, bool rosbag_record);
	void spinning();
	template <class Type> void publish(std::string name, Type value);
	void publishCarModel(Eigen::Vector2d value);
	void publishGUIInfo(State state, AckermannControl controls, double steering_angle, 
						double distance_full, double distance_start, 
						double tracking_error, double obstacle_distance, 
						double velocity_teach);
	void publishGUIInfo(State state, double steering_angle);
	void publishPath(std::vector<Eigen::Vector3d> positions, std::string name);
	void publishProgramm(Programm running, Programm to_run);
	void publishVCUInfos(std::string name, double value);
	void camCallback(const sensor_msgs::Image::ConstPtr& msg);
	void guiCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
	void laserCallback(const sensor_msgs::PointCloud2& msg);
	void rovioCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void rslamCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void steeringCallback(const std_msgs::Float64::ConstPtr& msg);
	void wheelRearLeftCallback(const std_msgs::Float64::ConstPtr& msg);
	void wheelRearRightCallback(const std_msgs::Float64::ConstPtr& msg);
private:
	//ROS node.
	ros::NodeHandle* node_;
	//ROS Topics of publisher and subscriber.
	Pubs pubs_;
	Subs subs_;
	//Mode.
	bool mode_;
	//Class elements.
	Information* infos_;
	Tracker* tracker_;
	CarModel* car_model_;
	Guard* guard_;
	ObstacleDetection* obstacle_detection_;
	PurePursuit* pure_pursuit_;
	StateEstimation* state_estimation_;
	VCUInterface* vcu_interface_;
};

#endif