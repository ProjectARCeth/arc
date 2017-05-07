#ifndef INFORMATION_AND_TOOLS_ARC_HPP
#define INFORMATION_AND_TOOLS_ARC_HPP

#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <Eigen/StdVector>
#include <fstream>
#include <iostream>
#include <math.h>
#include <sstream>
#include <string>
#include <sys/time.h>
#include <vector>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>

struct AckermannControl{
	double steering_angle;
	double velocity;
};

struct Erod{
	double distance_front_rear_axis;
	double distance_wheel_axis;
	double distance_vi_rear_axis;
	double distance_laser_rear_axis;
	double height_laser_rear_axis;
	double max_lateral_acceleration;
	double max_steering_angle;
	double max_velocity;
	double mu_haft;
	double total_width;
	double width_wheel_axis;
	double wheel_diameter;
};

struct General{
	int my_port;
	int vcu_port;
	std::string vcu_mode;
};

struct Control{
	double current_array_searching_width;
	double distance_interpolation;
	double k1_lad_laser;
	double k2_lad_laser;
	double k1_lad_s;
	double k2_lad_s;
	double upperbound_lad_s;
	double lowerbound_lad_s;
	double k1_lad_v;
	double k2_lad_v;
	double shut_down_time;
	double slow_down_distance;
	double slow_down_puffer;
	double v_freedom;
	int number_of_emergency_cells;
	double emergency_distance_lb;
	double emergency_distance_ub;
	double obstacle_slow_down_distance;
	double obstacle_puffer_distance;
	double length_correction_path;
};

struct Physics{
	double g_earth;
};

struct Programm{
	bool obstacle_detection;
	bool pure_pursuit;
	bool rovio;
	bool rslam;
	bool state_estimation;
	bool vcu;
	bool gps;
	bool vi;
	bool velodyne;
	bool compare(const Programm &p, Programm to_run);
	void empty();
	void fill();
	void print();
};

struct Safety{
	double critical_obstacle_distance;
	double fos_velocity;
	double fos_dangergrid;
	double fos_braking_distance;
	double max_absolute_velocity;
	double max_deviation_from_teach_path;
	double max_orientation_divergence;
	double min_shutdown_velocity;
	double static_tolerance_laser;
	double factor_tolerance_laser;
	double obstacle_search_width;
	double down_time;
};

struct Sensor{
	double cam_init_quat_x;
	double cam_init_quat_y;
	double cam_init_quat_z;
	double cam_init_quat_w;
};

struct State{
	Eigen::Vector3d position;
	Eigen::Vector4d orientation;
	double velocity;
	int current_index;
	Eigen::Vector3d euler();
};

namespace arc_tools {
namespace geometry{
	Eigen::Vector3d transformEulerQuaternionVector(const Eigen::Vector4d quat);
	Eigen::Matrix3d getRotationMatrix(const Eigen::Vector4d quats);
	Eigen::Vector3d globalToLocal(Eigen::Vector3d global_koordinate, State pose);
	Eigen::Vector3d rotationLocalToGlobal(Eigen::Vector3d local, State pose);
	Eigen::Vector4d multQuaternion(Eigen::Vector4d q1,Eigen::Vector4d q2);
	Eigen::Vector4d inverseQuaternion(Eigen::Vector4d quat);
	Eigen::Vector4d diffQuaternion(Eigen::Vector4d base_quat, Eigen::Vector4d target_quat);
}//namespace geometry.

namespace path{
	double distanceBetween(int base_index,int target_index,std::vector<Eigen::Vector3d> positions);
	int indexOfDistanceFront(int index, float max_distance,std::vector<Eigen::Vector3d> positions);
	int indexOfDistanceBack(int index, float max_distance,std::vector<Eigen::Vector3d> positions);
}

namespace tf_viewer{
	void tfBroadcaster(Eigen::Vector4d quat, Eigen::Vector3d position, std::string from, std::string to);
}//namespace tf_viewer.

class Clock {
public:
	Clock();
	double getTimestep();
	double getTimeFromStart();
	void start();
private:
	double getTime();
	void takeTime();
	struct timeval real_time_start_;
	bool first_step_;
	double last_time_, current_time_, time_step_, real_time_ms_;
	static const double kSecondsToMiliseconds = 1000.0;
	static const double kMicrosecondsToMiliseconds = 0.001;
	static const double kMilisecondsToSeconds = 0.001;
};	

}//namespace arc_tools.

class Information{
public:
	Information();
	~Information();
	void init(const ros::NodeHandle* node, Programm to_launch, 
			  bool mode, std::string name);
	Control getControl();
	Erod getErod();
	General getGeneral();
	Physics getPhysics();
	Sensor getSensor();
	Safety getSafety();
	Programm getLaunchables();
	bool getMode();
	std::string getName();
	std::vector<Eigen::Vector3d> getTeachPositions();
	std::vector<Eigen::Vector4d> getTeachOrientations();
	std::vector<double> getTeachVelocities();
	void setMode(bool mode);
private:
	//Mode and Name.
	bool mode_;
	std::string name_;
	//Parameter vectors.
	Control control_;
	Erod erod_;
	General general_;
	Physics physics_;
	Sensor sensor_;
	Safety safety_;
	Programm to_launch_;
	//Teach path.
	std::vector<Eigen::Vector3d> teach_positions_;
	std::vector<Eigen::Vector4d> teach_orientations_;
	std::vector<double> teach_velocities_;
	//Helper functions.
	void readPathFile(const std::string teach_path_file);
};

class Tracker{
public:
	Tracker();
	~Tracker();
	void init(const std::string name);
	void addControls(AckermannControl control);
	void addDistanceStart(double distance_start);
	void addObstacleDistance(double obstacle_distance);
	void addState(State state, double steering_angle);
	void addTrackingError(double tracking_error);
private:	
	//File name.
	std::string file_name_;
	//Information vectors.
	std::vector<double> x_,y_,z_;
	std::vector<int> index_;
	std::vector<double> steering_angle_, velocity_;
	std::vector<double> should_steering_angle_, should_velocity_;
	std::vector<double> tracking_error_;
	std::vector<double> distance_start_;
	std::vector<double> obstacle_distance_;
	//Helper functions.
	std::string round(double value);
};


#endif