#ifndef GRID_ANALYSER_ARC_HPP
#define GRID_ANALYSER_ARC_HPP

#include <arc/Guard.hpp>
#include <arc/Information_and_Tools.hpp>
#include <arc/PurePursuit.hpp>

#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <vector>

#include <nav_msgs/OccupancyGrid.h>

class Guard;
class Information;
class PurePursuit;

class GridAnalyser{
public:
	GridAnalyser();
	~GridAnalyser();
	void init(Information* infos, Guard* guard, PurePursuit* pure_pursuit);
	void compareGrids();
	void createDangerZone(const nav_msgs::OccupancyGrid grid_map);
	void searchForObstacles();
	void whattodo(const int danger_index);
	bool getObstacleStop();
	void setObstacleMap(const nav_msgs::OccupancyGrid* map);
	void setState(const State state);
private:
	//Grid analyser states.
	bool obstacle_stop_;
	bool jumper_, grid_init_, state_init_;
	int crit_counter_;
	bool use_obstacle_detection_;
	double tracking_error_;
	double obstacle_distance_;
	double emergency_distance_;
	//Grid with inflated Tube around path.
	nav_msgs::OccupancyGrid	tube_map_;
	//Current state and grid map.
	State state_;
	nav_msgs::OccupancyGrid obstacle_map_;
	//Teach path.
	std::vector<Eigen::Vector3d> teach_positions_;
	//Parameters.
	Control control_;
	Erod erod_;
	Safety safety_;
	//Grid map parameters.
	int n_cells_, height_, width_;				
	float resolution_;
	//Class elements.
	Information* infos_;
	Guard* guard_;
	PurePursuit* pure_pursuit_;
	//Helper functions.
	Eigen::Vector2d getGridElement(const int index);
	int getGridElement(Eigen::Vector2d grid_point);
	int gridIndexOfGlobalPoint(const Eigen::Vector3d point);
	void inflate(int index);
	void inflate(int x, int y);
};

#endif