#ifndef OBSTACLE_DETECTION_ARC_HPP
#define OBSTACLE_DETECTION_ARC_HPP

#include <arc/GridAnalyser.hpp>
#include <arc/Information_and_Tools.hpp>

#include <iostream>
#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

class GridAnalyser;
class Information;

class DistanceHistogram {
public:
	DistanceHistogram();
	~DistanceHistogram();
	std::vector<std::vector<double> > d_histo_15_;
	std::vector<std::vector<double> > d_histo_13_;
	std::vector<std::vector<double> > d_histo_11_;
	std::vector<std::vector<double> > d_histo_9_;
	std::vector<std::vector<double> > d_histo_7_;
	std::vector<std::vector<double> > d_histo_5_;
	std::vector<std::vector<double> > d_histo_3_;
	std::vector<std::vector<double> >* d_histo_ptr_[7];
};

class ObstacleDetection{
public:
	ObstacleDetection();
	~ObstacleDetection();
	void init(Information* infos, GridAnalyser* grid_analyser);
  	void scan(const sensor_msgs::PointCloud2& cloud_message);
  	void Filter(pcl::PointCloud<pcl::PointXYZ>& filtered_cloud,
              	const pcl::PointCloud<pcl::PointXYZ>& temp_cloud,
              	double* inter_d_ptr);
  	void GridMap(pcl::PointCloud<pcl::PointXYZ>& filtered_cloud,
               	 nav_msgs::OccupancyGrid& grid);
private:
	//Searching parameter.
	double tolerance_factor_;
	double tolerance_m_;
	double y_limit_m_;
	//Class elements.
	Information* infos_;
	GridAnalyser* grid_analyser_;
	//Helper function.
	void conversion_PC2toPCL(const sensor_msgs::PointCloud2& cloud_message,
                           pcl::PointCloud<pcl::PointXYZ>& temp_cloud);
  	void conversion_PCLtoPC2(pcl::PointCloud<pcl::PointXYZ>& filtered_cloud,
                           sensor_msgs::PointCloud2& converted);
  	void histogram_allocation(double d, int j, DistanceHistogram& temp);
};

#endif