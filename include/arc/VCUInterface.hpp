#ifndef VCUInterface_ARC_HPP
#define VCUInterface_ARC_HPP

#include <arc/CarModel.hpp>
#include <arc/Guard.hpp>
#include <arc/Information_and_Tools.hpp>
#include <arc/ROSInterface.hpp>

#include <arpa/inet.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/socket.h>

#include <ros/ros.h>

#define buflen_ 512 

class CarModel;
class Guard;
class Information;
class ROSInterface;

class VCUInterface{
public:
	VCUInterface();
	~VCUInterface();
	void init(Information* infos, CarModel* car_model, Guard* guard, 
			  ROSInterface* ros_interface, bool rosbag_record_);
	void modeChange(bool mode);
	void send_msg(std::string symbol, double msg, bool requirement);
	void send_msg(std::string symbol, double msg, bool requirement,
				  double max_value, double min_value, double shift);
	void recv_msgs();

private:
	//Network.
	struct sockaddr_in si_me_, si_other_, si_VCU_;
	int sock_;
	socklen_t slen_;
	//First start.
	bool first_autonomous_;
	//Use interface.
	bool use_vcu_;
	//Rosbag.
	bool record_;
	//Class elements.
	CarModel* car_model_;
	Guard* guard_;
	Information* infos_;
	ROSInterface* ros_interface_;
	//Useful functions.
	void printError(std::string error);
};

#endif