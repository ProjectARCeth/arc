#include <arc/ROSInterface.hpp>

ROSInterface::ROSInterface(){}

ROSInterface::~ROSInterface(){}

void ROSInterface::init(ros::NodeHandle* node, Information* infos, Tracker* tracker, CarModel* car_model,
			  			Guard* guard, ObstacleDetection* obstacle_detection,
			  			PurePursuit* pure_pursuit, StateEstimation* state_estimation,VCUInterface* vcu_interface){
	//Set class pointer.
	tracker_ = tracker;
	car_model_ = car_model;
	guard_ = guard;
	obstacle_detection_ = obstacle_detection;
	pure_pursuit_ = pure_pursuit;
	state_estimation_ = state_estimation;
	vcu_interface_ = vcu_interface;
	//Set node pointer.
	node_ = node;
	//Get parameter.
	infos_ = infos;
	mode_ = infos_->getMode();
	//Init publisher.
	pubs_.car_model_velocity = node_->advertise<geometry_msgs::TwistStamped>("car_model_velocity",10);
	pubs_.gui_info = node_->advertise<std_msgs::Float64MultiArray>("gui/data", 10);
	pubs_.programms = node_->advertise<std_msgs::Int32MultiArray>("programms", 10);
	pubs_.repeat_path = node_->advertise<nav_msgs::Path>("path", 10);
	pubs_.teach_path = node_->advertise<nav_msgs::Path>("teach_path", 10);
	//Init subscriber.
	subs_.cam_sub = node_->subscribe("/cam0/image_raw", 10, &ROSInterface::camCallback, this);
	subs_.gui_sub = node_->subscribe("/gui/commands", 10, &ROSInterface::guiCallback, this);
	if(mode_) subs_.laser_sub = node_->subscribe("/velodyne_points", 10, &ROSInterface::laserCallback, this);
	subs_.rovio_sub = node_->subscribe("/rovio/odometry", 10, &ROSInterface::rovioCallback, this);
	subs_.rslam_sub = node_->subscribe("/orb_slam2/odometry",10,&ROSInterface::rslamCallback, this);
	//Initial spinning.
	ros::Rate rate(10);
	while(!guard_->initialProgrammCheck()){
		ros::spinOnce();
		vcu_interface_->recv_msgs();
		//Publish teach path.
		if(mode_) publishPath(infos_->getTeachPositions(), "teach");
		rate.sleep();
	}
}	

void ROSInterface::spinning(){
	ros::Rate rate(10);
	int gui_counter = 0;
	int tracker_counter = 0;
	while(ros::ok()){
		ros::spinOnce();
		//Getting vcu infos.
		vcu_interface_->recv_msgs();
		//Getting current states.
		State state = state_estimation_->getState();
		double steering_angle = car_model_->getSteeringAngle();
		if(mode_){
			AckermannControl controls = pure_pursuit_->getControls();
			double obstacle_distance = pure_pursuit_->getObstacleDistance();
			double distance_full = state_estimation_->getDistanceFull();
			double distance_start = state_estimation_->getDistanceStart();
			double tracking_error = guard_->getTrackingError();
			double velocity_teach = pure_pursuit_->getTeachVelocity(state.current_index);
			//GUI Info.
			gui_counter++;
			if(gui_counter >= 5){
				gui_counter = 0;
				publishGUIInfo(state, controls, steering_angle, distance_full, distance_start, 
						   	   tracking_error, obstacle_distance, velocity_teach);
			}
			//Tracking.
			tracker_counter++;
			if(tracker_counter >= 25){
				tracker_counter = 0;
				tracker_->addControls(controls);
				tracker_->addDistanceStart(distance_start); 
				tracker_->addObstacleDistance(obstacle_distance);
				tracker_->addState(state,steering_angle);
				tracker_->addTrackingError(tracking_error);
			}
		}
		else{
			//GUI Info.
			gui_counter++;
			if(gui_counter >= 5 && !mode_){
				gui_counter = 0;
				publishGUIInfo(state, steering_angle);
			}
		}
		//Update GUI Path.
		publishPath(state_estimation_->getPath(), "current");
		rate.sleep();
	}
}

template <class Type> void ROSInterface::publish(std::string name, Type value){
	ros::Publisher pub = node_->advertise<std_msgs::Float64>(name, 10);
	std_msgs::Float64 msg;
	msg.data = value;
	pub.publish(msg);
}

void ROSInterface::publishCarModel(Eigen::Vector2d value){
	geometry_msgs::TwistStamped msg;
	msg.twist.linear.x = value(0);
    msg.twist.linear.y  = value(1);
	msg.twist.linear.z = 0;
	pubs_.car_model_velocity.publish(msg);
}

void ROSInterface::publishGUIInfo(State state, AckermannControl controls, double steering_angle, 
								  double distance_full, double distance_start, 
								  double tracking_error, double obstacle_distance,
								  double velocity_teach){
	//Create FloatMultiArray.
	std_msgs::Float64MultiArray data;
	for(int i=0; i<=10; ++i) data.data.push_back(0);
	//Fill information.
	data.data[0] = state.velocity;
	data.data[1] = controls.velocity;
	data.data[2] = steering_angle;
	data.data[3] = controls.steering_angle;
	data.data[4] = state.current_index;
	data.data[5] = tracking_error;
	data.data[6] = obstacle_distance;
	data.data[7] = distance_start;
	data.data[8] = distance_full - distance_start;
	data.data[9] = velocity_teach;
	//Publish array.
	pubs_.gui_info.publish(data);
}

void ROSInterface::publishGUIInfo(State state, double steering_angle){
	//Create FloatMultiArray.
	std_msgs::Float64MultiArray data;
	for(int i=0; i<=10; ++i) data.data.push_back(0);
	//Fill information.
	data.data[0] = state.velocity;
	data.data[2] = steering_angle;
	data.data[4] = state.current_index;
	//Publish array.
	pubs_.gui_info.publish(data);
}

void ROSInterface::publishPath(std::vector<Eigen::Vector3d> positions, std::string name){
	//Create path msg.
	nav_msgs::Path path;
	//Push data.
	for(int i=0; i<positions.size()-1;++i){
		geometry_msgs::PoseStamped temp_pose;
		temp_pose.pose.position.x = positions[i](0);
		temp_pose.pose.position.y = positions[i](1);
		temp_pose.pose.position.z = positions[i](2);
		path.poses.push_back(temp_pose);
	}
	if(name == "current"){
		path.header.frame_id = "repeat_path";
		pubs_.repeat_path.publish(path);
	}
	else if(name == "teach"){
		path.header.frame_id = "teach_path";
		pubs_.teach_path.publish(path);
	}
	else std::cout << "ROSInterface: Error in publishing " << name << " path " << std::endl;
}

void ROSInterface::publishProgramm(Programm running, Programm to_run){
	std_msgs::Int32MultiArray array;
	for (int i = 0; i < 10; i++) array.data.push_back(-1);
	if(to_run.obstacle_detection) array.data[0] = (int)running.obstacle_detection;
	if(to_run.pure_pursuit) array.data[1] = (int)running.pure_pursuit;
	if(to_run.rovio) array.data[2] = (int)running.rovio;
	if(to_run.rslam) array.data[3] = (int)running.rslam;
	if(to_run.state_estimation) array.data[4] = (int)running.state_estimation;
	if(to_run.vcu) array.data[5] = (int)running.vcu;
	if(to_run.gps) array.data[6] = (int)running.gps;
	if(to_run.vi) array.data[7] = (int)running.vi;
	if(to_run.velodyne) array.data[8] = (int)running.velodyne;
	pubs_.programms.publish(array);
}

void ROSInterface::camCallback(const sensor_msgs::Image::ConstPtr& msg){
	//Resolve guard check.
	guard_->setRunningStates(true, "vi");	
}

void ROSInterface::guiCallback(const std_msgs::Int32MultiArray::ConstPtr& msg){
	//Convert to c++ array (start, shutdown, stop).
	bool array[3];
	int i = 0;
	for(std::vector<int>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it){
		array[i] = (bool)*it;
		i++;
	}
	//Reaction.
	vcu_interface_->modeChange(array[0]);
	if(array[1]) pure_pursuit_->setShutDown(true);
	if(array[2]) guard_->emergencyStop("GUI");
}

void ROSInterface::laserCallback(const sensor_msgs::PointCloud2& msg){
	obstacle_detection_->scan(msg);
	//Resolve guard check.
	guard_->setRunningStates(true, "velodyne");
}

void ROSInterface::rovioCallback(const nav_msgs::Odometry::ConstPtr& msg){
	//Resolve guard check.
	guard_->setRunningStates(true, "rovio");
}

void ROSInterface::rslamCallback(const nav_msgs::Odometry::ConstPtr& msg){
	geometry_msgs::Pose pose = msg->pose.pose;
	Eigen::Vector3d position(pose.position.x,pose.position.y,pose.position.z);
	Eigen::Vector4d orientation(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
	Eigen::Vector4d quat_soll(0,0,0,1);
	state_estimation_->updatePose(position,orientation,quat_soll);
	//Resolve guard check.
	guard_->setRunningStates(true, "rslam");
}
