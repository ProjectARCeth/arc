#include <arc/Guard.hpp>

Guard::Guard(){}

Guard::~Guard(){}

void Guard::init(Information* infos, GridAnalyser* grid_analyser,  ROSInterface* ros_interface, 
				 StateEstimation* state_estimation, VCUInterface* vcu_interface){
	//Set class pointer.
	grid_analyser_ = grid_analyser;
	ros_interface_ = ros_interface;
	state_estimation_ = state_estimation;
	vcu_interface_ = vcu_interface;
	//Init paramter.
	infos_ = infos;
	car_ = infos_->getErod();
	safety_ = infos_->getSafety();
	if(infos_->getMode()){
		teach_positions_ = infos_->getTeachPositions();
		teach_orientations_ = infos_->getTeachOrientations();
	}
	to_run_ = infos_->getLaunchables();
	//Starting all programms.
	running_programmes_.empty();
}

bool Guard::initialProgrammCheck(){
	//Ping VCU.
	bool mode = infos_->getMode();
	if(!mode) vcu_interface_->send_msg("cc",0.0,!running_programmes_.vcu);
	else vcu_interface_->send_msg("cc",5.0,!running_programmes_.vcu);
	std::string vcu_type = infos_->getGeneral().vcu_mode;
	if(vcu_type == "street") vcu_interface_->send_msg("ec",1.0,!running_programmes_.vcu);
	else if(vcu_type == "lift") vcu_interface_->send_msg("ec",0.0,!running_programmes_.vcu);
	//Publish running programmes.
	ros_interface_->publishProgramm(running_programmes_, to_run_);
	//Check if everything is initialised.
	return running_programmes_.compare(to_run_,to_run_);
}

void Guard::checkAndSendControlling(AckermannControl should_control){
	//Always send steering command.
	double steering_should_deg = should_control.steering_angle/M_PI*180;
	vcu_interface_->send_msg("ss", steering_should_deg,true,car_.max_steering_angle,-car_.max_steering_angle,10000);
	//Check emergency and send controlling to vcu.
	if(watchdog())
		vcu_interface_->send_msg("vs",should_control.velocity,true, 0.9*car_.max_velocity,-100, 0.0);
}

bool Guard::checkObstacles(){return !grid_analyser_->getObstacleStop();}

bool Guard::checkRunningPrograms(){
	if(running_programmes_.compare(to_run_,to_run_)){
		clock_.start();
		running_programmes_.empty();
	}
	else{
		double down_time = clock_.getTimeFromStart();
		if(down_time >= safety_.down_time/2){
			std::cout << "GUARD: Warning due to programm connection lack" << std::endl;
			ros_interface_->publishProgramm(running_programmes_, to_run_);
		}
		if(down_time >= safety_.down_time){
			std::cout << "GUARD: Failed programm check" << std::endl;
			ros_interface_->publishProgramm(running_programmes_, to_run_);
			return false;
		}
	}
	return true;
}

bool Guard::checkState(State state){
	bool check = true;
	//Check velocity.
	if (state.velocity >= safety_.max_absolute_velocity){
		check = false; 
		std::cout << "GUARD: Velocity too large" << std::endl;
	}
  	//Check tracking error.
  	tracking_error_ = fabs(arc_tools::geometry::globalToLocal(teach_positions_[state.current_index], state)(1));
  	if (tracking_error_>= safety_.max_deviation_from_teach_path){
  		check = false;
  		std::cout << "GUARD: Tracking error too large" << std::endl;
  	}
  	//Check orientation (normal to plane orientation).
  	float current_orientation, path_orientation;
  	current_orientation = arc_tools::geometry::transformEulerQuaternionVector(state.orientation)(2);
  	path_orientation = arc_tools::geometry::transformEulerQuaternionVector(teach_orientations_[state.current_index])(2);
  	orientation_error_ = fabs(current_orientation - path_orientation);
  	if (orientation_error_ >= safety_.max_orientation_divergence){
  		check = false;
  		std::cout << "GUARD: Orientation divergence too large" << std::endl;
  	}
  	return check;
}		

void Guard::emergencyStop(std::string reason){
	emergency_stop_ = true;
	std::cout << "GUARD: Emergency Stop !!! Due to " << reason << std::endl;
	for(int i=0; i<10; ++i) vcu_interface_->send_msg("vs", 0.0, true);
}

bool Guard::watchdog(){
	bool ok = true;
	//Check running programms.
	ok = checkRunningPrograms();
	//Check state.
	if(ok) ok = checkState(state_estimation_->getState());
	//Check obstacles.
	if(to_run_.obstacle_detection && ok) ok = checkObstacles();
	//Check emergency stop.
	if(ok) ok = !emergency_stop_;
	if(!ok) emergencyStop("Watchdog emergency");
	return ok;
}

float Guard::getOrientationError(){return orientation_error_;}

double Guard::getTrackingError(){return tracking_error_;}

Programm Guard::getRunningProgramms(){return running_programmes_;}

void Guard::setRunningStates(bool value, std::string name){
	if (name == "obstacle_detection") running_programmes_.obstacle_detection = value;
	else if(name == "pure_pursuit") running_programmes_.pure_pursuit = value;
	else if(name == "rovio") running_programmes_.rovio = value;
	else if(name == "rslam") running_programmes_.rslam = value;
	else if(name == "state_estimation") running_programmes_.state_estimation = value;
	else if(name == "vcu") running_programmes_.vcu = value;
	else if(name == "gps") running_programmes_.gps = value;
	else if(name == "vi") running_programmes_.vi = value;
	else if(name == "velodyne") running_programmes_.velodyne = value;
	else std::cout << "GUARD: Error in set Running State of " << name << std::endl;
}