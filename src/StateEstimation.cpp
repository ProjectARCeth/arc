#include <arc/StateEstimation.hpp>

StateEstimation::StateEstimation(){}

StateEstimation::~StateEstimation(){
    std::cout << "ARC: Saved path with " << state_.current_index 
              << " indizes to " << file_name_ << std::endl; 
}

void StateEstimation::init(Information* infos, Guard* guard, PurePursuit* pure_pursuit){
    //Set pure pursuit pointer.
    guard_ = guard;
    pure_pursuit_ = pure_pursuit;
	  //Getting parameters.
	  infos_ = infos;
	  mode_ = infos_->getMode();
	  if(mode_) file_name_ = infos_->getName() + "_repeat.txt";
	  else file_name_ = infos_->getName() + "_teach.txt";
	  control_ = infos_->getControl();
	  Sensor cam = infos_->getSensor();
	  init_quat_ = Eigen::Vector4d(cam.cam_init_quat_x,cam.cam_init_quat_y,
				                         cam.cam_init_quat_z, cam.cam_init_quat_w);
	  trans_vi_rear_axis_ = Eigen::Vector3d(0,-infos_->getErod().distance_vi_rear_axis, 0);
	  //Getting teach path.
	  if(mode_){
      teach_positions_ = infos_->getTeachPositions();
      distance_full_ = arc_tools::path::distanceBetween(0,teach_positions_.size()-1,teach_positions_);
      distance_start_ = 0.0;
    }
    //Clear path vector.
    path_.clear();
	  //Create or empty txt file.
 	  std::ofstream stream(file_name_.c_str());
    stream.close();
}

void StateEstimation::updatePose(Eigen::Vector3d position, Eigen::Vector4d quat){
  	//Update state.
	  state_.position = position;
	  state_.orientation = quat;
	  //Finding array index.
  	if(!mode_) state_.current_index += 1; 
  	if(mode_) state_.current_index = searchCurrentArrayIndex();
  	//Writing path File.
  	std::string filename_all;
  	std::ofstream stream(file_name_.c_str(), std::ios::out|std::ios::app);
  	stream <<state_.current_index<<" "<<
           	position(0)<<" "<<position(1)<<" "<<position(2)<<" "<<
           	quat(0)<<" "<<quat(1)<<" "<<quat(2)<<" "<<quat(3)<<" "<<
           	state_.velocity<<" "<<"|";
  	stream.close();
    //Invoke pure pursuit.
    if(mode_) pure_pursuit_->calculateControls(state_);
    //Update path.
    path_.push_back(state_.position);
    //Resolve guard check.
    guard_->setRunningStates(true, "state_estimation");
}

void StateEstimation::updatePose(Eigen::Vector3d position, Eigen::Vector4d quat, Eigen::Vector4d quat_init_soll){
  	//Rotate orientation.
    Eigen::Vector4d quat_init_trafo;
	  quat_init_trafo = arc_tools::geometry::diffQuaternion(init_quat_, quat_init_soll);
  	state_.orientation = arc_tools::geometry::diffQuaternion(quat_init_trafo, quat);
  	//Translate position to rear axis.
    Eigen::Vector3d trans_vi_rear_global;
  	trans_vi_rear_global = arc_tools::geometry::rotationLocalToGlobal(trans_vi_rear_axis_, state_);
	  state_.position = position + trans_vi_rear_global;
	  //Finding array index.
  	if(!mode_) state_.current_index += 1; 
  	if(mode_) state_.current_index = searchCurrentArrayIndex();
  	//Writing path File.
  	std::string filename_all;
  	std::ofstream stream(file_name_.c_str(), std::ios::out|std::ios::app);
  	stream <<state_.current_index<<" "<<
           	position(0)<<" "<<position(1)<<" "<<position(2)<<" "<<
           	quat(0)<<" "<<quat(1)<<" "<<quat(2)<<" "<<quat(3)<<" "<<
           	state_.velocity<<" "<<"|";
  	stream.close();
    //Invoke pure pursuit.
    if(mode_) pure_pursuit_->calculateControls(state_);
    //Update path.
    path_.push_back(state_.position);
    //Resolve guard check.
    guard_->setRunningStates(true, "state_estimation");
}

void StateEstimation::updateVelocity(double velocity){state_.velocity = velocity;}

int StateEstimation::searchCurrentArrayIndex(){
    //Finding last array position and current state.
    int last_array_position = state_.current_index;
    Eigen::Vector3d last_pose = teach_positions_[last_array_position];
    Eigen::Vector3d pose = state_.position;
    //Finding maximal index so that in maximal width.
    int max_index = 0;
    while((last_pose - teach_positions_[last_array_position+max_index]).norm()
                 < control_.current_array_searching_width){
        max_index += 1;
    }
    //Searching closest point.
    double shortest_distance = 100;
    int smallest_distance_index = last_array_position;
    int lower_searching_bound = std::max(0, last_array_position-max_index);
    int upper_searching_bound = std::min((int)teach_positions_.size(),last_array_position+max_index);
    for (int s = lower_searching_bound; s < upper_searching_bound; s++){
      double current_distance = (pose - teach_positions_[s]).norm();
      if (current_distance < shortest_distance){
        shortest_distance = current_distance;
        smallest_distance_index = s;
      }
    }
    return smallest_distance_index;
}

double StateEstimation::getDistanceFull(){return distance_full_;}

double StateEstimation::getDistanceStart(){
    if(state_.current_index == 0.0) return 0.0;
    int i = state_.current_index;
    double delta_distance = (teach_positions_[i] - teach_positions_[i-1]).norm();
    distance_start_ += delta_distance;
    return distance_start_;
} 

std::vector<Eigen::Vector3d> StateEstimation::getPath(){return path_;}

State StateEstimation::getState(){return state_;}