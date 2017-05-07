#include <arc/Information_and_Tools.hpp>

Eigen::Vector3d State::euler(){
    Eigen::Vector3d euler;
    Eigen::Vector4d quat = orientation;
    double ysqr = quat(1) * quat(1);
    // roll (x-axis rotation)
    double t0 = +2.0 * (quat(3) * quat(0) + quat(1) * quat(2));
    double t1 = +1.0 - 2.0 * (quat(0) * quat(0) + ysqr);
    euler(0) = std::atan2(t0, t1);
    // pitch (y-axis rotation)
    double t2 = +2.0 * (quat(3) * quat(1) - quat(2) * quat(0));
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    euler(1) = std::asin(t2);
    // yaw (z-axis rotation)
    double t3 = +2.0 * (quat(3) * quat(2) + quat(0) * quat(1));
    double t4 = +1.0 - 2.0 * (ysqr + quat(2) * quat(2));  
    euler(2) = std::atan2(t3, t4);
    return euler;
}

bool Programm::compare(const Programm &p, Programm to_run){
    bool equal = true;
    if(obstacle_detection != p.obstacle_detection && to_run.obstacle_detection) equal = false;
    if(pure_pursuit != p.pure_pursuit && to_run.pure_pursuit) equal = false;
    if(rovio != p.rovio && to_run.rovio) equal = false;
    if(rslam != p.rslam && to_run.rslam) equal = false;
    if(state_estimation != p.state_estimation && to_run.state_estimation) equal = false;
    if(gps != p.gps && to_run.gps) equal = false;
    if(vi != p.vi && to_run.vi) equal = false;
    if(velodyne != p.velodyne && to_run.velodyne) equal = false;
    return equal;
}

void Programm::empty(){
    obstacle_detection = false;
    pure_pursuit = false;
    rovio = false;
    rslam = false;
    state_estimation = false;
    vcu = false;
    gps = false;
    vi = false;
    velodyne = false;
}

void Programm::fill(){
    obstacle_detection = true;
    pure_pursuit = true;
    rovio = true;
    rslam = true;
    state_estimation = true;
    vcu = true;
    gps = true;
    vi = true;
    velodyne = true;
}

void Programm::print(){
    std::cout << std::endl << "-------------------" << std::endl;
    std::cout << "Obstacle Detection: " << obstacle_detection << std::endl;
    std::cout << "Pure Pursuit : " << pure_pursuit << std::endl;
    std::cout << "Rovio: " << rovio << std::endl;
    std::cout << "RSLAM: " << rslam << std::endl;
    std::cout << "State Estimation: " << state_estimation << std::endl;
    std::cout << "VCU: " << vcu << std::endl;
    std::cout << "GPS: " << gps << std::endl;
    std::cout << "VI: " << vi << std::endl;
    std::cout << "velodyne: " << velodyne << std::endl;
}

namespace arc_tools{
namespace geometry{

Eigen::Vector3d transformEulerQuaternionVector(const Eigen::Vector4d quat){
    Eigen::Vector3d euler;
    double ysqr = quat(1) * quat(1);
    // roll (x-axis rotation)
    double t0 = +2.0 * (quat(3) * quat(0) + quat(1) * quat(2));
    double t1 = +1.0 - 2.0 * (quat(0) * quat(0) + ysqr);
    euler(0) = std::atan2(t0, t1);
    // pitch (y-axis rotation)
    double t2 = +2.0 * (quat(3) * quat(1) - quat(2) * quat(0));
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    euler(1) = std::asin(t2);
    // yaw (z-axis rotation)
    double t3 = +2.0 * (quat(3) * quat(2) + quat(0) * quat(1));
    double t4 = +1.0 - 2.0 * (ysqr + quat(2) * quat(2));  
    euler(2) = std::atan2(t3, t4);
    return euler;
}

Eigen::Matrix3d getRotationMatrix(const Eigen::Vector4d quats){	
    double a = quats(3); double b = quats(0);
    double c = quats(1); double d = quats(2);
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix<<(1-2*(c*c + d*d)), 2*(b*c - a*d), 2*(b*d + a*c), 
                    2*(b*c + a*d), (1-2*(d*d + b*b)), 2*(c*d - a*b), 
                    2*(b*d - a*c), 2*(c*d + a*b), (1-2*(b*b + c*c));
    return rotation_matrix;
}

Eigen::Vector3d globalToLocal(Eigen::Vector3d global_koordinate, State pose){
  //Translatation
  Eigen::Vector3d temp = global_koordinate-pose.position;
  //Rotation
  Eigen::Matrix3d R = getRotationMatrix(pose.orientation).transpose();
  Eigen::Vector3d local=R*temp;
  //Change of coordinate frame.
  Eigen::Vector3d local_rotated;
  local_rotated(0) = -local(1);
  local_rotated(1) = local(0);
  local_rotated(2) = local(2);
  return local_rotated;
}

Eigen::Vector3d rotationLocalToGlobal(Eigen::Vector3d local, State pose){
    Eigen::Matrix3d R = getRotationMatrix(pose.orientation);
    Eigen::Vector3d global = R*local;
    return global;
}

Eigen::Vector4d multQuaternion(Eigen::Vector4d q1,Eigen::Vector4d q2){
    Eigen::Vector3d axes1(q1(0), q1(1), q1(2));
    Eigen::Vector3d axes2(q2(0), q2(1), q2(2));
    //Hamiltonian product to get new quaternion.
    Eigen::Vector4d new_quat;
    new_quat(3) = q1(3)*q2(3) + axes1.dot(axes2);
    Eigen::Vector3d new_axes = q1(3)*axes2 + q2(3)*axes1 + axes1.cross(axes2);
    new_quat(0) = new_axes(0);
    new_quat(1) = new_axes(1);
    new_quat(2) = new_axes(2);
    new_quat = new_quat/new_quat.norm();
    return new_quat;
}

Eigen::Vector4d inverseQuaternion(Eigen::Vector4d quat){
    Eigen::Vector4d inverse_quat;
    inverse_quat(0) = -quat(0);
    inverse_quat(1) = -quat(1);
    inverse_quat(2) = -quat(2);
    inverse_quat(3) = quat(3);
    return inverse_quat;
}

Eigen::Vector4d diffQuaternion(Eigen::Vector4d base_quat, Eigen::Vector4d target_quat){
    //Normalization.
    base_quat = base_quat/base_quat.norm();
    target_quat = target_quat/target_quat.norm();
    //Getting difference of unit quaternion.
    Eigen::Vector4d diff_quat = multQuaternion(base_quat, inverseQuaternion(target_quat));
    diff_quat.normalize();
    return diff_quat;
}
}//namespace geometry.

namespace path{

double distanceBetween(int base_index,int target_index,std::vector<Eigen::Vector3d> positions){
    double distance = 0.0;
    for (int i=base_index; i<target_index; ++i){
      if(i>(int)positions.size()-1 || i<0) break;
      distance += (positions[i] - positions[i+1]).norm();
    }
    return distance;
}

int indexOfDistanceFront(int index, float max_distance,std::vector<Eigen::Vector3d> positions){
    double distance = 0.0;
    while(distance<max_distance && index<(int)positions.size()-2){
      distance += distanceBetween(index+1,index,positions);
      index++;
    }
    return index+1;
}

int indexOfDistanceBack(int index, float max_distance,std::vector<Eigen::Vector3d> positions){
    double distance = 0;
    while(distance<max_distance && index>0){
      distance += distanceBetween(index-2,index-1,positions);
      index--;
    }
    return index;
}
}//namespace path.

namespace tf_viewer{

void tfBroadcaster(Eigen::Vector4d quat, Eigen::Vector3d position, std::string from, std::string to){
    // Init static broadcaster.
    static tf::TransformBroadcaster broadcaster;
    //Set orientation and vector.
    tf::Quaternion tf_quat(quat(0), quat(1), quat(2), quat(3));
    tf::Vector3 tf_vector(position(0), position(1), position(2)+0.2);
    //Setting tf - broadcast from odom to rear_axle.
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf_quat, tf_vector),
                           ros::Time::now(), from, to));
}
}//namespace tf_viewer.

Clock::Clock() { start(); first_step_ = true;}

double Clock::getTimestep(){
    if(first_step_){
      last_time_ = getTime();
      first_step_ = false;
    }
    current_time_ = getTime();
    time_step_ = current_time_ - last_time_;
    last_time_ = getTime();
    return time_step_ * kMilisecondsToSeconds;
}

double Clock::getTimeFromStart(){
    if(first_step_){
      last_time_ = getTime();
      first_step_ = false;
    }
    current_time_ = getTime();
    time_step_ = current_time_ - last_time_;
    return time_step_ * kMilisecondsToSeconds;
}

void Clock::start(){ gettimeofday(&real_time_start_, NULL); }

double Clock::getTime() { takeTime(); return real_time_ms_;}

void Clock::takeTime(){
    //Updating cpu Time
    struct timeval end;
    gettimeofday(&end, NULL);
    long seconds, useconds;
    seconds  = end.tv_sec  - real_time_start_.tv_sec;
    useconds = end.tv_usec - real_time_start_.tv_usec;
    real_time_ms_ = (seconds * kSecondsToMiliseconds +
                   useconds * kMicrosecondsToMiliseconds) + 0.5;
}
}//namespace arc_tools.

Information::Information(){}

Information::~Information(){}

void Information::init(const ros::NodeHandle* node, Programm to_launch, bool mode, std::string name){
    //Set mode.
    mode_ = mode;
    name_ = name;
    //Set up erod info.
    node->getParam("/erod/DISTANCE_FRONT_REAR_AXIS", erod_.distance_front_rear_axis);
    node->getParam("/erod/DISTANCE_WHEEL_AXIS", erod_.distance_wheel_axis);
    node->getParam("/erod/DISTANCE_VI_REAR_AXIS", erod_.distance_vi_rear_axis);
    node->getParam("/erod/DISTANCE_LASER_REAR_AXIS", erod_.distance_laser_rear_axis);
    node->getParam("/erod/HEIGHT_LASER_REAR_AXIS", erod_.height_laser_rear_axis);
    node->getParam("/erod/TOTAL_WIDTH", erod_.total_width);
    node->getParam("/erod/WIDTH_WHEEL_AXIS", erod_.width_wheel_axis);
    node->getParam("/erod/MAX_LATERAL_ACCELERATION", erod_.max_lateral_acceleration);
    node->getParam("/erod/MAX_STEERING_ANGLE", erod_.max_steering_angle);
    node->getParam("/erod/MAX_VELOCITY", erod_.max_velocity);
    node->getParam("/erod/MU_HAFT", erod_.mu_haft);
    node->getParam("/erod/WHEEL_DIAMETER", erod_.wheel_diameter);
    //Set up sensor info.
    node->getParam("/sensor/CAM_INIT_QUAT_X", sensor_.cam_init_quat_x);
    node->getParam("/sensor/CAM_INIT_QUAT_Y", sensor_.cam_init_quat_y);
    node->getParam("/sensor/CAM_INIT_QUAT_Z", sensor_.cam_init_quat_z);
    node->getParam("/sensor/CAM_INIT_QUAT_W", sensor_.cam_init_quat_w);
    //Set up general info.
    node->getParam("/general/MY_PORT", general_.my_port);
    node->getParam("/general/VCU_PORT", general_.vcu_port);
    node->getParam("/general/VCU_PARAMETER_MODE", general_.vcu_mode);
    //Set up safety info.
    node->getParam("/safety/CRITICAL_OBSTACLE_DISTANCE", safety_.critical_obstacle_distance);
    node->getParam("/safety/FOS_VELOCITY", safety_.fos_velocity);
    node->getParam("/safety/FOS_DANGERGRID", safety_.fos_dangergrid);
    node->getParam("/safety/FOS_BRAKING_DISTANCE", safety_.fos_braking_distance);
    node->getParam("/safety/MAX_ABSOLUTE_VELOCITY", safety_.max_absolute_velocity);
    node->getParam("/safety/MAX_DEVIATION_FROM_TEACH_PATH", safety_.max_deviation_from_teach_path);
    node->getParam("/safety/MAX_ORIENTATION_DIVERGENCE", safety_.max_orientation_divergence);
    node->getParam("/safety/MIN_SHUTDOWN_VELOCITY", safety_.min_shutdown_velocity);
    node->getParam("/safety/STATIC_TOLERANCE_LASER", safety_.static_tolerance_laser);
    node->getParam("/safety/FACTOR_TOLERANCE_LASER", safety_.factor_tolerance_laser);
    node->getParam("/safety/OBSTACLE_SEARCH_WIDTH", safety_.obstacle_search_width);
    node->getParam("/safety/DOWN_TIME", safety_.down_time);
    //Set up control info.
    node->getParam("/control/CURRENT_ARRAY_SEARCHING_WIDTH", control_.current_array_searching_width);
    node->getParam("/control/DISTANCE_INTERPOLATION", control_.distance_interpolation);
    node->getParam("/control/K1_LAD_LASER", control_.k1_lad_laser);
    node->getParam("/control/K2_LAD_LASER", control_.k2_lad_laser);
    node->getParam("/control/K1_LAD_S", control_.k1_lad_s);
    node->getParam("/control/K2_LAD_S", control_.k2_lad_s);
    node->getParam("/control/K1_LAD_V", control_.k1_lad_v);
    node->getParam("/control/K2_LAD_V", control_.k2_lad_v);
    node->getParam("/control/SHUT_DOWN_TIME", control_.shut_down_time);
    node->getParam("/control/SLOW_DOWN_DISTANCE", control_.slow_down_distance);
    node->getParam("/control/SLOW_DOWN_PUFFER", control_.slow_down_puffer);
    node->getParam("/control/V_FREEDOM", control_.v_freedom);
    node->getParam("/control/NUMBER_OF_EMERGENCY_CELLS", control_.number_of_emergency_cells);
    node->getParam("/control/UPPERBOUND_LAD_S", control_.upperbound_lad_s);
    node->getParam("/control/LOWERBOUND_LAD_S", control_.lowerbound_lad_s);
    node->getParam("/control/EMERGENCY_DISTANCE_LB", control_.emergency_distance_lb);
    node->getParam("/control/EMERGENCY_DISTANCE_UB", control_.emergency_distance_ub);
    node->getParam("/control/OBSTACLE_SLOW_DOWN_DISTANCE", control_.obstacle_slow_down_distance);
    node->getParam("/control/OBSTACLE_PUFFER_DISTANCE", control_.obstacle_puffer_distance);
    node->getParam("/control/LENGHT_CORRECTION_PATH", control_.length_correction_path);
    //Set up physics.
    physics_.g_earth = 9.81;
    //Set up launchable programmes.
    to_launch_ = to_launch;
    //Read teach path.
    if(mode_) readPathFile(name_ + "_teach.txt");
}

void Information::readPathFile(const std::string teach_path_file){
    //Open stream.
    std::fstream fin;
    fin.open(teach_path_file.c_str());
    if(!fin.is_open())
      std::cout<<"INFORMATION: Error with opening of  "<<teach_path_file<<std::endl;
    //Getting file length.
    fin.seekg (-2, fin.end); 
    int length = fin.tellg();
    fin.seekg (0, fin.beg);
    //Copy file.
    char * file = new char [length];
    fin.read (file,length);
    std::istringstream stream_last(file,std::ios::in);
    delete[] file;  
    fin.close () ;
    //Writing path from file.
    int i=0;
    int array_index;  
    while(!stream_last.eof()&& i<length){
      Eigen::Vector3d temp_position;
      Eigen::Vector4d temp_orientation;
      double temp_diff;
      stream_last>>array_index;
      stream_last>>temp_position(0);
      stream_last>>temp_position(1);
      stream_last>>temp_position(2);
      //Reading teach orientation
      stream_last>>temp_orientation(0);
      stream_last>>temp_orientation(1);
      stream_last>>temp_orientation(2);
      stream_last>>temp_orientation(3);
      //Reading teach velocity
      stream_last>>temp_diff;
      teach_positions_.push_back(temp_position);
      teach_orientations_.push_back(temp_orientation);
      teach_velocities_.push_back(temp_diff);
      stream_last.ignore (300, '|');
      i++;  
    }
}

Control Information::getControl(){return control_;}

Erod Information::getErod(){return erod_;}

General Information::getGeneral(){return general_;}

Physics Information::getPhysics(){return physics_;}

Sensor Information::getSensor(){return sensor_;}

Safety Information::getSafety(){return safety_;}

Programm Information::getLaunchables(){return to_launch_;}

bool Information::getMode(){return mode_;}

std::string Information::getName(){return name_;}

std::vector<Eigen::Vector3d> Information::getTeachPositions(){return teach_positions_;}

std::vector<Eigen::Vector4d> Information::getTeachOrientations(){return teach_orientations_;}

std::vector<double> Information::getTeachVelocities(){return teach_velocities_;}

void Information::setMode(bool mode){mode_ = mode;}


Tracker::Tracker(){

}

Tracker::~Tracker(){
    //Create txt. File.
    std::fstream fout;
    fout.open(file_name_.c_str(), std::ios::out | std::ios::trunc);
    if(!fout.is_open())
      std::cout<<"TRACKER: Error with opening of  "<<file_name_<<std::endl;
    //Push information.
    fout << "Dis    In     X      Y      Z      Vel    VeSh   StAn   StSh   TrEr   ObDi   " << std::endl;
    for(int i=0; i<x_.size()-2; ++i){
        fout << round(distance_start_[i]) <<" " << round(index_[i])
             << round(x_[i]) << round(y_[i]) << round(z_[i]) 
             << round(velocity_[i]) << round(should_velocity_[i]) 
             << round(steering_angle_[i]) << round(should_steering_angle_[i]) 
             << round(tracking_error_[i]) << round(obstacle_distance_[i]) << std::endl;
    }
    //Close txt file.
    fout.close();
    std::cout << "ARC: Saved information file to " << file_name_ << std::endl;
}

void Tracker::init(const std::string name){
    //Clear vectors.
    x_.clear(); y_.clear(); z_.clear();
    index_.clear();
    steering_angle_.clear(); velocity_.clear();
    should_steering_angle_.clear(); should_velocity_.clear();
    tracking_error_.clear();
    distance_start_.clear();
    obstacle_distance_.clear();
    //Set name.
    file_name_ = name + "_infos.txt";
}

void Tracker::addControls(AckermannControl control){
    should_steering_angle_.push_back(control.steering_angle);
    should_velocity_.push_back(control.velocity);
}

void Tracker::addDistanceStart(double distance_start){
    distance_start_.push_back(distance_start);
}

void Tracker::addObstacleDistance(double obstacle_distance){
    obstacle_distance_.push_back(obstacle_distance);
}

void Tracker::addState(State state, double steering_angle){
    x_.push_back(state.position[0]);
    y_.push_back(state.position[1]);
    z_.push_back(state.position[2]);
    velocity_.push_back(state.velocity);
    index_.push_back(state.current_index);
    steering_angle_.push_back(steering_angle);
}

void Tracker::addTrackingError(double tracking_error){
    tracking_error_.push_back(tracking_error);
}

std::string Tracker::round(double value){
    value = roundf(value * 100) / 100;
    std::ostringstream stream;
    stream << value;
    std::string buffer = stream.str();
    while(buffer.length()<=6) buffer += " ";
    return buffer.substr (0, 6); 
}