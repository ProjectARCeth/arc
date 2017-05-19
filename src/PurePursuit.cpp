#include <arc/PurePursuit.hpp>

PurePursuit::PurePursuit(){}

PurePursuit::~PurePursuit(){}

void PurePursuit::init(Information* infos, Guard* guard){
	//Set guard pointer.
	guard_ = guard;
	//Getting parameter and teach infos.
	infos_ = infos;
	control_ = infos_->getControl();
	erod_ = infos_->getErod();
	safety_ = infos_->getSafety();
	teachs_ = infos_->getTeachPositions();
	teach_velocities_ = infos_->getTeachVelocities();
	//Init safety controls.
	obstacle_distance_ = 100.0;
	shut_down_ = false;
	//Calculate slow down index.
	double remaining_distance = 0.0;
	slow_down_index_ = (int)teachs_.size()-1;
	while(remaining_distance<control_.slow_down_distance){
		remaining_distance += arc_tools::path::distanceBetween(slow_down_index_-1,slow_down_index_,teachs_);
		slow_down_index_--;
	}
}

void PurePursuit::calculateControls(State state){
	should_controls_.steering_angle = calculateSteering(state);
	should_controls_.velocity = calculateVel(state);
	guard_->checkAndSendControlling(should_controls_);
	guard_->setRunningStates(true,"pure_pursuit");
}

double PurePursuit::calculateSteering(State state){
	//Empirical linear function to determine the look-ahead-distance.
	double lad = control_.k2_lad_s + control_.k1_lad_s*state.velocity;
	lad = std::max(lad,control_.lowerbound_lad_s);
	lad = std::min(lad,control_.upperbound_lad_s);
	//Calculate reference steering index.
	int ref_index = arc_tools::path::indexOfDistanceFront(state.current_index, lad,teachs_);
	//In path.
	double steering_angle;
	if(ref_index < teachs_.size()){
		//Get short position.
		double distance_short;
		distance_short = arc_tools::path::distanceBetween(state.current_index, ref_index-1,teachs_);
		Eigen::Vector3d short_point = teachs_[ref_index-1];
		//Get long position.
		double distance_long;
		distance_long = arc_tools::path::distanceBetween(state.current_index, ref_index,teachs_);
		Eigen::Vector3d long_point = teachs_[ref_index];
		//Interpolation.
		Eigen::Vector3d interpolated_point;
		interpolated_point = linearInterpolation(short_point,long_point,distance_short, distance_long, lad);
		//Calculate slope.
		Eigen::Vector3d local_vector;
		local_vector = arc_tools::geometry::globalToLocal(interpolated_point, state);
		float dy = local_vector(1);
		float dx = local_vector(0);
		float alpha = atan2(dy,dx);
		//Calculate steering angle using Pure Pursuit.
		steering_angle = atan2(2*erod_.distance_wheel_axis*sin(alpha),lad);
	}
	//End of path.
	else steering_angle = 0.0;
	return steering_angle;
}

double PurePursuit::calculateVel(State state){
	//First calculate optimal velocity
	//for the moment take curvature at fix distance lad_v
	double lad = control_.k2_lad_v + control_.k1_lad_s*state.velocity;
	//Calculate reference curvature index.
	int ref_index = arc_tools::path::indexOfDistanceFront(state.current_index, lad,teachs_);
	//Find upper velocity limits (physical, safety and teach).	
	double v_bounded = sqrt(erod_.max_lateral_acceleration*curveRadius(ref_index));
	v_bounded = std::min(v_bounded, safety_.max_absolute_velocity);
	v_bounded = std::min(v_bounded, teach_velocities_[state.current_index]+control_.v_freedom);
    //Penalisations (static and dynamic with tracking error).
	double tracking_error = fabs(arc_tools::geometry::globalToLocal(teachs_[state.current_index], state)(1));
	float c = safety_.fos_velocity/(1+tracking_error);
	//End slow down (gradually when arrive at SLOW_DOWN_DISTANCE from end of of path).
	if (state.current_index>=slow_down_index_){
		double distance_to_end, puffer;
		distance_to_end = arc_tools::path::distanceBetween(state.current_index,teachs_.size()-1,teachs_);
		puffer = control_.slow_down_puffer;
		std::cout<<"PURE PURSUIT: Slownig down. Distance to end: "<<distance_to_end<<std::endl;
		c *= (distance_to_end-puffer)/(control_.slow_down_distance-puffer);
	}
	//GUI shutdown.
	if(shut_down_ && BigBen_.getTimeFromStart()<=control_.shut_down_time){
		std::cout<<"PURE PURSUIT: Shutting down gradually"<<std::endl;
		c *= cos(BigBen_.getTimeFromStart()* M_PI/2 /(control_.shut_down_time));	
	}
	else if (shut_down_ && BigBen_.getTimeFromStart()>control_.shut_down_time){
		std::cout<<"PURE PURSUIT: Shutted down"<<std::endl;
		c = 0;
	}
	//Obstacle slow down.
	if(obstacle_distance_ < control_.obstacle_slow_down_distance){
		std::cout<<" PURE PURSUIT: Slow down for obstacle"<<std::endl;
		double puffer = control_.obstacle_puffer_distance;
		double slow_down_dis = control_.obstacle_slow_down_distance;
		obstacle_distance_ = std::max(obstacle_distance_,slow_down_dis);
		obstacle_distance_ = std::min(obstacle_distance_,puffer);
		c *= (obstacle_distance_-puffer)/(slow_down_dis - puffer);
	}
	//Calculate control.
	double velocity = v_bounded * c;
	return velocity;
}

AckermannControl PurePursuit::getControls(){return should_controls_;}

double PurePursuit::getObstacleDistance(){return obstacle_distance_;}

double PurePursuit::getTeachVelocity(int index){return teach_velocities_[index];}

double PurePursuit::curveRadius(int index){
	int count=0;
	double radius_sum = 0.0;
	for(int t=1;t<=3;t++){	
		count++;
		int i = index;
		float D = control_.distance_interpolation/t;
		int n_front=arc_tools::path::indexOfDistanceFront(i-1,D,teachs_);
		int n_back=arc_tools::path::indexOfDistanceBack(i-1,D,teachs_);
		//Path beginning.
		if(n_back <= 0){
			i=arc_tools::path::indexOfDistanceFront(0,D,teachs_);
			n_front=arc_tools::path::indexOfDistanceFront(i-1,D,teachs_);
			n_back=arc_tools::path::indexOfDistanceBack(i-1,D,teachs_);
		}
		//Path end.
		else if(n_front >= (int)teachs_.size()-1){
			i=arc_tools::path::indexOfDistanceBack((int)teachs_.size()-1,D,teachs_);
			n_front=arc_tools::path::indexOfDistanceFront(i-1,D,teachs_);
			n_back=arc_tools::path::indexOfDistanceBack(i-1,D,teachs_);
		}
		Eigen::Vector3d i_back = teachs_[n_back];
		Eigen::Vector3d i_front = teachs_[n_front];
		Eigen::Vector3d back_front = i_front - i_back;
		//Angle between i back and i front.
		double argument = i_back.dot(-i_front)/i_back.squaredNorm();
		if(argument > 1.0) argument = 1.0;
		if(argument < -1.0) argument = -1.0;
		float gamma = acos(argument);
		if(fabs(sin(gamma))<=0.01) radius_sum += 9999999;
		else radius_sum += back_front.norm()/(2*fabs(sin(gamma)));	
	}
	double radius = radius_sum/count;
	//Preventing radius from nan.
	if(radius > 2000) radius = 2000;
	return radius;
}

Eigen::Vector3d PurePursuit::linearInterpolation(Eigen::Vector3d short_point, Eigen::Vector3d long_point, 
												double distance_short, double distance_long, double lad){
	//Init interpolation point.
	Eigen::Vector3d interpolated = short_point;
	//Check distance calculation.
	if(distance_short == distance_long){
		std::cout<<"Falsch interpoliert"<< std::endl;
		return interpolated;
	}
	interpolated =  short_point + (lad-distance_short)/(distance_long - distance_short)*(long_point - short_point);
	return interpolated;
}

void PurePursuit::setObstacleDistance(double distance){obstacle_distance_ = distance;}

void PurePursuit::setShutDown(bool shut_down){
	shut_down_ = shut_down;
	if(shut_down) BigBen_.start();
}
