#include <arc/GridAnalyser.hpp>

GridAnalyser::GridAnalyser(){}

GridAnalyser::~GridAnalyser(){}

void GridAnalyser::init(Information* infos, Guard* guard, PurePursuit* pure_pursuit){
	//Set class pointer.
	guard_ = guard;
	pure_pursuit_ = pure_pursuit;
	//Init grid analyser states.
	obstacle_stop_ = false;
	jumper_ = false; grid_init_ = false; state_init_ = false;
	crit_counter_ = 0;
	//Getting parameter.
	infos_ = infos;
	control_ = infos_->getControl();
	erod_ = infos_->getErod();
	safety_ = infos_->getSafety();
	use_obstacle_detection_ = infos_->getLaunchables().obstacle_detection;
	//Teach path.
	teach_positions_ = infos_->getTeachPositions();
}

void GridAnalyser::createDangerZone(const nav_msgs::OccupancyGrid grid_map){	
	tube_map_=grid_map;
	//Empty grid map.
	int n_cells = grid_map.info.height*grid_map.info.width;
	for (int i=0;i<n_cells;i++) tube_map_.data[i]=0;
	//Inflate path
	int max_index = arc_tools::path::indexOfDistanceFront(state_.current_index,5,teach_positions_);
	for(int i=state_.current_index; i<max_index; i++) 
		inflate(gridIndexOfGlobalPoint(teach_positions_[i]));	
}

void GridAnalyser::compareGrids(){	
	int counter = 0;
	int j=0;
	double distance_old=100;
	for(int i=0; i<n_cells_; i++){	
		int a = tube_map_.data[i];
		int b = obstacle_map_.data[i];
		//If obstacle in both maps at same point.
		if ((a!=0)&&(b!=0)){	
			counter++;	
			Eigen::Vector2d obstacle_point = getGridElement(i);
			double x = obstacle_point(0);
			double y = obstacle_point(1);
			double distance_new = sqrt(pow(y+(width_/2.0),2) + pow(x-(height_/2.0),2))*resolution_;			
			if(distance_new<distance_old){
				distance_old = distance_new;
				j=i;
			}
		}
	}
	//If minimal one cell is in the danger area and overlays with grid map.
	if (counter!=0)	{
		std::cout<<"GRID ANALYSER: Etwas auf dem Weg!"<<std::endl;
		whattodo(j);
	}
	else{	
		crit_counter_= 0;
		obstacle_stop_ = false;
		obstacle_distance_ = 100;
	}	
}

void GridAnalyser::searchForObstacles(){
	if(grid_init_ && state_init_){
		createDangerZone(obstacle_map_);
		compareGrids();
		if(use_obstacle_detection_){
			pure_pursuit_->setObstacleDistance(obstacle_distance_);
			if(obstacle_stop_) guard_->emergencyStop("Obstacle");
			guard_->setRunningStates(true, "obstacle_detection");
		}
	}
}

void GridAnalyser::whattodo(const int danger_index){	
	//Object diatance to frontest point of car.
	Eigen::Vector2d danger_point = getGridElement(danger_index);
	double x = danger_point(0);
	double y = danger_point(1);
	obstacle_distance_=sqrt(pow(x-height_/2,2)+pow(-y-width_/2,2))*resolution_-erod_.distance_front_rear_axis;		
	if(obstacle_distance_ < emergency_distance_){
		crit_counter_++;
		if(crit_counter_ >= control_.number_of_emergency_cells){
			std::cout<<"GRID ANALYSER: NOTSTOPP!"<<std::endl;
			obstacle_stop_ = 1;
		}
		else{
			obstacle_stop_ = 0;
			std::cout<<"GRID ANALYSER: crit_counter bei "<<crit_counter_<<std::endl;
		}	
	}
	else{	
		crit_counter_ = 0;
		obstacle_stop_ = 0;
		std::cout<<"GRID ANALYSER: LANGSAMER!"<<std::endl;
	}
}

bool GridAnalyser::getObstacleStop(){return obstacle_stop_;}

void GridAnalyser::inflate(int x, int y){	
	//Displace less and less with increasing distance.
	double distance=sqrt( pow(y+(width_/2.0),2) + pow(x-(height_/2.0),2) )*resolution_; 
	double displacement=std::max(tracking_error_*(1-distance/control_.length_correction_path),double(0));
	if(displacement!=0.0){
		bool left=false;
		if (tracking_error_ >= 0) left=true;
		if (left) y-=round(displacement/resolution_);
		else y+=round(displacement/resolution_);
	}
	//Choose radius
	double radius = (erod_.total_width/2)*safety_.fos_dangergrid+tracking_error_;
	if((x>0) && x<height_ && y<0 && (y>-width_)){	
		int R = round(radius)/resolution_;
		for(int i=(x-R); i<=(x+R); i++){
			for(int j=(y-R); j<=(y+R); j++){	
				//Calculate distance squared between (i, j) and (x, y).
				float d=(((x-i)*(x-i))+((y-j)*(y-j)));
				//If (i, j) is on the map and the distance to (x,y) is smaller than the defined.
				if((i>0) && (i<height_) && (j<0) && j>-width_ && (d<(R*R))){
					Eigen::Vector2d grid_point(i,j);
					tube_map_.data[getGridElement(grid_point)]=100;
				}
			}
		}
	}
}

void GridAnalyser::inflate(int index){	
	Eigen::Vector2d point = getGridElement(index);
	inflate(point(0),point(1));
}

int GridAnalyser::getGridElement(Eigen::Vector2d grid_point){
	int x = grid_point(0);
	int y = grid_point(1);
	int index = (-y-1+width_*(x-1));	
	return index;
}

Eigen::Vector2d GridAnalyser::getGridElement(const int index){	
	Eigen::Vector2d grid_element;
	grid_element(0) = int(index/width_)+1;
	grid_element(1) = -(index%width_)-1;
	return grid_element;
}

int GridAnalyser::gridIndexOfGlobalPoint(const Eigen::Vector3d point){	
	int index = -1;
	Eigen::Vector3d local = arc_tools::geometry::globalToLocal(point,state_);
	int x_index = round(local(0)/resolution_);
	int y_index = round(local(1)/resolution_);
	if((y_index<(width_/2))&&(y_index>(-width_/2))&&(x_index>(-height_/2))&&(x_index<(height_/2)))
      	index = round(-y_index+x_index*width_+width_/2+(height_/2)*width_);
	return index;
}

void GridAnalyser::setObstacleMap(const nav_msgs::OccupancyGrid* map){
	if(jumper_){
		obstacle_map_ = *map;
		n_cells_ = map->info.height*map->info.width;
		width_ = map->info.width;					
		height_ = map->info.height;				
		resolution_ = map->info.resolution; 
		grid_init_ = true;
		jumper_ = false;	
		searchForObstacles();
	}
}

void GridAnalyser::setState(const State state){
	if(!jumper_){
		state_ = state;
		tracking_error_ = arc_tools::geometry::globalToLocal(teach_positions_[state_.current_index],state_)(1);
		double braking_distance_ = pow(state_.velocity*3.6/10,2)/2*safety_.fos_braking_distance;
		emergency_distance_ = std::max(control_.emergency_distance_lb,braking_distance_);
		emergency_distance_ = std::min(control_.emergency_distance_ub,braking_distance_);
		state_init_ = true;
		jumper_ = true;
		searchForObstacles();
	}
}

