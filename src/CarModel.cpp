#include <arc/CarModel.hpp>

CarModel::CarModel(){}

CarModel::~CarModel(){}

void CarModel::init(Information* infos, ROSInterface* ros_interface, 
                    StateEstimation* state_estimation){
    //Set state estimation pointer.
    ros_interface_ = ros_interface;
    state_estimation_ = state_estimation;
    //Set eRod parameter.
    infos_ = infos;
    distance_rear_front_axis_ = infos_->getErod().distance_wheel_axis;
    width_axis_ = infos_->getErod().width_wheel_axis;  
}

void CarModel::updateModel(){
    //Geometric calculations: Equal angular velocities and current center of rotation on
    //horizontal line from rear axle.
    if(fabs(steering_angle_) <= 0.01) {
        local_velocity_(0) = (velocity_rear_right_ + velocity_rear_left_ ) / 2;
        local_velocity_(1) = 0; 
    }
    else { 
        double v_rear = (velocity_rear_right_ + velocity_rear_left_ ) / 2;
        double beta = M_PI / 2 - fabs(steering_angle_);
        double r1 = distance_rear_front_axis_ * tan(beta);
        double r2 = distance_rear_front_axis_ / cos(beta);
        double w = v_rear / r1;
        double v_front = w * r2;
        local_velocity_(0) = cos(steering_angle_) * v_front;
        local_velocity_(1) = sin (steering_angle_) * v_front;
    }
    //Update state estimation.
    state_estimation_->updateVelocity(local_velocity_.norm());
    ros_interface_->publishCarModel(local_velocity_);
}

double CarModel::getSteeringAngle(){return steering_angle_;}

Eigen::Vector2d CarModel::getVelocity(){return local_velocity_;}

void CarModel::setSteeringAngle(double steering_angle){steering_angle_ = steering_angle;}

void CarModel::setRearLeftWheelVel(double vel){velocity_rear_left_ = vel;}

void CarModel::setRearRightWheelVel(double vel){velocity_rear_right_ = vel;}