#include <arc/VCUInterface.hpp>

VCUInterface::VCUInterface(){}

VCUInterface::~VCUInterface(){
	  //Back to manuell mode.
	  send_msg("am", 0.0, true);
}

void VCUInterface::init(Information* infos,CarModel* car_model, Guard* guard, 
                        ROSInterface* ros_interface, bool rosbag_record){
    //Set class pointer.
    car_model_ = car_model;
    guard_ = guard;
    ros_interface_ = ros_interface;
    //Getting parameters.
    infos_ = infos;
    memset((char *) &si_me_, 0, sizeof(si_me_));
    si_me_.sin_family = AF_INET;
    si_me_.sin_port = htons(infos_->getGeneral().my_port);
    si_me_.sin_addr.s_addr = htonl(INADDR_ANY);
    si_VCU_.sin_family = AF_INET;
   	si_VCU_.sin_port = htons(infos_->getGeneral().vcu_port);
    si_VCU_.sin_addr.s_addr = inet_addr("10.0.0.8");
    slen_ = sizeof(si_other_);
    //Use interface and first autonomous.
    use_vcu_ = infos_->getLaunchables().vcu;
    first_autonomous_ = infos_->getMode();
    //Set up vcu udp binding (socket + binding).
    if(use_vcu_){
      if ((sock_=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) printError("socket");
      if(bind(sock_, (struct sockaddr*)&si_me_, sizeof(si_me_)) == -1) printError("binding");
    }
    //Rosbag. 
    record_ = rosbag_record;
}

void VCUInterface::modeChange(bool mode){
    bool init_mode = infos_->getMode();
    if(mode!=init_mode || first_autonomous_){
      send_msg("am",0.0,!mode);
      send_msg("am",5.0,mode);
      std::string modus = (mode) ? "autonomous" : "manuell";
      std::cout << "ARC: Change to " << modus << " mode !" << std::endl;
      infos_->setMode(mode);
      first_autonomous_ = false;
    }
}

void VCUInterface::send_msg(std::string symbol, double msg, bool requirement){
    if(!use_vcu_) return;
    //Convert msg to string.
    std::ostringstream stream;
    stream << msg;
    std::string value_string = stream.str();
    //Create char array.
    std::string sending = symbol + ":" + value_string;
    const char *buffer_out = sending.c_str();
    //Sending to VCU.
    if(requirement)
       if (sendto(sock_, buffer_out, sizeof(buffer_out), 0, (struct sockaddr*) &si_VCU_, slen_) == -1) 
          printError("sending " + symbol);
}

void VCUInterface::send_msg(std::string symbol, double msg, bool requirement, 
                            double max_value, double min_value, double shift){
  	if(!use_vcu_) return;
    //Check requirements.
  	if(msg>max_value) msg = max_value;
  	if(msg<min_value) msg = min_value;
  	//Shift value.
  	msg += shift;
  	//Convert msg to string.
  	std::ostringstream stream;
    stream << msg;
    std::string value_string = stream.str();
    //Create char array.
    std::string sending = symbol + ":" + value_string;
    const char *buffer_out = sending.c_str();
    //Sending to VCU.
    if(requirement)
  	   if (sendto(sock_, buffer_out, sizeof(buffer_out), 0, (struct sockaddr*) &si_VCU_, slen_) == -1) 
          printError("sending " + symbol);
}

void VCUInterface::recv_msgs(){
    if(!use_vcu_) return;
  	//Receiving.
    int recv_len;
    char buffer_in[buflen_];
    if ((recv_len = recvfrom(sock_, buffer_in, buflen_, 0, (struct sockaddr *) &si_other_, &slen_)) == -1) 
       printError("receiving");
    //Convert msg to string.
    std::string msg;
    for (int i = 0; i < recv_len; ++i){
       msg += buffer_in[i];
    }
    //Get kind of msg and value.
  	std::string kind = msg.substr(0,2);
  	std::string value_string(msg, 3, msg.length()-1);
    const char *buffer = value_string.c_str();
    double value = atof(buffer);
  	//Answers.
  	if(kind == "si"){
  	    value = (value-1000)*M_PI/180;
  	   	car_model_->setSteeringAngle(value);
        if(record_) ros_interface_->publishVCUInfos("steering_angle", value);
  	} 
  	else if(kind == "rr"){
      car_model_->setRearRightWheelVel(value);
      if(record_) ros_interface_->publishVCUInfos("wheel_rear_right", value);
    }
  	else if(kind == "rl"){
      car_model_->setRearLeftWheelVel(value);
      if(record_) ros_interface_->publishVCUInfos("wheel_rear_left", value);
    }
    else if(kind == "am" && value == 20.0) {}
  	else if(kind == "am" && value == 99.0) guard_->emergencyStop("VCU");
  	else if(kind == "cc" && value == 1.0) guard_->setRunningStates(true, "vcu");
  	else if(kind == "hn") guard_->emergencyStop("VCU");
  	else printError( "assigning msg " + msg);
    //Resolve guard check.
    guard_->setRunningStates(true, "vcu");
}

void VCUInterface::printError(std::string error){
    // std::cout << "ARC Interface: Error with " + error << std::endl;
}
