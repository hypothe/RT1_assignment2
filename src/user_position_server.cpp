#include "ros/ros.h"
#include <vector>
#include <utility>

#include <XmlRpcValue.h>    // for loading the vector of pairs used for positions
#include <XmlRpcException.h>

#include "nonholo_control/TargetPos.h"	// Service for target position

std::vector< std::pair<double, double> > target_pos_;
int target_pos_dim_ = 0;

/*********************************************//**
* 
************************************************/
bool find_in_target_pos_(double x, double y){
    int found = false;
    for(int i=0; i<target_pos_dim_; i++){ if(found = (target_pos_[i].first==x)&&(target_pos_[i].second==y) ){ break; } }
    // found is given the val True only if [x,y] appears in the set, at which point it jumps out of the cycle
    
    return found;
}

void print_target_pos_(){
    for(int i=0; i<target_pos_dim_; i++){
        printf("[%lf , %lf]\t", target_pos_[i].first, target_pos_[i].second);
    }
    printf("\n");
}
bool user_pos(nonholo_control::TargetPos::Request &req, nonholo_control::TargetPos::Response &res){
    double x=0.0, y=0.0;
	bool found = false;
	while(!found){  // the question is repeated as long until a valid position is inserted
    	printf("Insert the desired target position, knowing it has to be one of these\n");
    	print_target_pos_();
    	printf("X:\t"); scanf("%lf", &x);
    	printf("Y:\t"); scanf("%lf", &y);
    	found = find_in_target_pos_(x, y);
	}
	printf("Value accepted.\n");
	res.target_pos.x = x;
	res.target_pos.y = y;
	return true;
}

int main(int argc, char* argv[]){
	ros::init(argc, argv, "random_position_server");
	ros::NodeHandle n;
	std::vector<double> param_vector;
	
	ros::ServiceServer service = n.advertiseService("/target_position/user", user_pos);	 /* Server for the Service /target_position */
										/* note that the same topic could be used to hold target position generated in more "meaningful" way */
										
	if(!ros::param::get("goals_vector", param_vector)){
	    ROS_ERROR("No field 'goals_vector' found in the parameter file.");
	}
	for(int i = 0; i<param_vector.size(); i+=2){
	    target_pos_.push_back(std::make_pair(param_vector[i], param_vector[i+1]));
	}
	target_pos_dim_ = target_pos_.size();   // of the process
	ros::spin();

	return 0;
}
