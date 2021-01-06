#include "ros/ros.h"
#include <cstdlib>	// rand()
#include <ctime>	// rand() seed
#include <vector>
#include <utility>

#include "nonholo_control/TargetPos.h"	// Service for target position

std::vector< std::pair<double, double> > target_pos_;
int prev_choice = -1;
int target_pos_dim_ = 0;
bool avoid_duplicates = false;

/*********************************************//**
* 
************************************************/
bool rand_pos(nonholo_control::TargetPos::Request &req, nonholo_control::TargetPos::Response &res){
    int i;
    do{
        while( (i = rand()) > RAND_MAX - (RAND_MAX - (target_pos_dim_-1) )%target_pos_dim_ ){}   // discard values of rand whih would unbalance the randnomization
        i = i%target_pos_dim_;
    }while(avoid_duplicates && prev_choice==i);  // easy but time wasting way: more complex implementation could remove previous goal from the vector, saving it
                                                // elsewhere, and reinserting it when the next goal is defined
    prev_choice = i;
    res.target_pos.x = target_pos_[i].first;
	res.target_pos.y = target_pos_[i].second;
	return true;
}

int main(int argc, char* argv[]){
	ros::init(argc, argv, "random_position_server");
	ros::NodeHandle n;
	std::vector<double> param_vector;
	
	if(argc>1 && strcmp(argv[1],"true")==0){    // avoid choosing as the next random position one already visited
	    avoid_duplicates = true;
	}
	
	srand(time(0));	/* random seed initialization */
	ros::ServiceServer service = n.advertiseService("/target_position/rand", rand_pos);	 /* Server for the Service /target_position */
										/* note that the same topic could be used to hold target position generated in more "meaningful" way */
										
	if(!ros::param::get("goals_vector", param_vector)){ ROS_ERROR("No field 'goals_vector' found in the parameter file."); }
	for(int i = 0; i<param_vector.size(); i+=2){
	    target_pos_.push_back(std::make_pair(param_vector[i], param_vector[i+1]));
	}
	target_pos_dim_ = target_pos_.size();   // of the process
	ros::spin();

	return 0;
}
