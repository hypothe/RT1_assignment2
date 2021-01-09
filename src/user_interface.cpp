#include "ros/ros.h"
#include "nonholo_control/UIMenu.h"
#include "nonholo_control/map_library.h"    // contains the definition of a function operating on the map

std::string plan_algo_used; // name of the planning algorithm used
std::map<std::string, int> plan_algo_map;   // map of the available planning alogorithms "name":ID, retrieved from parameter server

/*********************************************//**
* Callback to a service call of type 
* nonholo_control::UIMenu on service /ui_menu
* 
* This function displays the menu with options
* the user can choose from. The input choice 
* is inserted in the 'response' field of the
* service.
*
* \param req (nonholo_control::UIMenu::Request &):	
* 			request field of the service;
* \param res (nonholo_control::UIMenu::Response &):	
* 			response field of the service;
*
* \retval success (bool):
* 			'true' by default, unused;
************************************************/
bool menuCllbck(nonholo_control::UIMenu::Request &req, nonholo_control::UIMenu::Response &res){
    int ans=10;
    char str[10], curr_alg[16], next_alg[16];
    
	if(!ros::param::get("active_plan_algorithm", plan_algo_used)){ ROS_ERROR("No parameter named 'active_plan_algorithm' found."); }
    while(ans > 5){
        ROS_INFO("\n1.\tRandom goal\n2.\tUser defined goal\n3.\tWall follow\n4.\tStop\n5.\tChange algorithm (from %s to %s)\n", 
                                                            plan_algo_used.c_str(), 
                                                            nextMapElement(plan_algo_map, plan_algo_used)->first.c_str()     );
                                                            // the string printed contain the current and next planning algorithm
                                                            // names, as present in the 'plan_algo_map' (note that the orderd in the 
                                                            // map is alphabetical on the algorithm names, keys in the map)
        scanf("%s", str);
        sscanf(str, "%d", &ans);
    }
    res.menu_case = ans; 
    return true;
}

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "nonholo_control");
  	ros::NodeHandle n;
  	
    ros::ServiceServer srv_ui = n.advertiseService("/ui_menu", menuCllbck);

	if(!ros::param::get("plan_algorithm", plan_algo_map))
	    { ROS_ERROR("No parameter named 'plan_algorithm' found."); }
    ros::spin();
}
