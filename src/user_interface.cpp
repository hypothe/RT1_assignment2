#include "ros/ros.h"
#include "nonholo_control/UIMenu.h"
#include "nonholo_control/map_library.h"

std::string plan_algo_used;
std::map<std::string, int> plan_algo_map;

bool menuCllbck(nonholo_control::UIMenu::Request &req, nonholo_control::UIMenu::Response &res){
    int ans=10;
    char str[10], curr_alg[16], next_alg[16];
    //if (!drive_algo){ sprintf(curr_alg, "move_base"); sprintf(next_alg, "bug0");  }
    //else { sprintf(curr_alg, "bug0"); sprintf(next_alg, "move_base");  }
    
	if(!ros::param::get("active_plan_algorithm", plan_algo_used)){ ROS_ERROR("No parameter named 'active_plan_algorithm' found."); }
    while(ans > 5){
        // ROS_INFO("\n1.\tRandom goal\n2.\tUser defined goal\n3.\tWall follow\n4.\tStop\n5.\tChange algorithm (from %s to %s)\n", curr_alg, next_alg);
        ROS_INFO("\n1.\tRandom goal\n2.\tUser defined goal\n3.\tWall follow\n4.\tStop\n5.\tChange algorithm (from %s to %s)\n", 
                                                            plan_algo_used.c_str(), 
                                                            next_map_element(plan_algo_map, plan_algo_used)->first.c_str()     );
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

	if(!ros::param::get("plan_algorithm", plan_algo_map)){ ROS_ERROR("No parameter named 'plan_algorithm' found."); }
    ros::spin();
}
