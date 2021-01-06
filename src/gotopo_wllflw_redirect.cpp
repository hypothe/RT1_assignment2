#include "ros/ros.h"
#include "std_srvs/SetBool.h"

ros::ServiceClient client_go_to_point;  /**< Enabling wall following directly */
ros::ServiceClient client_wall_follow;  /**< Enabling wall following directly */
std_srvs::SetBool wllflw, gotopo;

bool goToPoNHC(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    std::string plan_algo_used;
    if(!ros::param::get("active_plan_algorithm", plan_algo_used)){ ROS_ERROR("No parameter named 'active_plan_algorithm' found."); }
    
    if (plan_algo_used == "move_base")
    {
        ROS_INFO("NHC REQUESTED GO_TO_POINT");
        gotopo.request.data = req.data;
        client_go_to_point.call(gotopo);
        res.success = gotopo.response.success;
        return res.success;
    }
    else    return true;
}
bool goToPoBUG(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    std::string plan_algo_used;
    if(!ros::param::get("active_plan_algorithm", plan_algo_used)){ ROS_ERROR("No parameter named 'active_plan_algorithm' found."); }
    
    ROS_INFO("BUG0 TRIED TO REQUEST GO_TO_POINT");
    if (plan_algo_used == "bug0")
    {
        ROS_INFO("BUG0 REQUESTED GO_TO_POINT");
        gotopo.request.data = req.data;
        client_go_to_point.call(gotopo);
        res.success = gotopo.response.success;
        return res.success;
    }
    else    return true;
}
bool wllFlwNHC(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    std::string plan_algo_used;
    if(!ros::param::get("active_plan_algorithm", plan_algo_used)){ ROS_ERROR("No parameter named 'active_plan_algorithm' found."); }
    
    if (plan_algo_used == "move_base")
    {
        ROS_INFO("NHC REQUESTED WALL_FOLLOWER");
        wllflw.request.data = req.data;
        client_wall_follow.call(wllflw);
        res.success = wllflw.response.success;
        return res.success;
    }
    else    return true;
}
bool wllFlwBUG(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    std::string plan_algo_used;
    if(!ros::param::get("active_plan_algorithm", plan_algo_used)){ ROS_ERROR("No parameter named 'active_plan_algorithm' found."); }
    
    ROS_INFO("BUG0 TRIED TO REQUEST WALL_FOLLOWER");
    if (plan_algo_used == "bug0")
    {
        ROS_INFO("BUG0 REQUESTED WALL_FOLLOWER");
        wllflw.request.data = req.data;
        client_wall_follow.call(wllflw);
        res.success = wllflw.response.success;
        return res.success;
    }
    else    return true;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "redirect_server");
	ros::NodeHandle n;
    
    ros::ServiceServer srv_gotopo_nhc = n.advertiseService("/go_to_point_switch_nhc", goToPoNHC);
    ros::ServiceServer srv_gotopo_bug = n.advertiseService("/go_to_point_switch_bug", goToPoBUG);
    ros::ServiceServer srv_wllflw_nhc = n.advertiseService("/wall_follower_switch_nhc", wllFlwNHC);
    ros::ServiceServer srv_wllflw_bug = n.advertiseService("/wall_follower_switch_bug", wllFlwBUG);

  	client_wall_follow          =	n.serviceClient<std_srvs::SetBool>("/wall_follower_switch");
  	client_go_to_point          =	n.serviceClient<std_srvs::SetBool>("/go_to_point_switch");
  	
    ros::spin();

    return 0;
}
