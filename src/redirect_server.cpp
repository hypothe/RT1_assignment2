#include "ros/ros.h"
#include "std_srvs/SetBool.h"

bool foo(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    res.success = true;
    return true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "redirect_server");
	ros::NodeHandle n;
    
    ros::ServiceServer service = n.advertiseService("/redirect", foo);
    ros::spin();

    return 0;
}
