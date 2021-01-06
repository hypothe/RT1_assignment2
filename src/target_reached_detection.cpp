#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "actionlib_msgs/GoalStatusArray.h" // Service for reading current status of move_base planning algorithm
#include "move_base_msgs/MoveBaseActionGoal.h" // 

// the node considers valid only the output emitted by the currently used planning algorithm

std_msgs::Empty empty_msg;
bool target_reached = false;
ros::Publisher pub; // NOTE: the choice of using messages instead of services is due to the fact that, altough sporadic events,
                    // we want the system to know as soon as possible (and, more important, without loosing events) when a 
                    // target is reached.
/*
void moveGoalCllbck(const move_base_msgs::MoveBaseActionGoal::ConstPtr& move_msg){
    _target_reached = false; // every time a new goal is set the variable is reset
}*/
bool bugReachedCllbck(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    std::string plan_algo_used;
    if(!target_reached && !ros::param::get("active_plan_algorithm", plan_algo_used)){ ROS_ERROR("No parameter named 'active_plan_algorithm' found."); }
    
    if (plan_algo_used == "bug0")
    {
        target_reached = true; 
        ROS_INFO("Bug0 called for target reached.\n");
        pub.publish(empty_msg);  
    }
    // if the current planning algorithm is not bug0 ignore its call
    return true;
}
/**
    Read status messages from move_base;
    status number 3 corresponds to "target reached"
*/
void moveStatusCllbck(const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg){
    std::string plan_algo_used;// = "move_base";
    if(!ros::param::get("active_plan_algorithm", plan_algo_used)){ ROS_ERROR("No parameter named 'active_plan_algorithm' found."); }

    if (!target_reached && plan_algo_used == "move_base" && !status_msg->status_list.empty() && status_msg->status_list.back().status == 3){
    //if (!target_reached && !status_msg->status_list.empty() && status_msg->status_list[0].status == 3){
        target_reached = true;    // blocks sensitivity to status msgs
        ROS_INFO("MOVE_BASE called for target reached.\n");
        pub.publish(empty_msg); // status_list is a vector, the first element is accessed
   }
        
    // if the current planning algorithm is not move_base ignore its call
}

void moveGoalCllbck(const move_base_msgs::MoveBaseActionGoal::ConstPtr& move_msg){
    ROS_INFO("TRD RECEIVED MOVEGOAL CALLBACK\n");
    target_reached = false;   // this resets the sensitivity to the status msgs
}

void exogenousTargetReachedCllbck(const std_msgs::Empty::ConstPtr& empty){
    ROS_INFO("TRD RECEIVED TARGETREACHED CALLBACK\n");
    target_reached = true; // this ensures that if some other node publishes a message stating the target has been reached 
}

int main(int argc, char** argv){
	ros::init(argc, argv, "target_reached_server");
	ros::NodeHandle n;
    
    pub = n.advertise<std_msgs::Empty>("/target_reached", 1000);
    
    ros::ServiceServer service  = n.advertiseService("/redirect_bug_user_interface", bugReachedCllbck);

    ros::Subscriber status_sub  =   n.subscribe("/move_base/status", 1000, moveStatusCllbck);
    ros::Subscriber goal_sub    =   n.subscribe("/move_base/goal", 1000, moveGoalCllbck);
    ros::Subscriber reach_sub   =   n.subscribe("/target_reached", 1000, exogenousTargetReachedCllbck);
    // ros::Subscriber pose_sub = n.subscribe("/move_base/goal", 1000, moveGoalCllbck);

    ros::spin();
    

    return 0;
}
