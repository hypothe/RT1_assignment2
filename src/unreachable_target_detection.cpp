#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "move_base_msgs/MoveBaseActionGoal.h" // 

std_msgs::Empty empty_msg;
ros::Publisher pub;					/**< Publisher used to publish message when a target is reached */
ros::Time target_init;
bool started = false;
/**
    Read status messages from move_base;
    status number 3 corresponds to "target reached"

*/
void moveGoalCllbck(const move_base_msgs::MoveBaseActionGoal::ConstPtr& move_msg){
    started = true;
    target_init = ros::Time::now(); // every time a new goal is set the timer is reset
    ROS_INFO("UNREACH_DETECTION TIMER STARTED\n");
}

void targetReachedCallback(const std_msgs::Empty::ConstPtr& empty){
    started = false; 
}
int main(int argc, char** argv){
	ros::init(argc, argv, "unreachable_goal_detection_server");
	ros::NodeHandle n;
	double DD;
    
    ros::Publisher pub          =   n.advertise<std_msgs::Empty>("/target_unreachable", 1000);
  	ros::Subscriber reach_sub   =   n.subscribe("/target_reached", 1000, targetReachedCallback);
    ros::Subscriber pose_sub    =   n.subscribe("/move_base/goal", 1000, moveGoalCllbck);
    ros::Rate loop_rate(1);    // doesn't need to check with high frequency
    
    while (1){
        DD = (ros::Time::now() - target_init).toSec();
        if ( started && DD > 120){
            // more than 2 minutes passed from the instant the new target position was given, we can assume it got stuck
            // in the meantime
            started = false;
            ROS_ERROR("TIMEOUT DETECTED, TARGET SHOULD BE CONSIDERED UNREACHABLE\n");
            pub.publish(empty_msg);
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return 0;
}
