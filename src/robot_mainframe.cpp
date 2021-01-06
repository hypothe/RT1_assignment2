#include "ros/ros.h"
#include "nonholo_control/TargetPos.h"	// Service for target position
#include "nonholo_control/UIMenu.h"	// Service for target position
#include "move_base_msgs/MoveBaseActionGoal.h" // Service for setting a goal that move_base can understand
#include "std_srvs/SetBool.h"   // to reuse the wall_follow script
#include "std_msgs/Empty.h"
#include "topic_tools/MuxSelect.h"
#include "nav_msgs/Odometry.h"		// Message containing estimated position

#include "nonholo_control/map_library.h"

#include <sstream>
#include <iostream>
#include <math.h>

ros::Publisher pub;					/**< Publisher used to publish velocities */

ros::ServiceClient client_target_rand;	/**< Service client used for obtaining random target positions */
ros::ServiceClient client_target_user;	/**< Service client used for obtaining user defined target positions */
ros::ServiceClient client_go_to_point;  /**< Enabling wall following directly */
ros::ServiceClient client_wall_follow;  /**< Enabling wall following directly */
ros::ServiceClient client_target_reached;
ros::ServiceClient client_target_unreachable;
ros::ServiceClient client_ui_menu;

ros::ServiceClient mux_cmd_vel;         /**< Allowing for multiplexing between multiple robot driving sources*/
ros::ServiceClient mux_wall_follow_nhc;     /**< Allowing for multiplexing between multiple robot driving sources*/
ros::ServiceClient mux_go_to_point_nhc;     /**< Allowing for multiplexing between multiple robot driving sources*/
ros::ServiceClient mux_wall_follow_bug;     /**< Allowing for multiplexing between multiple robot driving sources*/
ros::ServiceClient mux_go_to_point_bug;     /**< Allowing for multiplexing between multiple robot driving sources*/


geometry_msgs::Point target_position;	/**< Target position to reach, defined as a Point */
geometry_msgs::Point previous_target_position;	/**< Target position to reach, defined as a Point */
geometry_msgs::Point current_position;	/**< Current position of the robot, defined as a Point */
int drive_algo = 0; // 0 -> move_base, 1 -> bug0
double _position_error;
char state_ = 0;
bool _target_reached = false;
bool _target_unreachable = false;

std::map<std::string, int> plan_algo_map;
std::map<std::string, int>::iterator plan_algo_map_it;
std::string plan_algo_used;

/* statuses:
    0: waiting for target (includes following a wall)
    1: moving, non interruptible
*/
/*  Note the different way the two algorithms require to indicate the goal position, with move_base reading it from a topic (move_base_action_goal)
    and bug0 reading it from the parameter server
*/
void publish_target(){
    // if (drive_algo == 0){
    //if (drive_algo == 0 || drive_algo == 1){
        move_base_msgs::MoveBaseActionGoal move_msg;
        // fill it with the local data and publish it
        move_msg.goal.target_pose.header.frame_id = "map";
        move_msg.goal.target_pose.pose.position.x = target_position.x;
        move_msg.goal.target_pose.pose.position.y = target_position.y;
        move_msg.goal.target_pose.pose.orientation.w = 1;
        
        pub.publish(move_msg);
    //}
    //else if (drive_algo == 1){
        ros::param::set("/des_pos_x", target_position.x);
        ros::param::set("/des_pos_y", target_position.y);
   //}
}


// updates both local and param value
void set_plan_algo(std::string new_plan){

    drive_algo = plan_algo_map[new_plan];
    plan_algo_used = new_plan;
    ros::param::set("active_plan_algorithm", new_plan);
}
// new_drive:
//  0 -> move_base drives the robot, cmd_vel_move
//  1 -> bug0 drives the robot, cmd_vel_bug (to which wall_follow_switch publishes as well)
//  2 ->    wall_follow
void change_drive(int new_drive){
    topic_tools::MuxSelect drive;
    // topic_tools::MuxSelect wllflw_nhc, wllflw_bug;
    // topic_tools::MuxSelect gotopo_nhc, gotopo_bug;
    std_srvs::SetBool wall_follow, go_to_point;
        
    if (new_drive == 0){
        set_plan_algo("move_base");
        // --- Make this script drive the wall_follower_service ---
        /*
        wllflw_nhc.request.topic = "wall_follower_switch";
        gotopo_nhc.request.topic = "go_to_point_switch";
        wllflw_bug.request.topic = "redirect";
        gotopo_bug.request.topic = "redirect";
        */
            
	    wall_follow.request.data = false;
	    go_to_point.request.data = false;
        // ---                                                  ---
        drive.request.topic = "cmd_vel_move";
    }
    else if (new_drive == 1){
        set_plan_algo("bug0");
        // --- Make the bug_m drive the wall_follower_service   ---
        /*
        wllflw_bug.request.topic = "redirect";
        gotopo_bug.request.topic = "redirect";
        wllflw_bug.request.topic = "wall_follower_switch";
        gotopo_bug.request.topic = "go_to_point_switch";
        */
        
	    wall_follow.request.data = false;
	    go_to_point.request.data = true;
        // ---                                                  ---
        drive.request.topic = "cmd_vel_bug";
    }
    else if (new_drive == 2){ // wall follow
        // --- Make this script drive the wall_follower_service ---
        /*
        wllflw_nhc.request.topic = "wall_follower_switch";
        gotopo_nhc.request.topic = "go_to_point_switch";
        wllflw_bug.request.topic = "redirect";
        gotopo_bug.request.topic = "redirect";
        */
        
	    wall_follow.request.data = true;
	    go_to_point.request.data = false;
        // ---                                                  ---
        drive.request.topic = "cmd_vel_bug";
    }
    else if (new_drive == 3){
        // --- Make this script drive the wall_follower_service ---
        /*
        wllflw_nhc.request.topic = "wall_follower_switch";
        gotopo_nhc.request.topic = "go_to_point_switch";
        wllflw_bug.request.topic = "redirect";
        gotopo_bug.request.topic = "redirect";
        */
            
	    wall_follow.request.data = false;
	    go_to_point.request.data = false;
        // ---   
        drive.request.topic = "cmd_vel_stop";
    }
    
    mux_cmd_vel.call(drive);
    /*
    mux_go_to_point_nhc.call(gotopo_nhc);
    mux_wall_follow_nhc.call(wllflw_nhc);
    mux_go_to_point_bug.call(gotopo_bug);
    mux_wall_follow_bug.call(wllflw_bug);
    */
    
    client_wall_follow.call(wall_follow);
    client_go_to_point.call(go_to_point);
}

void subscriberCallback(const nav_msgs::Odometry::ConstPtr& pose_msg){
    current_position = pose_msg->pose.pose.position;
}

float pointDist(geometry_msgs::Point p1, geometry_msgs::Point p2){
	return sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y, 2));
}
double err_pos(){
    return pointDist(target_position, current_position);
}

void targetReachedCallback(const std_msgs::Empty::ConstPtr& empty){
    _target_reached = true; 
}
void targetUnreachableCallback(const std_msgs::Empty::ConstPtr& empty){
    _target_unreachable = true; 
}

void simpleRecovery(){
    target_position = previous_target_position;
    publish_target();
    ROS_ERROR("The target position [%f %f] seems to be unreachable\nReturning to last visited target position", target_position.x, target_position.y);
    ros::Duration(2).sleep();
    change_drive(0);    // start driving towards last visited target position, would stop when it reaches it
    // Using move_base algorithm, way more robust than bug0, to get back, but without 
    // overwriting the current dribing algorithm chosen by the user.
}
/*
std::map<std::string, int>::iterator next_map_element(std::map<std::string, int> map, std::string current_key){
    std::map<std::string, int>::iterator it, next;
    if((it = map.find(current_key)) == map.end()){
        ROS_ERROR("Planning algorithm not found in paramater file map description");
        return map.end();
    }
    next = it==--map.end()?map.begin():++it;
    return next;
}
*/
/*
char menu(){
    int ans=10;
    char str[10], curr_alg[16], next_alg[16];
    //if (!drive_algo){ sprintf(curr_alg, "move_base"); sprintf(next_alg, "bug0");  }
    //else { sprintf(curr_alg, "bug0"); sprintf(next_alg, "move_base");  }
    while(ans > 5){
        // ROS_INFO("\n1.\tRandom goal\n2.\tUser defined goal\n3.\tWall follow\n4.\tStop\n5.\tChange algorithm (from %s to %s)\n", curr_alg, next_alg);
        ROS_INFO("\n1.\tRandom goal\n2.\tUser defined goal\n3.\tWall follow\n4.\tStop\n5.\tChange algorithm (from %s to %s)\n", 
                                                            plan_algo_used.c_str(), 
                                                            next_map_element(plan_algo_map, plan_algo_used)->first.c_str()     );
        scanf("%s", str);
        sscanf(str, "%d", &ans);
    }
    return ans;
}*/

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "nonholo_control");
  	ros::NodeHandle n;
  	ros::Rate loop_rate(10);
  	nonholo_control::TargetPos target_srv;
    std_srvs::SetBool wall_follow, go_to_point;
    std_msgs::Empty empty;
    nonholo_control::UIMenu user_input;
    
  	char in;
	
	client_ui_menu              =   n.serviceClient<nonholo_control::UIMenu>("/ui_menu");
  	client_target_rand          =	n.serviceClient<nonholo_control::TargetPos>("/target_position/rand");
  	client_target_user          =	n.serviceClient<nonholo_control::TargetPos>("/target_position/user");
  	client_wall_follow          =	n.serviceClient<std_srvs::SetBool>("/wall_follower_switch");
  	client_go_to_point          =	n.serviceClient<std_srvs::SetBool>("/go_to_point_switch");
  	
    ros::Subscriber reached_sub =   n.subscribe("/target_reached", 1000, targetReachedCallback);
    ros::Subscriber unreach_sub =   n.subscribe("/target_unreachable", 1000, targetUnreachableCallback);
  	  	
  	mux_cmd_vel                 =   n.serviceClient<topic_tools::MuxSelect>("/mux_cmdvel/select");
  	/*mux_wall_follow_nhc             =   n.serviceClient<topic_tools::MuxSelect>("/mux_wllflw_nhc/select");
  	mux_go_to_point_nhc             =   n.serviceClient<topic_tools::MuxSelect>("/mux_gotopo_nhc/select");
  	mux_wall_follow_bug             =   n.serviceClient<topic_tools::MuxSelect>("/mux_wllflw_bug/select");
  	mux_go_to_point_bug             =   n.serviceClient<topic_tools::MuxSelect>("/mux_gotopo_bug/select");*/
  	
	pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000);
	ros::Publisher pub_err = n.advertise<std_msgs::Empty>("/target_reached", 1000);
	
	ros::Subscriber pose_sub = n.subscribe("/odom", 1000, subscriberCallback); 	/* subscriber to the topic relative to
																					robot odometry, setting the callback
																					computing velocity
																				 */
	if(!ros::param::get("plan_algorithm", plan_algo_map)){ ROS_ERROR("No parameter named 'plan_algorithm' found."); }
	if(!ros::param::get("active_plan_algorithm", plan_algo_used)){ ROS_ERROR("No parameter named 'active_plan_algorithm' found."); }
	// tolerance defining goal proximity
    if(!ros::param::get("xy_tolerance", _position_error)){ ROS_ERROR("No parameter named 'xy_goal_tolerance' found."); } 
	drive_algo = plan_algo_map[plan_algo_used];
	
    change_drive(drive_algo);
    ros::Rate(20);
    
	while(ros::ok()){
	    if (state_ == 0){
	        //in = menu();
	        
	        client_ui_menu.call(user_input);
	        in = user_input.response.menu_case;
	        
	        // wall_follow.request.data = false;
	        // go_to_point.request.data = false;
	        
	        if (in == 1 || in ==2){
	            if (in == 1)    {client_target_rand.call(target_srv);}  // empty request, service returns the new target position
	            else            {client_target_user.call(target_srv);}  // empty request, service returns the new target position
	            
                _target_reached = false;    // we are trying to reach a new target
	            previous_target_position = target_position;
	            target_position.x = target_srv.response.target_pos.x;
	            target_position.y = target_srv.response.target_pos.y;
	            change_drive(drive_algo);
	            publish_target();
	            state_ = 1;
	        }
	        else if (in == 3){
    	        ROS_INFO("Wall follow procedure started\n");
    	        change_drive(2);
	        }
	        else if (in == 4){
	            ROS_INFO("Standing still in [%f %f]\n", target_position.x, target_position.y);
	            ros::shutdown();
	            // should I exit now?
	        }
	        else if (in == 5){
    	        ROS_INFO("Changing movement algorithm.\n");
    	        plan_algo_map_it = next_map_element(plan_algo_map, plan_algo_used); // points to the new planning algorithm among those insered in the param file
    	        
    	        set_plan_algo(plan_algo_map_it->first);
    	        //drive_algo = plan_algo_map_it->second;
    	        
    	        //plan_algo_used = plan_algo_map_it->first;
    	        //ros::param::set("active_plan_algorithm", plan_algo_used);
    	        //drive_algo = (drive_algo+1)%2;
	        }
	    }
	    else if (state_ == 1){
	        if (_target_reached){
	            ROS_INFO("Goal reached [%f %f]\n", target_position.x, target_position.y);
	            state_ = 0;
	        }
	        else if (err_pos() < _position_error){ // try with 0.35
	            ROS_INFO("Distance from target lower than threshold, stopping.\n");
	            pub_err.publish(empty); 
	            _target_reached = true;
	            // this is to be sure that the robots stops by proximity, in case move_base keeps going (which appens
	            // almost always).
	            // sending the message instead of simply going into the next 'if' ensures that any other node relying 
	            // on the target being reached is made aware of this.
	        }
	        else if(_target_unreachable){
	            simpleRecovery();
	            _target_unreachable = false;
	        }
	        else{
	            ROS_INFO("At [[%f %f]] going towards [%f %f]\nError [%f]\n", current_position.x, current_position.y, target_position.x, target_position.y, err_pos());
	        }
	    }
	    
        ros::spinOnce();
        loop_rate.sleep();
	}
	
	return 0;
}
