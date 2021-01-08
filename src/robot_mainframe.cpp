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

ros::Publisher pub;					    /**< Publisher used to publish velocities */

ros::ServiceClient client_target_rand;	/**< Service client used for obtaining random target positions */
ros::ServiceClient client_target_user;	/**< Service client used for obtaining user defined target positions */
ros::ServiceClient client_go_to_point;  /**< Service client enabling go to point directly */
ros::ServiceClient client_wall_follow;  /**< Service client enabling wall follow directly */
ros::ServiceClient client_ui_menu;      /**< Service client used to communicate with the UI */

ros::ServiceClient mux_cmd_vel;         /**< Allowing for multiplexing between multiple robot driving sources*/


geometry_msgs::Point target_position;	/**< Target position to reach, defined as a Point */
geometry_msgs::Point previous_target_position;	/**< Target position to reach, defined as a Point */
geometry_msgs::Point current_position;	/**< Current position of the robot, defined as a Point */
int drive_algo = 0; // 0 -> move_base, 1 -> bug0
double _position_error;
char state_ = 0;
bool _target_reached = false;
bool _target_unreachable = false;

std::map<std::string, int> plan_algo_map;   /**< Map of the available planning alogorithms "name":ID, 
                                                retrieved from parameter server */
std::map<std::string, int>::iterator plan_algo_map_it;  /**< Iterator moving on the planning algorithms map */
std::string plan_algo_used;                 /**< Name of the planning algorithm used */

/*  STATUSES:
    0: waiting for target (includes following a wall)
    1: moving, non interruptible
*/
/*  
*/
/*********************************************//**
* Routine to publish a target position (goal), 
* stored as a global attribute
* 
* This function displays the menu with options
* the user can choose from. The input choice 
* is inserted in the 'response' field of the
* service.
* Note the different way the two algorithms 
* require to indicate the goal position, with 
* move_base reading it from a topic 
* (move_base_action_goal) and bug0 reading it 
* from the parameter server.
*
************************************************/
void publish_target(){
        move_base_msgs::MoveBaseActionGoal move_msg;
        // fill it with the local data and publish it
        move_msg.goal.target_pose.header.frame_id = "map";
        move_msg.goal.target_pose.pose.position.x = target_position.x;
        move_msg.goal.target_pose.pose.position.y = target_position.y;
        move_msg.goal.target_pose.pose.orientation.w = 1;
        pub.publish(move_msg);
        
        ros::param::set("/des_pos_x", target_position.x);
        ros::param::set("/des_pos_y", target_position.y);
}


// updates both local and param value
/*********************************************//**
* Function that updates the reference to the 
* planning algorithm used.
*
* The update is both local and on the value
* stored in the parameter server.
*
* \param new_plan (std::string):
*           name of the algorithm, as presented
*           in the keys of the map in the
*           parameters;
*
************************************************/
void set_plan_algo(std::string new_plan){

    drive_algo = plan_algo_map[new_plan];
    plan_algo_used = new_plan;
    ros::param::set("active_plan_algorithm", new_plan);
}

/*********************************************//**
* Function that updates the behaviour of the
* planning algorithm used.
*
* It first updates the reference to the algorithm,
* then uses a multiplexer to select which of the
* remapped cmd_vel outputs to use, either the
* move_base or bug0 one. It also calls the 
* swithcer for wall_follow and go_to_point 
* behaviour to set that behaviour (notice how
* they do not impact the driving mode move_base
* as a planner to a goal). In the function the
* value of the parameter 'wall_follow_active'
* is set as well, denoting if the wall follow
* behaviour is in act, in order to "mute" the
* bug0 service calls to go_to_point and 
* wall_follow in the meantime.
*
* \param new_drive (int):
*           the ID of the planning parameter to
*           use:\n
*           0 -> move_base\n
*           1 -> bug0\n
*           2 -> wall follow\n
*           3 -> stop (unused)\n
*
************************************************/
void change_drive(int new_drive){
    topic_tools::MuxSelect drive;
    std_srvs::SetBool wall_follow, go_to_point;
    bool wall_follow_active = false;
        
    if (new_drive == 0){
        set_plan_algo("move_base");
        // ---                              ---
	    wall_follow.request.data = false;
	    go_to_point.request.data = false;
        // ---                              ---
        drive.request.topic = "cmd_vel_move";
    }
    else if (new_drive == 1){
        set_plan_algo("bug0");
        // ---                              ---
	    wall_follow.request.data = false;
	    go_to_point.request.data = true;
        // ---                              ---
        drive.request.topic = "cmd_vel_bug";
    }
    else if (new_drive == 2){
        wall_follow_active = true;
        // ---                              ---
	    wall_follow.request.data = true;
	    go_to_point.request.data = false;
        // ---                              ---
        drive.request.topic = "cmd_vel_bug";
    }
    else if (new_drive == 3){
        // ---                              ---
	    wall_follow.request.data = false;
	    go_to_point.request.data = false;
        // ---                              ---
        drive.request.topic = "cmd_vel_stop";
    }
    
    if(!ros::param::has("/wall_follow_active"))
        { ROS_ERROR("No parameter named 'wall_follow_active' found."); }
    else
        ros::param::set("/wall_follow_active", wall_follow_active);
              
    mux_cmd_vel.call(drive);
    
    client_wall_follow.call(wall_follow);
    client_go_to_point.call(go_to_point);
}

/*********************************************//**
* Callback executed every time a message on topic
* /odom is published, updating the current robot
* position.
*
* \param pose_msg (const nav_msgs::Odometry::ConstPtr&):	
* 			content of the message read;
*
************************************************/
void subscriberCallback(const nav_msgs::Odometry::ConstPtr& pose_msg){
    current_position = pose_msg->pose.pose.position;
}

/*********************************************//**
* Routine returning the distance between two 
* points.
*
* \param p1 (geometry_msgs::Point&):	
* 			first point;
* \param p2 (geometry_msgs::Point&):	
* 			second point;
* \retval pointDist (double):
*           the euclidean distance between p1 
*           and p2;
*
************************************************/
double pointDist(geometry_msgs::Point p1, geometry_msgs::Point p2){
	return sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y, 2));
}

/*********************************************//**
* Function computing the distance of the current
* robot position with respect to the target
* position.
*
* \retval errPos (double):
*           the euclidean distance between 
*           current and target position;
*           
************************************************/
double errPos(){
    return pointDist(target_position, current_position);
}

/*********************************************//**
* Callback executed every time a message on topic
* /target_reached is published. 
*
* The global variable _target_reached is set to 
* 'true'.
*
* \param empty (const std_msgs::Empty::ConstPtr&):	
* 			reference to empty message (unused);
*
************************************************/
void targetReachedCallback(const std_msgs::Empty::ConstPtr& empty){
    _target_reached = true; 
}

/*********************************************//**
* Callback executed every time a message on topic
* /target_unreachable is published. 
*
* The global variable _target_unreachable is set  
* to 'true'.
*
* \param empty (const std_msgs::Empty::ConstPtr&):	
* 			reference to empty message, unused;
*
************************************************/
void targetUnreachableCallback(const std_msgs::Empty::ConstPtr& empty){
    _target_unreachable = true; 
}


/*********************************************//**
* Routine performing a trivial form of recovery
* behaviour.
*
* The robot is sent back to the previous target
* position, considered to be safe, using 
* 'move_base' planning (more robust than bug0).
*
************************************************/
void simpleRecovery(){
    target_position = previous_target_position;
    publish_target();
    ROS_ERROR("The target position [%f %f] seems to be unreachable\nReturning to last visited target position", target_position.x, target_position.y);
    ros::Duration(2).sleep();
    change_drive(0);    // start driving towards last visited target position, would stop when it reaches it
    // Using move_base algorithm, way more robust than bug0, to get back, but without 
    // overwriting the current dribing algorithm chosen by the user.
}

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
	        
	        client_ui_menu.call(user_input);
	        in = user_input.response.menu_case;
	        
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
	        }
	        else if (in == 5){
    	        ROS_INFO("Changing movement algorithm.\n");
    	        plan_algo_map_it = nextMapElement(plan_algo_map, plan_algo_used); 
    	        // points to the next planning algorithm among those inserted in the param 
    	        // file map: note that they are ordered alphabetically with respect to the
    	        // algorithm names
    	        
    	        set_plan_algo(plan_algo_map_it->first);
	        }
	    }
	    else if (state_ == 1){
	        if (_target_reached){
	            ROS_INFO("Goal reached [%f %f]\n", target_position.x, target_position.y);
	            state_ = 0;
	        }
	        else if (errPos() < _position_error){ // default to .5
	            ROS_INFO("Distance from target lower than threshold, stopping.\n");
	            pub_err.publish(empty); 
	            _target_reached = true;
	            // this is to be sure that the robots stops by proximity, in case move_base keeps going (which almost 
	            //always appens).
	            // sending the message instead of simply going into the next 'if' ensures that any other node relying 
	            // on the target being reached is made aware of this.
	        }
	        else if(_target_unreachable){
	            simpleRecovery();
	            _target_unreachable = false;
	        }
	        else{
	            ROS_INFO("At [[%f %f]] going towards [%f %f]\nError [%f]\n", 
	                                    current_position.x, current_position.y, 
	                                    target_position.x, target_position.y, 
	                                    werrPos());
	        }
	    }
	    
        ros::spinOnce();
        loop_rate.sleep();
	}
	
	return 0;
}
