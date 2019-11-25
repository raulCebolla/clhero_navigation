
//----------------------------------------------------
//    Includes
//----------------------------------------------------

#include <ros/ros.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <clhero_gait_controller/GaitPatternControl.h>

//----------------------------------------------------
//    Defines
//----------------------------------------------------

#define NAV_COMMAND_TOPIC_NAME "cmd_vel"
#define CLHERO_COMMAND_SRV_NAME "/clhero_gait_control/gait_pattern_control"
#define LINEAR_GAIT_PATTERN_NAME "open_loop_alternating_tripod"
#define POS_ANGULAR_GAIT_PATTERN_NAME "alternating_tripod_turn_left"
#define NEG_ANGULAR_GAIT_PATTERN_NAME "alternating_tripod_turn_right"
#define REST_GAIT_PATTERN_NAME "stand_up"

#define MIN_LINEAR_VELOCITY 0.01
#define MIN_ANGULAR_VELOCITY 0.01
#define LEG_RADIUS 0.073
#define ROTATION_LENGTH 0.289
#define PI 3.14159265359
#define DEFAULT_GROUND_ANGLE 1.3089969389957472 // 75[º]
#define MAX_MOTOR_VELOCITY 7 //[rad/s]
#define GROUND_ANGLE_INCREASE 0.08726646259971647 //5[º]
#define MAX_GROUND_ANGLE 1.7453292519943295 // 100 [º]

//----------------------------------------------------
//    Global variables
//----------------------------------------------------

ros::ServiceClient clhero_command_client;

std::string current_gait_pattern;

//----------------------------------------------------
//    Functions
//----------------------------------------------------

//Function that computes the motor velocity required
//to achieve certain linear velocity
double motorVelocityLinear (double v, double ap){

	double num = v * ap;
	double den = LEG_RADIUS * (ap - sin(ap/2 + PI) + sin(-ap/2 + PI));

	return num/den;

}

//Function that computes the motor velocity required
//to achieve certain angular velocity
double motorVelocityAngular (double w, double ap){

	double num = w * ROTATION_LENGTH * ap;
	double den = LEG_RADIUS * (ap - sin(ap/2 + PI) + sin(-ap/2 + PI));

	return num/den;

}

//Function that obtains the air_angle
inline double airVelocity (double ga, double gv){
	return (2*PI - ga)/ga * gv;
}

//Function that obtains the air_angle
inline double airAngle (double ga){
	return (2*PI - ga);
}

//Function that checks for errors in gait control srv
inline bool checkErrorsGaitCommand (int code){
	
	switch(code){
		case 1: 
			std::cout << "[clhero_vel_interface] ERROR: " << "Order not recognized" << std::endl;
			return true;
			break;
		case 2:
			std::cout << "[clhero_vel_interface] ERROR: " << "Gait pattern is not in the list" << std::endl;
			return true;
			break;
		case 3:
			std::cout << "[clhero_vel_interface] ERROR: " << "Gait pattern does not match with the current active pattern" << std::endl;
			return true;
			break;
		case 4:
			std::cout << "[clhero_vel_interface] ERROR: " << "Could not change gait pattern at the command interface" << std::endl;
			return true;
			break;
		default:
			break;
	}

	return false;
}

//callback for navigation stack's command msgs
void nav_command_msg_callback (const geometry_msgs::Twist::ConstPtr& msg){

	clhero_gait_controller::GaitPatternControl command_msg;
	clhero_gait_controller::GaitPatternControl start_pattern_msg;

	double ground_angle = DEFAULT_GROUND_ANGLE;
	double ground_velocity;

	std::string args;

	//Obtains the velocities as: (Vx, w)
	double vx = msg->linear.x;
	double w = msg->angular.z;

	//Checks if the navigation stack's command is linear or angular movement
	if(fabs(vx) > fabs(w)){
		
		//-- Linear movement --

		//checks if the velocity is greater than the limit
		if(fabs(vx) > MIN_LINEAR_VELOCITY){
			//Sets the alternating tripod as the gait pattern active
			command_msg.request.pattern_name = LINEAR_GAIT_PATTERN_NAME;
			//First update the args
			command_msg.request.order = "update_args";
			//Obtains the motor velocity
			ground_velocity = motorVelocityLinear(vx, ground_angle);
			//Checks if the corresponding air motor velocity exceedes
			//the motor velocity limits 
			while(airVelocity(ground_angle, ground_velocity) > MAX_MOTOR_VELOCITY){
				ground_angle += GROUND_ANGLE_INCREASE;
				if(ground_angle >= MAX_GROUND_ANGLE){
					break;
				}
			}
			//Creates the argument string
			args = "ground_angle: " + std::to_string(ground_angle) + ", ground_velocity: " + std::to_string(ground_velocity);
			//Adds the string to the msg
			command_msg.request.args = args;
			//Sends the msg
			clhero_command_client.call(command_msg);
			//If something went wrong exit the callback
			if(checkErrorsGaitCommand(command_msg.response.ans)){
				return;
			}
			//Now checks which gait pattern is currently active
			if(current_gait_pattern.compare(LINEAR_GAIT_PATTERN_NAME) != 0){
				//Calls the service to activate the gait pattern
				start_pattern_msg.request.pattern_name = LINEAR_GAIT_PATTERN_NAME;
				start_pattern_msg.request.order = "start";
				clhero_command_client.call(start_pattern_msg);
				//If something went wrong exit the callback
				if(checkErrorsGaitCommand(command_msg.response.ans)){
					return;
				}else{
					current_gait_pattern = LINEAR_GAIT_PATTERN_NAME;
				}
			}
		}else{
			//Now checks which gait pattern is currently active
			if(current_gait_pattern.compare(REST_GAIT_PATTERN_NAME) != 0){
				//If the velocity is lower just keeps the position
				start_pattern_msg.request.pattern_name = REST_GAIT_PATTERN_NAME;
				start_pattern_msg.request.order = "start";
				clhero_command_client.call(start_pattern_msg);
				//If something went wrong exit the callback
				if(checkErrorsGaitCommand(command_msg.response.ans)){
					return;
				}else{
					current_gait_pattern = REST_GAIT_PATTERN_NAME;
				}
			}
		}

	}else{

		//-- Rotation movement --

		//checks if the velocity is greater than the limit
		if(fabs(w) > MIN_ANGULAR_VELOCITY){
			//Sets the alternating tripod as the gait pattern active
			if(w > 0){
				//Counter-clockwise movement
				command_msg.request.pattern_name = POS_ANGULAR_GAIT_PATTERN_NAME;
			}else{
				//Clockwise movement
				command_msg.request.pattern_name = NEG_ANGULAR_GAIT_PATTERN_NAME;
			}
			//First update the args
			command_msg.request.order = "update_args";
			//Obtains the motor velocity
			ground_velocity = motorVelocityAngular(w, ground_angle);
			//Checks if the corresponding air motor velocity exceedes
			//the motor velocity limits 
			while(airVelocity(ground_angle, ground_velocity) > MAX_MOTOR_VELOCITY){
				ground_angle += GROUND_ANGLE_INCREASE;
				if(ground_angle >= MAX_GROUND_ANGLE){
					break;
				}
			}
			//Creates the argument string
			args = "ground_angle: " + std::to_string(ground_angle) + ", ground_velocity: " + std::to_string(ground_velocity);
			//Adds the string to the msg
			command_msg.request.args = args;
			//Sends the msg
			clhero_command_client.call(command_msg);
			//If something went wrong exit the callback
			if(checkErrorsGaitCommand(command_msg.response.ans)){
				return;
			}
			//Now checks which gait pattern is currently active
			if(current_gait_pattern.compare(command_msg.request.pattern_name) != 0){
				//Calls the service to activate the gait pattern
				start_pattern_msg.request.pattern_name = command_msg.request.pattern_name;
				start_pattern_msg.request.order = "start";
				clhero_command_client.call(start_pattern_msg);
				//If something went wrong exit the callback
				if(checkErrorsGaitCommand(command_msg.response.ans)){
					return;
				}else{
					current_gait_pattern = command_msg.request.pattern_name;
				}
			}
		}else{
			//Now checks which gait pattern is currently active
			if(current_gait_pattern.compare(REST_GAIT_PATTERN_NAME) != 0){
				//If the velocity is lower just keeps the position
				start_pattern_msg.request.pattern_name = REST_GAIT_PATTERN_NAME;
				start_pattern_msg.request.order = "start";
				clhero_command_client.call(start_pattern_msg);
				//If something went wrong exit the callback
				if(checkErrorsGaitCommand(command_msg.response.ans)){
					return;
				}else{
					current_gait_pattern = REST_GAIT_PATTERN_NAME;
				}
			}
		}

	}

	return;
}

//----------------------------------------------------
//    Main function
//----------------------------------------------------

int main(int argc, char **argv){

  //----------------------------------------------------
  //    ROS starting statements
  //----------------------------------------------------

  ros::init(argc, argv, "clhero_vel_interface");
  ros::NodeHandle nh;

  current_gait_pattern = "none";

  clhero_command_client = nh.serviceClient<clhero_gait_controller::GaitPatternControl>(CLHERO_COMMAND_SRV_NAME);
  ros::Subscriber nav_sub = nh.subscribe(NAV_COMMAND_TOPIC_NAME, 1000, nav_command_msg_callback);

  ros::spin();

  return 0;

}