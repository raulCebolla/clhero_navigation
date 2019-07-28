//=====================================================================
//	Author:	Raúl Cebolla Arroyo
//	File:
//	Version:
//	Description:
//	Changelog:
//=====================================================================

//---------------------------------------------------------------
//    Includes
//---------------------------------------------------------------

#include <ros/ros.h>
#include <clhero_gait_controller/LegState.h>
#include <nav_msgs/Odometry.h>
#include <map>
#include <vector>
#include <string>
#include <cstring>
#include <cmath>
#include <Eigen/Dense>

//---------------------------------------------------------------
//    Defines
//---------------------------------------------------------------

#define CONFIG_FILE_CHECK_RATE 3
#define LEG_NUMBER 6
#define MAX_PARAM_SEARCH_LOOP_ITERATIONS 180
#define PI 3.14159265359

//---------------------------------------------------------------
//    Struct definition
//---------------------------------------------------------------

//Struct with the relative position information of a point in 
//the clhero robot
struct Clhero_position {
  double x;
  double y;
  double alpha;
  double beta;
  double dist;
};

struct Pose {
  double x;
  double y;
  double giro;
};

//---------------------------------------------------------------
//    Typedefs
//---------------------------------------------------------------

typedef struct Clhero_position Clhero_position;
typedef struct Pose pose;

//---------------------------------------------------------------
//    Global variables
//---------------------------------------------------------------

//Vector with the position of each legs
std::vector<Clhero_position> leg (6, {0, 0, 0, 0, 0});

//Default geometry parent parameter name
const std::string geometry_param_name = "/clhero_geom";

//---------------------------------------------------------------
//    Functions
//---------------------------------------------------------------

//Function that gathers the clhero geometry configuration and 
//obtains other derived properties required for the odometry 
//calculation
bool setGeometryConfig (){

  //ROS Node handler
  ros::NodeHandle nh;

  //Rate to check for the configuration file to become avaiable
  ros::Rate check_rate (CONFIG_FILE_CHECK_RATE);

  //Flag that sets if the parameter exists
  bool param_found = false;

  //parameter key string
  std::string geometry_key;

  //Checks whether the geometry parameter has been set
  if(!(param_found = nh.searchParam(geometry_param_name, geometry_key))){

    //If the parameter has not been found sends a warning msg
    ROS_WARN("[clhero_odom] Clhero geometry parent name couldn't be found. The search will be repeated %d times each second up to %d times.", CONFIG_FILE_CHECK_RATE, MAX_PARAM_SEARCH_LOOP_ITERATIONS);

    //Creates a loop in which the parameter is searched each time
    int n = 0;
    while (n < MAX_PARAM_SEARCH_LOOP_ITERATIONS){
      
      if(param_found = nh.searchParam(geometry_param_name, geometry_key)){
        ROS_INFO("[clhero_odom] Clhero geometry parent found");
        break;
      }else{
        n++;
        check_rate.sleep();
        continue;
      }

    }

    //if the parameter couldn't be found 
    if(!param_found){
      ROS_ERROR("[clhero_odom] Clhero geometry parameter couldn't be found.");
      return false;
    }
  }else{
    ROS_INFO("[clhero_odom] Clhero geometry parent found");
  }

  //Once it is confirmed that the parameters exist, retrieves
  //the information
  
  //Information buffer
  std::map<std::string, double> pos;

  //For each leg
  for(int i=0; i<LEG_NUMBER; i++){

    //Clears the position buffer
    pos.clear();

    //Checks if the param exist and retrieves it
    if(nh.getParam(geometry_key + "/leg_" + std::to_string(i+1) + "/position", pos)){
      
      //Checks if the component x exists and assings it if true
      if(pos.find("x") != pos.end()){
        leg[i].x = pos["x"];
      }else{
        ROS_WARN("[clhero_odom] Component X for position in leg %d is not defined.", i+1);
        continue;
      }

      //Checks if the component y exists and assings it if true
      if(pos.find("y") != pos.end()){
        leg[i].y = pos["y"];
      }else{
        ROS_WARN("[clhero_odom] Component Y for position in leg %d is not defined.", i+1);
        continue;
      }

      //Once the position has been properly set, obtains the 
      //rest of the required properties

      //Distance to the point
      leg[i].dist = sqrt(pow(leg[i].x, 2) + pow(leg[i].y, 2));

      //Angle alpha
      leg[i].alpha = atan2(leg[i].y, leg[i].x);

      //Angle beta
      leg[i].beta = PI/2 - leg[i].alpha;


    }else{
      //If the position information does not exist, returns
      ROS_ERROR("[clhero_odom] Position parameter for leg %d does not exist.", i+1);
      return false;
    }

  }

  return true;
}

//---------------------------------------------------------------
//    Main function
//---------------------------------------------------------------

int main(int argc, char **argv){

  //----------------------------------------------------
  //    ROS starting statements
  //----------------------------------------------------

  ros::init(argc, argv, "clhero_common");
  ros::NodeHandle nh;

  //Configure the required geometry information
  if(!setGeometryConfig()){
    //If the function fails the node shall end
    ROS_ERROR("[clhero_odom] The required clhero geometry parameters does not exist. Terminating node.");
    return 0;
  }

  //----------------------------------------------------
  //    Core loop of the node
  //----------------------------------------------------

  ros::spin();

  return 0;

}
