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
#include <iostream>
#include <cstdio>
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
//    Using namespaces
//---------------------------------------------------------------

using namespace Eigen;

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
  Matrix<double, 2, 3> Rt;
};

//---------------------------------------------------------------
//    Typedefs
//---------------------------------------------------------------

typedef struct Clhero_position Clhero_position;
typedef Eigen::Vector3d Pose2D; //Defines pose as: x, y, angle
typedef Eigen::Matrix<double, 6, 1> Pose3D; //Defines pose as: x, y, z, roll, pitch, yaw

//---------------------------------------------------------------
//    Global variables
//---------------------------------------------------------------

//Vector with the position of each legs
std::vector<Clhero_position> leg (6, {0, 0, 0, 0, 0});

//Default geometry parent parameter name
const std::string geometry_param_name = "/clhero_geom";

//Limit angles which define the aerial and ground movement's
//limits
double max_take_off_angle;
double min_landing_angle;

//Leg radius and minimum height
double leg_radius;
//double min_height;

//Publisher of the odometry msg
ros::Publisher odometry_pub;

//Vector containing the current and former pose
std::vector<Pose3D> robot_pose (2, Pose3D::Zero());
std::vector<Pose3D> robot_velocity (2, Pose3D::Zero());

//Current and former time
std::vector<ros::Time> t;

//---------------------------------------------------------------
//    Functions
//---------------------------------------------------------------

template <class T, class V>
std::vector<T> quicksort_index (std::vector<T> ind, std::vector<V> ref){

  int last = ind.size()-1;

  //if size is lower or equal than 1 it is already sorted
  if(ind.size() <= 1){
    return ind;
  }

  //creates two partitions
  std::vector<T> p1, p2;

  //Divides into lower and greater than the pivot, which is the last element
  for(int i=0; i < (ind.size() - 1); i++){
    
    if(ref[ind[i]-1] > ref[ind[last]-1]){
      p1.push_back(ind[i]);
    }else{
      p2.push_back(ind[i]);
    }

  }

  //Applies quicksort to each of the partitions
  if(p1.size() < 1){
    p1.push_back(ind[last]);
    p2 = quicksort_index(p2, ref);
  }else if(p2.size() < 1){
    p1 = quicksort_index(p1, ref);
    p2.push_back(ind[last]);
  }else{
    p1 = quicksort_index(p1, ref);
    p1.push_back(ind[last]);
    p2 = quicksort_index(p2, ref);
  }

  //Once each of the partitions are sorted, join them
  for(int i = 0; i < p2.size(); i++){
    p1.push_back(p2[i]);
  }

  return p1;

}

//Function that calculates the leg velocity
Eigen::Vector2d calcLegVelocity(double ang, double ang_vel){

  //According to the cycloid kinematics the velocity can be obtained through:
  double vel = leg_radius*(ang_vel - ang_vel*cos(ang + PI));

  Eigen::Vector2d ans; ans << vel, 0;

  return ans;
  
}


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
        ROS_INFO("[clhero_odom] Clhero geometry parameter found");
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

      //Angle beta
      leg[i].beta = atan2(leg[i].x, leg[i].y);

      //Angle alpha
      leg[i].alpha = PI/2 - leg[i].beta;

      //Robot-Leg rotation matrix
      leg[i].Rt << Matrix<double, 2, 2>::Identity(), (Matrix<double, 2, 1>() << -leg[i].dist*cos(leg[i].beta), leg[i].dist*sin(leg[i].beta)).finished();


    }else{
      //If the position information does not exist, returns
      ROS_ERROR("[clhero_odom] Position parameter for leg %d does not exist.", i+1);
      return false;
    }

  }

  //Checks for the limit angles
  if(!nh.getParam(geometry_key + "/max_take_off_angle", max_take_off_angle)){
    ROS_ERROR("[clhero_odom] Maximum take off angle has not been defined.");
    return false;
  }
  if(!nh.getParam(geometry_key + "/min_landing_angle", min_landing_angle)){
    ROS_ERROR("[clhero_odom] Minimum landing angle has not been defined.");
    return false;
  }

  //Checks for other geometry parameters
  if(!nh.getParam(geometry_key + "/leg_radius", leg_radius)){
    ROS_ERROR("[clhero_odom] Leg radius has not been defined.");
    return false;
  }
  /*if(!nh.getParam(geometry_key + "/min_height", min_height)){
    ROS_ERROR("[clhero_odom] Minimum height has not been defined.");
    return false;
  }*/

  return true;
}


//Function that gives a value depending on how close an angle is to the 
//zero position
double getZeroScore(double pos){

  double score = pow((pos - PI), 2);
  return score;

}

//Function that obtains which three legs are more likely to be in contact
//with the ground.
std::vector<int> getLegsOnGround (std::vector<double> pos){

  std::vector<double> score;
  std::vector<int> leg_id;
  std::vector<int> legs_on_ground;

  //Obtains the score that evaluates how close a position is to 
  //the zero position
  for(int i=0; i < pos.size(); i++){

    //First checks if the leg is in an available ground position
    if((pos[i] < max_take_off_angle) || (pos[i] > min_landing_angle)){
      //in this case, obtains the score
      score.push_back(getZeroScore(pos[i]));
      //and adds the id
      leg_id.push_back(i+1);
    }
  }

  //The indexes are sorted from greater to lower using the score
  leg_id = quicksort_index(leg_id, score);

  //Picks only the three with the highest score 
  if(leg_id.size() < 3){
    //if there is less than 3 legs on ground
    //it shall be returned the same sorted idex array
    return leg_id;
  }
  //otherwise picks the higher 3
  for(int i=0; i < 3; i++){
    legs_on_ground.push_back(leg_id[i]);
  }

  return legs_on_ground;

}

//Function that returns the velocity vector of the three legs in 
//contact with the ground
Eigen::Matrix<double, 6, 1> calcCycloidVel (std::vector<int> leg_id, 
                      std::vector<double> pos, 
                      std::vector<double> vel){

  Eigen::Matrix<double, 6, 1> v = Eigen::Matrix<double, 6, 1>::Zero();

  //For each leg
  for(int i = 0; i < leg_id.size(); i++){
    //X component
    //Vx = R*[vel - vel*cos(pos)]
    v[2*i] = leg_radius * (vel[leg_id[i]-1] - vel[leg_id[i]-1]*cos(pos[leg_id[i]-1] + PI));
    //Y component
    //Vy = 0
    //As it is already initialized as 0, it shall not be modified
  }

  return v;

}

//Function that returns the rotation matrix for calculating the velocity
//vector of the robot
Eigen::Matrix<double, Dynamic, 3> robot_leg_rotation_matrix(std::vector<int> legs, std::vector<Clhero_position> geom){

  //Rotation matrix R
  Eigen::Matrix<double, Dynamic, 3> R = Eigen::Matrix<double, Dynamic, 3>::Zero(2*legs.size(), 3);

  //For each specified leg
  for(int i = 0; i < legs.size(); i++){
    //Builds the rotation matrix which the model specifies as:
    // | 1 0 -d*cos(beta) |
    // | 0 1  d*sin(beta) |
    R.block<2,3>(2*i, 0) = geom[legs[i]-1].Rt;
  }

  return R;

}

//Obtains the velocity of the robot in the robot's coordinate frame
Pose3D calcPoseVelocity(Matrix<double, Dynamic, 3> R, Matrix<double, Dynamic, 1> vl){

  //Using the least square method, the velocity of 2D movement may be obtained through:
  //  Vl = R * Vr -> Vr = inv(Rt*R)*R * Vl
  //Where:
  //  Vl = Velocity in leg coordinate frame
  //  R  = Rotation matrix robot -> Leg
  //  Vr = Velocity in robot coordinate frame = [x', y', theta']
  Matrix<double, 3, 1> Vr_2d = (R.transpose()*R).inverse()*R.transpose() * vl;

  //As only planar movement is considered, fills the rest of the pose velocity with
  //null velocity
  //  Vr = [x', y', z' = 0, roll' = 0, pitch' = 0, yaw']
  Pose3D Vr = Pose3D::Zero();
  Vr(0) = Vr_2d(0);
  Vr(1) = Vr_2d(1);
  Vr(5) = Vr_2d(2);

  return Vr;
}

//Function that obtains the new orientation of the robot given the twist velocity
//previously obtained.
inline double getNewOrientation (double prev_orientation, double twist_vel, double prev_twist_vel, double interval){

  return (1.0/2.0)*(twist_vel + prev_twist_vel)*interval + prev_orientation;

}

//Function that transforms the robot velocity from the robot frame to the odometry frame
Pose3D tf_RobotFrame_2_OdomFrame (Pose3D pos, double orientation){

  //Creates the Robot-Odom rotation matrix
  Matrix<double, 3, 3> R = Matrix<double, 3, 3>::Identity();

  R.block<2,2>(0,0) = (Matrix<double, 2, 2>() << cos(orientation), -sin(orientation), sin(orientation), cos(orientation)).finished();

  //Gets only the components of the pose which are related to position
  Matrix<double, 3, 1> p;
  p << pos[0], pos[1], pos[3]; //p = [x', y', z']

  //Obtains the transformed vector
  Matrix<double, 3, 1> pos_tf;
  pos_tf = R * p;

  //Builds the complete pose and returns it
  pos.block<3,1>(0,0) = pos_tf;
  return pos;

}

//Function that integrates the velocity to obtain the new pose
Pose3D getNewPose (Pose3D prev_pose, Pose3D curr_vel, Pose3D prev_vel, double interval){

  return (1.0/2.0)*(curr_vel + prev_vel)*interval + prev_pose;

}

//Function that returns the corresponding quaternion represetation of a 3D rotation
//given the unit vector and the angle.
Matrix<double, 4, 1> rotationQuaternion (double orientation, Matrix<double, 3, 1> u){

  Matrix<double, 4, 1> q = Matrix<double, 4, 1>::Zero();
  q.block<3,1>(0,0) = sin(orientation/2)*u;
  q[3] = cos(orientation/2);

  return q;

}

//Function that builds the odometry msg
nav_msgs::Odometry buildOdomMsg (ros::Time t, Pose3D pose, Pose3D robot_vel){

  nav_msgs::Odometry msg;

  //Sets the header
  msg.header.stamp = t;
  msg.header.frame_id = "odom";

  //Child frame 
  msg.child_frame_id = "base_link";

  //Sets the position
  msg.pose.pose.position.x = pose[0];
  msg.pose.pose.position.y = pose[1];
  msg.pose.pose.position.z = pose[2];

  //Sets the orientation
  Matrix<double,4,1> q = rotationQuaternion(pose[5], {0,0,1});

  msg.pose.pose.orientation.x = q[0];
  msg.pose.pose.orientation.y = q[1];
  msg.pose.pose.orientation.z = q[2];
  msg.pose.pose.orientation.w = q[3];

  //Sets the velocity, the velocity shall be represented in the
  //child frame specified.

  //Linear
  msg.twist.twist.linear.x = robot_vel[0];
  msg.twist.twist.linear.y = robot_vel[1];
  msg.twist.twist.linear.z = robot_vel[2];

  //Angular
  msg.twist.twist.angular.x = robot_vel[3];
  msg.twist.twist.angular.x = robot_vel[4];
  msg.twist.twist.angular.x = robot_vel[5];

  return msg;
}

//---------------------------------------------------------------
//    Callbacks
//---------------------------------------------------------------

//Callback for the state msg;
void leg_state_sub_callback(const clhero_gait_controller::LegState::ConstPtr& msg){

  double interval;

  //Legs on ground
  std::vector <int> legs_on_ground;

  //Checks if the time corresponds to a future state
  if(msg->stamp <= t[1]){
    ROS_INFO("[clhero_odom] Leg state msg received refers to a previous time.");
    return;
  }else{
    //if it corresponds to a new msg updates the time
    t[0] = msg->stamp;
    interval = (t[0] - t[1]).toSec(); 
    t[1] = t[0];
  }

  //Obtains the Legs state
  std::vector<double> legs_pos;
  std::vector<double> legs_vel;

  for(int i=0; i < msg->pos.size(); i++){
    legs_pos.push_back(msg->pos[i]);
    legs_vel.push_back(msg->vel[i]);
  }

  //Once that the msg is identified as a new state, which legs
  //are in contact with the ground shall be identified to 
  //build the rest of the algorithm
  legs_on_ground = getLegsOnGround(legs_pos);

  //Checks if the number of legs in ground is less than 3, which
  //in that case does not updates the velocity since is considered
  //as not moving
  if(legs_on_ground.size() < 3){

    //Legs on ground less than 2

    //in this case the velocity is assumed as zero since with 2 legs is
    //suposed not no move
    robot_velocity[1] = robot_velocity[0];
    robot_velocity[0] = Pose3D::Zero();
    robot_pose[1] = robot_pose[0];

    //builds and sends the msg
    odometry_pub.publish(buildOdomMsg(t[0], robot_pose[0], robot_velocity[0]));

    return;
  }

  //Now that the following situation is identified as a regular
  //motion state, the stored poses are moved as former.
  //  [0] -> Current
  //  [1] -> Former
  robot_pose[1] = robot_pose[0];
  robot_velocity[1] = robot_velocity[0];

  //If the legs on ground are 3 or more, the cycloid velocity of 
  //each leg is obtained
  Matrix<double, 6, 1> cycloid_vel = calcCycloidVel(legs_on_ground, legs_pos, legs_vel);

  //Next, the robot->leg rotation matrix is obtained
  Matrix<double, 6, 3> R = robot_leg_rotation_matrix(legs_on_ground, leg);

  //With both, the velocity of the legs and rotation matrix obtained, 
  //the robot velocity in the robot frame can be calculated as following
  //  Vr = inv(R.transpose()*R)*R.transpose() * V_legs
  Pose3D robot_vel = calcPoseVelocity(R, cycloid_vel);

  //The transformatio to the odometry frame may be obtained through the 
  //new rotation calculated through integration
  robot_velocity[0] = tf_RobotFrame_2_OdomFrame(robot_vel, getNewOrientation(robot_pose[1][5], robot_vel[5], robot_velocity[1][5], interval));

  //And the current pose:
  robot_pose[0] = getNewPose(robot_pose[1], robot_velocity[0], robot_velocity[1], interval);

  //Finally the odometry msg is built and sent
  odometry_pub.publish(buildOdomMsg(t[0], robot_pose[0], robot_vel));

  return;
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

  //Initializes parameters
  t = std::vector<ros::Time>(2, ros::Time::now());

  robot_pose[0] = robot_pose[1] = Pose3D::Zero();
  robot_velocity[0] = robot_velocity[1] = Pose3D::Zero();

  //Odometry publisher
  odometry_pub = nh.advertise<nav_msgs::Odometry>("/odom", 100);

  //Legs' state subscriber
  ros::Subscriber leg_state_sub = nh.subscribe("/clhero_gait_control/legs_state", 1000, leg_state_sub_callback);

  //----------------------------------------------------
  //    Core loop of the node
  //----------------------------------------------------

  ros::spin();

  return 0;

}
