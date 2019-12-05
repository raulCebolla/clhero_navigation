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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <string>

//---------------------------------------------------------------
//    Global variables
//---------------------------------------------------------------

ros::Publisher odom_fill_pub;

std::string child_frame_id;

//---------------------------------------------------------------
//    Callbacks
//---------------------------------------------------------------

void odom_combined_callback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg){

	nav_msgs::Odometry odom_msg;

	//Copies the header and the pose
	odom_msg.header = pose_msg->header;
	odom_msg.pose = pose_msg->pose;

	//Sets the child frame id
	odom_msg.child_frame_id = child_frame_id;

	//Builds the twist part
	odom_msg.twist.twist.linear.x = 0;
	odom_msg.twist.twist.linear.y = 0;
	odom_msg.twist.twist.linear.z = 0;

	odom_msg.twist.twist.angular.x = 0;
	odom_msg.twist.twist.angular.y = 0;
	odom_msg.twist.twist.angular.z = 0;

	odom_msg.twist.covariance = {0, 0, 0, 0, 0, 0,
								 0, 0, 0, 0, 0, 0,
								 0, 0, 0, 0, 0, 0,
								 0, 0, 0, 0, 0, 0,
								 0, 0, 0, 0, 0, 0,
								 0, 0, 0, 0, 0, 0};

	odom_fill_pub.publish(odom_msg);		

	return;					 

}


//---------------------------------------------------------------
//    Main Function
//---------------------------------------------------------------


int main (int argc, char** argv){


	ros::init(argc, argv, "odom_filler");

	ros::NodeHandle nh;
	ros::NodeHandle nh_private ("~");

	//Params
	nh_private.param("base_frame", child_frame_id, std::string("base_footprint"));

	ros::Subscriber odom_combined_sub = nh.subscribe("/odom_combined", 1000, odom_combined_callback);
	odom_fill_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1000);

	while(ros::ok()){
		ros::spin();
	}

	return 0;

}