/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2018 \n
 *   TU Delft
 *
 *****************************************************************
 *
 * \note
 *   Project name:
 * \note
 *   ROS stack name:
 * \note
 *   ROS package name: mobile_robot_state_publisher
 *
 * \author
 *   Author: Bruno Brito, email: Bruno.deBrito@tudelft.nl
 *
 * \date Date of creation: May, 2018
 *
 * \brief
 *   This package provides a generic mobile_robot_stsate_publisher
 *
 ****************************************************************/

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/SetLinkState.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf/transform_broadcaster.h>

using namespace std;

geometry_msgs::Pose pos;
geometry_msgs::Twist vel;
nav_msgs::Odometry odom_msg;


void VelocityCallBack(const gazebo_msgs::LinkStates& msg){
	int index=0;
	std::string str2 ("base_link");
	for(int i =0; i< sizeof(msg.name);i++){

		if(msg.name[i].find(str2)!=std::string::npos)
			break;
		index+=1;
	}
	//ROS_INFO_STREAM("fOUND IN: " << index);
    pos = msg.pose[index];
    odom_msg.twist.twist = msg.twist[index];
    odom_msg.pose.pose = msg.pose[index];
    odom_msg.child_frame_id="base_link";
    odom_msg.header.frame_id="odom";
    odom_msg.header.stamp=ros::Time::now();
/* This is done because the ukf does not match the position in Gazebo */
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(pos.position.x,pos.position.y,pos.position.z) );
	tf::Quaternion q(pos.orientation.x,pos.orientation.y,pos.orientation.z,pos.orientation.w);

	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mobile_robot_state_publisher_node");
	ros::NodeHandle n;
	ros::Subscriber robot_state_sub_;


	double node_rate;
	if (!n.getParam(ros::this_node::getName()+"/rate", node_rate))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName()+"/rate not set");
		return 0;
	}

	string root_frame;
	if (!n.getParam(ros::this_node::getName()+"/root_frame", root_frame))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName()+"/root_frame not set");
		return 0;
	}

	string base_frame;
	if (!n.getParam(ros::this_node::getName()+"/base_frame", base_frame))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName()+"/base_frame not set");
		return 0;
	}

	string robot_state_topic;
	if (!n.getParam(ros::this_node::getName()+"/robot_state_topic", robot_state_topic))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName()+"/robot_state_topic not set");
		return 0;
	}

	string vel_state_topic;
	if (!n.getParam(ros::this_node::getName()+"/vel_state_topic", vel_state_topic))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName()+"/vel_state_topic not set");
		return 0;
	}

    robot_state_sub_ = n.subscribe(vel_state_topic, 1, VelocityCallBack);

	ros::Publisher state_pub_ =
		n.advertise<geometry_msgs::PoseStamped>(robot_state_topic, 10);
	//ros::Publisher link_state_pub_ =
	//		n.advertise<geometry_msgs::Pose>("/gazebo/set_link_state", 10);
	ros::ServiceClient link_state_client_ = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");

	gazebo_msgs::SetLinkState link;
	link.request.link_state.link_name="base_link";

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	ros::Rate rate(node_rate);
	geometry_msgs::PoseStamped pose_msg;

    ros::Publisher odom_pub_ = n.advertise<nav_msgs::Odometry>("/odometry/filtered", 10);

	//Intermidiate variables
	double ysqr, t3, t4;
	geometry_msgs::TransformStamped transformStamped;
	ros::Rate r(20.0);
	while (n.ok()){
		/*
		try{
			transformStamped = tfBuffer.lookupTransform(root_frame, base_frame,
														ros::Time(0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		//CONVERT FROM QUATERNION TO JOINT ANGLE ROTATION
		*/
		pose_msg.header.frame_id = "odom";
		pose_msg.header.stamp = ros::Time::now();;

		ysqr = odom_msg.pose.pose.orientation.y * odom_msg.pose.pose.orientation.y;
		t3 = +2.0 * (odom_msg.pose.pose.orientation.w * odom_msg.pose.pose.orientation.z
					 + odom_msg.pose.pose.orientation.x * odom_msg.pose.pose.orientation.y);
		t4 = +1.0 - 2.0 * (ysqr + odom_msg.pose.pose.orientation.z * odom_msg.pose.pose.orientation.z);

		pose_msg.pose.orientation.z = atan2(t3, t4);
		pose_msg.pose.position.x = odom_msg.pose.pose.position.x;
		pose_msg.pose.position.y = odom_msg.pose.pose.position.y;

		pose_msg.pose.position.z = std::sqrt(std::pow(odom_msg.twist.twist.linear.x,2)+std::pow(odom_msg.twist.twist.linear.y,2));

		state_pub_.publish(pose_msg);
		/*
		link.request.link_state.pose.position.x = transformStamped.transform.translation.x;
		link.request.link_state.pose.position.y = transformStamped.transform.translation.y;
		link.request.link_state.pose.position.z = transformStamped.transform.translation.z;
		link.request.link_state.pose.orientation.x = transformStamped.transform.rotation.x;
		link.request.link_state.pose.orientation.y = transformStamped.transform.rotation.y;
		link.request.link_state.pose.orientation.z = transformStamped.transform.rotation.z;
		link.request.link_state.pose.orientation.w = transformStamped.transform.rotation.w;
		link_state_client_.call(link);
		 */
		//link_state_pub_.publish(link);
        odom_pub_.publish(odom_msg);
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
