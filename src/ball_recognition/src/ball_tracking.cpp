/*
 * ball_tracking.cpp
 *
 *  Created on: 17 Nov 2019
 *      Author: Dominic and Tim
 *
 * 		Description: This node subscribes to the topic "/ball_position" and uses a controller with
 * 					 a p-gain to publish the command velocity "/vrep/cmd_vel".
 * 					 It also turns off the Lidar.
 *
 */

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <iostream>

using namespace std;

class BallTracker
{
	ros::NodeHandle n;
	ros::Subscriber ballPos_sub_;
	ros::Publisher vel_pub_;
	ros::Publisher lidar_switch_pub_;
	geometry_msgs::Twist prior_msg;

	double p_gain_turn = 0.01;
	double p_gain_x = 0.1;
	double des_circle_size = 115;

public:
	BallTracker()
	{
		ballPos_sub_ = n.subscribe("/ball_position", 1000, &BallTracker::ballTrackerCallback, this);
		vel_pub_ = n.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 1000);
		lidar_switch_pub_ = n.advertise<std_msgs::Bool>("/vrep/laser_switch", 10);
	}

	void ballTrackerCallback(const geometry_msgs::Point::ConstPtr& circle)
	{
		std_msgs::Bool laser_switch;
		laser_switch.data = false;
		lidar_switch_pub_.publish(laser_switch);

		geometry_msgs::Twist msg;

		msg.angular.z = 0;
		msg.linear.x = 0;

		//apply angular velocity if circle is too far away from the center of the image
		if(abs(circle->x) > 50 ){
			double angular_z = -p_gain_turn*circle->x;
			msg.angular.z = angular_z;
		}

		//otherwise, apply linear velocity depending on the size of the ball
		//to keep the same distance from the ball.
		else{
			double circle_size_err = des_circle_size - circle->z;
			double linear_x = p_gain_x * circle_size_err;

			//set max velocity to 1.5
			if (linear_x > 1.5){
				msg.linear.x = 1.5;
			}
			else if (linear_x < -1.5){
				msg.linear.x = -1.5;
			}
			else {
				msg.linear.x = linear_x;
			}

		}

		//if the circle is very far away, don't start to track it yet
		if (circle->z < 5){
			msg.angular.z = 0;
			msg.linear.x = 0;
			prior_msg = msg;
			vel_pub_.publish(msg);
			return;
		}

		prior_msg = msg;
		vel_pub_.publish(msg);
		return;
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ball_tracker");
	BallTracker br;

	ros::spin();

	return 0;
}


