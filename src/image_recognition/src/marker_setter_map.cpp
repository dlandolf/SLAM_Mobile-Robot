/*
 * marker_setter_map.cpp
 *
 *  Created on: 22 Nov 2019
 *      Author: Dominic and Tim
 *
 *      Description: This node converts the position of the detected pictures on the wall from the
 *      			 laserscan frame to the map frame and publishes markers to indicate the position
 *      			 of these pictures in the map as well as the name of the detected person.
 */

#include "ros/ros.h"
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv_apps/FaceArrayStamped.h>
#include <opencv_apps/FaceArray.h>
#include <opencv_apps/Face.h>
#include <opencv_apps/Rect.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <image_recognition/StringPointStamped.h>
#include <iostream>
#include <queue>

using namespace ros;
using namespace std;
using namespace message_filters;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace visualization_msgs;
using namespace opencv_apps;
using namespace tf2_ros;
using namespace image_recognition;

class MarkerSetter
{

public:
	MarkerSetter():
		tf2(tfBuffer), tf2_filter(point_laser, tfBuffer, "map", 10, 0)
	{
		marker_pub = n.advertise<visualization_msgs::MarkerArray>("rviz/DisplayTypes/MarkerArray", 10);

		point_laser.subscribe(n, "/marker_setter/point_laser", 10);

		tf2_filter.registerCallback( boost::bind(&MarkerSetter::callback, this, _1) );

	}

	void callback(const StringPointStampedConstPtr &laser_point)
	{

		geometry_msgs::PointStamped map_point;

		try
		    {
		      tfBuffer.transform(laser_point->point, map_point, "map");
		    }
		    catch (tf2::TransformException &ex)
		    {
		      ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
		    }

		bool new_image = true;

		int size = marker_array.markers.size();

		//don't set new marker if there is already a marker closeby, so the same picture is not marked more than once.
		for(int i = 0; i < size; i++){
			Marker temp = marker_array.markers[i];

			float distance = sqrt(pow((map_point.point.x - temp.pose.position.x),2) + pow((map_point.point.y - temp.pose.position.y),2));

			if(distance<2){
				new_image = false;
			}
		}

		//only set a marker if a face is detected 3 times at approximately the same position
		int queue_size = candidates.size();
		int count = 0;
		for(int i = 0; i < queue_size; i++){
			PointStamped temp = candidates.front();
			if(distance(temp, map_point)<0.5){
				count++;
			}

			candidates.push(temp);
			candidates.pop();
		}
		if(count<3){
			new_image = false;

			while(queue_size > 15){
				candidates.pop();
				queue_size = candidates.size();
			}

			candidates.push(map_point);
		}
		else{
			queue_size = candidates.size();
			for(int i = 0; i < queue_size; i++){
				candidates.pop();
			}
		}

		if(new_image == true){
			//set marker and textmarker which will be published
			Marker marker;

			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time();
			marker.id = 0;
			marker.type = Marker::SPHERE;
			marker.action = Marker::ADD;
			marker.pose.position.x = map_point.point.x;
			marker.pose.position.y = map_point.point.y;
			marker.pose.position.z = map_point.point.z;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.3;
			marker.scale.y = 0.3;
			marker.scale.z = 0.3;
			marker.color.a = 1.0; // Don't forget to set the alpha!
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;

			Marker text_marker;

			text_marker.header.frame_id = "map";
			text_marker.header.stamp = ros::Time();
			text_marker.id = 1;
			text_marker.type = Marker::TEXT_VIEW_FACING;
			text_marker.action = Marker::ADD;
			text_marker.pose.position.x = map_point.point.x;
			text_marker.pose.position.y = map_point.point.y - 0.3;
			text_marker.pose.position.z = map_point.point.z + 0.3;
			text_marker.scale.z = 0.75;
			text_marker.color.a = 1.0; // Don't forget to set the alpha!
			text_marker.color.r = 1.0;
			text_marker.color.g = 0.0;
			text_marker.color.b = 0.0;

			text_marker.text =laser_point->person;

			text_marker.ns = text_marker.text;
			marker.ns = text_marker.text;

			marker_array.markers.push_back(marker);
			marker_array.markers.push_back(text_marker);

		}

		marker_pub.publish(marker_array);

		return;

	}

	float distance(PointStamped point_1, PointStamped point_2){
		return sqrt(pow((point_1.point.x - point_2.point.x),2) + pow((point_1.point.y - point_2.point.y),2));
	}

private:

	NodeHandle n;

	Publisher marker_pub;
	MarkerArray marker_array;

	queue <PointStamped> candidates;

	message_filters::Subscriber<StringPointStamped> point_laser;

	Buffer tfBuffer;
	TransformStamped transformStamped;
	MessageFilter<StringPointStamped> tf2_filter;
	TransformListener tf2;

};

int main(int argc, char **argv)
{
	init(argc, argv, "marker_setter_map");

	MarkerSetter ms;

	spin();

	return 0;
}



