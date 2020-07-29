/*
 * marker_setter_laser.cpp
 *
 *  Created on: 22 Nov 2019
 *      Author: Dominic and Tim
 *
 *      Description: This node subscribes to the output of the face recognition node
 *      			 and the laserscan. It determines the position of the pictures on the walls
 *      			 in the frame of the laserscan and publishes a msg with this position as well as
 *      			 the name of the detected face.
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
#include <geometry_msgs/PointStamped.h>
#include <image_recognition/StringPointStamped.h>

#include <iostream>
#include <iomanip>

using namespace ros;
using namespace std;
using namespace message_filters;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace visualization_msgs;
using namespace opencv_apps;
using namespace image_recognition;

class PointGetter
{

public:
	PointGetter()
	{
		point_pub = n.advertise<StringPointStamped>("/marker_setter/point_laser", 10);

		faces_sub.subscribe(n, "/face_recognition/output", 10);
		laser_sub.subscribe(n, "/vrep/scan", 10);

		sync.reset(new Sync(MySyncPolicy(10), faces_sub, laser_sub));
		sync->registerCallback(boost::bind(&PointGetter::callback, this, _1, _2));

	}

	void callback(const FaceArrayStampedConstPtr &array, const LaserScanConstPtr &laser)
	{
		int number_of_faces = array->faces.size();

		//only consider a face only 1 face is detected in current image
		if(number_of_faces == 1){
			Face first_face = array->faces[0];

			Rect rectangle = first_face.face;
			float x = rectangle.x;
			float y = rectangle.y;
			float width = rectangle.width;
			float height = rectangle.height;
			string label = first_face.label;

			//only consider the face if it is in upper area of the image (on the wall, not the floor)
			if(y<300){
				float number_of_measurements = laser->ranges.size();
				float angle_laser = 180;
				float step_size = angle_laser / number_of_measurements;
				float start_angle = ((angle_laser) -45) /2;
				float distance;
				float alpha;
				float index;
				float x_laser;
				float y_laser;
				float z_laser = 0;

				//convert position in image to angle of the laser to choose corresponding index
				float pixel_degree = 45.0/512.0;
				alpha = start_angle + x * pixel_degree;

				index = (int) alpha/step_size;

				distance = laser->ranges[index];
				x_laser = distance * cos(alpha*(M_PI/180));
				y_laser = - distance * sin(alpha*(M_PI/180));

				//Only consider the face if it is not too far away,
				//because position estimate would be more inaccurate
				if(distance < 4.5 && width > 60){
					geometry_msgs::PointStamped laser_point;
					laser_point.point.x = x_laser;
					laser_point.point.y = y_laser;
					laser_point.point.z = z_laser;
					laser_point.header.frame_id = "laser_link";
					laser_point.header.stamp = Time();
					laser_point.header.seq = 0;

					cout << "label laser: " << label << endl;
					StringPointStamped string_point;
					string_point.person = label;
					string_point.point = laser_point;
					string_point.header.seq = 0;
					string_point.header.stamp = Time();
					string_point.header.frame_id = "laser_link";

					point_pub.publish(string_point);
				}

			}

		}

		return;

	}

private:

	NodeHandle n;
	Publisher point_pub;

	message_filters::Subscriber<FaceArrayStamped> faces_sub;
	message_filters::Subscriber<LaserScan> laser_sub;

	typedef sync_policies::ApproximateTime<FaceArrayStamped, LaserScan> MySyncPolicy;
	typedef Synchronizer<MySyncPolicy> Sync;
	boost::shared_ptr<Sync> sync;

};

int main(int argc, char **argv)
{
	init(argc, argv, "marker_setter_laser");

	PointGetter pg;

	spin();

	return 0;
}



