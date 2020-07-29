/*
 * ball_recognition.cpp
 *
 *  Created on: 17 Nov 2019
 *      Author: Dominic and Tim
 *
 * 		Description: This node subscribes to "/vrep/image", flips it, filters out non-yellow colors
 * 					 and detects a circle (if there is one).
 * 					 It publishes the topic "/ball_position": the distance of the center of the ball
 * 					 (the detected circle) to the center of the image.
 * 					 For debugging purposes it also publishes "/image_processed": the color filtered image.
 *
 */

#include <opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;
using namespace cv_bridge;

class BallRecognizer
{
	ros::NodeHandle n;
	ros::Subscriber image_sub_;
	ros::Publisher ball_pub_;
	ros::Publisher image_pub_;

public:
	BallRecognizer()
	{
		image_sub_ = n.subscribe("/vrep/image", 1000, &BallRecognizer::ballRecognitionCallback, this);
		ball_pub_ = n.advertise<geometry_msgs::Point>("/ball_position", 1000);
		image_pub_ = n.advertise<sensor_msgs::Image>("/image_processed", 1000);
	}


	void ballRecognitionCallback(const sensor_msgs::Image::ConstPtr& img)
	{

		int height = 512;
		int width = 512;

		CvImagePtr cv_img_ptr;
		Mat originalImg,flippedImg;
		cv_img_ptr = toCvCopy(img, sensor_msgs::image_encodings::BGR8);
		originalImg = cv_img_ptr->image;
		flip(originalImg,flippedImg,1);

		Mat frame = flippedImg;
		Mat hsv_frame;
		Mat filtered_frame;
		Mat smoothed_frame;
		Mat gray_frame;

		// Convert color space to HSV as it is easier to filter colors in the HSV color-space.
		cvtColor(frame, hsv_frame, CV_BGR2HSV);

		// Filter out colors which are out of range.
		Mat mask1;
		inRange(hsv_frame, Scalar(0,100,100), Scalar(100,255,255), mask1);

		Mat kernel = Mat::ones(3,3,CV_32F);
		morphologyEx(mask1,mask1,cv::MORPH_OPEN,kernel);
		morphologyEx(mask1,mask1,cv::MORPH_DILATE,kernel);

		bitwise_and(frame,frame, filtered_frame, mask1);

		cv_bridge::CvImagePtr cv_ptr_processed = cv_img_ptr;
		cv_ptr_processed->image = filtered_frame;
		image_pub_.publish(cv_ptr_processed->toImageMsg());

		//make frame gray
		cvtColor(filtered_frame, gray_frame, CV_BGR2GRAY);

		//smooth for circle detection
		GaussianBlur(gray_frame, smoothed_frame, Size(9, 9), 2, 2);

		//detect the ball as a circle with Hough circles
		vector<Vec3f> circles;
		HoughCircles(smoothed_frame, circles, CV_HOUGH_GRADIENT, 2,smoothed_frame.rows/4,100, 75, 0, 1000);

		//create a msg with the relative location of the center of the circle to the middle of the image
		//and store the radius of the circle in the z component of the msg
		geometry_msgs::Point msg;
		if(circles.size() > 0){
			msg.x = circles[0][0]-260;
			msg.y = 260 - circles[0][1];
			msg.z = circles[0][2];
		}

		ball_pub_.publish(msg);
		return;
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ball_recognition");
	BallRecognizer br;
	ros::spin();

	return 0;
}



