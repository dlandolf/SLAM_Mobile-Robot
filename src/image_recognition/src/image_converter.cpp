/*
 * image_converter.cpp
 *
 *  Created on: 13 Nov 2019
 *      Author: Dominic and Tim
 *
 *      Description: This node subscribes to the topic "/vrep/image"
 *      			 publishes the flipped image in "/vrep/image_flipped".
 *
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;
using namespace cv_bridge;

class ImageConverter
{
  ros::NodeHandle n;
  ros::Subscriber image_sub_;
  ros::Publisher image_pub_;

public:
  ImageConverter()
  {
    image_sub_ = n.subscribe("/vrep/image", 1000, &ImageConverter::imageCallBack, this);
    image_pub_ = n.advertise<sensor_msgs::Image>("/vrep/image_flipped", 1000);
  }

  void imageCallBack(const sensor_msgs::ImageConstPtr& msg)
  {
    CvImagePtr cv_ptr;
    cv_ptr = toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    CvImagePtr cv_ptr_flipped;
	Mat originalImg,flippedImg;
	originalImg = cv_ptr->image;

	flip(originalImg,flippedImg,1);
	cv_ptr_flipped = cv_ptr;
	cv_ptr_flipped->image = flippedImg;

    image_pub_.publish(cv_ptr_flipped->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}




