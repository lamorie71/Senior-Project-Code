// Filename:                     	camera_capture.cpp
// Creation Date:					04/20/2022
// Last Revision Date:			04/20/2022
// Author(s) [email]:				Taylor Lamorie [tlamorie@lssu.edu]
// Revisor(s) {Date}:        
// Organization/Institution:	Lake Superior State University

//...................................................About camera_capture.cpp.....................................................................
//subscribes to both the right and left camera on the Zed2i and uses opencv to convert into a showable image


//................................................Included Libraries and Message Types..........................................
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "zed_interfaces/ObjectsStamped.h"
#include "zed_interfaces/Object.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "iostream"
#include "stdio.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <std_msgs/String.h>
#include <iostream>
#include <vector>
#include <string>

using namespace cv;
using namespace std;


// THIS FUNCTION: Subscribed to the Right side of the camera and uses opencv to show in a window 
// ACCEPTS: (sensor_msgs/Image pointer)
// RETURNS: (VOID)
// =============================================================================
//To use the picture option set right_camera to 1 buy using rosparam set right_camera 1  //A piture will be taken every 5 seconds set right_camera back to 0 when done
void imageRightRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	try 
	{
		ROS_INFO("Right Rectified image received from ZED - Size: %dx%d", msg->width, msg->height);
		cv::Mat org_img;
		org_img = cv_bridge::toCvShare(msg, "bgr8") -> image;		//converts message camera_msg into bgr8 type image
		cv::imshow("Right_side", org_img);                                     //displays the image in a window of default size
		
		int s;
		if (ros::param::get("/right_camera", s))                                   //set param to 1 to take a picture 
		{
			ROS_INFO("Got param for right");
			if (s == 1)
			{
				static int image_count_right;                                //static varaible to keep count of picture 
				std::stringstream sstream;                               //string object converted to stream for saving 
				sstream << "my_image_right" << image_count_right << ".png" ;                  //name of image file 
				ROS_ASSERT(cv::imwrite(sstream.str(),  org_img));      //runs the imwrite command 
				image_count_right++; 
				ros::Duration(5.0).sleep();  //waits 5 seconds until next picture is taken
			}                                     //counts image
        }
        else
        {
			ROS_ERROR("Failed to get param 'right_camera");
        }
	
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e) //looks for errors 
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg -> encoding.c_str()); //prints out the encoding string
	}
} //END

// THIS FUNCTION: Subscribed to the Left side of the camera and uses opencv to show in a window 
// ACCEPTS: (sensor_msgs/Image pointer)
// RETURNS: (VOID)
// =============================================================================
//To use the picture option set left_camera to 1 buy using rosparam set left_camera 1  //A piture will be taken every 5 seconds set left_camera back to 0 when done
void imageLeftRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	try 
	{
		ROS_INFO("Left Rectified image received from ZED - Size: %dx%d", msg->width, msg->height);
		cv::Mat org_img1;
		org_img1 = cv_bridge::toCvShare(msg, "bgr8") -> image;		//converts message camera_msg into bgr8 type image
		cv::imshow("Left_side", org_img1);                                     //displays the image in a window of default size
		
		int s1;
		if (ros::param::get("/left_camera", s1))                                  //left camera param set to 1
		{
			ROS_INFO("Got param for left");
			if (s1 = 1)
			{
				static int image_count_left;                                //static varaible to keep count of picture 
				std::stringstream sstream1;                                  //string object converted to stream for saving 
				sstream1 << "my_image_left" << image_count_left << ".png" ;                    //name of image file 
				ROS_ASSERT(cv::imwrite(sstream1.str(),  org_img1));       //runs the imwrite command 
				image_count_left++; 
				ros::Duration(5.0).sleep();  
			}                                        //counts image
        }
        else
        {
			ROS_ERROR("Failed to get param 'left_camera");
        }
		
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e) //looks for errors 
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg -> encoding.c_str()); //prints out the encoding string
	}
} //END

// THIS FUNCTION: Subscribed to messages below to give a ist of detected objects using zed classes
// ACCEPTS: (zed_interface/Object and zed_interface/ObjectsStamped) these are custom messages that are apart of the zed wrapper
// RETURNS: (VOID)
// =============================================================================

void objectListCallback(const zed_interfaces::ObjectsStamped::ConstPtr& msg)
{
  ROS_INFO("***** New object list *****");
  for (int i = 0; i < msg->objects.size(); i++) //loop for all objects detectec
  {
    if (msg->objects[i].label_id == -1)
      continue;

    ROS_INFO_STREAM(msg->objects[i].label << " [" << msg->objects[i].label_id << "] - Pos. ["
                                          << msg->objects[i].position[0] << "," << msg->objects[i].position[1] << ","
                                          << msg->objects[i].position[2] << "] [m]"
                                          << "- Conf. " << msg->objects[i].confidence
                                          << " - Tracking state: " << static_cast<int>(msg->objects[i].tracking_state));
  }
} //END

//...............................................................Main Program..............................................................
int main(int argc, char** argv)
{
	ros::init(argc, argv, "zed_video_subscriber");
  
  	cv::namedWindow("Right_side", cv::WINDOW_AUTOSIZE);					
	cv::namedWindow("Left_side", cv::WINDOW_AUTOSIZE);
  
	ros::NodeHandle n, n1, n2;
  
  	image_transport::ImageTransport it1(n);		// transports the images from published node to subscriber
	image_transport::ImageTransport it2(n1);

	image_transport::Subscriber subRightRectified = it1.subscribe("/zed2i/zed_node/right/image_rect_color", 1, imageRightRectifiedCallback); //subscribes to right camera rectified 
	image_transport::Subscriber subLeftRectified = it2.subscribe("/zed2i/zed_node/left/image_rect_color", 1, imageLeftRectifiedCallback);  //subscribes to left camera rectified 

	ros::Subscriber object_detection = n2.subscribe("/zed2i/zed_node/obj_det/objects", 1, objectListCallback); //subscribes to the object detection node /zed2i/zed_node/obj_det/objects

	ros::spin();

	cv::destroyWindow("Right_side"); //destroys the new windows
	cv::destroyWindow("Left_side");
	
	ros::Rate loop_rate(1);
	
	while(ros::ok()) {
	
	ros::spinOnce();
	loop_rate.sleep();
	
	}
	
}

