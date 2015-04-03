#pragma once

#include <home_obj_recognizer/PanCamera.h>
#include <highgui.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/CvBridge.h>

class PersonRecognizerProsilica {
 protected:
  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::Image> image_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;
  sensor_msgs::CvBridge m_bridge;
  
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> sync;
  ros::Publisher pt_cloud_pub;
  ros::Timer timer_;

 public:
  PersonRecognizerProsilica();
  
  bool ReceiveImage(const sensor_msgs::ImageConstPtr& img, 
		    const sensor_msgs::CameraInfoConstPtr& cam_info);
  void timerCallback(const ros::TimerEvent& e);
};
