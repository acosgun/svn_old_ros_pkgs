#include <PersonRecognizerProsilica.h>

PersonRecognizerProsilica::
PersonRecognizerProsilica () :
  nh_("~"),
  image_sub(nh_,"/prosilica/image_rect_color", 1),
  info_sub(nh_, "/prosilica/camera_info", 1),
  sync (image_sub, info_sub, 1){
  //  this->timer_ = nh_.createTimer(ros::Duration(0.01),&PersonRecognizerProsilica::timerCallback,this);
  sync.registerCallback(boost::bind(&PersonRecognizerProsilica::ReceiveImage, this, _1, _2));

}
/*
void PersonRecognizerProsilica::
timerCallback(const ros::TimerEvent& e) 
{
ROS_INFO("timer triggered");
RequestImage();
}
*/
bool PersonRecognizerProsilica::
ReceiveImage(const sensor_msgs::ImageConstPtr& img, 
	     const sensor_msgs::CameraInfoConstPtr& cam_info) {
  IplImage* iplimg = m_bridge.imgMsgToCv(img);
  IplImage* image = cvCloneImage(iplimg);
  cvNamedWindow("Test",1);
  cvShowImage("Test", image);
  cvWaitKey(10);
  cvReleaseImage(&image);
  return true;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "PersonRecognizerProsilica");
  PersonRecognizerProsilica ps;
  ros::spin();
}
