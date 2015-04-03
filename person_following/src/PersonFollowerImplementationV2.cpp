#include <iostream>
#include <PersonFollowerV2.h>


int main(int argc,char** argv) 
{
  ros::init(argc,argv, "PersonFollowerV2");
  PersonFollower follower;
  ros::spin();
  
  return 0;
}
