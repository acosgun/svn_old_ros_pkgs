#include <PersonRecognizerRobocup.h>
#include <iostream>
//using std::cout;
//using std::endl;


int main(int argc,char** argv) 
{
  ros::init(argc,argv, "PersonRecognizerRobocup");
  PersonRecognizer recognizer; //use this once.

  ros::spin();
  
  return 0;
}



