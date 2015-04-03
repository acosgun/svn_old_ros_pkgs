#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <string>
using std::string;
using std::cin;
using std::cout;
using std::endl;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "command_generator_node");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("korus_control_topic", 1);
  string str1;
  cout<<"-------COMMANDS---------"<<endl;
cout<<"f =starts people following"<<endl;
cout<<"i =stops people following"<<endl;
cout<<"q =closes people follower application and this application"<<endl;
cout<<"------------------------"<<endl;
  while (ros::ok())
  {
    std_msgs::String msg;
    cin>>str1;
    msg.data=str1;
    chatter_pub.publish(msg);
    ros::spinOnce();
	  
	if(str1.compare("q")==0)
	{
	sleep(1);
	return EXIT_SUCCESS;
	}

  }
}

