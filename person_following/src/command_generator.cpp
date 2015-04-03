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
  cout<<"r_i : recognizer idle state"<<endl;
  cout<<"r_r : recognizer recognize state"<<endl;
  cout<<"r_l_name : recognizer learn person called 'name'"<<endl;
  cout<<"r_s_name : recognizer save person called 'name' to database"<<endl;
  cout<<"r_q : recognizer node quit"<<endl;

  cout<<"f_i : follower, idle state"<<endl;
  cout<<"f_f : follower, start following"<<endl;
  cout<<"f_w : follower, wait for 10 sec"<<endl;
  cout<<"f_s : follower, stop and wait for MASTER"<<endl;
  cout<<"r_q : follower, node quit"<<endl;

  cout<<"g_r_complete : global msg from recognizer, learning completed"<<endl;

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

