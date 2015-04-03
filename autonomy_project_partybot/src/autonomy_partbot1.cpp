#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
using namespace std;
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <std_msgs/String.h>
#include <string>
using std::string;

#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <geometry_msgs/Twist.h>

#include <geometry_msgs/PointStamped.h> //tf
#include <tf/transform_listener.h> //tf


#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include "Socket.h"
#include "ClientSocket.h"
#include "SocketException.h"



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


void sendStringViaSocket(const string msg)
//void sendStringViaSocket(std_msgs::String msg)
{
//ClientSocket client("192.168.1.51",1000);
ClientSocket client("143.215.105.163",1000);
ROS_INFO("Sending string thru socket");
cout<<"Message is: "<<msg<<endl;
client<<msg;
}


void commandCallback  (const std_msgs::StringConstPtr& msg)
{

ROS_INFO("Rcvd string: [%s]", msg->data.c_str());
//sendStringViaSocket("SUCCEEDED");


string rcvd=msg->data.c_str();


static MoveBaseClient ac("move_base", true);

	if(rcvd.compare("q")==0) //quit program
	{
	
	}
	if(rcvd.compare("table1 drink")==0)
	{
	  cout<<"Receiving navigation command - go to table1 drink"<<endl;
          geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(-1.98);
				  move_base_msgs::MoveBaseGoal goal;
				  goal.target_pose.header.frame_id = "map";
				  goal.target_pose.header.stamp = ros::Time::now();
				  goal.target_pose.pose.position.x = 94.035;
				  goal.target_pose.pose.position.y = 98.076;
				  goal.target_pose.pose.orientation=quat;		
				  ac.waitForServer();
				  cout<<"Before send goal"<<endl;				  
				  ac.sendGoal(goal);
				  cout<<"After send goal"<<endl;
				  cout<<"Before wait for result"<<endl;
				  ac.waitForResult();
				  cout<<"After wait for result"<<endl;
				if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
				cout<<"NAV TO TABLE1 DRINK SUCCESSFUL!"<<endl;
				ROS_INFO("NAV TO TABLE1 DRINK SUCCESSFUL!");
				sendStringViaSocket("TABLE1 DRINK SUCCEEDED");
				}

	}
	if(rcvd.compare("table1 napkin")==0)
	{
	  cout<<"Receiving navigation command - go to table1 napkin"<<endl;
          geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(-1.98);

				  move_base_msgs::MoveBaseGoal goal;
				  goal.target_pose.header.frame_id = "map";
				  goal.target_pose.header.stamp = ros::Time::now();
				  goal.target_pose.pose.position.x = 94.035;
				  goal.target_pose.pose.position.y = 98.076;
				  goal.target_pose.pose.orientation=quat;		
				  ac.waitForServer();				  
				  ac.sendGoal(goal);
			  	  ac.waitForResult();
				if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
				cout<<"NAV TO TABLE1 NAPKIN SUCCESSFUL!"<<endl;
				ROS_INFO("NAV TO TABLE1 NAPKIN SUCCESSFUL!");
				sendStringViaSocket("TABLE1 NAPKIN SUCCEEDED");
				}

	}
	if(rcvd.compare("table1 check")==0)
	{
	  cout<<"Receiving navigation command - go to table1 check"<<endl;
          geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(-1.98);

				  move_base_msgs::MoveBaseGoal goal;
				  goal.target_pose.header.frame_id = "map";
				  goal.target_pose.header.stamp = ros::Time::now();
				  goal.target_pose.pose.position.x = 94.035;
				  goal.target_pose.pose.position.y = 98.076;
				  goal.target_pose.pose.orientation=quat;		
				  ac.waitForServer();				  
				  ac.sendGoal(goal);
			  	  ac.waitForResult();
				if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
				cout<<"NAV TO TABLE1 CHECK SUCCESSFUL!"<<endl;
				ROS_INFO("NAV TO TABLE1 CHECK SUCCESSFUL!");
				sendStringViaSocket("TABLE1 CHECK SUCCEEDED");
				}

	}
	if(rcvd.compare("table1 service")==0)
	{
	  cout<<"Receiving navigation command - go to table1 service"<<endl;
          geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(-1.98);

				  move_base_msgs::MoveBaseGoal goal;
				  goal.target_pose.header.frame_id = "map";
				  goal.target_pose.header.stamp = ros::Time::now();
				  goal.target_pose.pose.position.x = 94.035;
				  goal.target_pose.pose.position.y = 98.076;
				  goal.target_pose.pose.orientation=quat;					
				  ac.waitForServer();	  
				  ac.sendGoal(goal);
			  	  ac.waitForResult();
				if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
				cout<<"NAV TO TABLE1 SERVICE SUCCESSFUL!"<<endl;
				ROS_INFO("NAV TO TABLE1 SERVICE SUCCESSFUL!");
				sendStringViaSocket("TABLE1 SERVICE SUCCEEDED");
				}

	}
	if(rcvd.compare("table1 norecognize")==0)
	{
	  cout<<"Receiving navigation command - go to table1 norecognize"<<endl;
          geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(-1.98);

				  move_base_msgs::MoveBaseGoal goal;
				  goal.target_pose.header.frame_id = "map";
				  goal.target_pose.header.stamp = ros::Time::now();
				  goal.target_pose.pose.position.x = 94.035;
				  goal.target_pose.pose.position.y = 98.076;
				  goal.target_pose.pose.orientation=quat;		
				  ac.waitForServer();				  
				  ac.sendGoal(goal);
			  	  ac.waitForResult();
				if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
				cout<<"NAV TO TABLE1 NORECOGNIZE SUCCESSFUL!"<<endl;
				ROS_INFO("NAV TO TABLE1 NORECOGNIZE SUCCESSFUL!");
				sendStringViaSocket("TABLE1 NORECOGNIZE SUCCEEDED");
				}

	}
	if(rcvd.compare("kitchen")==0)
	{
          cout<<"Receiving navigation command - go to kitchen"<<endl;
          geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(-0.169);
				  move_base_msgs::MoveBaseGoal goal;
				  goal.target_pose.header.frame_id = "map";
				  goal.target_pose.header.stamp = ros::Time::now();
				  goal.target_pose.pose.position.x = 101.915;
				  goal.target_pose.pose.position.y = 99.996;
				  goal.target_pose.pose.orientation=quat;	
				  ac.waitForServer();					  
				  ac.sendGoal(goal);
				  ac.waitForResult();
				if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
				cout<<"NAV TO KITCHEN SUCCESSFUL!"<<endl;
				ROS_INFO("NAV TO KITCHEN SUCCESSFUL!");
				sendStringViaSocket("KITCHEN SUCCEEDED");
				}
	}

} 





int main(int argc, char** argv)
{

  ros::init(argc, argv, "autonomy_partbot_node");
  ros::NodeHandle n;

  
  //tf::TransformListener listener(ros::Duration(10)); //tf
  ros::Subscriber chatter_sub2=n.subscribe("korus_control_topic",1,commandCallback);  
  tf::TransformListener listener(ros::Duration(10)); //tf	

/*  
  goal.target_pose.pose.position.x = 102.099189758;//102.077212845;
  goal.target_pose.pose.position.y = 100.080055237;//99.8262;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = -0.758970317261;//-0.813337;
  goal.target_pose.pose.orientation.w =  0.651125224144;//0.5817919;
*/
	/*try{
	client("128.61.126.6",50000);
	}
  catch ( SocketException& e )
    {
      std::cout << "Exception was caught:" << e.description() << "\n";
    }*/


ROS_INFO("GONNA SPIN NOW!");
ros::spin();
}

