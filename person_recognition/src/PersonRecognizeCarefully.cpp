#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <string.h>
#include <actionlib/server/simple_action_server.h>
#include <person_recognition/Recognizer_LearnAction.h>
#include <person_recognition/Recognizer_RecogAction.h>
#include <person_recognition/PersonIdentifier.h>
#include <person_recognition/PersonRecControl.h>

using namespace std;

class PersonRecognizeCarefully{

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<person_recognition::Recognizer_LearnAction> learn_as_;
  actionlib::SimpleActionServer<person_recognition::Recognizer_RecogAction> recog_as_;
  person_recognition::Recognizer_LearnFeedback learn_feedback_;
  person_recognition::Recognizer_RecogFeedback recog_feedback_;
  person_recognition::Recognizer_LearnResult learn_result_;
  person_recognition::Recognizer_RecogResult recog_result_;

  ros::Subscriber command_subscriber_;
  ros::Subscriber person_subscriber_;
ros::Publisher person_control_publisher_;
  bool waitForRecognitionCompleteFlag;
  
  bool looking_for_people;

  //vector<person_recognition::PersonIdentifier> people_detected;


public:
  PersonRecognizeCarefully() :
    learn_as_(nh_,"learn_action",boost::bind(&PersonRecognizeCarefully::executeLearnCB,this,_1)),
    recog_as_(nh_,"recog_action",boost::bind(&PersonRecognizeCarefully::executeRecogCB,this,_1))
  {
    //speechGoto_sub  = nh_.subscribe("/speech_Goto",1,&PersonRecognizeCarefully::GotoCallback,this);
    command_subscriber_=nh_.subscribe<std_msgs::String>("/korus_control_topic",10,boost::bind(&PersonRecognizeCarefully::commandCallback, this, _1));
    person_subscriber_=nh_.subscribe<person_recognition::PersonIdentifier>("/personIdTopic", 30,boost::bind(&PersonRecognizeCarefully::personrecognizerCallback, this, _1) );
    person_control_publisher_=nh_.advertise<person_recognition::PersonRecControl>("/PersonRecControlTopic", 30);

waitForRecognitionCompleteFlag = false;
    looking_for_people = false;
  }
  
  void commandCallback  (const std_msgs::String::ConstPtr& msg)
  {
    char key0 = msg->data.c_str()[0];
    switch (key0)
      {
      case 'g': //message is global, to all modules.
	if(msg->data.length()>2)
	  {
	    char key2 = msg->data.c_str()[2];
	    switch (key2)
	      {
	      case 'r': //global message is from recognizer module.
		if(msg->data.length()>3)
		  {
		    string rcvdStr;
		    rcvdStr=msg->data.substr(4);
		    if(waitForRecognitionCompleteFlag==true)
		      {
			if(rcvdStr.compare("completed") == 0)
			  {
			    cout<<"GOT COMPLETION FLAG in personrecogCarefully!!"<<endl;
			    waitForRecognitionCompleteFlag=false;
			  }
		      }
		  }
		break;
	      default:
		break;
	      }
	  }
	
	break;
	
      case 'q':
	//exit(0);
	break;
      default:
	break;
      }
  }
  
  void personrecognizerCallback(person_recognition::PersonIdentifierConstPtr msg)
  {
    if(looking_for_people){
      //people_detected.push_back(*msg);
      recog_result_.people.push_back(*msg);
    }
  }

  void executeRecogCB(const person_recognition::Recognizer_RecogGoalConstPtr &goal)
  {
    
    ros::Rate r(0.05);
/*
    nh_.setParam("/PersonRecognizerRobocup/recognitionThreshold",0.5);
    nh_.setParam("/PersonRecognizerRobocup/searchedPersonsName","SEARCH_EVERYBODY");
    nh_.setParam("/PersonRecognizerRobocup/currentState",2);
*/


	person_recognition::PersonRecControl msg2;
	msg2.recognitionThresholdChange=true;
	msg2.searchedPersonsNameChange=true;
	msg2.currentStateChange=true;
	msg2.isNameNeededChange=false;
	msg2.learnedPersonsNameChange=false;
	msg2.recognitionThreshold=0.5;
	msg2.searchedPersonsName="SEARCH_EVERYBODY";
	msg2.currentState=2;
	person_control_publisher_.publish(msg2);

looking_for_people = true;


    ros::Time start = ros::Time::now();
    while((ros::Time::now() - start) < ros::Duration(goal->watch_duration)){
      if(recog_as_.isPreemptRequested())
	{
	  ROS_INFO("PersonRecognizer action: User requested preempt! Going to Recognition mode");
	  recog_as_.setPreempted();
	  //people_detected.clear();
	  recog_result_.people.clear();
	  looking_for_people = false;
	  nh_.setParam("/PersonRecognizerRobocup/currentState", 0);
	  return;
	}
	r.sleep();
    }
    nh_.setParam("/PersonRecognizerRobocup/currentState", 0);
    looking_for_people = false;
    //recog_result_.people = people_detected;
    recog_as_.setSucceeded(recog_result_);
    //people_detected.clear();
    recog_result_.people.clear();
  }
  
  
  void executeLearnCB(const person_recognition::Recognizer_LearnGoalConstPtr &goal)
  {
    looking_for_people = false;
    string name2=goal->name;
    /*
    nh_.setParam("/PersonRecognizerRobocup/isNameNeeded", true);
    nh_.setParam("/PersonRecognizerRobocup/learnedPersonsName", name2);
    nh_.setParam("/PersonRecognizerRobocup/currentState", 1);
    */

    person_recognition::PersonRecControl msg2;
	msg2.recognitionThresholdChange=false;
	msg2.searchedPersonsNameChange=false;
	msg2.currentStateChange=true;
	msg2.isNameNeededChange=true;
	msg2.learnedPersonsNameChange=true;
	msg2.learnedPersonsName=name2;
	msg2.isNameNeeded=true;
	msg2.currentState=1;
	person_control_publisher_.publish(msg2);


    waitForRecognitionCompleteFlag=true;
    ros::Rate r(0.05);
    
    
    while(waitForRecognitionCompleteFlag==true)
      {
	if(learn_as_.isPreemptRequested())
	  {
	    ROS_INFO("PersonRecognizer action: User requested preempt! Going to Recognition mode");
	    learn_as_.setPreempted();
	    nh_.setParam("/PersonRecognizerRobocup/currentState", 0);
	    return;
	  }
	r.sleep();
      }
    //When we are here, learning is completed..
    
    
    ROS_INFO("Person Recognizer Learn Action: Done executing");
    
    learn_result_.result=true;
    learn_as_.setSucceeded(learn_result_);
    //learn_as_.setAborted(learn_result_);
  }
  
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "PersonRecognizeCarefully");
	PersonRecognizeCarefully prc;
	ros::spin();
	return 0;
}

