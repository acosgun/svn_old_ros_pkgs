#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <jeeves_actions/PerformPTUScanAction.h>
#include <jeeves_actions/Listen_GotoAction.h>
#include <jeeves_actions/Listen_GetNameAction.h>
#include <person_recognition/Recognizer_LearnAction.h>
#include <person_recognition/Recognizer_RecogAction.h>
#include <person_recognition/PersonIdentifier.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <boost/thread.hpp>
#include <sound_play/sound_play.h>
#include <home_obj_recognizer/camera_helpers.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/CameraInfo.h>

struct bear_person {
  double bearing;
  double range;
  geometry_msgs::Point32 pos;
  std::map<std::string,int> name_count;
  bear_person(double b_) {
    bearing = b_;
  }
};

void spinThread()
{
  ros::spin();
}



class WhoIsWho {
protected:
  sensor_msgs::CameraInfo cam;
  ros::NodeHandle nh_;
  sound_play::SoundClient sc;
  ros::Subscriber info_sub_;
  bool got_cam_info;
  //vector<std::string> name_list;

public:

  WhoIsWho(){
    got_cam_info = false;
    //info_sub(nh_, "/prosilica/camera_info", 10),
    info_sub_ = nh_.subscribe("/prosilica/camera_info",10, &WhoIsWho::cameraInfoCB, this);
    
  }

  void cameraInfoCB(const sensor_msgs::CameraInfoConstPtr& cam_info) 
  {
    //ROS_INFO("GOT CAMERA INFOZZ!!!!!!!!!!!!!!!");
    if(!got_cam_info){
    got_cam_info = true;
    cam = *cam_info;
    }
    //ROS_INFO("Got camera info!");
  }

  //int main(int argc, char **argv)
  void run()
  {
      
    while(!got_cam_info && ros::ok()){
	ROS_INFO("waiting for a cam info!");
	ros::Duration(1.0).sleep();
      }

    // create the move base action client
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac("move_base", true);
    
    ROS_INFO("who is who: Waiting for move base AC!");
    move_base_ac.waitForServer();
    ROS_INFO("who is who: Move Base connected!");
    
    // create the PTU action client
    actionlib::SimpleActionClient<jeeves_actions::PerformPTUScanAction> ptu_scan_ac("perform_ptu_scan");
    ROS_INFO("who is who: Waiting for PTU action!");
    ptu_scan_ac.waitForServer();
    ROS_INFO("who is who: PTU action connected!");
    
    //create person learning action client
    actionlib::SimpleActionClient<person_recognition::Recognizer_LearnAction> recognizer_learn_ac("learn_action");
    ROS_INFO("who is who: Waiting for Person Learn action!");
    recognizer_learn_ac.waitForServer();
    ROS_INFO("who is who: Person Learn action connected!");
    
    //create person recognizer action client
    actionlib::SimpleActionClient<person_recognition::Recognizer_RecogAction> recognizer_recog_ac("recog_action");
    ROS_INFO("who is who: Waiting for Person Recog action!");
    recognizer_recog_ac.waitForServer();
    ROS_INFO("who is who: Person Learn action connected!");
    
	  //create the speech action client
	  actionlib::SimpleActionClient<jeeves_actions::Listen_GetNameAction> getname_ac("getname_action");
	  actionlib::SimpleActionClient<jeeves_actions::Listen_GotoAction> goto_ac("goto_action");
	  ROS_INFO("Go get it: Waiting Speech action!");
	  getname_ac.waitForServer();
	  goto_ac.waitForServer();
	  ROS_INFO("Go get it: Speech action connected!");
    

    sc.say("Jeeves activated! Initiating who is who task!");
    ros::Duration(5.0).sleep();
    

//MOVE TO THE START POSITION BY ENTERING THE DOOR



//LISTEN FOR THE NAME OF THE FIRST PERSON

  sc.say("I am ready to meet the first person.");
  jeeves_actions::Listen_GetName listen_name;
  listen_name.get_name = true;
  getname_ac.sendGoal(listen_name);


  bool heard_before_timeout = getname_ac.waitForResult(ros::Duration(60.0));

  jeeves_actions::Listen_GetNameResultConstPtr listen_result = getname_ac.getResult();

  if(heard_before_timeout){
    sc.say(listen_result->cmd.placeName.c_str());
  } else {
    sc.say("I did not hear a name!");
  }


//LEARN THE FIRST PERSON
    sc.say("OK Please look into the camera while I take your picture");  
    ros::Duration(5.0).sleep();
    
    person_recognition::Recognizer_LearnGoal learn_goal;
    learn_goal.name = listen_result->cmd.placeName.c_str();
    recognizer_learn_ac.sendGoal(learn_goal);
    sc.say("Stay one meter in front of me"); 
    bool learned_before_timeout = recognizer_learn_ac.waitForResult(ros::Duration(70.0));
    if(learned_before_timeout){
      ROS_INFO("Got recognizer_learn result!");
    } else {
      ROS_INFO("timed out on recognizer_learn");
    }
    sc.say("Nice to meet you!");
    ros::Duration(3.0).sleep();



//LISTEN FOR THE NAME OF THE SECOND PERSON

  sc.say("I am ready to meet the second person.");
  jeeves_actions::Listen_GetName listen_name;
  listen_name.get_name = true;
  getname_ac.sendGoal(listen_name);


  bool heard_before_timeout = getname_ac.waitForResult(ros::Duration(60.0));

  jeeves_actions::Listen_GetNameResultConstPtr listen_result = getname_ac.getResult();

  if(heard_before_timeout){
    sc.say(listen_result->cmd.placeName.c_str());
  } else {
    sc.say("I did not hear a name!");
  }



//LEARN THE SECOND PERSON
    
    sc.say("OK Please look into the camera while I take your picture");  
    ros::Duration(5.0).sleep();
    
    person_recognition::Recognizer_LearnGoal learn_goal2;
    learn_goal2.name = listen_result->cmd.placeName.c_str();
    recognizer_learn_ac.sendGoal(learn_goal2);
    sc.say("Stay one meter in front of me");  
    bool learned_before_timeout2 = recognizer_learn_ac.waitForResult(ros::Duration(60.0));
    if(learned_before_timeout2){
      ROS_INFO("Got recognizer_learn result!");
    } else {
      ROS_INFO("timed out on recognizer_learn");
    }
    sc.say("Nice to meet you!");
    

//LISTEN TO THE GO TO ROOM COMMAND

  jeeves_actions::Listen_GotoGoal listen_goal;
  listen_goal.get_confirmation = true;
  goto_ac.sendGoal(listen_goal);


  bool heard_before_timeout = goto_ac.waitForResult(ros::Duration(60.0));

  jeeves_actions::Listen_GotoResultConstPtr listen_result = goto_ac.getResult();

  if(heard_before_timeout){
    sc.say(listen_result->cmd.placeName.c_str());
  } else {
    sc.say("I did not get a speech command!");
  }


//CYCLE THROUGH DIFFERENT SEARCH LOCATIONS LOOKING FOR PEOPLE    
    //TODO: navigate to the room
    
    
    //recognize
    person_recognition::Recognizer_RecogGoal recog_goal;
    recog_goal.watch_duration = 15.0;
    recognizer_recog_ac.sendGoal(recog_goal);
    
    bool recoged_before_timeout = recognizer_recog_ac.waitForResult(ros::Duration(60.0));
    if(recoged_before_timeout){
      ROS_INFO("Got recognizer_recog result!");
    } else {
      ROS_INFO("timed out on recognizer_recog");
    }
    
    person_recognition::Recognizer_RecogResultConstPtr recog_result = recognizer_recog_ac.getResult();
    ROS_INFO("Person recognition detected %d measurements!",recog_result->people.size());
    
    //do something with measurements
    
    std::vector<bear_person> bear_peeps;
    for(int i = 0; i < recog_result->people.size(); i++){
      int facex_px = recog_result->people[i].faceRegion.x_offset+
	recog_result->people[i].faceRegion.width/2;
      int facey_py = recog_result->people[i].faceRegion.y_offset+
	recog_result->people[i].faceRegion.height/2;
      
      

      double face_bearing = get_bearing(unproject_pixel(cam.P,point2(facex_px,facey_py)));
      ROS_INFO("bear peep detected at bearing: %lf",face_bearing);
      bool matched= false;
      //check against people we know
      for(int j = 0; j < bear_peeps.size(); j++){
	double this_bearing = bear_peeps[j].bearing;
	if (fabs(this_bearing - face_bearing) < 0.3) {
	  matched = true;
	  if (bear_peeps[j].name_count.find(recog_result->people[i].name) != bear_peeps[j].name_count.end()) {
	    bear_peeps[j].name_count[recog_result->people[i].name] ++;
	    
	  }
	  else 
	    bear_peeps[j].name_count.insert(std::pair<std::string, int>(recog_result->people[i].name, 1));
	}
      }
      if (!matched){ //insert it into the map
	bear_person bp(face_bearing);
	bp.name_count.insert(std::pair<std::string, int>(recog_result->people[i].name, 1));
	bear_peeps.push_back(bp);
      }
    }

    ROS_INFO("cool, I found %d bear peeps!",bear_peeps.size());
    std::vector<bear_person> assigned_people;
    //count up the names
    int best_cluster = 0;
    int best_cluster_count = 0;
    for(int i = 0; i < bear_peeps.size(); i++){
      //figure out the best assignment for this one
      int best_name_count = 0;
      std::string best_name;
      for(std::map<std::string,int>::const_iterator itr = 
	    bear_peeps[i].name_count.begin();
	  itr != bear_peeps[i].name_count.end();
	  itr++){
	if(itr->second > best_name_count){
	  best_name_count = itr->second;
	  best_name = itr->first;
	}
      }

      ROS_INFO("Best name for this cluster: %s\n",best_name.c_str());
      if(best_name_count > best_cluster_count){
	best_cluster_count = best_name_count;
	best_cluster = i;
      }
    }
    //assign best name and remove that cluster and name
    assigned_people.push_back(bear_peeps[best_cluster]);
    bear_peeps.erase(bear_peeps.begin()+best_cluster);
    

//APPROACH, IDENTIFY, AND GREET ALL PEOPLE
    //find a point to approach to



//EXIT THE ARENA
	//navigate to the entry point
	//use linear controller to exit door (???)

  }
  
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "who_is_who");
  boost::thread spin_thread(&spinThread);
    


  WhoIsWho task;
  task.run();
  spin_thread.join();
  return 0;
}
