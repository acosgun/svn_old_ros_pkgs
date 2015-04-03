#include <PersonFollower.h>


PersonFollower::PersonFollower(): nh_("~"),ac("move_base", true),listener3(ros::Duration(10))
{
	using namespace std;

	periodicTimer1sec= nh_.createTimer(ros::Duration(1.0),boost::bind(&PersonFollower::periodicTimer1secCallback, this, _1),false);
	periodicTimer1sec.stop();

	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.5,0.5,0,1);

	targetNotMovingCounter=0;
	featWindowCounter=0;
	fillingForImages=255; //255 for visualization, 1 for non-visual
	intNumOfSegments=0;
	floatGridLength=0.05;
	floatUncertainty=0.02;
	//plotParam1=301; //400
	//plotParam2=75; //133

	plotParam1=501; //400
	plotParam2=125; //133

	distanceTreshold=0.1;
	numberOfPointsTreshold=3;
	boolFilterSegmentsAccordingToLengthInSegmentation=false;
	qUncertainty=0.01;
	rUncertainty=0.01;
	minPixelsToCountAllowedForMovementDetection=2;
	floatHitRatioTresholdForMovementDetection=0.10;
	maxEuclidianDistOfSegmentsThatCanBePersonCandids=0.45; //0.6 before
	minEuclidianDistOfSegmentsThatCanBePersonCandids=0.04; //0.1 before


	virgin=true;


	inhibitLinearMovement=false;//for peopleBot
	inhibitAngularMovement=false;//for peopleBot
	inhibitNavigationMovement=false;//for both

	personrecognizerCallbackReception=false; // should be false to start with, for robocup task3.
	enableComeNearbyBehavior=false;//for segway
	isMasterLearntByRecognizer=false; //internal variables.don't touch
	waitForRecognitionCompleteFlag=false;
	oneShotPersonRecognizeChance=false;
	decreaseRecognitionThresholdEnable=false;
	searchTwoSegmentsEnable=true;
	checkForPersonToStop=true;
	tetaNearby=CV_PI/2;
	velocityCounter=0;
	maxDistanceToMatchPrediction=0.45;
	floatMaxRadius=4.0; //max range of the laser for ppl following
	defaultStartingRecognitionThreshold=0.65;
	periodicRecThreshold=0.95;



	featWindowInitPhase=true;
	boolUseFixedFeatureParams=false;

	//IMPORTANT PARAMS
	//plotLaser=true;
	//usePTU=true;
	//useOdometry=false;
	//useOdomForSpeed=true;
	//useMoveBase=true;
	//useVision=true;
	//extraVelocityBeCareful=0.02;
	//permanentInhibitNavigationMovement=false; //only if using nav stack active. true:robot never moves. false:no restriction.
	//distanceToKeepWithPerson=1.1;
	//inverted=false;
	//intDataPoints=181; //# of data pts in a laser scan
	//rectangularBoxXover2sizeForMovDetection=0.6;
	//rectangularBoxYsizeForMovDetection=1.95;
	//operatingFrequency=9.37; //4.64 or 9.08
	//checkpoint3detectionRadius=2.0; //for robocup follow me task#3
	//

	//IMPORTANT PARAMS
	nh_.param("usePTU", usePTU, false);
	nh_.param("useOdometry", useOdometry, false);
	nh_.param("useOdomForSpeed", useOdomForSpeed, true);
	nh_.param("useMoveBase", useMoveBase, true);
	nh_.param("useVision", useVision, true);
	nh_.param("extraVelocityBeCareful", extraVelocityBeCareful, 0.0);
	nh_.param("permanentInhibitNavigationMovement", permanentInhibitNavigationMovement, false);
	nh_.param("distanceToKeepWithPerson", distanceToKeepWithPerson, 1.1);
	nh_.param("inverted", inverted, false);
	nh_.param("intDataPoints", intDataPoints, 181);
	nh_.param("angularResolution",angularResolution,1.0);
	nh_.param("rectangularBoxXover2sizeForMovDetection", rectangularBoxXover2sizeForMovDetection, 0.6);
	nh_.param("rectangularBoxYsizeForMovDetection", rectangularBoxYsizeForMovDetection, 1.65);
	nh_.param("operatingFrequency", operatingFrequency, 9.37);
	nh_.param("checkpoint3detectionRadius", checkpoint3detectionRadius, 2.0);
	nh_.param("plotLaser",plotLaser,true);
	nh_.param("useVoice",useVoice,true);
	nh_.param("useSpeechRec",useSpeechRec,true);
	nh_.param("logLaser",logLaser,true);
	nh_.param("featureUpdateWindowSize",featureUpdateWindowSize,20);
	//

	deltaT=1/operatingFrequency;
	coeff1=(0.1*0.25*angularResolution)/1.0;
	coeff2=(0.1*0.75*angularResolution)/2.7;

	imgSize.height=(int)( (floatMaxRadius/floatGridLength)+1 +0.5);
	imgSize.width=(int)( (2*floatMaxRadius/floatGridLength)+1 +0.5);
	meineKernel= cvCreateStructuringElementEx(2,1,0,0,CV_SHAPE_RECT);
	imgGridMap = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	imgGridMapPrevious = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	imgGridMapIncludingSegmentNumbers = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	imgGridMapPreviousNegated = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	cleanedGridMapIncludingSegmentNumbers = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	DifferenceMap = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	cleanedGridMap = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	outImg= cvCreateImage(cvSize(2*plotParam1, plotParam1), IPL_DEPTH_8U , 3);	
	kalman=cvCreateKalman(4,2,0);
	kalmanOdomFrame=cvCreateKalman(4,2,0);
	initVectors();

	if(logLaser)
	{
		logfile = fopen("bagfile.txt","w"); // open this for saving laser datas
	}
	if(useMoveBase)
	{
		distanceToKeepWithPerson=distanceToKeepWithPerson-0.01;
		while(!ac.waitForServer(ros::Duration(5.0)))
		{
			ROS_INFO("Waiting for the move_base action server to come up");
		}
	}
	if(usePTU)
	{
		PTUClient = nh_.serviceClient<ptu_46::PtuCmd>("/ptu_cmd");
		ros::service::waitForService("/ptu_cmd");
		ptu_46::PtuCmd cmd;
		cmd.request.pan_pos = 0.0;
		cmd.request.tilt_pos = 0.35;
		if (PTUClient.call(cmd))
		{

		}
		else
		{
			ROS_ERROR("PTU returned false in PERSON FOLLOWER module");
		}
	}
	cout<<"usePTU: "<<usePTU<<endl;
	cout<<"useOdometry: "<<useOdometry<<endl;
	cout<<"useOdomForSpeed: "<<useOdomForSpeed<<endl;
	cout<<"useMoveBase: "<<useMoveBase<<endl;
	cout<<"useVision: "<<useVision<<endl;
	cout<<"extraVelocityBeCareful: "<<extraVelocityBeCareful<<endl;
	cout<<"permanentInhibitNavigationMovement: "<<permanentInhibitNavigationMovement<<endl;
	cout<<"distanceToKeepWithPerson: "<<distanceToKeepWithPerson<<endl;
	cout<<"inverted: "<<inverted<<endl;
	cout<<"intDataPoints: "<<intDataPoints<<endl;
	cout<<"rectangularBoxXover2sizeForMovDetection: "<<rectangularBoxXover2sizeForMovDetection<<endl;
	cout<<"rectangularBoxYsizeForMovDetection: "<<rectangularBoxYsizeForMovDetection<<endl;
	cout<<"operatingFrequency: "<<operatingFrequency<<endl;
	cout<<"distanceToKeepWithPerson: "<<distanceToKeepWithPerson<<endl;
	cout<<"checkpoint3detectionRadius: "<<checkpoint3detectionRadius<<endl;
	cout<<"useVoice: "<<useVoice<<endl;
	cout<<"useSpeechRec: "<<useSpeechRec<<endl;
	cout<<"logLaser: "<<logLaser<<endl;

	if(useVoice)
	{
		sc.say("Waiting for follow me command");
		cvWaitKey(5.0);
	}



	/*
	//ABOUT_SPEECH_START
	if(useSpeechRec)
	{
		followMeCommandReceptionEnable=false;
		stopCommandReceptionEnable=false;
		waitCommandReceptionEnable=false;

		//activate follow me action server
		cout<<"Waiting for FollowMe action server"<<endl;
		follow_ac.waitForServer();
		cout<<"Done!"<<endl;
		cout<<"Waiting for Wait action server"<<endl;
		wait_ac.waitForServer();
		cout<<"Done!"<<endl;
		cout<<"Waiting for Stop action server"<<endl;
		stop_ac.waitForServer();
		ROS_INFO("follow Me receive action successfully connected!");



		jeeves_actions::Listen_FollowMeGoal follow_goal; //give the goal to listen for the "follow me" speech now
		follow_goal.get_confirmation = false;




                ROS_INFO("creating listen and wait goal..");
                jeeves_actions::Listen_WaitGoal wait_goal; //give the goal to the wait_ac action server. Now listening for the "wait" speech command
		  wait_goal.get_confirmation = false;


                jeeves_actions::Listen_StopGoal stop_goal; //give the goal to the stop_ac action server. Now listening for the "stop" speech command
                stop_goal.get_confirmation =false;
		// stop_ac.sendGoal(stop_goal);


         }
	else
	{
	followMeCommandReceptionEnable=false;
	stopCommandReceptionEnable=false;
	waitCommandReceptionEnable=false;
	}
	//ABOUT_SPEECH_END */

	command_subscriber_=nh_.subscribe<std_msgs::String>("/korus_control_topic",10,boost::bind(&PersonFollower::commandCallback, this, _1));
	laser_subscriber_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, boost::bind(&PersonFollower::scanCallback, this, _1));
	commander_pub = nh_.advertise<geometry_msgs::Twist> ("/cmd_vel", 3);
	//timer10sec = nh_.createTimer(ros::Duration(10.0),boost::bind(&PersonFollower::timer10secCallback, this, _1),true);
	person_subscriber_=nh_.subscribe<person_recognition::PersonIdentifier>("/personIdTopic", 30,boost::bind(&PersonFollower::personrecognizerCallback, this, _1) );
	person_control_publisher_=nh_.advertise<person_recognition::PersonRecControl>("/PersonRecControlTopic", 30);


	currentState=IDLE;
	currentMotionState=REGULAR_FOLLOW;
	ROS_INFO("Initialized Person Follower node..");
}

void PersonFollower::commandCallback  (const std_msgs::String::ConstPtr& msg)
{
	cout<<"Got command "<<msg->data<<endl;
	char key0 = msg->data.c_str()[0];

	switch (key0)
	{

	case 'f': //message is to follower module.

		if(msg->data.length()>2)
		{
			char key2 = msg->data.c_str()[2];
			switch (key2)
			{
			case 'h': //f_h: halt
				if(permanentInhibitNavigationMovement==false)
				{
					permanentInhibitNavigationMovement=true;
				}
				else
				{
					permanentInhibitNavigationMovement=false;
				}				
				break;

			case 'i': //f_i: idle
				stopMotors();

				currentState=IDLE;

				break;
			case 'f': //f_f: follow
				if(useVision)
				{
					currentState=IDLE;


					if(useVoice)
					{
						sc.say("Please look at the camera while I take your pictures Stay one meter in front of me");
					}


					person_recognition::PersonRecControl msg2;
					msg2.recognitionThresholdChange=false;
					msg2.searchedPersonsNameChange=false;
					msg2.currentStateChange=true;
					msg2.isNameNeededChange=true;
					msg2.learnedPersonsNameChange=false;

					msg2.isNameNeeded=false;
					msg2.currentState=1;
					person_control_publisher_.publish(msg2);

					/*
					if (nh_.hasParam("/PersonRecognizerRobocup/isNameNeeded"))
					{
						nh_.setParam("/PersonRecognizerRobocup/isNameNeeded", false);
					}
					if (nh_.hasParam("/PersonRecognizerRobocup/learnedPersonsName"))
					{
						nh_.setParam("/PersonRecognizerRobocup/learnedPersonsName", "MASTER");
					}
					if (nh_.hasParam("/PersonRecognizerRobocup/currentState")) //start LEARN_PERSON routine
					{
						nh_.setParam("/PersonRecognizerRobocup/currentState", 1);
					}
					 */


					waitForRecognitionCompleteFlag=true;
					cout<<"Waiting for Recognition Completion Flag!"<<endl;
				}
				else
				{
					if(useVoice)
					{
						sc.say("I am following you");
					}
					currentState=NO_TARGET_TO_TRACK_MODE;
				}
				break;

			case 'w': //f_w: wait
				if(useVoice)
				{
					sc.say("I am going to wait for ten seconds and continue following you");
				}
				//stopMotors();
				inhibitNavigationMovement=true;
				timer10sec = nh_.createTimer(ros::Duration(15.0),boost::bind(&PersonFollower::timer10secCallback, this, _1),true); //TODO: change this to 10secCallback!!!!!
				ros::spin();
				break;
			case 's': //f_s: stop
				if(useVoice)
				{
					sc.say("I will stop and wait until the user shows up again");
				}
				if(useVision && isMasterLearntByRecognizer)
				{

					stopMotors();
					searchTwoSegmentsEnable=false;
					currentState=SEARCH_TWO_SEGMENTS;
					timer10sec = nh_.createTimer(ros::Duration(10.0),boost::bind(&PersonFollower::timer10secCallback, this, _1),true);
					//timer4sec = nh_.createTimer(ros::Duration(2.0),boost::bind(&PersonFollower::timer4secCallback, this, _1),true); //for offline testing

					ros::spin();
				}
				else
				{
					cout<<"Error: Vision is disabled or Master isn't learnt yet!"<<endl;
				}
				break;

			case 'q':
				exit(0);
				break;
			default:
				break;
			}
		}

		break;

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
							if(useVoice)
							{
								sc.say("I will follow you when you start moving..");
							}
							cout<<"GOT THE COMPETION FLAG!!"<<endl;
							waitForRecognitionCompleteFlag=false;
							isMasterLearntByRecognizer=true;
							currentState=NO_TARGET_TO_TRACK_MODE;
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
		if(logLaser)
		{
			fclose(logfile);
		}
		exit(0);
		break;
	default:
		break;
	}



}

void PersonFollower::personrecognizerCallback(person_recognition::PersonIdentifierConstPtr msg)
{

	if(personrecognizerCallbackReception==true)
	{

		string personName=msg->name;
		cout<<"DETECTED: "<<personName<<endl;


		//ESTIMATE POSITION ACCORDING TO SIZE,LOCATION
		sensor_msgs::RegionOfInterest reg;
		reg=msg->faceRegion;
		int facePixelWidth=reg.width;
		int facePixelLeft=reg.x_offset;
		int camImgWidth=msg->camImgWidth;

		double x=152.0/facePixelWidth;

		double y_pixels=static_cast<double>(camImgWidth/2)-static_cast<double>(facePixelWidth/2)-static_cast<double>(facePixelLeft);
		double y=-(y_pixels*0.18)/(static_cast<double>(facePixelWidth));

		//cout<<"x: "<<x<<endl;
		//cout<<"y: "<<y<<endl;

		//TRANSFORM TO LASER'S COORDINATE FRAME

		//cvZero( outImg );
		//PLOT

		double targetXmyCoords=-y;
		double targetYmyCoords=x;
		if(plotLaser==true)
		{
			double tempx=plotParam1 - plotParam2 * targetXmyCoords;
			double tempy=plotParam1 - plotParam2 * targetYmyCoords;
			cvCircle(outImg, cvPoint(tempx,tempy), 5, cvScalar(50,175,255),3);
			cvShowImage("LaserData", outImg);
			cvWaitKey(4);
		}

		float valsXk[4];
		double dbltemp1=0.0;
		double dbltemp2=0.0;


		dbltemp1=targetXmyCoords;
		dbltemp2=targetYmyCoords;
		valsXk[0]=(float)dbltemp1;
		valsXk[1]=(float)dbltemp2;
		valsXk[2]=0.0;
		valsXk[3]=0.0;
		memcpy(kalman->state_post->data.fl,valsXk,sizeof(valsXk)); // set the Xk
		memcpy(kalman->state_pre->data.fl,valsXk,sizeof(valsXk)); // set the XkPrior



		pointTransformer(listener3,targetYmyCoords,-targetXmyCoords,dbltemp1,dbltemp2,true,"laser","odom");
		valsXk[0]=(float)dbltemp1;
		valsXk[1]=(float)dbltemp2;
		valsXk[2]=0.0;
		valsXk[3]=0.0;
		memcpy(kalmanOdomFrame->state_post->data.fl,valsXk,sizeof(valsXk)); // set the Xk
		memcpy(kalmanOdomFrame->state_pre->data.fl,valsXk,sizeof(valsXk)); // set the XkPrior


		oneShotPersonRecognizeChance=true;
		currentState=ONE_LEG_TRACK_MODE;

	}
}


void PersonFollower::initVectors()
{
	doubleLaserDataXprevious.clear();
	doubleLaserDataYprevious.clear();
	doubleLaserDataX.clear();
	doubleLaserDataY.clear();

	doubleLaserDataXprevious.resize(intDataPoints);
	doubleLaserDataYprevious.resize(intDataPoints);
	doubleLaserDataX.resize(intDataPoints);
	doubleLaserDataY.resize(intDataPoints);

	valsF[0]=1;
	valsF[1]=0;			
	valsF[2]=deltaT;	
	valsF[3]=0;
	valsF[4]=0;	
	valsF[5]=1;
	valsF[6]=0;
	valsF[7]=deltaT;
	valsF[8]=0;
	valsF[9]=0;
	valsF[10]=1;
	valsF[11]=0;	
	valsF[12]=0;
	valsF[13]=0;
	valsF[14]=0;
	valsF[15]=1;

	originPtt[0]=0;
	originPtt[1]=0;

	featureMeans[0]=0.138; //width.
	featureMeans[1]=2.235; //IAV. Perfect circle=1.57, Perfect line=3.14
	featureMeans[2]=0.258; //circularity. Perfect circle=1.0, Perfect line= 0.0

	featureVariances[0]=0.034;
	featureVariances[1]=0.404;
	featureVariances[2]=0.15;

	featureTotals[0]=0.0;
	featureTotals[1]=0.0;
	featureTotals[2]=0.0;
}

void PersonFollower::stopMotors()
{
	if(useMoveBase)
	{
		ac.cancelAllGoals();
	}
	//	else
	//	{
	cmd.linear.x=0;
	cmd.angular.z=0;
	commander_pub.publish(cmd);
	//	}

}

void PersonFollower::updateFeatureParameters(double currentSegmentWidth,double currentSegmentIAV,double currentSegmentCircCriterion,int featureUpdateWindowSize)
{
	featureMeans[0]=(double)((featureMeans[0]*(featureUpdateWindowSize-1)+currentSegmentWidth)/featureUpdateWindowSize); //width.
	featureMeans[1]=(double)((featureMeans[1]*(featureUpdateWindowSize-1)+currentSegmentIAV)/featureUpdateWindowSize); //iav.
	featureMeans[2]=(double)((featureMeans[2]*(featureUpdateWindowSize-1)+currentSegmentCircCriterion)/featureUpdateWindowSize); //circularity.
}

void PersonFollower::periodicTimer1secCallback(const ros::TimerEvent& e)
{
	ROS_INFO("1 sec interrupt triggered");
	if(decreaseRecognitionThresholdEnable==true)
	{
		periodicRecThreshold=periodicRecThreshold-0.01;
		person_recognition::PersonRecControl msg2;
		msg2.recognitionThresholdChange=true;
		msg2.searchedPersonsNameChange=false;
		msg2.currentStateChange=false;
		msg2.isNameNeededChange=false;
		msg2.learnedPersonsNameChange=false;

		msg2.recognitionThreshold=periodicRecThreshold;
		person_control_publisher_.publish(msg2);
		cout<<"Thres: "<<periodicRecThreshold<<endl;


	}
}

void PersonFollower::timer10secCallback(const ros::TimerEvent& e) 
{
	ROS_INFO("10 sec wait ended");
	inhibitNavigationMovement = false;
	/*
	//ABOUT_SPEECH_START
	if(waitCommandReceptionEnable==false) //re-activate the reception of wait and stop speech commands.
	  {
	    ROS_INFO("setting waitcommandreception true"); 
		waitCommandReceptionEnable=true;
		stopCommandReceptionEnable=true;
	}
	//ABOUT_SPEECH_END
	 */
	if(searchTwoSegmentsEnable==false)
	{
		searchTwoSegmentsEnable=true;
	}
}
void PersonFollower::timer4secCallback(const ros::TimerEvent& e)
{
	ROS_INFO("2sec wait ended");

	currentState=IDLE;


	periodicRecThreshold=0.95;


	person_recognition::PersonRecControl msg2;
	msg2.recognitionThresholdChange=true;
	msg2.searchedPersonsNameChange=true;
	msg2.currentStateChange=true;
	msg2.isNameNeededChange=false;
	msg2.learnedPersonsNameChange=false;
	msg2.recognitionThreshold=periodicRecThreshold;
	msg2.searchedPersonsName="MASTER";
	msg2.currentState=2;
	person_control_publisher_.publish(msg2);
	cvWaitKey(500);


	maxDistanceToMatchPrediction=0.45;
	personrecognizerCallbackReception=true;
	periodicTimer1sec.start();
	decreaseRecognitionThresholdEnable=true;



	/*
	if (nh_.hasParam("/PersonRecognizerRobocup/recognitionThreshold"))
	{
		nh_.setParam("/PersonRecognizerRobocup/recognitionThreshold", periodicRecThreshold);
	}
	if (nh_.hasParam("/PersonRecognizerRobocup/searchedPersonsName"))
	{
		nh_.setParam("/PersonRecognizerRobocup/searchedPersonsName", "MASTER");
	}
	if (nh_.hasParam("/PersonRecognizerRobocup/currentState"))
	{
		nh_.setParam("/PersonRecognizerRobocup/currentState", 2);
	}
	 */

}

void PersonFollower::pointTransformer(const tf::TransformListener& listener, double inX, double inY, double &outX, double &outY, bool verbose, string from, string to)
{

	geometry_msgs::PointStamped laser_point; //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
	//laser_point.header.frame_id = "laser";
	laser_point.header.frame_id = from;

	laser_point.header.stamp = ros::Time(); //we'll just use the most recent transform available for our simple example

	laser_point.point.x = inX; //this is reversed; to match the coordinate frame "laser" to my convention.
	laser_point.point.y = inY;
	laser_point.point.z = 0.0;

	/*	laser_point.point.x = inY;
	laser_point.point.y = inX;
	laser_point.point.z = 0.0; */

	try
	{
		geometry_msgs::PointStamped base_point;
		listener.transformPoint(to, laser_point, base_point);
		//listener.transformPoint("odom", laser_point, base_point);
		outX=base_point.point.x;
		outY=base_point.point.y;
		if(verbose==true)
		{
			ROS_INFO("laser:(%.2f,%.2f)->odom:(%.2f,%.2f)",
					laser_point.point.x, laser_point.point.y,
					base_point.point.x, base_point.point.y);
		}
	}
	catch(tf::TransformException& ex)
	{
		ROS_ERROR("Received an exception :%s", ex.what());
	}
}

void PersonFollower::printMat(CvMat *A)
{
	int i, j;
	for (i = 0; i < A->rows; i++)
	{
		printf("\n");
		switch (CV_MAT_DEPTH(A->type))
		{
		case CV_32F:
		case CV_64F:
			for (j = 0; j < A->cols; j++)
				printf ("%8.3f ", (float)cvGetReal2D(A, i, j));
			break;
		case CV_8U:
		case CV_16U:
			for(j = 0; j < A->cols; j++)
				printf ("%6d",(int)cvGetReal2D(A, i, j));
			break;
		default:
			break;
		}
	}
	printf("\n");
}


double PersonFollower::findDistanceBetweenTwoPoints(const double pt1[], const double pt2[])
{
	double tot=sqrt(pow((pt1[0]-pt2[0]),2)+pow((pt1[1]-pt2[1]),2));
	return tot;
}
double PersonFollower::calculateIAV(Segment segTemp)
{
	double pt1[2];
	double pt2[2];
	double pt3[2];
	double angleTotals=0.0;
	double angleCount=0.0;
	double angle1=0.0;
	double angle2=0.0;

	pt1[0]=doubleLaserDataX[segTemp[0]];
	pt1[1]=doubleLaserDataY[segTemp[0]];
	pt3[0]=doubleLaserDataX[segTemp[segTemp.size()-1]];
	pt3[1]=doubleLaserDataY[segTemp[segTemp.size()-1]];

	for(unsigned int j=0;j<segTemp.size()-2;j++)
	{
		pt2[0]=doubleLaserDataX[segTemp[j+1]];
		pt2[1]=doubleLaserDataY[segTemp[j+1]];
		angle1=atan2( (pt2[1]-pt1[1]),(pt2[0]-pt1[0]) );
		angle2=atan2( (pt3[1]-pt2[1]),(pt3[0]-pt2[0]) );
		angleTotals=angleTotals+(CV_PI-abs(acos(cos(angle1)*cos(angle2)+sin(angle1)*sin(angle2)))  );
		angleCount=angleCount+1.0;
	}
	return (angleTotals/angleCount);

}

double PersonFollower::calculateCircCriterion(Segment segTemp,double currentSegmentWidth)
{
	unsigned int segLength=segTemp.size();
	/*
cout<<"Segment Length: "<<segLength<<endl;
	   for(unsigned int hu=0;hu<segLength;hu++)
	    {
	    	cout<<segTemp[hu]<<" ";
	    }
	 */



	int midIndex;
	/*if ( segLength % 2 == 0 ) //even
    {
        midIndex=segLength/2+segTemp[0];
    }
    else //odd
    {
	midIndex=static_cast<int>(segLength/2);    	
    }*/
	midIndex=static_cast<int>(segLength)/2;
	double p1[2];
	double p2[2];
	double p3[2];


	/*
cout<<"midIndex: "<<midIndex<<endl;
cout<<"segTemp[0]: "<<segTemp[0]<<endl;
cout<<"segTemp[midIndex]: "<<segTemp[midIndex]<<endl;
cout<<"segTemp[segLength-1]: "<<segTemp[segLength-1]<<endl;
	 */

	p3[0]=doubleLaserDataX[segTemp[0]];
	p3[1]=doubleLaserDataY[segTemp[0]];
	p2[0]=doubleLaserDataX[segTemp[midIndex]];
	p2[1]=doubleLaserDataY[segTemp[midIndex]];
	p1[0]=doubleLaserDataX[segTemp[segLength-1]];
	p1[1]=doubleLaserDataY[segTemp[segLength-1]];
	p3[0]=p3[0]-p1[0];
	p3[1]=p3[1]-p1[1];
	p2[0]=p2[0]-p1[0];
	p2[1]=p2[1]-p1[1];

	//cout<<"p3[0]:"<<p3[0]<<"p3[1]:"<<p3[1]<<endl;

	double teta=atan2(p3[1],p3[0]);

	//cout<<"teta: "<<teta<<endl;
	double y2=p2[1]*cos(teta)-p2[0]*sin(teta);
	//cout<<"y2: "<<y2<<endl;




	return (abs(y2)/currentSegmentWidth);
}

double PersonFollower::calculateSegmentDistance(double currentSegmentWidth,double currentSegmentIAV,double currentSegmentCircCriterion,bool boolIncludePhysicalDistance,double physicalDist)
{
	double feat1dist=pow( (currentSegmentWidth-featureMeans[0]),2) /featureVariances[0];
	double feat2dist=0.0;
	double feat3dist=0.0;

	if((currentSegmentIAV-featureMeans[1])>0)
	{
		feat2dist=pow( (currentSegmentIAV-featureMeans[1]),2)/featureVariances[1];
	}
	else
	{
		feat2dist=pow( (currentSegmentIAV-featureMeans[1]),2)/featureVariances[1]*0.15;
	}

	if((currentSegmentCircCriterion-featureMeans[2])<0)
	{
		feat3dist=pow( (currentSegmentCircCriterion-featureMeans[2]),2)/featureVariances[2];
	}
	else
	{
		feat3dist=pow( (currentSegmentCircCriterion-featureMeans[2]),2)/featureVariances[2]*0.3;
	}
	if(boolIncludePhysicalDistance)
	{
		//CvMat* covMatrix=kalman->error_cov_pre;
		//printMat(covMatrix);


		double varianceTotal=0.1;
		double feat0dist=physicalDist*physicalDist/varianceTotal;

		//cout<<"d0: "<<feat0dist<<" d1: "<<feat1dist<<" d2: "<<feat2dist<<" d3: "<<feat3dist<<endl;
		return(sqrt(0.5*feat0dist+0.15*feat1dist+0.2*feat2dist+0.15*feat3dist));
	}
	else
	{
		return(sqrt(0.35*feat1dist+0.39*feat2dist+0.26*feat3dist));
	}

}

void PersonFollower::findSegmentsAndCreateCurrentGridMap(const vector<double> &xArray, const vector<double> &yArray, IplImage* imgGridMap, IplImage* imgGridMapIncludingSegmentNumbers, int &intNumOfSegments, vector <Segment>& SegmentIndexesVectorOfVectors)
{
	double currentPoint[2];
	double nextPoint[2];
	double originPoint[2];

	Segment tempArray;

	intNumOfSegments=0;
	originPoint[0]=0.0;
	originPoint[1]=0.0;
	currentPoint[0]=xArray[0];
	currentPoint[1]=0.0;

	for (int i=0; i< (intDataPoints); i++)
	{
		nextPoint[0]=xArray[i+1];
		nextPoint[1]=yArray[i+1];

		if(tempArray.empty())
		{
			tempArray.push_back((double)(i));
		}

		//distanceTreshold=((0.1*0.25)+( findDistanceBetweenTwoPoints(currentPoint,originPtt) )/2.7*0.1*0.75)*(angularResolution/1.0);
		distanceTreshold=coeff1+findDistanceBetweenTwoPoints(currentPoint,originPtt)*coeff2;

		if( (findDistanceBetweenTwoPoints(currentPoint,nextPoint)<distanceTreshold) && (findDistanceBetweenTwoPoints(currentPoint,originPoint)<(floatMaxRadius-floatUncertainty)) ) //distanceTreshold condition OK
		{
			if(i==(intDataPoints-1)) //if it is the last point of the scan.
			{
				if(!boolFilterSegmentsAccordingToLengthInSegmentation) // the segment is valid in terms of both distance and #of pts. add it
				{
					intNumOfSegments++;
					SegmentIndexesVectorOfVectors.push_back(tempArray);

					for(unsigned int p=0;p<tempArray.size();p++) //grid map construction here!
					{
						cvScalarTemp=cvScalar(fillingForImages);
						cvSet2D(imgGridMap, (int)((yArray[tempArray[p]]/floatGridLength)+0.5), (int)( ((xArray[tempArray[p]]+floatMaxRadius) /floatGridLength)+0.5), cvScalarTemp);
						cvScalarTemp=cvScalar(intNumOfSegments);
						cvSet2D(imgGridMapIncludingSegmentNumbers, (int)((yArray[tempArray[p]]/floatGridLength)+0.5), (int)( ((xArray[tempArray[p]]+floatMaxRadius) /floatGridLength)+0.5), cvScalarTemp);
						//						cout<<"ham x: "<<yArray[tempArray[p]]<<" ham y: "<<xArray[tempArray[p]]<<endl;
						//						cout<<"x: "<<(int)( (yArray[tempArray[p]]/floatGridLength)+0.5)<<" y: "<< (int)( ((xArray[tempArray[p]]+floatMaxRadius) /floatGridLength)+0.5)<<endl;
					}

				}
				else
				{
					double pFirst[2];
					double pLast[2];
					pFirst[0]=xArray[tempArray[0]];
					pFirst[1]=yArray[tempArray[0]];
					pLast[0]=xArray[tempArray[tempArray.size()-1]];
					pLast[1]=yArray[tempArray[tempArray.size()-1]];
					double euclidianDistanceOfTheSegment=findDistanceBetweenTwoPoints(pFirst,pLast);

					if(euclidianDistanceOfTheSegment<maxEuclidianDistOfSegmentsThatCanBePersonCandids && euclidianDistanceOfTheSegment> minEuclidianDistOfSegmentsThatCanBePersonCandids)
					{
						intNumOfSegments++;
						SegmentIndexesVectorOfVectors.push_back(tempArray);

						for(unsigned int p=0;p<tempArray.size();p++) //grid map construction here!
						{

							cvSet2D(imgGridMap, (int)((yArray[tempArray[p]]/floatGridLength)+0.5), (int)( ((xArray[tempArray[p]]+floatMaxRadius) /floatGridLength)+0.5), cvScalar(fillingForImages));

							cvSet2D(imgGridMapIncludingSegmentNumbers, (int)((yArray[tempArray[p]]/floatGridLength)+0.5), (int)( ((xArray[tempArray[p]]+floatMaxRadius) /floatGridLength)+0.5), cvScalar(intNumOfSegments));
							//						cout<<"ham x: "<<yArray[tempArray[p]]<<" ham y: "<<xArray[tempArray[p]]<<endl;
							//						cout<<"x: "<<(int)( (yArray[tempArray[p]]/floatGridLength)+1  +0.5)<<" y: "<< (int)( ((xArray[tempArray[p]]+floatMaxRadius) /floatGridLength)+1 +0.5)<<endl;
						}
					}
				}

			}
			else
			{
				tempArray.push_back((double)(i+1));
			}

		}
		else //distanceTreshold condition did not hold
		{
			if(tempArray.size()<numberOfPointsTreshold)
			{
				// empty :)
			}
			else
			{
				if(!boolFilterSegmentsAccordingToLengthInSegmentation) // the segment is valid in terms of both distance and #of pts. add it
				{
					intNumOfSegments++;
					SegmentIndexesVectorOfVectors.push_back(tempArray);

					for(unsigned int p=0;p<tempArray.size();p++) //grid map construction here!
					{
						cvScalarTemp=cvScalar(fillingForImages);
						cvSet2D(imgGridMap, (int)((yArray[tempArray[p]]/floatGridLength)+0.5), (int)( ((xArray[tempArray[p]]+floatMaxRadius) /floatGridLength)+0.5), cvScalarTemp);
						cvScalarTemp=cvScalar(intNumOfSegments);
						cvSet2D(imgGridMapIncludingSegmentNumbers, (int)((yArray[tempArray[p]]/floatGridLength)+0.5), (int)( ((xArray[tempArray[p]]+floatMaxRadius) /floatGridLength)+0.5), cvScalarTemp);
						//						cout<<"ham x: "<<yArray[tempArray[p]]<<" ham y: "<<xArray[tempArray[p]]<<endl;
						//						cout<<"x: "<<(int)( (yArray[tempArray[p]]/floatGridLength)+0.5)<<" y: "<< (int)( ((xArray[tempArray[p]]+floatMaxRadius) /floatGridLength)+0.5)<<endl;
					}

				}
				else
				{

					double pFirst[2];
					double pLast[2];
					pFirst[0]=xArray[tempArray[0]];
					pFirst[1]=yArray[tempArray[0]];
					pLast[0]=xArray[tempArray[tempArray.size()-1]];
					pLast[1]=yArray[tempArray[tempArray.size()-1]];
					double euclidianDistanceOfTheSegment=findDistanceBetweenTwoPoints(pFirst,pLast);

					if(euclidianDistanceOfTheSegment<maxEuclidianDistOfSegmentsThatCanBePersonCandids && euclidianDistanceOfTheSegment> minEuclidianDistOfSegmentsThatCanBePersonCandids)
					{
						intNumOfSegments++;
						SegmentIndexesVectorOfVectors.push_back(tempArray);

						for(unsigned int p=0;p<tempArray.size();p++) //grid map construction here!
						{
							cvScalarTemp=cvScalar(fillingForImages);
							cvSet2D(imgGridMap, (int)((yArray[tempArray[p]]/floatGridLength)+0.5), (int)( ((xArray[tempArray[p]]+floatMaxRadius) /floatGridLength)+0.5), cvScalarTemp);
							cvScalarTemp=cvScalar(intNumOfSegments);
							cvSet2D(imgGridMapIncludingSegmentNumbers, (int)((yArray[tempArray[p]]/floatGridLength)+0.5), (int)( ((xArray[tempArray[p]]+floatMaxRadius) /floatGridLength)+0.5), cvScalarTemp);
							//						cout<<"ham x: "<<yArray[tempArray[p]]<<" ham y: "<<xArray[tempArray[p]]<<endl;
							//						cout<<"x: "<<(int)( (yArray[tempArray[p]]/floatGridLength)+0.5)<<" y: "<< (int)( ((xArray[tempArray[p]]+floatMaxRadius) /floatGridLength)+0.5)<<endl;
						}
					}
				}
			}
			tempArray.clear(); //clear tempArray vector here
		}

		currentPoint[0]=nextPoint[0]; //get prepared for the next turn
		currentPoint[1]=nextPoint[1];
	}

}

void PersonFollower::calculateCandidateSegments(const IplImage* cleanedGridMapIncludingSegmentNumbers,  const vector <Segment> SegmentIndexesVectorOfVectors , const int intNumOfSegments, CvSize imgSize, Segment &validSegments)
{
	int valOfThePixel=0;
	int totalSegmentHits[intNumOfSegments];
	int hitsFromGridMap[intNumOfSegments];
	Segment temperSegment;
	float currentHitRatio=0;

	for(int i=0;i<intNumOfSegments;i++) //for printing an image. i is the y axis, j is the x axis. GIMP=(j,i)
	{
		totalSegmentHits[i]=0;
		hitsFromGridMap[i]=0;
	}

	for(int i=0;i<imgSize.height;i++) //for printing an image. i is the y axis, j is the x axis. GIMP=(j,i)
	{
		for(int j=0;j<imgSize.width;j++)
		{
			valOfThePixel=cvGet2D(cleanedGridMapIncludingSegmentNumbers,i,j).val[0];
			if(!(valOfThePixel==0))
			{
				hitsFromGridMap[valOfThePixel-1]++;
			}
		}
	}


	for(unsigned int i=0;i<SegmentIndexesVectorOfVectors.size();i++) //for printing an image. i is the y axis, j is the x axis. GIMP=(j,i)
	{
		temperSegment=SegmentIndexesVectorOfVectors[i];
		totalSegmentHits[i]=temperSegment.size();
	}

	for(int i=0;i<intNumOfSegments;i++)
	{
		currentHitRatio=((float)hitsFromGridMap[i]/(float)totalSegmentHits[i]);
		if(currentHitRatio>floatHitRatioTresholdForMovementDetection && hitsFromGridMap[i]>=minPixelsToCountAllowedForMovementDetection)
		{
			validSegments.push_back(i);
		}
	}
}



void PersonFollower::scanCallback  (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{	
	sensor_msgs::PointCloud cloud;
	laser_geometry::LaserProjection projector_;
	projector_.projectLaser(*scan_in, cloud,floatMaxRadius);
	//static tf::TransformListener listener(ros::Duration(10)); //tf
	//static MoveBaseClient ac("move_base", true);

	float currentRangeValue;

	/*
	//ABOUT_SPEECH_START
	if(useSpeechRec)
	{
		if(followMeCommandReceptionEnable)
		{
			//poll for the follow me speech command
			if(follow_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) //if received; do your magic. + activate stop & wait goals seperately.
			{
				followMeCommandReceptionEnable=false; //we don't wanna look for the follow me command when it is received once.
				ROS_INFO("in personfollow: FOLLOW ACTION SUCCEEDED");


				//follow_ac.sendGoal(follow_goal);
                //wait_ac.sendGoal(wait_goal);

				//----------don't touch this part. Activates person following behavior------
				if(useVision)
				{
					currentState=IDLE;
					if(useVoice)
					{
						sc.say("Please look at the camera while I take your pictures Stay one meter in front of me");
					}

					person_recognition::PersonRecControl msg2;
					msg2.recognitionThresholdChange=false;
					msg2.searchedPersonsNameChange=false;
					msg2.currentStateChange=true;
					msg2.isNameNeededChange=true;
					msg2.learnedPersonsNameChange=false;

					msg2.isNameNeeded=false;
					msg2.currentState=1;
					person_control_publisher_.publish(msg2);

					waitForRecognitionCompleteFlag=true;
					cout<<"Waiting for Recognition Completion Flag!"<<endl;
				}//usevision
				else
				{
					currentState=NO_TARGET_TO_TRACK_MODE;
				}//else
				//--------------------------------------------------------------------------


//                ROS_INFO("creating listen and wait goal..");    if ( num % 2 == 0 )
    {
        cout << num << " is even ";
    }
	 *
//
//				jeeves_actions::Listen_WaitGoal wait_goal; //give the goal to the wait_ac action server. Now listening for the "wait" speech command
//				wait_goal.get_confirmation = true;
//				wait_ac.sendGoal(wait_goal);
//
//				jeeves_actions::Listen_StopGoal stop_goal; //give the goal to the stop_ac action server. Now listening for the "stop" speech command
//				stop_goal.get_confirmation = true;
//				stop_ac.sendGoal(stop_goal);

				stopCommandReceptionEnable=true;  //now hearing the "stop" speech command
				waitCommandReceptionEnable=true;  //now hearing the "wait" speech command
			}

		}
		if(waitCommandReceptionEnable)
		{
			//poll for the stop speech command
			if(wait_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) //if succeeded; do your magic.
			{
				waitCommandReceptionEnable=false;	 //temporarily deactivate the reception of the command. It is going to be opened when the wait task is completed.
				stopCommandReceptionEnable=false;    //also disable this, because we don't want to hear any commands when we are executing the task.

				ROS_INFO("in personfollow: WAIT ACTION SUCCEED");

				//don't touch this part of code. Things to do when wait command is received-----
				if(useVoice)
				{
					sc.say("I am going to wait for ten seconds and continue following you");
				}
				//stopMotors();
				inhibitNavigationMovement=true;
				timer10sec = nh_.createTimer(ros::Duration(10.0),boost::bind(&PersonFollower::timer10secCallback, this, _1),true); //TODO: change this to 10secCallback!!!!!
				ros::spin();
				//---------------------------------------------------------------------------

			}
		}
		if(stopCommandReceptionEnable)
		{

			if(stop_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) //if succeeded; do your magic.
			{
				ROS_INFO("in personfollow: STOP ACTION SUCCEED");
				stopCommandReceptionEnable=false; //temporarily deactivate the reception of the command. It is going to be opened when the stop task is completed.
				waitCommandReceptionEnable=false; //also disable this, because we don't want to hear any commands when we are executing the task.

				//don't touch this part of code. Things to do when stop command is received-----
				if(useVoice)
				{
					sc.say("I will stop and wait until the user shows up again");
				}
				if(useVision && isMasterLearntByRecognizer)
				{

					stopMotors();
					searchTwoSegmentsEnable=false;
					currentState=SEARCH_TWO_SEGMENTS;
					timer10sec = nh_.createTimer(ros::Duration(10.0),boost::bind(&PersonFollower::timer10secCallback, this, _1),true);
					ros::spin();
				}
				else
				{
					cout<<"Error: Vision is disabled or Master isn't learnt yet!"<<endl;
				}
				//---------------------------------------------------------------------------

			}

		}
	}
	//ABOUT_SPEECH_END */

	if(virgin==true)
	{
		printf("1st Laser | ");

		for(int b=0;b<intDataPoints;b++)
		{
			if(inverted==true)
			{
				currentRangeValue=scan_in->ranges[b];
			}
			else
			{
				currentRangeValue=scan_in->ranges[intDataPoints-1-b];
			}

			if(currentRangeValue>floatMaxRadius)
			{
				currentRangeValue=floatMaxRadius;
			}
			doubleLaserDataXprevious[b]=currentRangeValue*cos(b*CV_PI/(intDataPoints-1));
			doubleLaserDataYprevious[b]=currentRangeValue*sin(b*CV_PI/(intDataPoints-1));		
		}

		if(plotLaser==true)
		{
			cvNamedWindow("LaserData", CV_WINDOW_AUTOSIZE);
			cvMoveWindow("LaserData", 00, 100);
		}
		virgin=false;


	}
	else //(virgin==false)
	{
		if(plotLaser==true)
		{ cvZero( outImg ); }

		//printf("NL | ");



		for(int b=0;b<intDataPoints;b++) //load the data to buffers
		{
			if(inverted==true)
			{
				currentRangeValue=scan_in->ranges[b];
			}
			else
			{
				currentRangeValue=scan_in->ranges[intDataPoints-1-b];
			}
			if(logLaser)
			{
				fprintf (logfile,"%lf ", currentRangeValue); //log laser data
				fprintf(logfile,"\t"); //log laser data
			}
			if(currentRangeValue>floatMaxRadius)
			{
				currentRangeValue=floatMaxRadius;
			}
			doubleLaserDataX[b]=currentRangeValue*cos(b*CV_PI/(intDataPoints-1));
			doubleLaserDataY[b]=currentRangeValue*sin(b*CV_PI/(intDataPoints-1));

			if(plotLaser==true)
			{
				cvCircle(outImg, cvPoint(plotParam1 - plotParam2 * doubleLaserDataX[b], plotParam1 - plotParam2 * doubleLaserDataY[b]), 1, CV_RGB(255,255,255), 1.2);
			} 		
		}
		if(logLaser)
		{
			fprintf(logfile,"\n"); //log laser data
			//fclose(logfile);
		}

		vector <Segment> SegmentIndexesVectorOfVectors;
		findSegmentsAndCreateCurrentGridMap(doubleLaserDataX,doubleLaserDataY,imgGridMap,imgGridMapIncludingSegmentNumbers,intNumOfSegments,SegmentIndexesVectorOfVectors);
		if (currentState==IDLE)
		{
			//cout<<"IDLE |"<<endl;
			doubleLaserDataXprevious=doubleLaserDataX;
			doubleLaserDataYprevious=doubleLaserDataY;

		}
		else if (currentState==NO_TARGET_TO_TRACK_MODE)
		{
			//cout<<"NO_TARGET_TO_TRACK_MODE |"<<endl;

			if(plotLaser==true)
			{
				double tempx1=plotParam1 - plotParam2 * -rectangularBoxXover2sizeForMovDetection;
				double tempy1=plotParam1 - plotParam2 * rectangularBoxYsizeForMovDetection;
				double tempx2=plotParam1 - plotParam2 * rectangularBoxXover2sizeForMovDetection;
				double tempy2=plotParam1 - plotParam2 * 0;
				cvRectangle(outImg, cvPoint(tempx1,tempy1), cvPoint(tempx2,tempy2), cvScalar(0,255,0), 1);
				cvShowImage("LaserData", outImg);
				cvWaitKey(5);
			}
			for(int i=0;i<intDataPoints;i++) //filling out PreviousGridMap.
			{
				cvScalarTemp=cvScalar(fillingForImages);
				cvSet2D(imgGridMapPrevious, (int)((doubleLaserDataYprevious[i]/floatGridLength)+0.5), (int)( ((doubleLaserDataXprevious[i]+floatMaxRadius) /floatGridLength)+0.5), cvScalarTemp);
			}
			cvNot(imgGridMapPrevious,imgGridMapPreviousNegated);
			cvMul(imgGridMap,imgGridMapPreviousNegated,DifferenceMap,1.0);
			cvErode(DifferenceMap,cleanedGridMap,meineKernel);

			cvMul(cleanedGridMap,imgGridMapIncludingSegmentNumbers,cleanedGridMapIncludingSegmentNumbers,(double)1/255);
			Segment validSegments;
			calculateCandidateSegments(cleanedGridMapIncludingSegmentNumbers,SegmentIndexesVectorOfVectors,intNumOfSegments,imgSize,validSegments);


			if(validSegments.empty())
			{
				//	cout<<" No Valid Segments "<<endl;
			}
			else //there is a moved segment
			{
				double firstPt[2];
				double lastPt[2];
				double centerPtt[2];
				int segNumberToTrack;
				int closestMovingSegment=8;
				double minMovingSegmentDistance=100.0;

				for (unsigned int h=0; h<validSegments.size();h++)
				{
					//cout<<"Segmo: "<<validSegments[h]<<endl;
					Segment segmentTester=SegmentIndexesVectorOfVectors[validSegments[h]];
					firstPt[0]=doubleLaserDataX[segmentTester[0]];
					firstPt[1]=doubleLaserDataY[segmentTester[0]];
					lastPt[0]=doubleLaserDataX[segmentTester[segmentTester.size()-1]];
					lastPt[1]=doubleLaserDataY[segmentTester[segmentTester.size()-1]];
					centerPtt[0]=((firstPt[0]+lastPt[0])/2);
					centerPtt[1]=((firstPt[1]+lastPt[1])/2);
					//cout<<"centerPtt[0]: "<<centerPtt[0]<<" centerPtt[1]: "<<centerPtt[1]<<endl;
					double tempDist=findDistanceBetweenTwoPoints(centerPtt,originPtt);
					//cout<<"tempDist= "<<tempDist<<endl;
					if(tempDist<minMovingSegmentDistance)
					{
						minMovingSegmentDistance=tempDist;
						closestMovingSegment=h;
					}
				}

				if(abs(centerPtt[0])<rectangularBoxXover2sizeForMovDetection && abs(centerPtt[1])<rectangularBoxYsizeForMovDetection)
				{
					segNumberToTrack=validSegments[closestMovingSegment];
					Segment segmentToTrack=SegmentIndexesVectorOfVectors[segNumberToTrack];
					firstPt[0]=doubleLaserDataX[segmentToTrack[0]];
					firstPt[1]=doubleLaserDataY[segmentToTrack[0]];
					lastPt[0]=doubleLaserDataX[segmentToTrack[segmentToTrack.size()-1]];
					lastPt[1]=doubleLaserDataY[segmentToTrack[segmentToTrack.size()-1]];

					memcpy(kalman->transition_matrix->data.fl,valsF,sizeof(valsF)); //F=4x4
					cvSetIdentity(kalman->measurement_matrix,cvRealScalar(1)); //H=4x2
					cvSetIdentity(kalman->process_noise_cov,cvRealScalar(qUncertainty)); //Q=4x4
					cvSetIdentity(kalman->measurement_noise_cov,cvRealScalar(rUncertainty)); //R=2x2

					memcpy(kalmanOdomFrame->transition_matrix->data.fl,valsF,sizeof(valsF)); //F=4x4
					cvSetIdentity(kalmanOdomFrame->measurement_matrix,cvRealScalar(1)); //H=4x2
					cvSetIdentity(kalmanOdomFrame->process_noise_cov,cvRealScalar(qUncertainty)); //Q=4x4
					cvSetIdentity(kalmanOdomFrame->measurement_noise_cov,cvRealScalar(rUncertainty)); //R=2x2



					float valsXk[]={ ((firstPt[0]+lastPt[0])/2) , ((firstPt[1]+lastPt[1])/2) ,0,0};	//was: float valsXk[]
					double dbltemp1=0.0;
					double dbltemp2=0.0;





					dbltemp1=((firstPt[0]+lastPt[0])/2);
					dbltemp2=((firstPt[1]+lastPt[1])/2);
					valsXk[0]=(float)dbltemp1;
					valsXk[1]=(float)dbltemp2;
					memcpy(kalman->state_post->data.fl,valsXk,sizeof(valsXk)); // set the initial Xk


					pointTransformer(listener3, ((firstPt[1]+lastPt[1])/2),-((firstPt[0]+lastPt[0])/2),dbltemp1,dbltemp2,false,"laser","odom");
					valsXk[0]=(float)dbltemp1;
					valsXk[1]=(float)dbltemp2;
					memcpy(kalmanOdomFrame->state_post->data.fl,valsXk,sizeof(valsXk)); // set the initial Xk




					cout<<"STARTED TRACKING.."<<endl;
					//cout<<"Xcenter: "<<valsXk[0]<<" Ycenter: "<<valsXk[1]<<endl;

					currentState=ONE_LEG_TRACK_MODE;
					validSegments.clear();
				}
				else
				{
					cout<<"Moved segment not within range.."<<endl;
				}

			}

			cvCopy(imgGridMap, imgGridMapPrevious);
		} // end of NO_TARGET_TO_TRACK_MODE

		else if(currentState==ONE_LEG_TRACK_MODE)
		{
			//cout<<"ONE_LEG_TRACK_MODE | ";
			Segment segTemp;
			CvMat* z_k=cvCreateMat(2,1,CV_32FC1);


			const CvMat* y_k;
			const CvMat* y_kOdom;
			CvMat* test;

			y_kOdom=cvKalmanPredict(kalmanOdomFrame,0);
			y_k=cvKalmanPredict(kalman,0);
			if(useOdometry)
			{
				test=kalmanOdomFrame->state_pre;
			}
			else
			{
				test=kalman->state_pre;
			}


			//cout<<" XkPrior: "<<endl;
			//PrintMat(test);

			//			double minDistanceToPredictedState=100.0;
			//			int closestSegmentToPredictedState=100;
			double checkedCenterPoint[2];
			double checkedCenterPointLaserFrame[2];
			double estimatedCenterPointOfTarget[2];
			double firstPt[2];
			double lastPt[2];

			estimatedCenterPointOfTarget[0]=cvGet1D(test,0).val[0];
			estimatedCenterPointOfTarget[1]=cvGet1D(test,1).val[0];
			//cout<<"est X: "<<estimatedCenterPointOfTarget[0]<<" est Y: "<<estimatedCenterPointOfTarget[1]<<endl;


			double xxEst;
			double yyEst;
			double xxEstMyCoord;
			double yyEstMyCoord;
			if(useOdometry==true)
			{
				pointTransformer(listener3, estimatedCenterPointOfTarget[0],estimatedCenterPointOfTarget[1],xxEst,yyEst,false,"odom","laser");
				xxEstMyCoord=-yyEst;
				yyEstMyCoord=xxEst;
			}
			else
			{
				xxEstMyCoord=estimatedCenterPointOfTarget[0];
				yyEstMyCoord=estimatedCenterPointOfTarget[1];
			}
			if(plotLaser==true)
			{
				double tempx=plotParam1 - plotParam2 * xxEstMyCoord;
				double tempy=plotParam1 - plotParam2 * yyEstMyCoord;
				cvCircle(outImg, cvPoint(tempx,tempy), 4, cvScalar(0,255,0), 4);
			}



			vector<Segment> RefinedSegmentIndexesVectorOfVectors; //going to fill that!
			vector<double> RefinedSegmentsDistanceTotals;
			double minDistanceToPredictedState=100.0;
			int closestSegmentToPredictedState=100;

			if(featWindowInitPhase==false)
			{

				for(unsigned int t=0;t<SegmentIndexesVectorOfVectors.size();t++)
				{
					segTemp=SegmentIndexesVectorOfVectors[t];
					//cout<<endl<<"Segment #: "<<t<<endl;

					firstPt[0]=doubleLaserDataX[segTemp[0]];
					firstPt[1]=doubleLaserDataY[segTemp[0]];
					lastPt[0]=doubleLaserDataX[segTemp[segTemp.size()-1]];
					lastPt[1]=doubleLaserDataY[segTemp[segTemp.size()-1]];
					checkedCenterPoint[0]=((firstPt[0]+lastPt[0])/2);
					checkedCenterPoint[1]=((firstPt[1]+lastPt[1])/2);

					if(useOdometry==true)
					{
						pointTransformer(listener3, ((firstPt[1]+lastPt[1])/2),-((firstPt[0]+lastPt[0])/2),checkedCenterPoint[0],checkedCenterPoint[1],false,"laser","odom");
					}
					else
					{
						checkedCenterPoint[0]=((firstPt[0]+lastPt[0])/2);
						checkedCenterPoint[1]=((firstPt[1]+lastPt[1])/2);
					}

					double temperDist=findDistanceBetweenTwoPoints(estimatedCenterPointOfTarget,checkedCenterPoint);
					//cout<<"temperDist : "<<temperDist<<endl;
					double currentSegmentWidth=0.0;
					double currentSegmentIAV=0.0;
					double currentSegmentCircCriterion=0.0;
					double currentSegmentDistance=0.0;
//currentSegmentDistance=calculateSegmentDistance(currentSegmentWidth,currentSegmentIAV,currentSegmentCircCriterion,false,temperDist);

					//if(temperDist<5.0)
					if(temperDist<maxDistanceToMatchPrediction)
					{
						currentSegmentWidth=findDistanceBetweenTwoPoints(firstPt,lastPt);
						bool oneLegBool=true;

						if(featWindowInitPhase)
						{
							if(currentSegmentWidth>(maxEuclidianDistOfSegmentsThatCanBePersonCandids/2)) //BLOB case
							{
								oneLegBool=false;
							}
							else //1LEG case
							{
								oneLegBool=true;
							}
						}
						else //if(featWindowInitPhase==false)
						{
							if(currentSegmentWidth>(featureMeans[0]+2.5*featureVariances[0])) //BLOB case
							{
								oneLegBool=false;
							}
							else //1LEG case
							{
								oneLegBool=true;
							}
						} //end of if(featWindowInitPhase)

						oneLegBool=true;

						if(oneLegBool)//1LEG case
						{
							//cout<<"1LEG"<<endl;
							currentSegmentIAV=calculateIAV(segTemp);
							currentSegmentCircCriterion=calculateCircCriterion(segTemp,currentSegmentWidth);
							currentSegmentDistance=calculateSegmentDistance(currentSegmentWidth,currentSegmentIAV,currentSegmentCircCriterion,true,temperDist);
							RefinedSegmentsDistanceTotals.push_back(currentSegmentDistance);
							RefinedSegmentIndexesVectorOfVectors.push_back(segTemp);

							if(plotLaser==true)
							{
								double tempx=plotParam1 - plotParam2 * checkedCenterPoint[0];
								double tempy=plotParam1 - plotParam2 * checkedCenterPoint[1];
								int n;
								char commentBox[100];
								//n  = sprintf(commentBox,"Width:%1.2f|Linearity:%1.2f|Circularity:%1.2f|Distance:%2.2f",currentSegmentWidth,currentSegmentIAV,currentSegmentCircCriterion,currentSegmentDistance);
								n  = sprintf(commentBox,"Dist: %2.2f",currentSegmentDistance);
								cvPutText (outImg,commentBox,cvPoint(tempx,tempy), &font, cvScalar(0,255,0));
							}

						}
						else //BLOB case
						{
							//cout<<"BLOB"<<endl;
							int segLength=segTemp.size();
							int midIndex=static_cast<int>(segLength)/2;
							Segment blobSeg1;
							Segment blobSeg2;
							for(int v=0;v<segLength;v++)
							{
								if(v<midIndex)
								{
									blobSeg1.push_back(segTemp[v]);
								}
								else
								{
									blobSeg2.push_back(segTemp[v]);
								}
							}
							double blobSegfirstPt[2];
							double blobSeglastPt[2];
							double blobCheckedCenterPoint[2];


							double physicalDistBlob;
							blobSegfirstPt[0]=doubleLaserDataX[blobSeg1[0]];
							blobSegfirstPt[1]=doubleLaserDataY[blobSeg1[0]];
							blobSeglastPt[0]=doubleLaserDataX[blobSeg1[blobSeg1.size()-1]];
							blobSeglastPt[1]=doubleLaserDataY[blobSeg1[blobSeg1.size()-1]];
							blobCheckedCenterPoint[0]=((blobSegfirstPt[0]+blobSeglastPt[0])/2);
							blobCheckedCenterPoint[1]=((blobSegfirstPt[1]+blobSeglastPt[1])/2);

							currentSegmentWidth=findDistanceBetweenTwoPoints(blobSegfirstPt,blobSeglastPt);
							currentSegmentIAV=calculateIAV(blobSeg1);
							currentSegmentCircCriterion=calculateCircCriterion(blobSeg1,currentSegmentWidth);
							physicalDistBlob=findDistanceBetweenTwoPoints(estimatedCenterPointOfTarget,blobCheckedCenterPoint);
							currentSegmentDistance=calculateSegmentDistance(currentSegmentWidth,currentSegmentIAV,currentSegmentCircCriterion,true,physicalDistBlob);
							RefinedSegmentsDistanceTotals.push_back(currentSegmentDistance);
							RefinedSegmentIndexesVectorOfVectors.push_back(blobSeg1);
							if(plotLaser)
							{
								double tempx=plotParam1 - plotParam2 * blobCheckedCenterPoint[0];
								double tempy=plotParam1 - plotParam2 * blobCheckedCenterPoint[1];
								int n;
								char commentBox[100];
								//n  = sprintf(commentBox,"Width:%1.2f|Linearity:%1.2f|Circularity:%1.2f|Distance:%2.2f",currentSegmentWidth,currentSegmentIAV,currentSegmentCircCriterion,currentSegmentDistance);
								n  = sprintf(commentBox,"Dist: %2.2f",currentSegmentDistance);
								cvPutText (outImg,commentBox,cvPoint(tempx,tempy), &font, cvScalar(0,255,0));
							}

							blobSegfirstPt[0]=doubleLaserDataX[blobSeg2[0]];
							blobSegfirstPt[1]=doubleLaserDataY[blobSeg2[0]];
							blobSeglastPt[0]=doubleLaserDataX[blobSeg2[blobSeg2.size()-1]];
							blobSeglastPt[1]=doubleLaserDataY[blobSeg2[blobSeg2.size()-1]];
							blobCheckedCenterPoint[0]=((blobSegfirstPt[0]+blobSeglastPt[0])/2);
							blobCheckedCenterPoint[1]=((blobSegfirstPt[1]+blobSeglastPt[1])/2);

							currentSegmentWidth=findDistanceBetweenTwoPoints(blobSegfirstPt,blobSeglastPt);
							currentSegmentIAV=calculateIAV(blobSeg2);
							currentSegmentCircCriterion=calculateCircCriterion(blobSeg2,currentSegmentWidth);
							physicalDistBlob=findDistanceBetweenTwoPoints(estimatedCenterPointOfTarget,blobCheckedCenterPoint);
							currentSegmentDistance=calculateSegmentDistance(currentSegmentWidth,currentSegmentIAV,currentSegmentCircCriterion,true,physicalDistBlob);
							RefinedSegmentsDistanceTotals.push_back(currentSegmentDistance);
							RefinedSegmentIndexesVectorOfVectors.push_back(blobSeg2);
							if(plotLaser)
							{
								double tempx=plotParam1 - plotParam2 * blobCheckedCenterPoint[0];
								double tempy=plotParam1 - plotParam2 * blobCheckedCenterPoint[1];
								int n;
								char commentBox[100];
								//n  = sprintf(commentBox,"Width:%1.2f|Linearity:%1.2f|Circularity:%1.2f|Distance:%2.2f",currentSegmentWidth,currentSegmentIAV,currentSegmentCircCriterion,currentSegmentDistance);
								n  = sprintf(commentBox,"Dist: %2.2f",currentSegmentDistance);
								cvPutText (outImg,commentBox,cvPoint(tempx,tempy), &font, cvScalar(0,255,0));
							}

						} //end of BLOB case. From now on, refined segments are saved, scores are calculated.

					} //if(temperDist<maxDistanceToMatchPrediction) ends
				} //for all segments, ends.
			}
			else
			{
				for(unsigned int t=0;t<SegmentIndexesVectorOfVectors.size();t++)
				{
					segTemp=SegmentIndexesVectorOfVectors[t];
					//cout<<endl<<"Checking Segment: "<<t<<endl;

					firstPt[0]=doubleLaserDataX[segTemp[0]];
					firstPt[1]=doubleLaserDataY[segTemp[0]];
					lastPt[0]=doubleLaserDataX[segTemp[segTemp.size()-1]];
					lastPt[1]=doubleLaserDataY[segTemp[segTemp.size()-1]];
					checkedCenterPoint[0]=((firstPt[0]+lastPt[0])/2);
					checkedCenterPoint[1]=((firstPt[1]+lastPt[1])/2);

					if(useOdometry==true)
					{
						pointTransformer(listener3, ((firstPt[1]+lastPt[1])/2),-((firstPt[0]+lastPt[0])/2),checkedCenterPoint[0],checkedCenterPoint[1],false,"laser","odom");
					}
					else
					{
						checkedCenterPoint[0]=((firstPt[0]+lastPt[0])/2);
						checkedCenterPoint[1]=((firstPt[1]+lastPt[1])/2);
					}

					double temperDist=findDistanceBetweenTwoPoints(estimatedCenterPointOfTarget,checkedCenterPoint);
					//cout<<" temperDist : "<<temperDist<<endl;
					if(temperDist<minDistanceToPredictedState)
					{
						minDistanceToPredictedState=temperDist;
						closestSegmentToPredictedState=t;
					}

				}
			}

			//change the distance calculation(include location)
			bool boolAtLeastOneSegmentMatch;
			double minDist=1000.0;
			unsigned int minDistIndex=1000;
			if(RefinedSegmentsDistanceTotals.size()==0)
			{
				//cout<<"Candid Seg num: 0"<<endl;
				boolAtLeastOneSegmentMatch=false;
			}
			else
			{
				//cout<<"Candid Seg num: "<<RefinedSegmentsDistanceTotals.size()<<endl;
				boolAtLeastOneSegmentMatch=true;
			}
			if(boolAtLeastOneSegmentMatch)
			{
				for(unsigned int k=0;k<RefinedSegmentsDistanceTotals.size();k++) //find the minimum of distance among all distances.
				{
					if(RefinedSegmentsDistanceTotals[k]<minDist) //find the max score for this particular person
					{
						minDist=RefinedSegmentsDistanceTotals[k];
						minDistIndex=k;
					}
				}
			}

			//cout<<"minDist: "<<minDist<<endl;
			bool boolLegitimateSegment;
			if(featWindowInitPhase)
			{
				if(closestSegmentToPredictedState!=100 && minDistanceToPredictedState<maxDistanceToMatchPrediction)
				{
					boolLegitimateSegment=true;
				}
				else
				{
					boolLegitimateSegment=false;
				}
			}
			else
			{
				if(boolAtLeastOneSegmentMatch && minDist<1.1)
				{
					boolLegitimateSegment=true;
				}
				else
				{
					boolLegitimateSegment=false;
				}
			}

			if(boolLegitimateSegment)
			{

				if(oneShotPersonRecognizeChance==true) //Person is re-detected by personRecognizer module. now shut the service off.
				{
					personrecognizerCallbackReception=false;
					oneShotPersonRecognizeChance=false;
					inhibitNavigationMovement = false;
					decreaseRecognitionThresholdEnable=false;
					periodicTimer1sec.stop();


					person_recognition::PersonRecControl msg2;
					msg2.recognitionThresholdChange=true;
					msg2.searchedPersonsNameChange=false;
					msg2.currentStateChange=false;
					msg2.isNameNeededChange=false;
					msg2.learnedPersonsNameChange=false;

					msg2.recognitionThreshold=defaultStartingRecognitionThreshold;
					msg2.currentState=0;
					person_control_publisher_.publish(msg2);

					stopCommandReceptionEnable=true; //"STOP" task is completed. enable the reception of the speech command back.
					waitCommandReceptionEnable=true;
				}

				if(featWindowInitPhase)
				{
					segTemp=SegmentIndexesVectorOfVectors[closestSegmentToPredictedState];
					if(featWindowCounter==featureUpdateWindowSize)
					{
						if(!boolUseFixedFeatureParams)
						{
							featureMeans[0]=featureTotals[0]/featWindowCounter;
							featureMeans[1]=featureTotals[1]/featWindowCounter;
							featureMeans[2]=featureTotals[2]/featWindowCounter;
cout<<"FEAT TOTALS: "<<featureTotals[0]<<" "<<featureTotals[1]<<" "<<featureTotals[2]<<endl;
							cout<<"LEARNED FEAT MEANS: "<<featureMeans[0]<<" "<<featureMeans[1]<<" "<<featureMeans[2]<<endl;
						}
						featWindowInitPhase=false;
					}
					else
					{
						//add to totals
						double bblobSegfirstPt[2];
						double bblobSeglastPt[2];
						bblobSegfirstPt[0]=doubleLaserDataX[segTemp[0]];
						bblobSegfirstPt[1]=doubleLaserDataY[segTemp[0]];
						bblobSeglastPt[0]=doubleLaserDataX[segTemp[segTemp.size()-1]];
						bblobSeglastPt[1]=doubleLaserDataY[segTemp[segTemp.size()-1]];
						double curSegW=findDistanceBetweenTwoPoints(bblobSegfirstPt,bblobSeglastPt);

						featureTotals[0]=featureTotals[0]+findDistanceBetweenTwoPoints(bblobSegfirstPt,bblobSeglastPt);
						featureTotals[1]=featureTotals[1]+calculateIAV(segTemp);
						featureTotals[2]=featureTotals[2]+calculateCircCriterion(segTemp,curSegW);
						featWindowCounter++;
					}
				}
				else
				{
					segTemp=RefinedSegmentIndexesVectorOfVectors[minDistIndex];
				}

				if(!boolUseFixedFeatureParams)
				{
					double bblobSegfirstPt[2];
					double bblobSeglastPt[2];
					

					bblobSegfirstPt[0]=doubleLaserDataX[segTemp[0]];
					bblobSegfirstPt[1]=doubleLaserDataY[segTemp[0]];
					bblobSeglastPt[0]=doubleLaserDataX[segTemp[segTemp.size()-1]];
					bblobSeglastPt[1]=doubleLaserDataY[segTemp[segTemp.size()-1]];

					double curSegW=findDistanceBetweenTwoPoints(bblobSegfirstPt,bblobSeglastPt);
					updateFeatureParameters(curSegW,calculateIAV(segTemp),calculateCircCriterion(segTemp,curSegW),featureUpdateWindowSize);

				}

				firstPt[0]=doubleLaserDataX[segTemp[0]];
				firstPt[1]=doubleLaserDataY[segTemp[0]];
				lastPt[0]=doubleLaserDataX[segTemp[segTemp.size()-1]];
				lastPt[1]=doubleLaserDataY[segTemp[segTemp.size()-1]];
				checkedCenterPointLaserFrame[0]=((firstPt[0]+lastPt[0])/2);
				checkedCenterPointLaserFrame[1]=((firstPt[1]+lastPt[1])/2);

				checkedCenterPoint[0]=((firstPt[0]+lastPt[0])/2);
				checkedCenterPoint[1]=((firstPt[1]+lastPt[1])/2);

				cvmSet(z_k, 0, 0, checkedCenterPoint[0]);
				cvmSet(z_k, 1, 0, checkedCenterPoint[1]);
				//cout<<"X: "<<checkedCenterPoint[0]<<" Y: "<<checkedCenterPoint[1]<<endl;
				cvKalmanCorrect(kalman,z_k);



				pointTransformer(listener3, ((firstPt[1]+lastPt[1])/2),-((firstPt[0]+lastPt[0])/2),checkedCenterPoint[0],checkedCenterPoint[1],false,"laser","odom");
				cvmSet(z_k, 0, 0, checkedCenterPoint[0]);
				cvmSet(z_k, 1, 0, checkedCenterPoint[1]);
				//cout<<"X: "<<checkedCenterPoint[0]<<" Y: "<<checkedCenterPoint[1]<<endl;
				cvKalmanCorrect(kalmanOdomFrame,z_k);

				CvMat* test2=kalman->state_post;
				CvMat* test3=kalmanOdomFrame->state_post;
				//cout<<"Xk: "<<endl;
				//PrintMat(test2);

				double Vx;
				double Vy;
				if(useOdomForSpeed)
				{
					Vx=cvGet1D(test3,2).val[0];
					Vy=cvGet1D(test3,3).val[0];
				}
				else
				{
					Vx=cvGet1D(test2,2).val[0];
					Vy=cvGet1D(test2,3).val[0];
				}

				double VTotal=sqrt(Vx*Vx+Vy*Vy);
				//cout<<"VTotal: "<<VTotal<<endl;
				maxDistanceToMatchPrediction=0.3+VTotal/2;
				if(maxDistanceToMatchPrediction<0.3)
				{
					maxDistanceToMatchPrediction=0.3;
				}
				else if(maxDistanceToMatchPrediction>0.55)
				{
					maxDistanceToMatchPrediction=0.55;
				}
				//cout<<"maxMatchPredict: "<<maxDistanceToMatchPrediction<<endl;

				if(VTotal<0.007)
				{
					targetNotMovingCounter++;
					if( (static_cast<double>(targetNotMovingCounter/operatingFrequency)>8.0))
					{
						if(useVoice)
						{
							sc.say("Stopped following");
						}
						targetNotMovingCounter=0;
						currentState=NO_TARGET_TO_TRACK_MODE;
					}
				}
				else
				{
					targetNotMovingCounter=0;
				}

				double distToPerson=findDistanceBetweenTwoPoints(checkedCenterPointLaserFrame,originPtt);
				//double personAngle=atan2(checkedCenterPoint[1],checkedCenterPoint[0])*180/CV_PI-90; //+y axis is 0 degrees
				double personAngle=atan2(checkedCenterPointLaserFrame[1],checkedCenterPointLaserFrame[0])*180/CV_PI-90; //+y axis is 0 degrees

				double angleOfPersonRadiansRaw=atan2(checkedCenterPointLaserFrame[1],checkedCenterPointLaserFrame[0]);
				double angleOfPersonRadiansAdjusted=angleOfPersonRadiansRaw-CV_PI/2;

				/*cout<<"X: "<<checkedCenterPointLaserFrame[0]<<" Y: "<<checkedCenterPointLaserFrame[1]<<endl;
				cout<<"distToPerson: "<<distToPerson<<endl;
					cout<<"personAngle: "<<personAngle<<endl; 
					cout<<"angleOfPersonRadiansAdjusted: "<<angleOfPersonRadiansAdjusted<<endl;*/

				if(currentMotionState==REGULAR_FOLLOW)
				{
					if(useMoveBase==true)
					{


						if(distToPerson>distanceToKeepWithPerson)
						{
							/*
							double targetXmyCoordstest= checkedCenterPointLaserFrame[0]-distanceToKeepWithPerson*cos(angleOfPersonRadiansRaw+tetaNearby);
							double targetYmyCoordstest= checkedCenterPointLaserFrame[1]-distanceToKeepWithPerson*sin(angleOfPersonRadiansRaw+tetaNearby);
							if(plotLaser==true)
							{
								double tempx=plotParam1 - plotParam2 * targetXmyCoordstest;
								double tempy=plotParam1 - plotParam2 * targetYmyCoordstest;
								cvCircle(outImg, cvPoint(tempx,tempy), 4, cvScalar(0,128,255), 3);
							}
							 */
							double targetXmyCoords= checkedCenterPointLaserFrame[0]-distanceToKeepWithPerson*cos(angleOfPersonRadiansRaw);
							double targetYmyCoords= checkedCenterPointLaserFrame[1]-distanceToKeepWithPerson*sin(angleOfPersonRadiansRaw);
							if(plotLaser==true)
							{
								double tempx=plotParam1 - plotParam2 * targetXmyCoords;
								double tempy=plotParam1 - plotParam2 * targetYmyCoords;
								cvCircle(outImg, cvPoint(tempx,tempy), 4, cvScalar(255,0,255), 2);
							}

							//geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(angleOfPersonRadiansAdjusted);
							geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(-angleOfPersonRadiansAdjusted);
							move_base_msgs::MoveBaseGoal goal;
							goal.target_pose.header.frame_id = "laser";
							goal.target_pose.header.stamp = ros::Time::now();

							goal.target_pose.pose.position.x = targetYmyCoords;
							goal.target_pose.pose.position.y = targetXmyCoords;

							//goal.target_pose.pose.position.x = targetYmyCoords;
							//goal.target_pose.pose.position.y = -targetXmyCoords;

							goal.target_pose.pose.orientation=quat;

							//ROS_INFO("Sending positional goal");
							if(inhibitNavigationMovement==false && permanentInhibitNavigationMovement==false)
							{
								ac.sendGoal(goal);
							}

						}
						else
						{
							//geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(angleOfPersonRadiansAdjusted);
							geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(-angleOfPersonRadiansAdjusted);
							move_base_msgs::MoveBaseGoal goal;
							goal.target_pose.header.frame_id = "laser";
							goal.target_pose.header.stamp = ros::Time::now();

							goal.target_pose.pose.orientation=quat;

							//ROS_INFO("Sending orientation goal");
							if(inhibitNavigationMovement==false && permanentInhibitNavigationMovement==false)
							{
								ac.sendGoal(goal);
							}

						}

						if(enableComeNearbyBehavior==true)
						{

							if(checkForPersonToStop==false) //robot is looking if the person is moving again.
							{

								if(VTotal>0.1) //person is dynamic
								{
									velocityCounter++;
									if(velocityCounter>(unsigned int)(2*operatingFrequency))
									{
										velocityCounter=0;
										cout<<"PERSON IS DYNAMIC AGAIN--------------------------"<<endl;
										checkForPersonToStop=true;
									}

								}
								else //person is not dynamic
								{
									velocityCounter=0;
								}

							}
							else if(checkForPersonToStop==true)
							{

								if(VTotal<0.1) //person is static
								{
									velocityCounter++;
									if(velocityCounter>(unsigned int)(9*operatingFrequency))
									{
										velocityCounter=0;
										checkForPersonToStop=false;
										cout<<"PERSON STOPPED-----------------------------"<<endl;



										double targetXmyCoords= checkedCenterPointLaserFrame[0]-(distanceToKeepWithPerson+0.5)*cos(angleOfPersonRadiansRaw-tetaNearby);
										double targetYmyCoords= checkedCenterPointLaserFrame[1]-(distanceToKeepWithPerson+0.5)*sin(angleOfPersonRadiansRaw-tetaNearby);
										if(plotLaser==true)
										{
											double tempx=plotParam1 - plotParam2 * targetXmyCoords;
											double tempy=plotParam1 - plotParam2 * targetYmyCoords;
											cvCircle(outImg, cvPoint(tempx,tempy), 3, cvScalar(255,0,255), 2);
										}
										geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(-angleOfPersonRadiansAdjusted+tetaNearby);
										move_base_msgs::MoveBaseGoal goal;
										goal.target_pose.header.frame_id = "laser";
										goal.target_pose.header.stamp = ros::Time::now();
										goal.target_pose.pose.position.x = targetYmyCoords;
										goal.target_pose.pose.position.y = targetXmyCoords;
										goal.target_pose.pose.orientation=quat;
										currentMotionState=GO_NEARBY_PERSON;
										ac.sendGoal(goal);


									}

								}
								else //person is not static
								{
									velocityCounter=0;
								}
							}

						}



					} //end of useMoveBase==true
					else
					{


						if(inhibitAngularMovement==false) //determine angular speeds
						{
							thres0=23;
							thres1=41;
							thres2=54;
							thres3=66;
							thres4=74;
							if(checkedCenterPointLaserFrame[1]<distanceToKeepWithPerson)
							{
								if(personAngle>0)
								{
									if(personAngle<thres0)
									{
										cmd.angular.z = 0;
									}
									else if(personAngle>thres4) //person is on right.
									{
										cmd.angular.z = 0.72;
									}
									else if(personAngle>thres3) //person is on right.
									{
										cmd.angular.z = 0.62;
									}
									else if(personAngle>thres2) //person is on right.
									{
										cmd.angular.z = 0.53;
									}
									else if(personAngle>thres1) //person is on right.
									{
										cmd.angular.z = 0.44;
									}
									else if(personAngle>thres0) //person is on right.
									{
										cmd.angular.z = 0.3;
									}

								}
								else
								{
									if(personAngle<-thres4) //person is on left.
									{
										cmd.angular.z = -0.72;
									}
									else if(personAngle<-thres3) //person is on left.
									{
										cmd.angular.z = -0.62;
									}
									else if(personAngle<-thres2) //person is on L.
									{
										cmd.angular.z = -0.53;
									}
									else if(personAngle<-thres1) //person is on left
									{
										cmd.angular.z = -0.44;
									}
									else if(personAngle<-thres0) //person is on right.
									{
										cmd.angular.z = -0.3;
									}
								}
							}
							else //(checkedCenterPointLaserFrame[1]>distanceToKeepWithPerson)
							{
								thres0=23;
								thres1=32;
								thres2=45;
								thres3=60;
								thres4=70;

								double limit0=distanceToKeepWithPerson*tan(thres0*CV_PI/180);
								double limit1=distanceToKeepWithPerson*tan(thres1*CV_PI/180);
								double limit2=distanceToKeepWithPerson*tan(thres2*CV_PI/180);
								double limit3=distanceToKeepWithPerson*tan(thres3*CV_PI/180);
								double limit4=distanceToKeepWithPerson*tan(thres4*CV_PI/180);

								double absX=abs(checkedCenterPointLaserFrame[0]);
								double speed;
								if(absX>limit4)
								{
									speed = 0.72;
								}
								else if(absX>limit3)
								{
									speed = 0.62;
								}
								else if(absX>limit2)
								{
									speed = 0.52;
								}
								else if(absX>limit1)
								{
									speed = 0.44;
								}
								else if(absX>limit0)
								{
									speed = 0.3;
								}
								else
								{
									speed = 0.0;
								}

								if(personAngle>0) //person is on right.
										{
									cmd.angular.z=speed;
										}
								else //person is on L.
								{
									cmd.angular.z=-speed;
								}

							}
						}
						else //if(inhibitAngularMovement==true)
						{
							cmd.angular.z=0.0;
						}

						if(inhibitLinearMovement==false)
						{
							//now linear speeds
							if(distToPerson>distanceToKeepWithPerson+2.4)
							{
								cmd.linear.x=0.9+extraVelocityBeCareful;
							}
							else if(distToPerson>distanceToKeepWithPerson+1.9)
							{
								cmd.linear.x=0.8+extraVelocityBeCareful;
							}
							else if(distToPerson>distanceToKeepWithPerson+1.6)
							{
								cmd.linear.x=0.7+extraVelocityBeCareful;
							}
							if(distToPerson>distanceToKeepWithPerson+1.3)
							{
								cmd.linear.x=0.65+extraVelocityBeCareful;
							}
							else if(distToPerson>distanceToKeepWithPerson+1.1)
							{
								cmd.linear.x=0.61+extraVelocityBeCareful;
							}
							else if(distToPerson>distanceToKeepWithPerson+0.9)
							{
								cmd.linear.x=0.57+extraVelocityBeCareful;
							}
							else if(distToPerson>distanceToKeepWithPerson+0.7)
							{
								cmd.linear.x=0.51+extraVelocityBeCareful;
							}
							else if(distToPerson>distanceToKeepWithPerson+0.5)
							{
								cmd.linear.x=0.47+extraVelocityBeCareful;
							}
							else if(distToPerson>distanceToKeepWithPerson+0.45)
							{
								cmd.linear.x=0.41+extraVelocityBeCareful;
							}
							else if(distToPerson>distanceToKeepWithPerson+0.4)
							{
								cmd.linear.x=0.37+extraVelocityBeCareful;
							}
							else if(distToPerson>distanceToKeepWithPerson+0.35)
							{
								cmd.linear.x=0.33+extraVelocityBeCareful;
							}
							else if(distToPerson>distanceToKeepWithPerson+0.3)
							{
								cmd.linear.x=0.3+extraVelocityBeCareful;
							}
							else if(distToPerson>distanceToKeepWithPerson+0.25)
							{
								cmd.linear.x=0.28+extraVelocityBeCareful;
							}
							else if(distToPerson>distanceToKeepWithPerson+0.2)
							{
								cmd.linear.x=0.26+extraVelocityBeCareful;
							}
							else if(distToPerson>distanceToKeepWithPerson+0.15)
							{
								cmd.linear.x=0.23+extraVelocityBeCareful;
							}
							else if(distToPerson>distanceToKeepWithPerson+0.1)
							{
								cmd.linear.x=0.2+extraVelocityBeCareful;
							}
							else if(distToPerson>distanceToKeepWithPerson+0.05)
							{
								cmd.linear.x=0.17+extraVelocityBeCareful;
							}
							else if(distToPerson>distanceToKeepWithPerson)
							{
								cmd.linear.x=0.15;
							}
							else
							{
								cmd.linear.x=0;
							}
						}
						else //if(inhibitLinearMovement==true)
						{
							cmd.linear.x=0;
						}
						//cout<<"Person is @angle : " <<personAngle<<", @distance"<<distToPerson<<endl;
						if(inhibitNavigationMovement==false && permanentInhibitNavigationMovement==false)
						{
							commander_pub.publish(cmd);
						}

					} //end of useMoveBase==false
				} //end of currentMotionState==REGULAR_FOLLOW
				else if(currentMotionState==GO_NEARBY_PERSON)
				{
					cout<<"ENTERED GO_NEARBY_PERSON----------------"<<endl;
					ac.waitForResult();
					if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
					{
						ROS_INFO("GO_NEARBY_PERSON SUCCESSFUL!");
						currentMotionState=REGULAR_FOLLOW;
					}

				}


				if(plotLaser==true)
				{
					double tempx=plotParam1 - plotParam2 * checkedCenterPointLaserFrame[0];
					double tempy=plotParam1 - plotParam2 * checkedCenterPointLaserFrame[1];

					//cvCircle(outImg, cvPoint(tempx,tempy), 20, cvScalar(0,255,0), 1);
					//tempx=plotParam1 - plotParam2 * estimatedCenterPointOfTarget[0];
					//tempy=plotParam1 - plotParam2 * estimatedCenterPointOfTarget[1];
					cvCircle(outImg, cvPoint(tempx,tempy), 8, cvScalar(0,255,255), 2);
				}
			} //end of: if segment is a good match.
			else
			{
				cout<<"NO GOOD SEGMENT"<<endl;
				if(oneShotPersonRecognizeChance==true) //Person recognizer suggestion faulty
				{
					currentState=IDLE;
					oneShotPersonRecognizeChance=false;
					cout<<"1-SHOT PERSON RECOGNIZE DIDN'T HOLD"<<endl;
				}

				//cvKalmanCorrect(kalman,z_k);
				//cmd.linear.x=0;
				//cmd.angular.z=0;
				//commander_pub.publish(cmd);
				//CvMat* test2=kalman->state_post;
			}

		}// end of ONE_LEG_TRACK_MODE

		else if (currentState == TWO_LEGS_TRACK_MODE)
		{

		}// end of TWO_LEGS_TRACK_MODE
		else if (currentState == SEARCH_TWO_SEGMENTS)
		{

			if(searchTwoSegmentsEnable==true)
			{
				//cout<<"SEARCH_TWO_SEGMENTS | ";
				Segment segTemp;

				unsigned int segmentCounter=0;

				for(unsigned int t=0;t<SegmentIndexesVectorOfVectors.size();t++)
				{
					double checkedCenterPoint[2];
					double firstPt[2];
					double lastPt[2];

					segTemp=SegmentIndexesVectorOfVectors[t];
					//cout<<endl<<"Checking Segment: "<<t<<endl;
					firstPt[0]=doubleLaserDataX[segTemp[0]];
					firstPt[1]=doubleLaserDataY[segTemp[0]];
					lastPt[0]=doubleLaserDataX[segTemp[segTemp.size()-1]];
					lastPt[1]=doubleLaserDataY[segTemp[segTemp.size()-1]];
					checkedCenterPoint[0]=((firstPt[0]+lastPt[0])/2);
					checkedCenterPoint[1]=((firstPt[1]+lastPt[1])/2);

					double temperDist=findDistanceBetweenTwoPoints(originPtt,checkedCenterPoint);
					//cout<<" temperDist : "<<temperDist<<endl;
					if(temperDist<checkpoint3detectionRadius)
					{
						segmentCounter++;
					}
				}
				if(segmentCounter>1) //there must be at least 2 segments within range. Achieved. Changing state.
				{
					cout<<"2 segments found. changing state to: SEARCH_TWO_SEGMENTS"<<endl;
					currentState=IDLE;
					timer4sec = nh_.createTimer(ros::Duration(2.0),boost::bind(&PersonFollower::timer4secCallback, this, _1),true);
					ros::spin();
				}

			}

		}// end of SEARCH_TWO_SEGMENTS

		if (plotLaser == true)
		{
			cvShowImage("LaserData", outImg);
			cvWaitKey(2);
		}

		doubleLaserDataXprevious = doubleLaserDataX;
		doubleLaserDataYprevious = doubleLaserDataY;
		cvSetZero(imgGridMap);

	} //end of (virgin==false)
} // end of scanCallback function
