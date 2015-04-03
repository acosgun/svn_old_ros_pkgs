//#include <cv_bridge/CvBridge.h>
#include <PersonRecognizerRobocup.h>
#include <stdio.h>

#include <ptu_46/PtuCmd.h>
#include <ptu_46/PtuLookAt.h>

PersonRecognizer::PersonRecognizer(): nh_("~"),
  image_sub(nh_,"/prosilica/image_rect_color", 1),
  info_sub(nh_, "/prosilica/camera_info", 1),
  sync (image_sub, info_sub, 1){
  //  this->timer_ = nh_.createTimer(ros::Duration(0.01),&PersonRecognizerProsilica::timerCallback,this);
  sync.registerCallback(boost::bind(&PersonRecognizer::ReceiveImage, this, _1, _2));
cout<<"Entering constructor"<<endl;

/*
PersonRecognizer::PersonRecognizer(): nh_("~"){
  //  this->timer_ = nh_.createTimer(ros::Duration(0.01),&PersonRecognizerProsilica::timerCallback,this);
cout<<"Entering constructer"<<endl;
*/
	using namespace std;

    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.8, 0.8 , 0, 2, 6 );
    
    //IMPORTANT PARAMETERS
    trainForJustOneImage=false;
//    trainHowManyImages=20; //if trainForJustOneImage==false, look to this value. # of images used for every person.
//    maxNumberOfFaceTrainImages=100; //images saved for every person to decompose to eigenvalues and determine mean/variance.
//    learningRate=0.0; //every time the person is detected, update the histogram with this rate.
//    scoringBiasTowardsFaceRecog=0.55; //Scoring=(face recognition Score)*scoringBiasTowardsFaceRecog+(shirt color score)*(1-scoringBiasTowardsFaceRecog)
//    THRESHOLD_FOR_RATIO_TEST=1.05; //between [1-inf]. to be recognized as a person, the best scorer must be [tresholdForRatioTest] times of the second best scorer. 0 to disable ratio test
//    THRESHOLD_FOR_PERSON_RECOGNITION=0.7; //between [0-1].to be recognized as a person, the max Scorer must surpass this score. for 0 every face, for 1 none is accepted. 0.6 ideal?

    limitRatioForFaceDetectionInTraining=0.55; //while training, limit the face detection to the region horozontally to the center. 0.5 means %50 of image around center line.
    faceToTorsoLengthFactor=1.6; //1.6 ideal. The square region beneath the square offered by face detection. Defines where torso is physically under the face.
    h_bins = 40; //how many bins are for Channel 1 of color space?
    s_bins = 40; //how many bins are for Channel 2 of color space?
    h_max=255.0; //HSV=180, RGB=255, NormRGB=255
    s_max=255.0; //HSV=255, RGB=255, NormRGB=255
    trainingImageSize=100; //the face images are saved as squares. This parameter defines one
	learnStateVirgin=true;
	enableVarianceNormalization=false;
    currentNumberOfFaceImagesSaved=0;
    eigenValMat=0;
    Person candidatePerson;
	faceImgSize.width=trainingImageSize;
	faceImgSize.height=trainingImageSize;
	pAvgTrainImg=cvCreateImage(faceImgSize,IPL_DEPTH_32F,1);
	learnedPersonsName="NOT_SET";
	everyoneStr="SEARCH_EVERYBODY";
	knownStr="SEARCH_ONLY_KNOWN";
	unknownStr="UNKNOWN";

	if(nh_.getParam("packageDir",packageDir))
	{
		packageDirectory = new char [packageDir.size()+1];
		strcpy (packageDirectory, packageDir.c_str());
	}
    nh_.param("recognitionThreshold", THRESHOLD_FOR_PERSON_RECOGNITION, 0.65);
    nh_.param("searchedPersonsName", searchedPersonsName, searchedPersonsName);
    nh_.param("trainHowManyImages",trainHowManyImages,20);
    nh_.param("scoringBiasTowardsFaceRecog",scoringBiasTowardsFaceRecog,0.5);
    nh_.param("learningRate",learningRate,0.0);
    nh_.param("THRESHOLD_FOR_RATIO_TEST",THRESHOLD_FOR_RATIO_TEST,1.05);
    nh_.param("maxNumberOfFaceTrainImages",maxNumberOfFaceTrainImages,100);
    nh_.param("isNameNeeded", isNameNeeded, isNameNeeded);
    nh_.param("learnedPersonsName", learnedPersonsName, learnedPersonsName);
    nh_.param("showImages", showImages, true);
    nh_.param("verbose", verbose, true);
    nh_.param("faceDetectionQualityWhenRecognition",faceDetectionQualityWhenRecognition,4);
    nh_.param("faceDetectionQualityWhenLearning",faceDetectionQualityWhenLearning,5);
    nh_.param("adaptiveShirtHist",adaptiveShirtHist,false);
    nh_.param("usePTU", usePTU, false);


	setSearchedPersonsName(searchedPersonsName);

	string cascadeFile;
	if(nh_.getParam("cascadeFile",cascadeFile))
	{
		char * charCascad = new char [cascadeFile.size()+1];
		strcpy (charCascad, cascadeFile.c_str());

	    cascade = (CvHaarClassifierCascade*)cvLoad( charCascad, 0, 0, 0 ); // Load the HaarClassifierCascade. on Akansel's computer.
	    //cascade = (CvHaarClassifierCascade*)cvLoad( "/home/akan/ros/pkgs/vision_opencv/opencv2/build/opencv-svn/data/haarcascades/haarcascade_frontalface_alt.xml", 0, 0, 0 );
	    //cascade = (CvHaarClassifierCascade*)cvLoad( "/home/akan/Research/OpenCV-2.0.0/data/haarcascades/haarcascade_profileface.xml", 0, 0, 0 );
		//cascade =(CvHaarClassifierCascade*)cvLoad("/home/mobmanip/ros/pkgs/vision_opencv/opencv2/build/opencv-svn/data/haarcascades/haarcascade_frontalface_default.xml",0,0,0);
	    if( !cascade ) // Check whether the cascade has loaded successfully. Else report and error and quit
	    {
	    	fprintf( stderr, "ERROR: Could not load classifier cascade\n" );
	        return;
	    }
	}

	if(usePTU)
	{
		PTUClient = nh_.serviceClient<ptu_46::PtuCmd>("/ptu_cmd");
		ros::service::waitForService("/ptu_cmd");
		ptu_46::PtuCmd cmd;
		cmd.request.pan_pos = 0.0;
		cmd.request.tilt_pos = 0.4;
		if (PTUClient.call(cmd))
		{

		}
		else
		{
			ROS_ERROR("PTU returned false in PERSON RECOGNITION module");
		}
	}

    int temp;
    nh_.param("currentState", temp, 2);
    setState((State)temp);

    loadPersonListFromDatabase();
    loadFaceImageArray();
    doPCA(); //should be training at the start of the code.

    cout<<"Cascade File: "<<cascadeFile<<endl;
    cout<<"Package Directory: "<<packageDir<<endl;
    cout<<"Recognition Threshold: "<<THRESHOLD_FOR_PERSON_RECOGNITION<<endl;
    cout<<"# of training images: "<<trainHowManyImages<<endl;
    cout<<"Scoring coefficient of face recog: "<<scoringBiasTowardsFaceRecog<<endl;
    cout<<"Updating Rate of shirt Histograms: "<<learningRate<<endl;
    cout<<"Ratio Test Threshold: "<<THRESHOLD_FOR_RATIO_TEST<<endl;
    cout<<"# of images per person to calculate eigenval means: "<<maxNumberOfFaceTrainImages<<endl;
    cout<<"Is Name Needed while training?: "<<isNameNeeded<<endl;
    cout<<"Next Learned Person's name: "<<learnedPersonsName<<endl;
    cout<<"Show Images?: "<<showImages<<endl;
    cout<<"Verbose?: "<<verbose<<endl;
    cout<<"Current State: "<<currentState<<endl;
    cout<<"faceDetectionQualityWhenRecognition: "<<faceDetectionQualityWhenRecognition<<endl;
    cout<<"faceDetectionQualityWhenLearning: "<<faceDetectionQualityWhenLearning<<endl;
    cout<<"adaptiveShirtHist: " <<adaptiveShirtHist<<endl;
    cout<<"usePTU: "<<usePTU<<endl;
	if(showImages)
	{
    cvNamedWindow( "result", 1 );
	}

	command_subscriber_=nh_.subscribe<std_msgs::String>("/korus_control_topic",5,boost::bind(&PersonRecognizer::commandCallback, this, _1));
    //image_subscriber_ = nh_.subscribe<sensor_msgs::Image>("/CVimage", 1, boost::bind(&PersonRecognizer::ImageCallback, this, _1));
    person_publisher_=nh_.advertise<person_recognition::PersonIdentifier>("/personIdTopic", 30);
    command_generator_=nh_.advertise<std_msgs::String>("/korus_control_topic", 5);
    person_control_subscriber_=nh_.subscribe<person_recognition::PersonRecControl>("/PersonRecControlTopic", 30,boost::bind(&PersonRecognizer::personRecControlCallback, this, _1) );

    ROS_INFO("Initialized Person Recognition node..");

}

void PersonRecognizer::commandCallback  (const std_msgs::String::ConstPtr& msg)
{

	cout<<"Got command "<<msg->data<<endl;
	char key0 = msg->data.c_str()[0];

	switch (key0)
	{

	case 'r': //enable recognizer

		if(msg->data.length()>2)
		{
			char key2 = msg->data.c_str()[2];
			switch (key2)
			{
			case 'i': //r_i: idle
				setState(IDLE);
				break;
			case 'l': //r_l_name: set learned Person's name to 'name', activate learning state.
				if(msg->data.length()>3)
				{
					string rcvdStr;
					rcvdStr=msg->data.substr(4);
					//cout<<"rcvdStr: "<<rcvdStr<<endl;
					learnedPersonsName=rcvdStr; //make sure that this works!
					//nh_.setParam("learnedPersonsName", rcvdStr);
				}
				setState(LEARN_PERSON);
				break;
			case 'r': //r_r: recognize
				setState(RECOGNIZE_PEOPLE_FROM_FACES);
				break;
			case 's': //r_s_name: save person called 'name' to database.
				if(msg->data.length()>3)
				{
					string rcvdStr;
					rcvdStr=msg->data.substr(4);
					//cout<<"rcvdStr: "<<rcvdStr<<endl;
					for(unsigned int j=0;j<personList.size();j++) //for every person in the list
					{
						if(personList[j].getPersonName().compare(rcvdStr) == 0) //the name is found.
						{
							savePersonToDatabase(personList[j]);
						}
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
		break;
	case 'q':
		//exit(0);
		break;
	default:
		break;
	}
}

void PersonRecognizer::personRecControlCallback(person_recognition::PersonRecControlConstPtr msg2)
{
	if((msg2->recognitionThresholdChange)==true)
	{
		THRESHOLD_FOR_PERSON_RECOGNITION=msg2->recognitionThreshold;
	}
	if((msg2->searchedPersonsNameChange)==true)
	{
		searchedPersonsName=msg2->searchedPersonsName;
	}
	if((msg2->currentStateChange)==true)
	{
		int temp;
		temp=msg2->currentState;
		if(abs(temp-0)<0.01)
		{
		setState(IDLE);
		}
		else if(abs(temp-1)<0.01)
		{
		setState(LEARN_PERSON);
		}		
		else if(abs(temp-3)<0.01)
		{
		setState(RECOGNIZE_PEOPLE_FROM_FACES);
		}
		else
		{
		
		}
				
		

	}
	if((msg2->isNameNeededChange)==true)
	{
		isNameNeeded=msg2->isNameNeeded;
	}
	if((msg2->learnedPersonsNameChange)==true)
	{
		learnedPersonsName=msg2->learnedPersonsName;
	}

}

void PersonRecognizer::timerCallback(const ros::TimerEvent& e)
{
  ROS_INFO("timerCallback triggered");
}


/*
void PersonRecognizer::ImageCallback(const sensor_msgs::ImageConstPtr& msg) 
{
*/
bool PersonRecognizer::ReceiveImage(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
	if(currentState==IDLE)
	{

	}
	else if(currentState==QUIT)
	{
		//exit(0);
	}
	else if(currentState==FACE_DETECTION_TEST)
	{
    ros::Time tick = ros::Time::now();
    sensor_msgs::CvBridge bridge;
    global_image = bridge.imgMsgToCv(msg);
    detect_and_draw(global_image);
	}
	if(currentState==CONTINIOUS_CAPTURE_MODE)
	{
	    ros::Time tick = ros::Time::now();
	    sensor_msgs::CvBridge bridge;
	    global_image = bridge.imgMsgToCv(msg);
//	    if(showImages)
//	    {
//	    	cvShowImage( "result", global_image );
//	    	cvWaitKey(2);
//	    }
	}
	else if(currentState==GRAB_NEXT_IMAGE)
	{
	    ros::Time tick = ros::Time::now();
	    sensor_msgs::CvBridge bridge;
	    global_image = bridge.imgMsgToCv(msg);

	    setState(IDLE);
	}
	else if(currentState==LEARN_PERSON)
	{
	    ros::Time tick = ros::Time::now();
	    sensor_msgs::CvBridge bridge;
	    global_image = bridge.imgMsgToCv(msg);

		if(learnStateVirgin==true)
		{
			learnStateVirgin=false;

			if(isNameNeeded==true)
			{
				currentNumberOfFaceImagesSaved=0;
				candidatePerson.setPersonName(learnedPersonsName);
				candidatePerson.shirtHistogramInitialized=false;
			}
			else
			{
				learnedPersonsName="MASTER";
				currentNumberOfFaceImagesSaved=0;
				candidatePerson.setPersonName(learnedPersonsName);
				candidatePerson.shirtHistogramInitialized=false;
			}
			deletePerson(learnedPersonsName);


			int n; //create necessary folders
			char folderName [250];
			char folderData [250];
			char * cstr;
			cstr = new char [candidatePerson.getPersonName().size()+1];
			strcpy (cstr, candidatePerson.getPersonName().c_str());
			n  = sprintf(folderData,"mkdir %s/data",packageDirectory);
			if(system(folderData))
			{ } //couldn't create folder '..data'
			n  = sprintf(folderName,"mkdir %s/data/%s",packageDirectory,cstr);
			if(system(folderName))
			{ printf("Could not create folder: %s\n",folderName);	}
		}

	    vector<CvRect> faceRectanglesVector;
	    findFaces(global_image, CV_HAAR_FIND_BIGGEST_OBJECT,faceDetectionQualityWhenLearning,faceRectanglesVector,70); //looking for the biggest face in the image.
	    //findFaces(global_image,  CV_HAAR_DO_CANNY_PRUNING,12,faceRectanglesVector,70); //looking for the biggest face in the image.

	    if(faceRectanglesVector.size()==1) //there is a face in the image
	    {
	    cvRectangle( global_image, cvPoint(faceRectanglesVector[0].x, faceRectanglesVector[0].y), cvPoint(faceRectanglesVector[0].x+faceRectanglesVector[0].width, faceRectanglesVector[0].y+faceRectanglesVector[0].height) , CV_RGB(255,255,255), 1, 4, 0 ); // Draw the face rectangle in the input image

	    int limitLeft=(global_image->width)*(1-limitRatioForFaceDetectionInTraining)/2;
	    int limitRight=(global_image->width)*(1+limitRatioForFaceDetectionInTraining)/2;

			if( (faceRectanglesVector[0].x+faceRectanglesVector[0].width/2)>limitLeft && (faceRectanglesVector[0].x+faceRectanglesVector[0].width/2)<limitRight ) //face is between the lines.
			{

				if(currentNumberOfFaceImagesSaved<maxNumberOfFaceTrainImages) //number of photos taken isn't satisfied yet.
				{

					IplImage* gray_face_img_resized = cvCreateImage( cvSize(trainingImageSize,trainingImageSize), IPL_DEPTH_8U, 1 ); // Create a new image based on the input image

						cvSetImageROI(global_image, faceRectanglesVector[0]); //for saving the data
						IplImage *img2 = cvCreateImage(cvGetSize(global_image),global_image->depth,global_image->nChannels);
						cvCopy(global_image, img2, NULL);
						cvResetImageROI(global_image);
						IplImage* gray_face_img = cvCreateImage( cvSize(img2->width,img2->height), IPL_DEPTH_8U, 1 ); // Create a new image based on the input image
						cvCvtColor(img2,gray_face_img,CV_RGB2GRAY);

						cvEqualizeHist(gray_face_img,gray_face_img);

						cvResize(gray_face_img,gray_face_img_resized,CV_INTER_CUBIC);

						char * cstr;
						cstr = new char [candidatePerson.getPersonName().size()+1];
						strcpy (cstr, candidatePerson.getPersonName().c_str());
						int n;
						char outFileName [250];
						n  = sprintf(outFileName,"%s/data/%s/%s%i.jpg",packageDirectory,cstr,cstr,currentNumberOfFaceImagesSaved);
						//n += sprintf(outFileName+ n,"%%%2.0f",maxScore*100);
						if(!cvSaveImage(outFileName,gray_face_img_resized)) printf("Could not save: %s\n",outFileName);
						cvReleaseImage(&img2);
						cvReleaseImage(&gray_face_img);
						cvReleaseImage(&gray_face_img_resized);
						currentNumberOfFaceImagesSaved++;

				}

				if(candidatePerson.shirtHistogramInitialized==false) //shirt Histogram not initialized yet.
				{
					 if( (int) (faceRectanglesVector[0].y+faceRectanglesVector[0].height*(faceToTorsoLengthFactor+1)) < global_image->height) // if the torso rectangle doesn't exceed limits
					 {
					 CvRect torsoRect= cvRect(faceRectanglesVector[0].x,(int) (faceRectanglesVector[0].y+faceRectanglesVector[0].height*faceToTorsoLengthFactor),faceRectanglesVector[0].width,faceRectanglesVector[0].height);
					 cvRectangle( global_image, cvPoint(torsoRect.x, torsoRect.y), cvPoint(torsoRect.x+torsoRect.width,torsoRect.y+torsoRect.height) , CV_RGB(100,170,70), 1, 4, 0 ); // Draw the rectangle in the input image

					 int hist_size[] = { h_bins, s_bins };
					 float h_ranges[] = { 0, h_max };
					 float s_ranges[] = { 0, s_max };
					 float* ranges[] = { h_ranges, s_ranges };
					 CvHistogram* hist = cvCreateHist( 2, hist_size, CV_HIST_ARRAY, ranges, 1 );

					 findHistogramOfRectangle(global_image,torsoRect,hist);

					 cvSetImageROI(global_image, torsoRect); //save the shirt image to the data folder.
					 IplImage *img2 = cvCreateImage(cvGetSize(global_image),global_image->depth,global_image->nChannels);
					 cvCopy(global_image, img2, NULL);
					 cvResetImageROI(global_image);
					 char * cstr;
					 cstr = new char [candidatePerson.getPersonName().size()+1];
					 strcpy (cstr, candidatePerson.getPersonName().c_str());
					 int n;
					 char outFileName [250];
					 n  = sprintf(outFileName,"%s/data/%s/%sShirt.jpg",packageDirectory,cstr,cstr);
					 if(!cvSaveImage(outFileName,img2)) printf("Could not save: %s\n",outFileName);
					 candidatePerson.initShirtHistogram();
					 candidatePerson.setShirtHistogram(hist);

				     cvReleaseImage( &img2);

					 }
					 else // if the torso rectangle doesn't exceed limits
					 {
						 cout<<"I NEED TO SEE YOUR SHIRT. PLEASE BACK UP A STEP.."<<endl;
					 }
				}


				if((candidatePerson.shirtHistogramInitialized==true) && (currentNumberOfFaceImagesSaved==maxNumberOfFaceTrainImages)) //learning completed.
				{
			     setState(RECOGNIZE_PEOPLE_FROM_FACES);

				 candidatePerson.createRandomCvScalar();
				 addPerson(candidatePerson); //add person to the list.
				 loadFaceImageArray();
				 doPCA(); //run PCA

				 std_msgs::String strr; //publish the completion flag
				 strr.data="g_r_completed";
				 command_generator_.publish(strr);

				 currentNumberOfFaceImagesSaved=0;
				 candidatePerson.shirtHistogramInitialized=false;
				 setSearchedPersonsName(candidatePerson.getPersonName());
				 learnStateVirgin=true;

				 cout<<"LEARNING COMPLETED.NICE TO MEET YOU "<<candidatePerson.getPersonName()<<endl;

				 printPersonList();

				 //ros::spinOnce();
				}
				cvLine(global_image, cvPoint(limitLeft,1), cvPoint(limitLeft,global_image->height-1), cvScalar(120,255,0), 2); //draw a green line to indicate the face is in the proper region
				cvLine(global_image, cvPoint(limitRight,1), cvPoint(limitRight,global_image->height-1), cvScalar(120,255,0), 2); //draw a green line to indicate the face is in the proper region
			}
			else
			{
				cvLine(global_image, cvPoint(limitLeft,1), cvPoint(limitLeft,global_image->height-1), cvScalar(0,30,255), 2); //draw a red line to indicate the face isn't in the proper region.
				cvLine(global_image, cvPoint(limitRight,1), cvPoint(limitRight,global_image->height-1), cvScalar(0,30,255), 2); //draw a red line to indicate the face isn't in the proper region.
				cout<<"PLEASE STAY IN FRONT OF ME!"<<endl;
			}

	    }
	    else //there are no faces in the image.
	    {
	    	cout<<"CAN'T DETECT ANY FACES. PLEASE LOOK AT THE CAMERA.."<<endl;
	    }
	    if(showImages)
	    {
	    	cvShowImage( "result", global_image );
	    	cvWaitKey(2);
	    }
	}
	else if(currentState==RECOGNIZE_PEOPLE_FROM_FACES)
	{
	    ros::Time tick = ros::Time::now();
	    sensor_msgs::CvBridge bridge;
	    global_image = bridge.imgMsgToCv(msg);

	    vector<CvRect> faceRectanglesVector;
	    findFaces(global_image, CV_HAAR_DO_CANNY_PRUNING, faceDetectionQualityWhenRecognition ,faceRectanglesVector,37); // should be CV_HAAR_DO_CANNY_PRUNING. for 1 face=CV_HAAR_FIND_BIGGEST_OBJECT

			 vector<unsigned int> maxScorerIDListOfRectangles(faceRectanglesVector.size());
			 vector<double> maxScoresOfRectangles(faceRectanglesVector.size());
			 vector<bool> shirtEligibilityOfRectangles(faceRectanglesVector.size());


		 for(unsigned int h=0;h<faceRectanglesVector.size();h++) //for every rectangle
		 {
			 cvRectangle( global_image, cvPoint(faceRectanglesVector[h].x, faceRectanglesVector[h].y), cvPoint(faceRectanglesVector[h].x+faceRectanglesVector[h].width, faceRectanglesVector[h].y+faceRectanglesVector[h].height) , CV_RGB(255,255,255), 1, 4, 0 ); // Draw the face rectangle in the input image



			 bool eligibleForShirtHistogram=false;
			 for(unsigned int j=0;j<personList.size();j++) //for every person in the list
			 {
				eligibleForShirtHistogram=eligibleForShirtHistogram || personList[j].shirtHistogramInitialized;
			 }
			 eligibleForShirtHistogram=eligibleForShirtHistogram && ((int) (faceRectanglesVector[h].y+faceRectanglesVector[h].height*(faceToTorsoLengthFactor+1)) < global_image->height); // if the torso rectangle doesn't exceed limits

			 if(eligibleForShirtHistogram)
			 {
			 shirtEligibilityOfRectangles[h]=true;

			 CvRect torsoRect= cvRect(faceRectanglesVector[h].x,(int) (faceRectanglesVector[h].y+faceRectanglesVector[h].height*faceToTorsoLengthFactor),faceRectanglesVector[h].width,faceRectanglesVector[h].height);
			 cvRectangle( global_image, cvPoint(torsoRect.x, torsoRect.y), cvPoint(torsoRect.x+torsoRect.width,torsoRect.y+torsoRect.height) , CV_RGB(225,225,242), 1, 4, 0 ); // Draw the rectangle in the input image

			 int hist_size[] = { h_bins, s_bins };
			 float h_ranges[] = { 0, h_max };
			 float s_ranges[] = { 0, s_max };
			 float* ranges[] = { h_ranges, s_ranges };
			 CvHistogram* testHist = cvCreateHist( 2, hist_size, CV_HIST_ARRAY, ranges, 1 );

			 findHistogramOfRectangle(global_image,torsoRect,testHist);


			 if(verbose)
			 {
			 cout<<"--shirt "<<h<<":--"<<endl;
			 }
						 vector<double> totalScores(personList.size());

						 for(unsigned int j=0;j<personList.size();j++) //for every person in the list
						 {
							if(personList[j].shirtHistogramInitialized==true)
							{
								double score=compareTwoHistograms(testHist,personList[j].getShirtHistogram()); //compare this person's histogram with testHist (detected Histogram)
								totalScores[j]=(1-scoringBiasTowardsFaceRecog)*score;


									//cout<<"Shirt : "<<personList[j].getPersonName()<<" "<<score<<endl;


							}
							else // if shirt is not initialized before, give 0 points.
							{
								totalScores[j]=0;
							}
						 }

						    //START FACE RECOGNITION

						        IplImage* gray_face_img_resized = cvCreateImage( cvSize(trainingImageSize,trainingImageSize), IPL_DEPTH_8U, 1 ); // Create a new image based on the input image

							    cvSetImageROI(global_image, faceRectanglesVector[h]);
							    IplImage *img2 = cvCreateImage(cvGetSize(global_image),global_image->depth,global_image->nChannels);
							    cvCopy(global_image, img2, NULL);
							    cvResetImageROI(global_image);
							    IplImage* gray_face_img = cvCreateImage( cvSize(img2->width,img2->height), IPL_DEPTH_8U, 1 ); // Create a new image based on the input image
						    	cvCvtColor(img2,gray_face_img,CV_RGB2GRAY);

						    	cvEqualizeHist(gray_face_img,gray_face_img);

						    	cvResize(gray_face_img,gray_face_img_resized,CV_INTER_CUBIC);
//							    if(showImages)
//							    {
//						    	cvShowImage("testFace",gray_face_img_resized);
//							    }

						    float* projectedTestFace=(float*)cvAlloc(nEigens*sizeof(float));
						    cvEigenDecomposite(gray_face_img_resized,nEigens,eigenVectArray,0,0,pAvgTrainImg,projectedTestFace);

						    cvReleaseImage( &gray_face_img_resized);
						    cvReleaseImage( &img2);
						    cvReleaseImage( &gray_face_img);

						    for(unsigned int j=0;j<personList.size();j++) //for every person in the list, find Mahalanobis distance
						    {
						    	double distance=findMahalanobisDistance(projectedTestFace,personList[j].getEigenValueAveragesArray(),personList[j].getEigenValueVarianceArray());

						    	if(personList[j].shirtHistogramInitialized==true)
						    	{
						    		totalScores[j]=totalScores[j]+scoringBiasTowardsFaceRecog*exp(-(distance/3.75)*(distance/3.75));

							    		//cout<<" Face %: "<<personList[j].getPersonName()<<": "<<exp(-(distance/3.75)*(distance/3.75))<<endl;

						    	}
						    	else
						    	{
						    		totalScores[j]=0;
						    	}


						    }

						    for(unsigned int j=0;j<personList.size();j++)
						    {
						    	//cout<<"  TOTAL: "<<personList[j].getPersonName()<<": "<<totalScores[j]<<endl;
						    }

						    double maxScore;
						    unsigned int maxScoreIndex;
						    double SecondMaxScore;
						    unsigned int SecondMaxScoreIndex;

						    maxScore=totalScores[0];
						    maxScoreIndex=0;


						    for(unsigned int j=0;j<personList.size();j++)
						    {
						    	if(totalScores[j]>maxScore) //find the max score for this particular person
						    	{
						    		maxScore=totalScores[j];
						    		maxScoreIndex=j;
						    	}
						    }
						    unsigned int guessIndex;
						    if((maxScoreIndex+1)<totalScores.size())
						    {
						    	guessIndex=maxScoreIndex+1;
						    }
						    else
						    {
						    	guessIndex=maxScoreIndex-1;
						    }
						    SecondMaxScore=totalScores[guessIndex];
						    SecondMaxScoreIndex=guessIndex;
						    for(unsigned int j=0;j<personList.size();j++)
						    {
						    	if(totalScores[j]>SecondMaxScore && j!=maxScoreIndex) //find the 2nd max score for this particular person
						    	{
						    		SecondMaxScore=totalScores[j];
						    		SecondMaxScoreIndex=j;
						    	}
						    }



						    if(verbose)
						    {
						    	cout<<"2ND= "<<SecondMaxScore<<".From: "<<personList[SecondMaxScoreIndex].getPersonName()<<endl;
						    }



			 //cout<<"RATIO: "<<(maxScore/SecondMaxScore)<<endl;
			 cout<<"BEST: "<<maxScore<<".From: "<<personList[maxScoreIndex].getPersonName()<<endl;



			 CvPoint pt1;
			 pt1.x = faceRectanglesVector[h].x;
			 pt1.y = faceRectanglesVector[h].y;

			 if ((maxScore>THRESHOLD_FOR_PERSON_RECOGNITION && ((maxScore/SecondMaxScore)>THRESHOLD_FOR_RATIO_TEST))) //this tested face rectange faceRectanglesVector[h] with torso rectange torsoRect matches the person personList[j]
			 {

				 maxScorerIDListOfRectangles[h]=maxScoreIndex; //place maxScoreIndex in the vector
				 maxScoresOfRectangles[h]=maxScore; //place maxScore in the vector.

				 if(personList[maxScoreIndex].shirtHistogramInitialized==true && adaptiveShirtHist) // UPDATE THE HISTOGRAM
				 {
					 personList[maxScoreIndex].updateShirtHistogram(testHist,learningRate);
				 }
				 char * cstr;
				 string str=personList[maxScoreIndex].getPersonName();
				 cstr = new char [str.size()+1];
				 strcpy (cstr, str.c_str());
				 int n;
				 char buffer [50];
				 n  = sprintf(buffer,"%s ",cstr);
				 n += sprintf(buffer+ n,"%%%2.0f",maxScore*100);

				 CvRect torsoRect= cvRect(faceRectanglesVector[h].x,(int) (faceRectanglesVector[h].y+faceRectanglesVector[h].height*faceToTorsoLengthFactor),faceRectanglesVector[h].width,faceRectanglesVector[h].height);
				 cvRectangle( global_image, cvPoint(torsoRect.x, torsoRect.y), cvPoint(torsoRect.x+torsoRect.width,torsoRect.y+torsoRect.height) , personList[maxScoreIndex].getPersonColor(), 1, 4, 0 ); // Draw the rectangle in the input image
				 cvRectangle( global_image, cvPoint(faceRectanglesVector[h].x, faceRectanglesVector[h].y), cvPoint(faceRectanglesVector[h].x+faceRectanglesVector[h].width, faceRectanglesVector[h].y+faceRectanglesVector[h].height) , personList[maxScoreIndex].getPersonColor(), 1, 4, 0 ); // Draw the face rectangle in the input image
				 cvPutText(global_image,buffer,pt1,&font, cvScalar(0,255,0));

			 }
			 else
			 {
				 maxScorerIDListOfRectangles[h]=999;//place 999 in the vector
				 maxScoresOfRectangles[h]=999.0;
				 cvPutText(global_image,"UNKNOWN",pt1,&font,cvScalar(255,255,255));
			 }

		 } // end of if(eligibleForShirtHistogram)
		 else
		 {
			 shirtEligibilityOfRectangles[h]=false;
		 }
	} //end of 'for' every rectangle.

	for(unsigned int h=0;h<maxScorerIDListOfRectangles.size();h++) //for every rectangle. now place the names & decide if no duplicates are made in the same frame.
	{

			if(shirtEligibilityOfRectangles[h])
			{

				 bool passedDuplicateTest=true;
				 if(maxScorerIDListOfRectangles[h]!=999)		 // if if the rectangle isn't tagged as unknown && clears for shirt detection
				 {
					 for(unsigned int k=0;k<maxScorerIDListOfRectangles.size();k++) //for every rectangle. now place the names & decide if no duplicates are made in the same frame.
					 {
						 if(k!=h) //if the controlled rectangle isn't the one just checked.
						 {
							 if(maxScorerIDListOfRectangles[h]==maxScorerIDListOfRectangles[k])
							 {
							 passedDuplicateTest=passedDuplicateTest&&false;
//								 if(maxScoresOfRectangles[h]>maxScoresOfRectangles[k])
//								 {
//									 setRecognitionThreshold(maxScoresOfRectangles[k]);
//									 if(verbose)
//									 {
//										 cout<<"THRESHOLD_FOR_PERSON_RECOGNITION set to: "<<getRecognitionThreshold()<<endl;
//									 }
//								 }
//								 else
//								 {
//									 setRecognitionThreshold(maxScoresOfRectangles[h]);
//									 if(verbose)
//									 {
//										 cout<<"THRESHOLD_FOR_PERSON_RECOGNITION set to: "<<getRecognitionThreshold()<<endl;
//									 }
//								 }
							 }
						 }
					 }
				 }

				 if(passedDuplicateTest)
				 {

					 if(maxScorerIDListOfRectangles[h]==999) //This ractangle is tagged as unknown.
					 {

						 if(everyoneStr.compare(searchedPersonsName) == 0)
						 {
							 person_recognition::PersonIdentifier msg;
							 sensor_msgs::RegionOfInterest reg;
							 CvRect rect=faceRectanglesVector[h];
							 msg.name="UNKNOWN";
							 msg.detectionConfidence=1.0;
							 reg.x_offset=rect.x;
							 reg.y_offset=rect.y;
							 reg.height=rect.height;
							 reg.width=rect.width;
							 msg.faceRegion=reg;
							 msg.camImgWidth=global_image->width;
							 msg.camImgHeight=global_image->height;
							 msg.positionDefined=false;
							 person_publisher_.publish(msg);
						 }

					 }
					 else //this rectangle is recognized as a known person.
					 {
						 if(everyoneStr.compare(searchedPersonsName) == 0 || knownStr.compare(searchedPersonsName) == 0)
						 {
							 person_recognition::PersonIdentifier msg;
							 sensor_msgs::RegionOfInterest reg;
							 CvRect rect=faceRectanglesVector[h];
							 msg.name=personList[maxScorerIDListOfRectangles[h]].getPersonName();
							 msg.detectionConfidence=maxScoresOfRectangles[h];
							 reg.x_offset=rect.x;
							 reg.y_offset=rect.y;
							 reg.height=rect.height;
							 reg.width=rect.width;
							 msg.faceRegion=reg;
							 msg.camImgWidth=global_image->width;
							 msg.camImgHeight=global_image->height;
							 msg.positionDefined=false;
							 person_publisher_.publish(msg);
						 }
						 else
						 {
							 if(personList[maxScorerIDListOfRectangles[h]].getPersonName().compare(searchedPersonsName) == 0) //the name is found. delete person from vector.
							 {
								 person_recognition::PersonIdentifier msg;
								 sensor_msgs::RegionOfInterest reg;
								 CvRect rect=faceRectanglesVector[h];
								 msg.name=personList[maxScorerIDListOfRectangles[h]].getPersonName();
								 msg.detectionConfidence=maxScoresOfRectangles[h];
								 reg.x_offset=rect.x;
								 reg.y_offset=rect.y;
								 reg.height=rect.height;
								 reg.width=rect.width;
								 msg.faceRegion=reg;
								 msg.camImgWidth=global_image->width;
								 msg.camImgHeight=global_image->height;
								 msg.positionDefined=false;
								 person_publisher_.publish(msg);

							 }

						 }
					 }
				 }
				 else //couldn't pass the duplicate test.
				 {
					 CvPoint pt1;
					 pt1.x = faceRectanglesVector[h].x;
					 pt1.y = faceRectanglesVector[h].y;
					 cvPutText(global_image,"UNKNOWN",pt1,&font,cvScalar(255,255,255));
				 }
			}

		 }
	if(showImages)
	{
		cvShowImage( "result", global_image );
		cvWaitKey(2);
	}
		return true;
	}
return false;

}

void PersonRecognizer::calculateNormalizedRGB(IplImage* srcImg,IplImage* dstImg)
{
	CvScalar scalar;
	int r;
	int g;
	int b;
	int total;
	for (int h = 0; h < srcImg->height; h++)
	{
		for (int w = 0; w < srcImg->width; w++)
		{
			scalar=cvGet2D(srcImg, h, w);
			r=scalar.val[0];
			g=scalar.val[1];
			b=scalar.val[2];
			total=r+g+b;
			//cout<<"r="<<r<<" g="<<g<<" b="<<b<<endl;

			if(total==0)
			{
				scalar.val[0]=(unsigned int)(255/3);
				scalar.val[1]=(unsigned int)(255/3);
				scalar.val[2]=(unsigned int)(255/3);
			}
			else
			{
				scalar.val[0]=(unsigned int)(255*r/total);
				scalar.val[1]=(unsigned int)(255*g/total);
				scalar.val[2]=(unsigned int)(255*b/total);
			}
			//cout<<"r="<<scalar.val[0]<<" g="<<scalar.val[1]<<" b="<<scalar.val[2]<<endl;
			cvSet2D(dstImg,h,w,scalar);
		}
	}
	//cvNormalize(srcImg,dstImg, 255.0, 0.0, CV_MINMAX);
}



double PersonRecognizer::compareTwoHistograms(CvHistogram* hist1,CvHistogram* hist2)
{
	CvMat* sig1;
	CvMat* sig2;
	int numrows=h_bins*s_bins;
	sig1=cvCreateMat(numrows,3,CV_32FC1);
	sig2=cvCreateMat(numrows,3,CV_32FC1);
	for (int h = 0; h < h_bins; h++)
	{
		for (int s = 0; s < s_bins; s++)
		{
			float bin_val=cvQueryHistValue_2D(hist1,h,s);
			cvSet2D(sig1,h*s_bins + s,0,cvScalar(bin_val));
			cvSet2D(sig1,h*s_bins + s,1,cvScalar(h));
			cvSet2D(sig1,h*s_bins + s,2,cvScalar(s));

			bin_val=cvQueryHistValue_2D(hist2,h,s);
			cvSet2D(sig2,h*s_bins + s,0,cvScalar(bin_val));
			cvSet2D(sig2,h*s_bins + s,1,cvScalar(h));
			cvSet2D(sig2,h*s_bins + s,2,cvScalar(s));
		}
	}

	float emd=cvCalcEMD2(sig1,sig2,CV_DIST_L2);
	//printf("EARTH MOOVER: %F /",emd);
	double score;
	double coeff=4.8;
	if(emd>coeff)
	{
		score=0.0;
	}
	else
	{
		score=1-(emd/coeff)*(emd/coeff);
	}
	return score;

	//return (1-cvCompareHist(hist1,hist2,CV_COMP_BHATTACHARYYA));
	//return (cvCompareHist(hist1,hist2,CV_COMP_INTERSECT));
	//return (cvCompareHist(hist1,hist2,CV_COMP_CORREL));

}

void PersonRecognizer::findHistogramOfRectangle(IplImage* img1, CvRect rectangle, CvHistogram* hist)
{
	    cvSetImageROI(img1, rectangle);
	    IplImage *img2 = cvCreateImage(cvGetSize(img1),img1->depth,img1->nChannels);
	    cvCopy(img1, img2, NULL);
	    cvResetImageROI(img1);

	    IplImage* hsv = cvCreateImage(cvGetSize(img2), IPL_DEPTH_8U, 3 );


	  	//cvCvtColor( img2, hsv, CV_RGB2HSV ); //HSV
	  	//cvCopy(img2, hsv, NULL); // RGB
	  	calculateNormalizedRGB(img2,hsv); //Normalized RGB

	  	IplImage* h_plane = cvCreateImage( cvGetSize( hsv ), 8, 1 );
	  	IplImage* s_plane = cvCreateImage( cvGetSize( hsv ), 8, 1 );
	  	IplImage* v_plane = cvCreateImage( cvGetSize( hsv ), 8, 1 );
	  	IplImage* planes[] = { h_plane, s_plane };
	  	cvCvtPixToPlane( hsv, h_plane, s_plane, v_plane, 0 );




/*
	  	CvScalar scalarV;
		CvSize imgSize = CvSize();
		imgSize = cvGetSize(h_plane);
		//cout<< "w: " <<h_plane->width<< "h: "<< h_plane->height<<endl;
		int heMax=0;
		for (int h = 0; h < imgSize.height; h++)
		{
			for (int w = 0; w < imgSize.width; w++)
			{
				scalarV=cvGet2D(h_plane, h, w);
				if(scalarV.val[0]>heMax)
				{
					heMax=scalarV.val[0];
				}
			}
		}
		cout<<"Max val CH1:"<<heMax<<endl;

		heMax=0;
		for (int h = 0; h < imgSize.height; h++)
		{
			for (int w = 0; w < imgSize.width; w++)
			{
				scalarV=cvGet2D(s_plane, h, w);
				if(scalarV.val[0]>heMax)
				{
					heMax=scalarV.val[0];
				}
			}
		}
		cout<<"Max val CH2:"<<heMax<<endl;

		heMax=0;
		for (int h = 0; h < imgSize.height; h++)
		{
			for (int w = 0; w < imgSize.width; w++)
			{
				scalarV=cvGet2D(v_plane, h, w);
				if(scalarV.val[0]>heMax)
				{
					heMax=scalarV.val[0];
				}
			}
		}
		cout<<"Max val CH3:"<<heMax<<endl;
*/

	  	cvCalcHist( planes, hist, 0, 0 ); // Compute histogram
		cvNormalizeHist( hist, 1 ); // Normalize it

		if(false) //visualize the histogram, H and S values.
		{
		int scale = 12;
		IplImage* hist_img = cvCreateImage( cvSize( h_bins * scale, s_bins * scale ), 8, 3 ); // Create an image to visualize the histogram
		cvZero ( hist_img );
		float max_value = 0;
		cvGetMinMaxHistValue( hist, 0, &max_value, 0, 0 );


		for( int h = 0; h < h_bins; h++ )
		{
			for( int s = 0; s < s_bins; s++ )
			{
				float bin_val = cvQueryHistValue_2D( hist, h, s );
				int intensity = cvRound( bin_val * 255 / max_value );
				cvRectangle( hist_img, cvPoint( h*scale, s*scale ),
							cvPoint( (h+1)*scale - 1, (s+1)*scale - 1 ),
							CV_RGB( intensity, intensity, intensity ),
							CV_FILLED );
			}
		}


		cvNamedWindow( "H-S Histogram", 1) ;// Show histogram equalized
		cvShowImage( "H-S Histogram", hist_img );
		//cvNamedWindow("H", 1) ;
		//cvShowImage("H", h_plane );
		//cvNamedWindow("S", 1) ;
		//cvShowImage("S", s_plane );
		cvWaitKey(2);
		}

		cvReleaseImage(&img2);
		cvReleaseImage(&hsv);
		cvReleaseImage(&h_plane);
		cvReleaseImage(&s_plane);
		cvReleaseImage(&v_plane);
}

void PersonRecognizer::printMat(CvMat *A)
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


IplImage* PersonRecognizer::grabNextImage()
{
	setState(GRAB_NEXT_IMAGE);
	while(currentState==GRAB_NEXT_IMAGE)
	{
		//wait till next image is grabbed.
	}
	return global_image;
}
IplImage* PersonRecognizer::grabLatestImage() //use only when the currentState is CONTINIOUS_CAPTURE_MODE
{
	return global_image;
}

void PersonRecognizer::setState(State newState)
{
	currentState=newState;
}

PersonRecognizer::State PersonRecognizer::getState()
{
	return currentState;
}

void PersonRecognizer::addPerson(Person newPerson)
{
	personList.push_back(newPerson);
}

void PersonRecognizer::deletePerson(string name)
{
	unsigned int indexDelete=999;
	 for(unsigned int h=0;h<personList.size();h++) //for every person in the list
	 {

		 if(personList[h].getPersonName().compare(name) == 0) //the name is found. delete person from vector.
		 {
			 indexDelete=h;
		 }
	 }
	 if(indexDelete != 999)
	 {
		 if(verbose)
		 {
			 cout<<"Deleting "<<personList[indexDelete].getPersonName()<<" from list.."<<endl;
		 }
	 personList.erase(personList.begin()+indexDelete);
	 }
}

vector<Person> PersonRecognizer::getPersonList()
{
	return personList;
}

void PersonRecognizer::printPersonList()
{
	 for(unsigned int h=0;h<personList.size();h++) //for every person in the list
	 {
			 cout<<"PERSON "<<h<<": "<<personList[h].getPersonName()<<endl;
	 }
	 cout<<"TOTAL: "<<personList.size()<<" people"<<endl;
}

void PersonRecognizer::setSearchedPersonsName(string name)
{
	searchedPersonsName=name;
}
string PersonRecognizer::getSearchedPersonsName()
{
	return searchedPersonsName;
}

void PersonRecognizer::findFaces(IplImage* img, int flags, int min_neighbors,vector<CvRect>& faceRectanglesVector,int maxSizeInPixels)
{
	faceRectanglesVector.clear();
    IplImage* gray_img = cvCreateImage( cvSize(img->width,img->height), IPL_DEPTH_8U, 1 );



    IplImage* gray_img_pre = cvCreateImage( cvSize(img->width,img->height), IPL_DEPTH_8U, 1 );
    cvCvtColor(img,gray_img_pre,CV_RGB2GRAY);
    cvEqualizeHist(gray_img_pre,gray_img);

    /*cvCvtColor(img,gray_img,CV_RGB2GRAY);*/



    CvMemStorage* storage = cvCreateMemStorage(0); // Create memory for calculations
    cvClearMemStorage( storage ); // Clear the memory storage which was used before

    if(!cascade ) // Find whether the cascade is loaded, to find the faces. If yes, then:
    {
    cout<<"Cascade not loaded properly for face detection"<<endl;
    }

        CvSeq* faces = cvHaarDetectObjects( gray_img, cascade, storage,1.2, min_neighbors, flags,cvSize(maxSizeInPixels, maxSizeInPixels)); //FOR TRAINING=70 pixels=detect max 2m, recog max 45 pixels, approx 5m.

        for( int i = 0; i < (faces ? faces->total : 0); i++ )
        {
            CvRect* r = (CvRect*)cvGetSeqElem( faces, i );// Create a new rectangle for drawing the face
            faceRectanglesVector.push_back(*r);
        }
        cvReleaseMemStorage(&storage);
        cvReleaseImage( &gray_img);
        cvReleaseImage( &gray_img_pre);
}



// Function to detect and draw any faces that is present in an image
void PersonRecognizer::detect_and_draw( IplImage* img )
{
    int scale = 1;
    
    IplImage* gray_img = cvCreateImage( cvSize(img->width/scale,img->height/scale), 8, 1 ); // Create a new image based on the input image
    cvCvtColor(img,gray_img,CV_RGB2GRAY);
    CvPoint pt1, pt2;// Create two points to represent the face locations
    int i;
    // Create a new named window with title: result
    CvMemStorage* storage = cvCreateMemStorage(0); // Create memory for calculations
    cvClearMemStorage( storage ); // Clear the memory storage which was used before

    if( cascade ) // Find whether the cascade is loaded, to find the faces. If yes, then:
    {
        //CvSeq* faces = cvHaarDetectObjects( img, cascade, storage,1.1, 4, CV_HAAR_DO_CANNY_PRUNING,cvSize(40, 40));
        CvSeq* faces = cvHaarDetectObjects( gray_img, cascade, storage,1.1, 5, CV_HAAR_FIND_BIGGEST_OBJECT,cvSize(70, 70)); //FOR TRAINING 70 pixels=detect max 2m.

        for( i = 0; i < (faces ? faces->total : 0); i++ )
        {
            CvRect* r = (CvRect*)cvGetSeqElem( faces, i );// Create a new rectangle for drawing the face

            pt1.x = r->x*scale;// Find the dimensions of the face,and scale it if necessary
            pt2.x = (r->x+r->width)*scale;
            pt1.y = r->y*scale;
            pt2.y = (r->y+r->height)*scale;

            cvRectangle( img, pt1, pt2, CV_RGB(50,200,50), 3, 8, 0 );// Draw the rectangle in the input image
            cvPutText(img,"PERSON",pt1,&font,cvScalar(50,200,50));

        }
    }

    cvShowImage( "result", img );
    cvReleaseMemStorage(&storage);
    cvReleaseImage( &gray_img);
    cvWaitKey(2);
}

void PersonRecognizer::loadFaceImageArray()
{




			 if(trainForJustOneImage==true)
			 {
				 faceImgArray=(IplImage **)cvAlloc(personList.size()*trainingImageSize*trainingImageSize);
				 for(unsigned int j=0;j<personList.size();j++) //for every person in the list
				 {

					 if(personList[j].loadedFromDatabase==true)
					 {
							char * cstr;
							string str=personList[j].getPersonName();
							cstr = new char [str.size()+1];
							strcpy (cstr, str.c_str());
							int n;
							char outFileName [250];
							n  = sprintf(outFileName,"%s/faces/%s/%s0.jpg",packageDirectory,cstr,cstr);

							char frameName [30];
							n  = sprintf(frameName,"%s",cstr);

							faceImgArray[j]=cvLoadImage(outFileName,CV_LOAD_IMAGE_GRAYSCALE);
//						    if(showImages)
//						    {
//							cvShowImage(frameName,faceImgArray[j]);
//						    }
					 }
					 else
					 {
							char * cstr;
							string str=personList[j].getPersonName();
							cstr = new char [str.size()+1];
							strcpy (cstr, str.c_str());
							int n;
							char outFileName [250];
							n  = sprintf(outFileName,"%s/data/%s/%s0.jpg",packageDirectory,cstr,cstr);

							char frameName [30];
							n  = sprintf(frameName,"%s",cstr);

							faceImgArray[j]=cvLoadImage(outFileName,CV_LOAD_IMAGE_GRAYSCALE);
//						    if(showImages)
//						    {
//							cvShowImage(frameName,faceImgArray[j]);
//						    }
					 }
				 }
			 }
			 else if(trainForJustOneImage==false)
			 {

				 unsigned int counter=0;
				 faceImgArray=(IplImage **)cvAlloc(personList.size()*trainHowManyImages*trainingImageSize*trainingImageSize);

				 srand(time(0));
				 int randomInts[trainHowManyImages];
				 for(int ii=0;ii<trainHowManyImages;ii++) //initialize array
				 {
					 randomInts[ii]=0;
				 }
				 unsigned int numberOfCompletedRandNums=0;


				 while(numberOfCompletedRandNums!=(unsigned int)trainHowManyImages)
				 {
					 int randomTemp=rand()%maxNumberOfFaceTrainImages;
					 cvWaitKey(2);
					 bool passed=true;
					 for(unsigned int ii=0;ii<numberOfCompletedRandNums;ii++)
					 {
						 if(randomInts[ii]==randomTemp)
						 {
							 passed=passed&&false;
						 }
					 }
					 if(passed==true)
					 {
						 randomInts[numberOfCompletedRandNums]=randomTemp;
						 numberOfCompletedRandNums++;
					 }

				 }


				 for(unsigned int j=0;j<personList.size();j++) //for every person in the list
				 {
					 for(int ii=0;ii<trainHowManyImages;ii++)
					 {
						 if(personList[j].loadedFromDatabase==true)
						 {
							 char * cstr;
							 string str=personList[j].getPersonName();
							 cstr = new char [str.size()+1];
							 strcpy (cstr, str.c_str());
							 int n;
							 char outFileName [250];

							 n  = sprintf(outFileName,"%s/faces/%s/%s%i.jpg",packageDirectory,cstr,cstr,randomInts[ii]);
							 //cout<<outFileName<<endl;
							 char frameName [30];
							 n  = sprintf(frameName,"%s",cstr);

							 faceImgArray[counter]=cvLoadImage(outFileName,CV_LOAD_IMAGE_GRAYSCALE);
//							 if(showImages)
//							 {
//								 cvShowImage(frameName,faceImgArray[counter]);
//							 }
						 }
						 else
						 {
							 char * cstr;
							 string str=personList[j].getPersonName();
							 cstr = new char [str.size()+1];
							 strcpy (cstr, str.c_str());
							 int n;
							 char outFileName [250];
							 n  = sprintf(outFileName,"%s/data/%s/%s%i.jpg",packageDirectory,cstr,cstr,randomInts[ii]);
							 //cout<<outFileName<<endl;
							 char frameName [30];
							 n  = sprintf(frameName,"%s",cstr);

							 faceImgArray[counter]=cvLoadImage(outFileName,CV_LOAD_IMAGE_GRAYSCALE);
//							 if(showImages)
//							 {
//								 cvShowImage(frameName,faceImgArray[counter]);
//							 }
						 }
						 counter++;
					 }

				 }
			 }


			 if(verbose)
			 {
				 cout<<"out loadFaceImageArray"<<endl;
			 }
}
void PersonRecognizer::loadPersonListFromDatabase()
{
	int n;
	char personDatFile [250];
	n  = sprintf(personDatFile,"%s/faces/personDatabase.dat",packageDirectory);

	ifstream inClientFile(personDatFile, ios::in);
	if(!inClientFile)
	{
		cerr<<"personDatabase.dat can't be opened.."<<endl;
		exit(1);
	}
	char name[50];

	while(inClientFile>>name)
	{
		Person insan;
		insan.setPersonName(name);

		int hist_size[] = { h_bins, s_bins };
		float h_ranges[] = { 0, h_max };
		float s_ranges[] = { 0, s_max };
		float* ranges[] = { h_ranges, s_ranges };
		CvHistogram* hist = cvCreateHist( 2, hist_size, CV_HIST_ARRAY, ranges, 1 );

		char * cstr; //load the .dat
		string str=insan.getPersonName();
		cstr = new char [str.size()+1];
		strcpy (cstr, str.c_str());
		int n;
		char histDatFileName [250];
		n  = sprintf(histDatFileName,"%s/faces/%s/%sHistogram.dat",packageDirectory,cstr,cstr); //create folder in faces.
		ifstream datClientFile(histDatFileName, ios::in);
		char fl[33];

		if(!datClientFile)
		{
			cout<<"xHistogram.dat can't be opened.."<<endl;
			exit(1);
		}
		else
		{
			for (int h = 0; h < h_bins; h++)
			{
				for (int w = 0; w < s_bins; w++)
				{
					datClientFile>>fl;
					float* dstValPtr=cvGetHistValue_2D(hist,h,w);
//					float dstVal=*dstValPtr;
					*dstValPtr=atof(fl);
				}
			}
		}

		insan.loadedFromDatabase=true;
		insan.initShirtHistogram();
		insan.setShirtHistogram(hist);
		addPerson(insan);
	}
	printPersonList();


}
void PersonRecognizer::savePersonToDatabase(Person newPerson)
{
	char * cstr;
	string str=newPerson.getPersonName();
	cstr = new char [str.size()+1];
	strcpy (cstr, str.c_str());
	int n;

	if(newPerson.loadedFromDatabase==false) //the person is newly learnt
	{
		char origFileName [250];
		char folderName [250];	//create folder in faces
		n  = sprintf(folderName,"mkdir %s/faces/%s",packageDirectory,cstr); //create folder in faces.
		if(system(folderName))
		{ }

		n  = sprintf(origFileName,"%s/data/%s/",packageDirectory,cstr);
		char cmdBuff[450];
		sprintf(cmdBuff,"cp -r %s %s/faces/",origFileName,packageDirectory); //copy pictures into the folder
		if(system(cmdBuff))
		{ printf("Could not copy folder");	}


		bool unique=true; //now manipulate .dat file



		char personDatFile [250];
		n  = sprintf(personDatFile,"%s/faces/personDatabase.dat",packageDirectory);

		ifstream inClientFile(personDatFile, ios::in); //check if the name is in the list
		if(!inClientFile)
		{
			cerr<<"File can't be opened.."<<endl;
			exit(1);
		}
		char name[50];
		while(inClientFile>>name)
		{
			if(str.compare(name) == 0)
			{
				unique=unique&&false;
			}
		}
		if(unique) //if name is not in the .dat file, add it.
		{
			ofstream myfile;

			char personDatFile [250];
			n  = sprintf(personDatFile,"%s/faces/personDatabase.dat",packageDirectory);

			myfile.open (personDatFile,ios::app);
			if(!myfile)
			{
				cerr<<"File can't be opened.."<<endl;
				exit(1);
			}
			else
			{
				myfile<<str<<"\n";
			}
		}

		//create .dat file for histogram
		char histDatFileName [200];
		n  = sprintf(histDatFileName,"%s/faces/%s/%sHistogram.dat",packageDirectory,cstr,cstr);
//		char cmdBuff2[120];
//		n  = sprintf(cmdBuff2,"touch %s",histDatFileName);
//		if(system(cmdBuff2))
//		{ }

		ofstream myfile2;
		myfile2.open (histDatFileName,ios::out);
		if(!myfile2)
		{
			cerr<<".dat File can't be opened for write operation.."<<endl;
			exit(1);
		}

		//Now save the latest histogram
		CvHistogram* hist=newPerson.getShirtHistogram();
		for (int h = 0; h < h_bins; h++)
		{
			for (int w = 0; w < s_bins; w++)
			{
				float bin_val = cvQueryHistValue_2D( hist, h, w );
				myfile2<<bin_val<<"\n";
			}
		}
		cout<<"Person "<<str<<" successfully saved to database.."<<endl;
	}
	else
	{
		cout<<"Person is in database. Updating the shirt histogram.."<<endl; //TODO
		char histDatFileName [200];
		n  = sprintf(histDatFileName,"%s/faces/%s/%sHistogram.dat",packageDirectory,cstr,cstr);
		ofstream myfile2;
		myfile2.open (histDatFileName,ios::out);
		if(!myfile2)
		{
			cerr<<".dat File can't be opened for write operation.."<<endl;
			exit(1);
		}

		//Now save the latest histogram
		CvHistogram* hist=newPerson.getShirtHistogram();
		for (int h = 0; h < h_bins; h++)
		{
			for (int w = 0; w < s_bins; w++)
			{
				float bin_val = cvQueryHistValue_2D( hist, h, w );
				myfile2<<bin_val<<"\n";
			}
		}
		cout<<"Person "<<str<<"successfully saved to database.."<<endl;
	}
}

void PersonRecognizer::doPCA()
{
	if(verbose)
	{
		cout<<"in doPCA"<<endl;
	}
	if(trainForJustOneImage)
	{
		nEigens=personList.size()-1;
	}
	else
	{
		nEigens=personList.size()*trainHowManyImages-1;
	}

	CvTermCriteria calcLimit;
	IplImage** eigenVectArrayTemper=(IplImage**)cvAlloc(nEigens*trainingImageSize*trainingImageSize);

	for(int i=0;i<nEigens;i++)
	{
		eigenVectArrayTemper[i]=cvCreateImage(faceImgSize,IPL_DEPTH_32F,1);
	}
	eigenValMat=cvCreateMat(1,nEigens,CV_32FC1);
	calcLimit=cvTermCriteria(CV_TERMCRIT_ITER,nEigens,1);


	 if(trainForJustOneImage==true)
	 {
	 cvCalcEigenObjects(personList.size(),(void*)faceImgArray,(void*)eigenVectArrayTemper,CV_EIGOBJ_NO_CALLBACK,0,0,&calcLimit,pAvgTrainImg,eigenValMat->data.fl);
	 }
	 else
	 {
	 cvCalcEigenObjects(personList.size()*trainHowManyImages,(void*)faceImgArray,(void*)eigenVectArrayTemper,CV_EIGOBJ_NO_CALLBACK,0,0,&calcLimit,pAvgTrainImg,eigenValMat->data.fl);
	 }
	//printMat(eigenValMat);
	eigenVectArray=eigenVectArrayTemper;
	//cvShowImage("avg Image",pAvgTrainImg);


	for(unsigned int j=0;j<personList.size();j++) //for every person in the list
	//for(unsigned int j=0;j<1;j++) //for 1st person
	{
		cout<<"Loading: "<<personList[j].getPersonName()<<endl;

		char * cstr;
		string str=personList[j].getPersonName();
		cstr = new char [str.size()+1];
		strcpy (cstr, str.c_str());


		vector<double> eigenValueAveragesArray(nEigens);
		vector<double> eigenValueVarianceArray(nEigens);
		for(int ii=0;ii<maxNumberOfFaceTrainImages;ii++)  //calculate the mean.
		{
			int n;
			char outFileName [200];
			if(personList[j].loadedFromDatabase==true)
			{
				n  = sprintf(outFileName,"%s/faces/%s/%s%i.jpg",packageDirectory,cstr,cstr,ii);
			}
			else
			{
				n  = sprintf(outFileName,"%s/data/%s/%s%i.jpg",packageDirectory,cstr,cstr,ii);
			}
			IplImage* currentFacePhoto=cvLoadImage(outFileName,CV_LOAD_IMAGE_GRAYSCALE);
			float* currentEigenvalues=(float*)cvAlloc(nEigens*sizeof(float));
			cvEigenDecomposite(currentFacePhoto,nEigens,eigenVectArray,0,0,pAvgTrainImg,currentEigenvalues);
			cvReleaseImage(&currentFacePhoto);

			for(int jj=0;jj<nEigens;jj++)
			{

//				if(jj==2)
//				{
//					cout<<"ii "<<ii<<endl;
//					cout<<"jj "<<jj<<endl;
//					cout<<"currentEigenvalues["<<jj<<"]: "<<currentEigenvalues[jj]<<endl;
//				}

				//cout<<currentEigenvalues[jj]<<" ";



				eigenValueAveragesArray[jj]=static_cast<double>(ii)/(ii+1)*eigenValueAveragesArray[jj]+static_cast<double>(1)/(ii+1)*currentEigenvalues[jj];

				if(ii==0)
				{
				eigenValueVarianceArray[jj]=0;
				}
				else
				{
				eigenValueVarianceArray[jj]=static_cast<double>(ii)/(ii+1)*eigenValueVarianceArray[jj]+pow((currentEigenvalues[jj]-eigenValueAveragesArray[jj]),2)/(ii);
				}
				/*cout<<"eigenValueAveragesArray["<<jj<<"]: "<<eigenValueAveragesArray[jj]<<endl;
				cout<<"eigenValueVarianceArray["<<jj<<"]: "<<eigenValueVarianceArray[jj]<<endl;*/
			}
			//cout<<";";
		}

//		if(verbose)
//		{
//			cout<<"PERSON: "<<personList[j].getPersonName()<<endl;
//			for(int jj=0;jj<nEigens;jj++)
//			{
//				cout<<"eigenValueAveragesArray[jj]: "<<eigenValueAveragesArray[jj]<<endl;
//				cout<<"eigenValueVarianceArray[jj]: "<<eigenValueVarianceArray[jj]<<endl;
//			}
//		}


		personList[j].setEigenValueAveragesArray(eigenValueAveragesArray); //assign mean&var to the person.
		personList[j].setEigenValueVarianceArray(eigenValueVarianceArray); //assign mean&var to the person.

	}

	if(enableVarianceNormalization) //modify the assigned variances
	{
		vector<double> varianceTotalsOfPeople(personList.size());
		vector<double> varianceCoeffsOfPeople(personList.size());
		double varianceAvgOfAll=0.0;

		for(unsigned int j=0;j<personList.size();j++) //for every person in the list
		{
			vector<double> eigValVarArrTemp(nEigens);
			eigValVarArrTemp=personList[j].getEigenValueVarianceArray();

			for(int k=0;k<nEigens;k++)
			{
				varianceTotalsOfPeople[j]=varianceTotalsOfPeople[j]+eigValVarArrTemp[k];
				varianceAvgOfAll=varianceAvgOfAll+eigValVarArrTemp[k];
			}
		}
		varianceAvgOfAll=varianceAvgOfAll/nEigens/personList.size();
		cout<<"varianceAvgOfAll: "<<varianceAvgOfAll<<endl;

		for(unsigned int j=0;j<personList.size();j++) //for every person in the list
		{
			varianceCoeffsOfPeople[j]=varianceAvgOfAll/varianceTotalsOfPeople[j];
			vector<double> eigValVarArrTemp(nEigens);
			eigValVarArrTemp=personList[j].getEigenValueVarianceArray();
			for(int k=0;k<nEigens;k++)
			{
				eigValVarArrTemp[k]=eigValVarArrTemp[k]*varianceCoeffsOfPeople[j];
			}
			personList[j].setEigenValueVarianceArray(eigValVarArrTemp); //assign mean&var to the person.

//
//			if(verbose)
//			{
//				cout<<"PERSON: "<<personList[j].getPersonName()<<endl;
//				for(int jj=0;jj<nEigens;jj++)
//				{
//					cout<<"eigValVarArrTemp[jj]: "<<eigValVarArrTemp[jj]<<endl;
//				}
//			}

		}


	}



	cout<<"out doPCA"<<endl;
}


double PersonRecognizer::findMahalanobisDistance(float* projectedTestFace,vector<double> meanArr,vector<double> varArray)
{
	double distance=0.0;

	for(int i=0;i<nEigens;i++)
	{
		double inputVal=static_cast<double>(projectedTestFace[i]);
		//cout<<inputVal<<" ";

		//cout<<"eigenValue["<<i<<"]: "<<projectedTestFace[i]<<endl;
		//cout<<"inputVal: "<<inputVal<<endl;
		//cout<<"meanArr["<<i<<"]: "<<meanArr[i]<<endl;
		//cout<<"varArray["<<i<<"]: "<<varArray[i]<<endl;



		distance=distance+pow((inputVal-meanArr[i]),2)/varArray[i];
		//distance2=distance2+pow((inputVal-meanArr[i]),2);
		//distance=distance+abs(inputVal-meanArr[i])/varArray[i];

	}

	distance=sqrt(distance/nEigens);
//	if(verbose)
//	{
//		cout<<" dist2: "<<distance2;
//	}
	return distance;
}

void PersonRecognizer::increaseRecognitionThreshold(double addedValue)
{
	THRESHOLD_FOR_PERSON_RECOGNITION=THRESHOLD_FOR_PERSON_RECOGNITION+addedValue;
	if(THRESHOLD_FOR_PERSON_RECOGNITION>0.9)
	{
		THRESHOLD_FOR_PERSON_RECOGNITION=0.9;
	}
	else if(THRESHOLD_FOR_PERSON_RECOGNITION<0.1)
	{
		THRESHOLD_FOR_PERSON_RECOGNITION=0.1;
	}
}
double PersonRecognizer::getRecognitionThreshold()
{
	return THRESHOLD_FOR_PERSON_RECOGNITION;
}
void PersonRecognizer::setRecognitionThreshold(double setValue)
{
	THRESHOLD_FOR_PERSON_RECOGNITION=setValue;
	if(THRESHOLD_FOR_PERSON_RECOGNITION>0.95)
	{
		THRESHOLD_FOR_PERSON_RECOGNITION=0.95;
	}
	else if(THRESHOLD_FOR_PERSON_RECOGNITION<0.05)
	{
		THRESHOLD_FOR_PERSON_RECOGNITION=0.05;
	}
}

