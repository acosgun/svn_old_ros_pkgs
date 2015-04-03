#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <std_msgs/String.h>
#include <string>
using std::string;


#include "personFollowerProject.h" //new
#include <opencv/cxcore.h> //new
#include <opencv/cv.h> //new
#include <opencv/cvwimage.h> //new
#include <opencv/highgui.h> //new





#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <geometry_msgs/Twist.h>

#include <geometry_msgs/PointStamped.h> //tf
#include <tf/transform_listener.h> //tf



const int plotParam1=301; //400
const int plotParam2=75; //133
const double operatingFrequency=9.37; //4.64 or 9.08

const float rectangularBoxXover2sizeForMovDetection=0.6;
const float rectangularBoxYsizeForMovDetection=1.2;

	//const float floatFieldOfView=CV_PI;
	int intNumOfSegments=0;

	float deltaT=1/operatingFrequency;
	float qUncertainty=0.01;
	float rUncertainty=0.01;
	float valsF[]={1,0,deltaT,0,		0,1,0,deltaT,	0,0,1,0		,	0,0,0,1};

float maxDistanceToMatchPrediction=0.4;
const int minPixelsToCountAllowedForMovementDetection=1; //for 5 Hz,this is 2. for 10 hz, set to 1.
const float floatHitRatioTresholdForMovementDetection=0.10;
const float floatMaxRadius=4;
const int intDataPoints=181; //361
const float distanceTreshold=0.1;
const unsigned int numberOfPointsTreshold=3;
const bool boolFilterSegmentsAccordingToLengthInSegmentation=true;
const float maxEuclidianDistOfSegmentsThatCanBePersonCandids=0.5; //0.6 before
const float minEuclidianDistOfSegmentsThatCanBePersonCandids=0.01; //0.1 before
const float floatGridLength=0.05;
const int fillingForImages=255; //255 for visualization, 1 for non-visual
CvScalar cvScalarTemp;
geometry_msgs::Twist cmd;


bool virgin=true;
unsigned int personStopCounter=0;
const double secondsToDeclarePersonStopped=2;

const bool plotLaser=true;
const bool inhibitLinearMovement=true;
const bool inhibitAngularMovement=true;
bool inhibitNavigationMovement=false;

const bool enableComeNearbyBehavior=false;

bool checkForPersonToStop=true;
const double extraVelocityBeCareful=0.1;
double distanceToKeepWithPerson=1.25;
double tetaNearby=CV_PI/2;
unsigned int velocityCounter=0;

	vector<double> doubleLaserDataXprevious(intDataPoints);
	vector<double> doubleLaserDataYprevious(intDataPoints);
	vector<double> doubleLaserDataX(intDataPoints);
	vector<double> doubleLaserDataY(intDataPoints);
double originPtt[2]={0,0};


float thres1,thres2,thres3,thres4;
IplImage *outImg;

using namespace std;
typedef vector <int> Segment;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

enum Status{NO_TARGET_TO_TRACK_MODE,ONE_LEG_TRACK_MODE,TWO_LEGS_TRACK_MODE,IDLE};
Status currentStatus=IDLE;
enum motionStatus{REGULAR_FOLLOW,GO_NEARBY_PERSON};
motionStatus currentMotionStatus=REGULAR_FOLLOW; 

void transformPt(const tf::TransformListener& listener, double inX, double inY, double &outX, double &outY, bool verbose)
{
  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "laser";

  //we'll just use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  //laser_point.point.x = inX;
  //laser_point.point.y = inY;

  laser_point.point.x = inY; //this is reversed; to match the coordinate frame "laser" to my convention.
  laser_point.point.y = inX;

  laser_point.point.z = 0.0;

  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("odom", laser_point, base_point);
			outX=base_point.point.x;
			outY=base_point.point.y;
		if(verbose==true)
		{
		    ROS_INFO("laser: (%.2f, %.2f. %.2f) -----> odom: (%.2f, %.2f, %.2f) at time %.2f",
			laser_point.point.x, laser_point.point.y, laser_point.point.z,
			base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
		}
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception :%s", ex.what());
  }

} 

void transformPtFromLaserToCam(const tf::TransformListener& listener, double inX, double inY, double &outX, double &outY, bool verbose)
{
  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "laser";

  //we'll just use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  //laser_point.point.x = inX;
  //laser_point.point.y = inY;

  laser_point.point.x = inY; //this is reversed; to match the coordinate frame "laser" to my convention.
  laser_point.point.y = inX;

  laser_point.point.z = 0.0;

  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("camera", laser_point, base_point);
			outX=base_point.point.x;
			outY=base_point.point.y;
		if(verbose==true)
		{
		    ROS_INFO("laser: (%.2f, %.2f. %.2f) -----> camera: (%.2f, %.2f, %.2f) at time %.2f",
			laser_point.point.x, laser_point.point.y, laser_point.point.z,
			base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
		}
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception :%s", ex.what());
  }

} 





void scanCallback  (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{	 
	ros::NodeHandle n;
	ros::Publisher commander_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 3);
	ros::Rate pub_rate(20);

	geometry_msgs::PointStamped laser_point; //tf
	laser_point.header.frame_id = "laser"; //tf
	static tf::TransformListener listener(ros::Duration(10)); //tf
	
  static MoveBaseClient ac("move_base", true);
  





  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  float currentRangeValue;
  sensor_msgs::PointCloud cloud;
  projector_.projectLaser(*scan_in, cloud,floatMaxRadius);

//ros::Publisher commander_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	static CvSize imgSize = CvSize();
	imgSize.height=(int)( (floatMaxRadius/floatGridLength)+1 +0.5); //static?
	imgSize.width=(int)( (2*floatMaxRadius/floatGridLength)+1 +0.5); //static?

	static IplConvKernel* meineKernel= cvCreateStructuringElementEx(2,1,0,0,CV_SHAPE_RECT);
	static IplImage* imgGridMap = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	static IplImage* imgGridMapPrevious = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	static IplImage* imgGridMapIncludingSegmentNumbers = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	static IplImage* imgGridMapPreviousNegated = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	static IplImage* cleanedGridMapIncludingSegmentNumbers = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	static IplImage* DifferenceMap = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	static IplImage* cleanedGridMap = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
//	cout<<"w: "<<imgSize.width<<"h: "<<imgSize.height<<endl;
	static CvKalman* kalman=cvCreateKalman(4,2,0);
	//cvNamedWindow("cleanedGridMap",CV_WINDOW_AUTOSIZE);
	//cvNamedWindow( "imgGridMap", CV_WINDOW_AUTOSIZE );
	//cvNamedWindow( "imgGridMapPrevious", CV_WINDOW_AUTOSIZE );
	//cvNamedWindow( "DifferenceMap", CV_WINDOW_AUTOSIZE );


//if(virgin==true && NO_TARGET_TO_TRACK_MODE)
if(virgin==true)
{
  

	printf("1st Laser | ");
		for(int b=0;b<intDataPoints;b++)
		{
		currentRangeValue=scan_in->ranges[b];

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
	cvMoveWindow("LaserData", 0, 10);
 	outImg= cvCreateImage(cvSize(2*plotParam1, plotParam1), IPL_DEPTH_8U , 3);	
	}
	cout << "Got initial data. Starting Main Loop"<<endl;
virgin=false;


}
else
{
  //static FILE* logfile = fopen("bagfile.txt","w"); // open this for saving laser datas
  //printf("Got a scan with angle_min: %lf and angle_max: %lf\n",scan_in->angle_min, (*scan_in).angle_max);
	if(plotLaser==true)
	{ cvZero( outImg ); }

	printf("New Laser | ");
		
	for(int b=0;b<intDataPoints;b++)
		{
		currentRangeValue=scan_in->ranges[b];
		//printf("Degree = %u\n",b);
		//printf("Range: %lf\n",currentRangeValue);
		//fprintf (logfile,"%lf ", currentRangeValue); //log laser data
		//fprintf(logfile,"\t"); //log laser data

			if(currentRangeValue>floatMaxRadius)
			{
			currentRangeValue=floatMaxRadius;
			}
		doubleLaserDataX[b]=currentRangeValue*cos(b*CV_PI/(intDataPoints-1));
		doubleLaserDataY[b]=currentRangeValue*sin(b*CV_PI/(intDataPoints-1));

			if(plotLaser==true)
			{
			cvCircle(outImg, cvPoint(plotParam1 - plotParam2 * doubleLaserDataX[b], plotParam1 - plotParam2 * doubleLaserDataY[b]), 1.3, CV_RGB(255,255,255), 1.3);
			} 		
		}




			   vector <Segment> SegmentIndexesVectorOfVectors;
			   findSegmentsAndCreateCurrentGridMap(doubleLaserDataX,doubleLaserDataY,imgGridMap,imgGridMapIncludingSegmentNumbers,intNumOfSegments,SegmentIndexesVectorOfVectors);

	    	if (currentStatus==IDLE)
		{
		//ros::spinOnce();
	    	cout<<"IDLE |"<<endl;
	    		doubleLaserDataXprevious=doubleLaserDataX;
			doubleLaserDataYprevious=doubleLaserDataY;	
		}
		else if (currentStatus==NO_TARGET_TO_TRACK_MODE)
		{
		cout<<"NO_TARGET_TO_TRACK_MODE |"<<endl;
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
//				   cout<<"ham x: "<<doubleLaserDataYprevious[i]<<" ham y: "<<doubleLaserDataXprevious[i]<<endl;
//				   cout<<"x: "<<(int)((doubleLaserDataYprevious[i]/floatGridLength)+0.5)<<" y: "<< (int)( ((doubleLaserDataXprevious[i]+floatMaxRadius) /floatGridLength)+0.5)<<endl;
			   }
			   cvNot(imgGridMapPrevious,imgGridMapPreviousNegated);
			   cvMul(imgGridMap,imgGridMapPreviousNegated,DifferenceMap,1.0);
			   cvErode(DifferenceMap,cleanedGridMap,meineKernel);

			    //cvShowImage("cleanedGridMap",cleanedGridMap);
			    //cvShowImage("imgGridMap", imgGridMap);
			    //cvShowImage("imgGridMapPrevious", imgGridMapPrevious);
			    //cvShowImage("DifferenceMap", DifferenceMap);

				cvMul(cleanedGridMap,imgGridMapIncludingSegmentNumbers,cleanedGridMapIncludingSegmentNumbers,(double)1/255);


				/*cvSaveImage("data/cleanedSegNum.jpg",cleanedGridMapIncludingSegmentNumbers);
				cvSaveImage("data/DifferenceMap.jpg",DifferenceMap);
				cvSaveImage("data/imgGridMapPrevious.jpg",imgGridMapPrevious);
				cvSaveImage("data/imgGridMapPreviousNegated.jpg",imgGridMapPreviousNegated);
				cvSaveImage("data/imgGridMapIncludingSegmentNumbers.jpg",imgGridMapIncludingSegmentNumbers);
				cvSaveImage("data/cleanedGridMap.jpg",cleanedGridMap);
				cvSaveImage("data/imgGridMap.jpg",imgGridMap); */

				Segment validSegments;
				calculateCandidateSegments(cleanedGridMapIncludingSegmentNumbers,SegmentIndexesVectorOfVectors,intNumOfSegments,imgSize,validSegments);


				if(validSegments.empty())
				{
//					cout<<" No Valid Segments "<<endl;
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

												float valsXk[]={ ((firstPt[0]+lastPt[0])/2) , ((firstPt[1]+lastPt[1])/2) ,0,0};										//was: float valsXk[]

double dbltemp1;
double dbltemp2;

												transformPt(listener, ((firstPt[0]+lastPt[0])/2),((firstPt[1]+lastPt[1])/2),dbltemp1,dbltemp2,false);

valsXk[0]=(float)dbltemp1;
valsXk[1]=(float)dbltemp2;

												memcpy(kalman->state_post->data.fl,valsXk,sizeof(valsXk)); // set the initial Xk

												cout<<"STARTED TRACKING.."<<endl;
												//cout<<"Xcenter: "<<valsXk[0]<<" Ycenter: "<<valsXk[1]<<endl;





												currentStatus=ONE_LEG_TRACK_MODE;
												validSegments.clear();
						}
						else
						{
							cout<<"Moved segment not within range.."<<endl;
						}

				}

				cvCopy(imgGridMap, imgGridMapPrevious);




		 } // end of NO_TARGET_TO_TRACK_MODE
		else if(currentStatus==ONE_LEG_TRACK_MODE)
		{
			cout<<"ONE_LEG_TRACK_MODE | ";
			Segment segTemp;
			CvMat* z_k=cvCreateMat(2,1,CV_32FC1);
			const CvMat* y_k=cvKalmanPredict(kalman,0);
			CvMat* test=kalman->state_pre;

			//cout<<" XkPrior: "<<endl;
			//PrintMat(test);

			double minDistanceToPredictedState=100.0;
			int closestSegmentToPredictedState=100;
			double checkedCenterPoint[2];
			double checkedCenterPointLaserFrame[2];

			double estimatedCenterPointOfTarget[2];
			double firstPt[2];
			double lastPt[2];


			estimatedCenterPointOfTarget[0]=cvGet1D(test,0).val[0];
			estimatedCenterPointOfTarget[1]=cvGet1D(test,1).val[0];
			//cout<<"est X: "<<estimatedCenterPointOfTarget[0]<<" est Y: "<<estimatedCenterPointOfTarget[1]<<endl;

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
										transformPt(listener, ((firstPt[0]+lastPt[0])/2),((firstPt[1]+lastPt[1])/2),checkedCenterPoint[0],checkedCenterPoint[1],false);

											double temperDist=findDistanceBetweenTwoPoints(estimatedCenterPointOfTarget,checkedCenterPoint);
											cout<<" temperDist : "<<temperDist<<endl;
											if(temperDist<minDistanceToPredictedState)
											{
												minDistanceToPredictedState=temperDist;
												closestSegmentToPredictedState=t;
											}

						   			   }
									   cout<<"BEST MATCH DISTANCE: "<<minDistanceToPredictedState<<endl;
						   			   
									if(closestSegmentToPredictedState!=100 && minDistanceToPredictedState<maxDistanceToMatchPrediction)
						   			   {
							   			    segTemp=SegmentIndexesVectorOfVectors[closestSegmentToPredictedState];
											firstPt[0]=doubleLaserDataX[segTemp[0]];
											firstPt[1]=doubleLaserDataY[segTemp[0]];
											lastPt[0]=doubleLaserDataX[segTemp[segTemp.size()-1]];
											lastPt[1]=doubleLaserDataY[segTemp[segTemp.size()-1]];
											checkedCenterPointLaserFrame[0]=((firstPt[0]+lastPt[0])/2);
											checkedCenterPointLaserFrame[1]=((firstPt[1]+lastPt[1])/2);

											//TODO: transform checkedCenterPoint
										transformPt(listener, ((firstPt[0]+lastPt[0])/2),((firstPt[1]+lastPt[1])/2),checkedCenterPoint[0],checkedCenterPoint[1],true);



											cvmSet(z_k, 0, 0, checkedCenterPoint[0]);
											cvmSet(z_k, 1, 0, checkedCenterPoint[1]);
											//cout<<"X: "<<checkedCenterPoint[0]<<" Y: "<<checkedCenterPoint[1]<<endl;
											cvKalmanCorrect(kalman,z_k);

											CvMat* test2=kalman->state_post;
											//cout<<"Xk: "<<endl;
											//PrintMat(test2);
		  double Vx=cvGet1D(test2,2).val[0];
		  double Vy=cvGet1D(test2,3).val[0];
		  double VTotal=sqrt(Vx*Vx+Vy*Vy);
cout<<"VTotal: "<<VTotal<<endl;
		  maxDistanceToMatchPrediction=0.35+VTotal/2;
if(maxDistanceToMatchPrediction<0.35)
{
maxDistanceToMatchPrediction=0.35;
}
else if(maxDistanceToMatchPrediction>0.55)
{
maxDistanceToMatchPrediction=0.55;
}
cout<<"maxMatchPredict: "<<maxDistanceToMatchPrediction<<endl;



											double distToPerson=findDistanceBetweenTwoPoints(checkedCenterPointLaserFrame,originPtt);
											//double personAngle=atan2(checkedCenterPoint[1],checkedCenterPoint[0])*180/CV_PI-90; //+y axis is 0 degrees
											double personAngle=atan2(checkedCenterPointLaserFrame[1],checkedCenterPointLaserFrame[0])*180/CV_PI-90; //+y axis is 0 degrees
											
double angleOfPersonRadiansRaw=atan2(checkedCenterPointLaserFrame[1],checkedCenterPointLaserFrame[0]);
double angleOfPersonRadiansAdjusted=angleOfPersonRadiansRaw-CV_PI/2;


cout<<"X: "<<checkedCenterPointLaserFrame[0]<<" Y: "<<checkedCenterPointLaserFrame[1]<<endl;
/*cout<<"distToPerson: "<<distToPerson<<endl;
cout<<"personAngle: "<<personAngle<<endl; 
cout<<"angleOfPersonRadiansAdjusted: "<<angleOfPersonRadiansAdjusted<<endl;*/


		  




if(currentMotionStatus==REGULAR_FOLLOW)
{
	if(distToPerson>distanceToKeepWithPerson)
	{

				  double targetXmyCoordstest= checkedCenterPointLaserFrame[0]-distanceToKeepWithPerson*cos(angleOfPersonRadiansRaw+tetaNearby);
				  double targetYmyCoordstest= checkedCenterPointLaserFrame[1]-distanceToKeepWithPerson*sin(angleOfPersonRadiansRaw+tetaNearby);
				  if(plotLaser==true)
				  {
				  double tempx=plotParam1 - plotParam2 * targetXmyCoordstest;
				  double tempy=plotParam1 - plotParam2 * targetYmyCoordstest;
				  cvCircle(outImg, cvPoint(tempx,tempy), 4, cvScalar(0,128,255), 3);
				  }	




	  double targetXmyCoords= checkedCenterPointLaserFrame[0]-distanceToKeepWithPerson*cos(angleOfPersonRadiansRaw);
	  double targetYmyCoords= checkedCenterPointLaserFrame[1]-distanceToKeepWithPerson*sin(angleOfPersonRadiansRaw);
		  if(plotLaser==true)
		  {
		  double tempx=plotParam1 - plotParam2 * targetXmyCoords;
		  double tempy=plotParam1 - plotParam2 * targetYmyCoords;
		  cvCircle(outImg, cvPoint(tempx,tempy), 4, cvScalar(255,0,255), 2);
		  }


	  
	  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(angleOfPersonRadiansAdjusted);
//geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(-angleOfPersonRadiansAdjusted);
	  move_base_msgs::MoveBaseGoal goal;
	  goal.target_pose.header.frame_id = "laser";
	  goal.target_pose.header.stamp = ros::Time::now();
	  //goal.target_pose.pose.position.x = targetYmyCoords;
	  //goal.target_pose.pose.position.y = targetXmyCoords;

	  goal.target_pose.pose.position.x = targetYmyCoords;
	  goal.target_pose.pose.position.y = -targetXmyCoords;

	  goal.target_pose.pose.orientation=quat;

	  //ROS_INFO("Sending positional goal");
	  if(inhibitNavigationMovement==false)
	  {
	  ac.sendGoal(goal);
	  }



	}
	else
	{
	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(angleOfPersonRadiansAdjusted);
//geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(-angleOfPersonRadiansAdjusted);
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "laser";
	goal.target_pose.header.stamp = ros::Time::now();
	
	goal.target_pose.pose.orientation=quat;
	//goal.target_pose.pose.position.x=0;
	//goal.target_pose.pose.position.y=0;
	//goal.target_pose.pose.position.z=0;
	
	//goal.target_pose.pose.orientation.w = 1.0;

	//ROS_INFO("Sending orientation goal");
	  if(inhibitNavigationMovement==false)
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
			  	if(velocityCounter>(int)(2*operatingFrequency))
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
			  	if(velocityCounter>(int)(9*operatingFrequency))
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
				currentMotionStatus=GO_NEARBY_PERSON;				  
				ac.sendGoal(goal);
			  	  

				}

			  }
			  else //person is not static
			  {
			  velocityCounter=0;
			  }
		}			  

	}




}
else if(currentMotionStatus==GO_NEARBY_PERSON)
{
	cout<<"ENTERED GO_NEARBY_PERSON----------------"<<endl;
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
	ROS_INFO("GO_NEARBY_PERSON SUCCESSFUL!");
	currentMotionStatus=REGULAR_FOLLOW;
	}

}


										if(inhibitAngularMovement==false)
										{
										//determine angular speeds
											if(distToPerson<1.1)
											{
												thres1=20;
												thres2=35;
												thres3=50;
												thres4=70;
											}
											else if(distToPerson>2.5)
											{
												thres1=10;
												thres2=16;
												thres3=24;
												thres4=30;
											}
											else if (distToPerson>1.7)
											{
												thres1=13;
												thres2=22;
												thres3=33;
												thres4=43;
											}
											else if (distToPerson>1.1)
											{
												thres1=15;
												thres2=25;
												thres3=40;
												thres4=55;
											}

											if(abs(personAngle)<thres1)
											{
											cmd.angular.z = 0;
											}
											else if(personAngle>thres4) //person is on right.
											{
											cmd.angular.z = -0.6;
											}
											else if(personAngle>thres3) //person is on right.
											{
											cmd.angular.z = -0.5;
											}
											else if(personAngle>thres2) //person is on right.
											{
											cmd.angular.z = -0.4;
											}
											else if(personAngle>thres1) //person is on right.
											{
											cmd.angular.z = -0.3;
											}

											else if(personAngle<-thres4) //person is on left.
											{
											cmd.angular.z = 0.6;
											}
											else if(personAngle<-thres3) //person is on left.
											{
											cmd.angular.z = 0.5;
											}
											else if(personAngle<-thres2) //person is on L.
											{
											cmd.angular.z = 0.4;
											}
											else if(personAngle<-thres1) //person is on left
											{
											cmd.angular.z = 0.3;
											}
										}
											if(inhibitLinearMovement==false)
											{
											//now linear speeds
												if(distToPerson>2.75)
												{									
												cmd.linear.x=0.27+extraVelocityBeCareful;	
												}
												if(distToPerson>2.5)
												{									
												cmd.linear.x=0.24+extraVelocityBeCareful;	
												}
												if(distToPerson>2.0)
												{									
												cmd.linear.x=0.21+extraVelocityBeCareful;	
												}
												else if(distToPerson>1.5)
												{									
												cmd.linear.x=0.19+extraVelocityBeCareful;	
												}
												else if(distToPerson>1)
												{									
												cmd.linear.x=0.15+extraVelocityBeCareful;	
												}
												else if(distToPerson>0.75)
												{									
												cmd.linear.x=0.10;	
												}
												else
												{
												cmd.linear.x=0;
												}
											}
												//cout<<"Person is @angle : " <<personAngle<<", @distance"<<distToPerson<<endl;
						if(inhibitLinearMovement&&inhibitAngularMovement==false)
						{												
						commander_pub.publish(cmd);
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
						   			   }
									else
										{
										cout<<"No good Segment To Match."<<endl;
											//cvKalmanCorrect(kalman,z_k);
										//cmd.linear.x=0;
										//cmd.angular.z=0;
										//commander_pub.publish(cmd);
											//CvMat* test2=kalman->state_post;
										}




		}// end of ONE_LEG_TRACK_MODE
		else if(currentStatus==TWO_LEGS_TRACK_MODE)
		{


		}// end of TWO_LEGS_TRACK_MODE
		

				if(plotLaser==true)
				{
				cvShowImage("LaserData",outImg);
				cvWaitKey(4);
				}

//				cvSetZero(imgGridMapIncludingSegmentNumbers);
//				cvCopy(cleanedGridMapIncludingSegmentNumbers, imgGridMapIncludingSegmentNumbers);
				doubleLaserDataXprevious=doubleLaserDataX;
				doubleLaserDataYprevious=doubleLaserDataY;
				
				//char stringTest[50];
				//sprintf( stringTest, "data/img%d.jpg", testFrame );
				//cvWaitKey(0);			
				//cvSaveImage(stringTest,outImg);

				cvSetZero(imgGridMap);								
		 


} //end of virgin==false




//fprintf(logfile,"\n"); //log laser data
//fclose(logfile);

}


void tenSecondsCallback(const ros::TimerEvent& e)
{
	ROS_INFO("tenSecondsCallback triggered");
	inhibitNavigationMovement=false;
}

void commandCallback  (const std_msgs::StringConstPtr& msg)
{
ROS_INFO("Rcvd string: [%s]", msg->data.c_str());
string rcvd=msg->data.c_str();

ros::NodeHandle n;
geometry_msgs::Twist cmd;
ros::Publisher commander_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 3);
ros::Rate pub_rate(20);

	if(rcvd.compare("q")==0) //quit program
	{
		cmd.linear.x=0;
		cmd.angular.z=0;
		commander_pub.publish(cmd); //TODO:open it up later
		cvWaitKey(3);
	    
	    /*cvDestroyAllWindows(); //TODO: deal with the release images later.
	    cvReleaseImage(&cleanedGridMap);
	    cvReleaseImage(&imgGridMap);
	    cvReleaseImage(&imgGridMapPrevious);
	    cvReleaseImage(&DifferenceMap);*/
		
		/*if(plotLaser==true)
		{    
		cvReleaseImage(&outImg);
		} */
		exit(0);
	}
	if(rcvd.compare("i")==0) //idle state
	{
	cmd.linear.x=0;
	cmd.angular.z=0;
	commander_pub.publish(cmd);
	currentStatus=IDLE;
	}
	if(rcvd.compare("f")==0) //follow people
	{
	cmd.linear.x=0;
	cmd.angular.z=0;
	commander_pub.publish(cmd);
	currentStatus=NO_TARGET_TO_TRACK_MODE;
	}
	if(rcvd.compare("pplfollow_stop_and_wait")==0 || rcvd.compare("sw")==0)
	{

		ros::NodeHandle n;
		ros::Timer timer1 = n.createTimer(ros::Duration(10),tenSecondsCallback,true);
		inhibitNavigationMovement=true;
		ros::spin();
	}
} 



void findSegmentsAndCreateCurrentGridMap(const vector<double> &xArray, const vector<double> &yArray, IplImage* imgGridMap, IplImage* imgGridMapIncludingSegmentNumbers, int &intNumOfSegments, vector <Segment>& SegmentIndexesVectorOfVectors)
{
	double currentPoint[2];
	double nextPoint[2];
	double originPoint[2];
	const float floatUncertainty=0.02;
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

double findDistanceBetweenTwoPoints(const double pt1[], const double pt2[])
{
double tot=sqrt(pow((pt1[0]-pt2[0]),2)+pow((pt1[1]-pt2[1]),2));
return tot;
}


void calculateCandidateSegments(const IplImage* cleanedGridMapIncludingSegmentNumbers,  const vector <Segment> SegmentIndexesVectorOfVectors , const int intNumOfSegments, CvSize imgSize, Segment &validSegments)
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
//			cout<<"i: " <<i<<" j: "<<j<<" v: "<<cvGet2D(cleanedGridMapIncludingSegmentNumbers,i,j).val[0]<<" "<<endl; //values bigger than 0.
			}
//						cout<<"i: " <<i<<" j: "<<j<<" v: "<<cvGet2D(cleanedGridMapIncludingSegmentNumbers,i,j).val[0]<<" "<<endl; // all values
		}
	}


	for(unsigned int i=0;i<SegmentIndexesVectorOfVectors.size();i++) //for printing an image. i is the y axis, j is the x axis. GIMP=(j,i)
	{
		temperSegment=SegmentIndexesVectorOfVectors[i];
		totalSegmentHits[i]=temperSegment.size();
	}

	for(int i=0;i<intNumOfSegments;i++)
	{
//		cout<<" Segment"<<i+1<<" : "<<"Total: "<<totalSegmentHits[i]<<" Hits: "<<hitsFromGridMap[i]<<endl;

		currentHitRatio=((float)hitsFromGridMap[i]/(float)totalSegmentHits[i]);
//		cout<<"currrentHitRatio: "<< currentHitRatio<<endl;
		if(currentHitRatio>floatHitRatioTresholdForMovementDetection && hitsFromGridMap[i]>=minPixelsToCountAllowedForMovementDetection)
		{
			validSegments.push_back(i);
		    //cout<<" Valid Segment: "<<i<<endl;
		}

	}



}

void PrintMat(CvMat *A)
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







int main(int argc, char** argv)
{

  ros::init(argc, argv, "ppl_follower_node");
  ros::NodeHandle n;

  
  //tf::TransformListener listener(ros::Duration(10)); //tf
  ros::Subscriber chatter_sub2=n.subscribe("korus_control_topic",1,commandCallback);  
  ros::Subscriber chatter_sub=n.subscribe("scan",1,scanCallback);
  //ros::Rate loop_rate(operatingFrequency); //work at 4.68 Hz
//ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
tf::TransformListener listener(ros::Duration(10)); //tf

	ros::Publisher commander_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 3);
	ros::Rate pub_rate(20);
	cmd.linear.x = cmd.linear.y = cmd.linear.z = 0;
	cmd.angular.x = cmd.angular.y = cmd.angular.z = 0;	



  MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "laser";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 0.0;
  goal.target_pose.pose.position.y = 0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
//ac.waitForResult(ros::Duration(10.0));
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
{
ROS_INFO("state= SUCCEEEDED");
}
else if(ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
{
ROS_INFO("state= ACTIVE");
}
else if(ac.getState() == actionlib::SimpleClientGoalState::PENDING)
{
ROS_INFO("state= PENDING");
}
else if(ac.getState() == actionlib::SimpleClientGoalState::LOST)
{
ROS_INFO("state= LOST");
}
else if(ac.getState() == actionlib::SimpleClientGoalState::REJECTED)
{
ROS_INFO("state= REJECTED");
}

else if(ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED)
{
ROS_INFO("state= PREEMPTED");
}

else if(ac.getState() == actionlib::SimpleClientGoalState::RECALLED)
{
ROS_INFO("state= RECALLED");
}

else
{ 
//cmd.linear.x=0;
//cmd.angular.z = 0;
ROS_INFO("The base failed to move forward 1 meter for some reason");
} 

ros::spin();
}



