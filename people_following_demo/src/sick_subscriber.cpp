
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


#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <geometry_msgs/Twist.h>

const int plotParam1=400;
const int plotParam2=133;
const double operatingFrequency=4.64; //4.64 or 9.08



const float rectangularBoxXover2sizeForMovDetection=0.75;
const float rectangularBoxYsizeForMovDetection=1.3;



const float maxDistanceToMatchPrediction=0.4;
const int minPixelsToCountAllowedForMovementDetection=2;
const float floatHitRatioTresholdForMovementDetection=0.10;
const float floatMaxRadius=3;
const int intDataPoints=181; //361
const float distanceTreshold=0.1;
const int numberOfPointsTreshold=3;
const bool boolFilterSegmentsAccordingToLengthInSegmentation=true;
const float maxEuclidianDistOfSegmentsThatCanBePersonCandids=1; //0.6 before
const float minEuclidianDistOfSegmentsThatCanBePersonCandids=0.1; //0.1 before
const float floatGridLength=0.05;
const int fillingForImages=255; //255 for visualization, 1 for non-visual
CvScalar cvScalarTemp;
const bool plotLaser=true;
const bool useRealLaser=true;
const bool infiniteLoop=true;
const bool inhibitLinearMovement=true;
const bool inhibitAngularMovement=true;
const double extraVelocityBeCareful=0.07;

	vector<double> doubleLaserDataXprevious(intDataPoints);
	vector<double> doubleLaserDataYprevious(intDataPoints);
	vector<double> doubleLaserDataX(intDataPoints);
	vector<double> doubleLaserDataY(intDataPoints);
double originPtt[2]={0,0};


float thres1,thres2,thres3,thres4;
IplImage *outImg;

using namespace std;
typedef vector <int> Segment;

enum Status{NO_TARGET_TO_TRACK_MODE,ONE_LEG_TRACK_MODE,TWO_LEGS_TRACK_MODE,IDLE};
//Status currentStatus=NO_TARGET_TO_TRACK_MODE;
Status currentStatus=IDLE;



void scanCallback  (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  float currentRangeValue;
  
  //static FILE* logfile = fopen("bagfile.txt","w"); // open this for saving laser datas

    //printf("Got a scan with angle_min: %lf and angle_max: %lf\n",scan_in->angle_min, (*scan_in).angle_max);
    sensor_msgs::PointCloud cloud;
    projector_.projectLaser(*scan_in, cloud,floatMaxRadius);

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

			if(plotLaser==true && currentStatus==IDLE)
			{
			cvCircle(outImg, cvPoint(plotParam1 - plotParam2 * doubleLaserDataX[b], plotParam1 - plotParam2 * doubleLaserDataY[b]), 1, CV_RGB(255,255,255), 2);
			}
		
		}
		if(plotLaser==true && currentStatus==IDLE)
		{
		cvShowImage("LaserData",outImg);
		cvWaitKey(4);
		}















//fprintf(logfile,"\n"); //log laser data
//fclose(logfile);
}



void commandCallback  (const std_msgs::StringConstPtr& msg)
{
ROS_INFO("Received command: [%s]", msg->data.c_str());
string rcvd=msg->data.c_str();

ros::NodeHandle n;
geometry_msgs::Twist cmd;
ros::Publisher commander_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
ros::Rate pub_rate(20);

	if(rcvd.compare("q")==0) //quit program
	{
	cmd.linear.x=0;
	cmd.angular.z=0;
	commander_pub.publish(cmd);
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

} 


int main(int argc, char** argv)
{

  ros::init(argc, argv, "ppl_follower_node");
  ros::NodeHandle n;

  ros::Subscriber chatter_sub2=n.subscribe("korus_control_topic",1,commandCallback);
  
  ros::Subscriber chatter_sub=n.subscribe("scan",1,scanCallback);

  ros::Rate loop_rate(operatingFrequency); //work at 4.68 Hz 

	ros::Publisher commander_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	ros::Rate pub_rate(20);
	geometry_msgs::Twist cmd;
	cmd.linear.x = cmd.linear.y = cmd.linear.z = 0;
	cmd.angular.x = cmd.angular.y = cmd.angular.z = 0;
	
	//const float floatFieldOfView=CV_PI;
	int counter1=0;
	const int testFrameStart=30; //78-89 in c++ is equal to frames 80-90 in matlab.
	int testFrameEnd=150; //45
	int intNumOfSegments=0;


	vector<double> cosineChart(intDataPoints);
	vector<double> sineChart(intDataPoints);


	CvSize imgSize = CvSize();
	imgSize.height=(int)( (floatMaxRadius/floatGridLength)+1 +0.5);
	imgSize.width=(int)( (2*floatMaxRadius/floatGridLength)+1 +0.5);

	IplConvKernel* meineKernel= cvCreateStructuringElementEx(2,1,0,0,CV_SHAPE_RECT);
	IplImage* imgGridMap = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	IplImage* imgGridMapPrevious = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	IplImage* imgGridMapIncludingSegmentNumbers = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	IplImage* imgGridMapPreviousNegated = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	IplImage* cleanedGridMapIncludingSegmentNumbers = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	IplImage* DifferenceMap = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	IplImage* cleanedGridMap = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
//	cout<<"w: "<<imgSize.width<<"h: "<<imgSize.height<<endl;
	




	CvKalman* kalman=cvCreateKalman(4,2,0);
	float deltaT=1/operatingFrequency;
	float qUncertainty=0.01;
	float rUncertainty=0.01;
	float valsF[]={1,0,deltaT,0,		0,1,0,deltaT,	0,0,1,0		,	0,0,0,1};


	//cvNamedWindow("cleanedGridMap",CV_WINDOW_AUTOSIZE);
	//cvNamedWindow( "imgGridMap", CV_WINDOW_AUTOSIZE );
	//cvNamedWindow( "imgGridMapPrevious", CV_WINDOW_AUTOSIZE );
	//cvNamedWindow( "DifferenceMap", CV_WINDOW_AUTOSIZE );

	if(plotLaser==true)
	{
	cvNamedWindow("LaserData", CV_WINDOW_AUTOSIZE);
	cvMoveWindow("LaserData", 00, 100);
 	outImg= cvCreateImage(cvSize(800, 400), IPL_DEPTH_8U , 3);	
	}

	ifstream myfile;
	double num;
	if(useRealLaser==false)
	{
		for(int p=0;p<intDataPoints;p++) //filling out cosine and sine values for 361 data points.
		 {
			 cosineChart[p]=cos(p*CV_PI/(intDataPoints-1));
			 sineChart[p]=sin(p*CV_PI/(intDataPoints-1));
		 }

		  
		  myfile.open("../data/laserHR2.out");
		  if (!(myfile.is_open()))
		  {
			 cout << "Unable to open file";
			 return 0;
		  }

		   
		   counter1=0;
		   int lineLimit=intDataPoints*testFrameStart;
		   while (counter1<lineLimit) //going to the desired frame in the complete laser data
		   {
		      myfile >> num;
		      counter1++;
		   }
		   counter1=0;
		   while ( !myfile.eof() && counter1<intDataPoints) // for initialization, need to have 2 laser scans to start. filling previous values
		   {
		      myfile >> num;
			  if (num>floatMaxRadius)
			  {
				  num=floatMaxRadius;
			  }
			  doubleLaserDataXprevious[counter1]=num*cosineChart[counter1];
			  doubleLaserDataYprevious[counter1]=num*sineChart[counter1];
		      counter1++;
	
		   }
	}
	else if(useRealLaser==true) //initialization of prev gridMap
	{
	loop_rate.sleep();
	ros::spinOnce();
	

	doubleLaserDataXprevious=doubleLaserDataX;
	doubleLaserDataYprevious=doubleLaserDataY;
	}
	cout << "Got initial data. Starting Main Loop"<<endl;

		 for(int testFrame=testFrameStart;testFrame<testFrameEnd;testFrame++) //main loop
		 {

if(infiniteLoop==true)
{
testFrameEnd=testFrame+2;
}

			 if(useRealLaser==false)
			{ 
				if(myfile.eof())
				   {
					   return EXIT_SUCCESS;
				   }
			}

			if(plotLaser==true)
			{ cvZero( outImg ); }
			  
			if(useRealLaser==false)
			{ 
				  counter1=0;
				   while ( !myfile.eof() && counter1<intDataPoints) //import laser data here. load doubleLaserDataX and doubleLaserDataY
				   {
					  myfile >> num;
						  if (num>floatMaxRadius)
						  {
							  num=floatMaxRadius;
						  }
					  doubleLaserDataX[counter1]=num*cosineChart[counter1]; //load new laser X and Y values
					  doubleLaserDataY[counter1]=num*sineChart[counter1];
					  
					cout<<num<<endl;
				          cout<<" p: "<<counter1<<" cos(p): "<< cosineChart[counter1]<<endl;
					  cout<<" p: "<<counter1<<" sin(p): "<< sineChart[counter1]<<endl;
					  cout<<" p: "<<counter1<<" X val: "<< doubleLaserDataX[counter1]<<" y val: "<<doubleLaserDataY[counter1]<<endl;
					  cout<<num<<" ";
				      
					counter1++;				
				   }
			}	
			else if(useRealLaser==true)  //get periodic laser
			{
			loop_rate.sleep();			
			ros::spinOnce();
			}		


                                      if(plotLaser==true) 
	 	                                                { 
	 	                                                double tempx,tempy; 
	 	                                                        for (int ii=0; ii<intDataPoints; ii++)  
	 	                                                        { 
	 	                                                                tempx = plotParam1 - plotParam2 * doubleLaserDataX[ii]; 
	 	                                                                tempy = plotParam1 - plotParam2 * doubleLaserDataY[ii];                  
	 	                                                             cvCircle(outImg, cvPoint(tempx, tempy), 1, CV_RGB(255,255,255),2);                                                              
	 	                                                        } 
	 	                                                cvShowImage("LaserData",outImg); 
	 	                                                cvWaitKey(5); 
	 	                                                }       


			   vector <Segment> SegmentIndexesVectorOfVectors;
			   findSegmentsAndCreateCurrentGridMap(doubleLaserDataX,doubleLaserDataY,imgGridMap,imgGridMapIncludingSegmentNumbers,intNumOfSegments,SegmentIndexesVectorOfVectors);

//			   			   cout<<intNumOfSegments<<endl; //for printing current segment values.
//			   			   for(int t=0;t<SegmentIndexesVectorOfVectors.size();t++)
//			   			   {
//			   			   Segment segTemp=SegmentIndexesVectorOfVectors[t];
//			   			   cout<<endl<<"Starting Segment: "<<t<<endl;
//			   				   for(int u=0;u<segTemp.size();u++)
//			   				   {
//			   					   cout<<segTemp[u]<<" ";
//			   				   }
//			   			   }
	    	if (currentStatus==IDLE)
		{
		ros::spinOnce();
	    	cout<<"IDLE |"<<endl;
	    				doubleLaserDataXprevious=doubleLaserDataX;
			doubleLaserDataYprevious=doubleLaserDataY;	
		}
		else if (currentStatus==NO_TARGET_TO_TRACK_MODE)
		{

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



//				cvCopy(cleanedGridMapIncludingSegmentNumbers, imgGridMapIncludingSegmentNumbers);

//				cout<<" Displaying:"<<" imgGridMap"<<endl;
//				for(int i=0;i<imgSize.height;i++) //for printing an image. i is the y axis, j is the x axis. GIMP=(j,i)
//				{
//					for(int j=0;j<imgSize.width;j++)
//					{
//						if(!(cvGet2D(imgGridMap,i,j).val[0]==0))
//						{
//						cout<<"i: " <<i<<" j: "<<j<<" v: "<<cvGet2D(imgGridMap,i,j).val[0]<<" "<<endl; //values bigger than 0.
//						}
////						cout<<"i: " <<i<<" j: "<<j<<" v: "<<cvGet2D(cleanedGridMapIncludingSegmentNumbers,i,j).val[0]<<" "<<endl; // all values
//					}
//				}
//
//
//				cout<<" Displaying:"<<" imgGridMapPrevious"<<endl;
//				for(int i=0;i<imgSize.height;i++) //for printing an image. i is the y axis, j is the x axis. GIMP=(j,i)
//				{
//					for(int j=0;j<imgSize.width;j++)
//					{
//						if(!(cvGet2D(imgGridMapPrevious,i,j).val[0]==0))
//						{
//						cout<<"i: " <<i<<" j: "<<j<<" v: "<<cvGet2D(imgGridMapPrevious,i,j).val[0]<<" "<<endl; //values bigger than 0.
//						}
////						cout<<"i: " <<i<<" j: "<<j<<" v: "<<cvGet2D(cleanedGridMapIncludingSegmentNumbers,i,j).val[0]<<" "<<endl; // all values
//					}
//				}
//
//
//				cout<<" Displaying:"<<" DifferenceMap"<<endl;
//				for(int i=0;i<imgSize.height;i++) //for printing an image. i is the y axis, j is the x axis. GIMP=(j,i)
//				{
//					for(int j=0;j<imgSize.width;j++)
//					{
//						if(!(cvGet2D(DifferenceMap,i,j).val[0]==0))
//						{
//						cout<<"i: " <<i<<" j: "<<j<<" v: "<<cvGet2D(DifferenceMap,i,j).val[0]<<" "<<endl; //values bigger than 0.
//						}
////						cout<<"i: " <<i<<" j: "<<j<<" v: "<<cvGet2D(cleanedGridMapIncludingSegmentNumbers,i,j).val[0]<<" "<<endl; // all values
//					}
//				}
//
//
//				cout<<" Displaying:"<<" cleanedGridMap"<<endl;
//				for(int i=0;i<imgSize.height;i++) //for printing an image. i is the y axis, j is the x axis. GIMP=(j,i)
//				{
//					for(int j=0;j<imgSize.width;j++)
//					{
//						if(!(cvGet2D(cleanedGridMap,i,j).val[0]==0))
//						{
//						cout<<"i: " <<i<<" j: "<<j<<" v: "<<cvGet2D(cleanedGridMap,i,j).val[0]<<" "<<endl; //values bigger than 0.
//						}
////						cout<<"i: " <<i<<" j: "<<j<<" v: "<<cvGet2D(cleanedGridMapIncludingSegmentNumbers,i,j).val[0]<<" "<<endl; // all values
//					}
//				}
//
//				cout<<" Displaying:"<<" cleanedGridMapIncludingSegmentNumbers "<<endl;
//				for(int i=0;i<imgSize.height;i++) //for printing an image. i is the y axis, j is the x axis. GIMP=(j,i)
//				{
//					for(int j=0;j<imgSize.width;j++)
//					{
//						if(!(cvGet2D(cleanedGridMapIncludingSegmentNumbers,i,j).val[0]==0))
//						{
//						cout<<"i: " <<i<<" j: "<<j<<" v: "<<cvGet2D(cleanedGridMapIncludingSegmentNumbers,i,j).val[0]<<" "<<endl; //values bigger than 0.
//						}
////						cout<<"i: " <<i<<" j: "<<j<<" v: "<<cvGet2D(cleanedGridMapIncludingSegmentNumbers,i,j).val[0]<<" "<<endl; // all values
//					}
//				}









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
//					cout<<" NO Valid Segments "<<endl;
				}
				else //there is a moved segment
				{

					double firstPt[2];
					double lastPt[2];
					double centerPtt[2];
					int segNumberToTrack;
					int closestMovingSegment;
					double minMovingSegmentDistance=100.0;					
					

						for (int h=0; h<validSegments.size();h++)
						{
						cout<<"Segmo: "<<validSegments[h]<<endl;
						Segment segmentTester=SegmentIndexesVectorOfVectors[validSegments[h]];
						firstPt[0]=doubleLaserDataX[segmentTester[0]];
						firstPt[1]=doubleLaserDataY[segmentTester[0]];
						lastPt[0]=doubleLaserDataX[segmentTester[segmentTester.size()-1]];
						lastPt[1]=doubleLaserDataY[segmentTester[segmentTester.size()-1]];
						centerPtt[0]=((firstPt[0]+lastPt[0])/2);
						centerPtt[1]=((firstPt[1]+lastPt[1])/2);
						cout<<"centerPtt[0]: "<<centerPtt[0]<<" centerPtt[1]: "<<centerPtt[1]<<endl;
						double tempDist=findDistanceBetweenTwoPoints(centerPtt,originPtt);
						cout<<"tempDist= "<<tempDist<<endl;
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

												float valsXk[]={ ((firstPt[0]+lastPt[0])/2) , ((firstPt[1]+lastPt[1])/2) ,0,0};
												memcpy(kalman->state_post->data.fl,valsXk,sizeof(valsXk)); // set the initial Xk

												cout<<"MOVEMENT DETECTED. STARTED TRACKING.."<<endl;
												cout<<"Xcenter: "<<valsXk[0]<<" Ycenter: "<<valsXk[1]<<endl;
												currentStatus=ONE_LEG_TRACK_MODE;
												validSegments.clear();
						}
						else
						{
							cout<<"Waiting for a human to move.."<<endl;
						}

				}

				cvCopy(imgGridMap, imgGridMapPrevious);




		 } // end of NO_TARGET_TO_TRACK_MODE
		else if(currentStatus==ONE_LEG_TRACK_MODE)
		{
			Segment segTemp;
			CvMat* z_k=cvCreateMat(2,1,CV_32FC1);
			const CvMat* y_k=cvKalmanPredict(kalman,0);
			CvMat* test=kalman->state_pre;

			//cout<<" XkPrior: "<<endl;
			//PrintMat(test);

			double minDistanceToPredictedState=20.0;
			int closestSegmentToPredictedState=100;
			double checkedCenterPoint[2];
			double estimatedCenterPointOfTarget[2];
			double firstPt[2];
			double lastPt[2];


			estimatedCenterPointOfTarget[0]=cvGet1D(test,0).val[0];
			estimatedCenterPointOfTarget[1]=cvGet1D(test,1).val[0];
			//cout<<"estimated Target X: "<<estimatedCenterPointOfTarget[0]<<" estimated Target Y: "<<estimatedCenterPointOfTarget[1]<<endl;

						   			   for(int t=0;t<SegmentIndexesVectorOfVectors.size();t++)
						   			   {
						   			   segTemp=SegmentIndexesVectorOfVectors[t];
						   			   //cout<<endl<<"Checking Segment: "<<t<<endl;

										firstPt[0]=doubleLaserDataX[segTemp[0]];
										firstPt[1]=doubleLaserDataY[segTemp[0]];
										lastPt[0]=doubleLaserDataX[segTemp[segTemp.size()-1]];
										lastPt[1]=doubleLaserDataY[segTemp[segTemp.size()-1]];
										checkedCenterPoint[0]=((firstPt[0]+lastPt[0])/2);
										checkedCenterPoint[1]=((firstPt[1]+lastPt[1])/2);

											double temperDist=findDistanceBetweenTwoPoints(estimatedCenterPointOfTarget,checkedCenterPoint);
										//	cout<<" temperDist : "<<temperDist<<endl;
											if(temperDist<minDistanceToPredictedState)
											{
												minDistanceToPredictedState=temperDist;
												closestSegmentToPredictedState=t;
											}

						   			   }
				//cout<<"minDistanceToPredictedState: "<<minDistanceToPredictedState<<endl;
						   			   
									if(closestSegmentToPredictedState!=100 && minDistanceToPredictedState<maxDistanceToMatchPrediction)
						   			   {
							   			    segTemp=SegmentIndexesVectorOfVectors[closestSegmentToPredictedState];
											firstPt[0]=doubleLaserDataX[segTemp[0]];
											firstPt[1]=doubleLaserDataY[segTemp[0]];
											lastPt[0]=doubleLaserDataX[segTemp[segTemp.size()-1]];
											lastPt[1]=doubleLaserDataY[segTemp[segTemp.size()-1]];
											checkedCenterPoint[0]=((firstPt[0]+lastPt[0])/2);
											checkedCenterPoint[1]=((firstPt[1]+lastPt[1])/2);
											cvmSet(z_k, 0, 0, checkedCenterPoint[0]);
											cvmSet(z_k, 1, 0, checkedCenterPoint[1]);
											cout<<"X: "<<checkedCenterPoint[0]<<" Y: "<<checkedCenterPoint[1]<<endl;
											cvKalmanCorrect(kalman,z_k);

											CvMat* test2=kalman->state_post;
											//cout<<"Xk: "<<endl;
											//PrintMat(test2);

											    double distToPerson=findDistanceBetweenTwoPoints(checkedCenterPoint,originPtt);
											double personAngle=atan2(checkedCenterPoint[1],checkedCenterPoint[0])*180/CV_PI-90; //+y axis is 0 degrees
											
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
											cmd.angular.z = 0.85;
											}
											else if(personAngle>thres3) //person is on right.
											{
											cmd.angular.z = 0.73;
											}
											else if(personAngle>thres2) //person is on right.
											{
											cmd.angular.z = 0.64;
											}
											else if(personAngle>thres1) //person is on right.
											{
											cmd.angular.z = 0.52;
											}

											else if(personAngle<-thres4) //person is on left.
											{
											cmd.angular.z = -0.85;
											}
											else if(personAngle<-thres3) //person is on left.
											{
											cmd.angular.z = -0.73;
											}
											else if(personAngle<-thres2) //person is on L.
											{
											cmd.angular.z = -0.64;
											}
											else if(personAngle<-thres1) //person is on left
											{
											cmd.angular.z = -0.52;
											}
										}
											if(inhibitLinearMovement==false)
											{
											//now linear speeds
												if(distToPerson>2.75)
												{									
												cmd.linear.x=0.75+extraVelocityBeCareful;	
												}
												if(distToPerson>2.5)
												{									
												cmd.linear.x=0.62+extraVelocityBeCareful;	
												}
												if(distToPerson>2.0)
												{									
												cmd.linear.x=0.52+extraVelocityBeCareful;	
												}
												else if(distToPerson>1.5)
												{									
												cmd.linear.x=0.42+extraVelocityBeCareful;	
												}
												else if(distToPerson>1)
												{									
												cmd.linear.x=0.32+extraVelocityBeCareful;	
												}
												else if(distToPerson>0.75)
												{									
												cmd.linear.x=0.2;	
												}
												else
												{
												cmd.linear.x=0;
												}
											}
												//cout<<"Person is @angle : " <<personAngle<<", @distance"<<distToPerson<<endl;
												commander_pub.publish(cmd);


		                                                                        if(plotLaser==true)                
		                                                                        {
		                                                                        double tempx=plotParam1 - plotParam2 * checkedCenterPoint[0];
		                                                                        double tempy=plotParam1 - plotParam2 * checkedCenterPoint[1];
		                                                                        
		                                                                        cvCircle(outImg, cvPoint(tempx,tempy), 20, cvScalar(0,255,0), 1);
		                                                                        tempx=plotParam1 - plotParam2 * estimatedCenterPointOfTarget[0];
		                                                                        tempy=plotParam1 - plotParam2 * estimatedCenterPointOfTarget[1];
		                                                                        cvCircle(outImg, cvPoint(tempx,tempy), 8, cvScalar(0,255,255), 2);

		                                                                        cvShowImage("LaserData", outImg);
		                                                                        cvWaitKey(5);
		                                                                        }
						   			   }
									else
										{
										cout<<"No good Segment To Match."<<endl;
											//cvKalmanCorrect(kalman,z_k);
										cmd.linear.x=0;
										cmd.angular.z=0;
										commander_pub.publish(cmd);
											//CvMat* test2=kalman->state_post;
										}




		}// end of ONE_LEG_TRACK_MODE
		else if(currentStatus==TWO_LEGS_TRACK_MODE)
		{


		}// end of TWO_LEGS_TRACK_MODE


//				cvSetZero(imgGridMapIncludingSegmentNumbers);
//				cvCopy(cleanedGridMapIncludingSegmentNumbers, imgGridMapIncludingSegmentNumbers);
				doubleLaserDataXprevious=doubleLaserDataX;
				doubleLaserDataYprevious=doubleLaserDataY;
				
				char stringTest[50];
				sprintf( stringTest, "data/img%d.jpg", testFrame );


				//cvWaitKey(0);
				//sprintf(
				cvSaveImage(stringTest,outImg);
				cvSetZero(imgGridMap);								
		 } //end of main loop


    cmd.linear.x=0;
	cmd.angular.z=0;
	commander_pub.publish(cmd);

    myfile.close();
    cvDestroyAllWindows();
    cvReleaseImage(&cleanedGridMap);
    cvReleaseImage(&imgGridMap);
    cvReleaseImage(&imgGridMapPrevious);
    cvReleaseImage(&DifferenceMap);
	if(plotLaser==true)
	{    
	cvReleaseImage(&outImg);
	}
	return EXIT_SUCCESS;
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


	for (int i=0; i< (intDataPoints); i++) //from i=0-359
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

				for(int p=0;p<tempArray.size();p++) //grid map construction here!
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

					for(int p=0;p<tempArray.size();p++) //grid map construction here!
					 {
						cvScalarTemp=cvScalar(fillingForImages);
						cvSet2D(imgGridMap, (int)((yArray[tempArray[p]]/floatGridLength)+0.5), (int)( ((xArray[tempArray[p]]+floatMaxRadius) /floatGridLength)+0.5), cvScalarTemp);

						cvScalarTemp=cvScalar(intNumOfSegments);
						cvSet2D(imgGridMapIncludingSegmentNumbers, (int)((yArray[tempArray[p]]/floatGridLength)+0.5), (int)( ((xArray[tempArray[p]]+floatMaxRadius) /floatGridLength)+0.5), cvScalarTemp);
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

				for(int p=0;p<tempArray.size();p++) //grid map construction here!
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

					for(int p=0;p<tempArray.size();p++) //grid map construction here!
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
//cout<<"New Turn"<<endl;
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


	for(int i=0;i<SegmentIndexesVectorOfVectors.size();i++) //for printing an image. i is the y axis, j is the x axis. GIMP=(j,i)
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
		    cout<<" Valid Segment: "<<i<<endl;
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









