#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>
#include <cv_bridge/CvBridge.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <omnimapper_msgs/LabeledPointingGesture.h>
#include <person_following_msgs/initFollow.h> 
#include <sound_play/sound_play.h>
#include <pthread.h>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include "SceneDrawer.h"

////
#if (XN_PLATFORM == XN_PLATFORM_MACOSX)
	#include <GLUT/glut.h>
#else
	#include <GL/glut.h>
#endif
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/String.h>
////

int masterUserID;
sensor_msgs::CvBridge bridge_;
IplImage* global_image;
ros::Subscriber rgbimage_sub_;
ros::Timer timer;
using std::string;
ros::Subscriber command_subscriber_;
//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
xn::Context g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator g_UserGenerator;
//xn::ImageGenerator g_ImageGenerator
ros::Publisher person_pub_;
ros::Publisher pointing_pub_;

pthread_mutex_t mutexMasterUserID=PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutexSc=PTHREAD_MUTEX_INITIALIZER;

XnBool g_bNeedPose = FALSE;
XnChar g_strPose[20] = "";
XnBool g_bDrawBackground = TRUE;
XnBool g_bDrawPixels = TRUE;
XnBool g_bDrawSkeleton = TRUE;
XnBool g_bPrintID = TRUE;
XnBool g_bPrintState = TRUE;

bool boolGestureWindowOpen=false;
string pointing_label_string; 

#if (XN_PLATFORM == XN_PLATFORM_MACOSX)
	#include <GLUT/glut.h>
#else
	#include <GL/glut.h>
#endif

#define GL_WIN_SIZE_X 720
#define GL_WIN_SIZE_Y 480

XnBool g_bPause = false;
XnBool g_bRecord = false;

XnBool g_bQuit = false;

sound_play::SoundClient* sc;
int flag_label_type;

void timerCallback(const ros::TimerEvent& event)
{
  // timer.stop();
 boolGestureWindowOpen=false;
 std::cout<<"Gesture Window Closed.."<<std::endl;
}







void CleanupExit()
{
	g_Context.Shutdown();

	exit (1);
}

// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
  printf("New User %d\n", nId);
	// New user found
	g_UserGenerator.GetSkeletonCap().StartTracking(nId);	
 	if (g_bNeedPose)
	{
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	}
	else
	{
	g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}
// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
  /*
  printf("Lost user %d\n", nId);
  pthread_mutex_lock(&mutexMasterUserID);
	if(nId==masterUserID)
	  {
	    masterUserID=-1;
	  }
	  pthread_mutex_unlock(&mutexMasterUserID);*/
}
// Callback: Detected a pose
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
  //printf("Pose %s detected for user %d\n", strPose, nId);
	g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
	g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}
// Callback: Started calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
  printf("Calibration started for user %d\n", nId);
}
// Callback: Finished calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie)
{
	if (bSuccess)
	{
	  /*
	      pthread_mutex_lock(&mutexSc);
	      sc->say("I see you");
	      pthread_mutex_unlock(&mutexSc);
	  */
		// Calibration succeeded
		printf("Calibration complete, start tracking user %d\n", nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
		/*
		pthread_mutex_lock(&mutexMasterUserID);
		if(masterUserID!=-1)
		  {
		    g_UserGenerator.GetSkeletonCap().StopTracking(masterUserID);

		    if (g_bNeedPose)
		      {
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, masterUserID);
		      }
		    else
		      {
			g_UserGenerator.GetSkeletonCap().RequestCalibration(masterUserID, TRUE);
		      }
		  }
		masterUserID=nId;
	       pthread_mutex_unlock(&mutexMasterUserID);*/
	}
	else
	{
		// Calibration failed
		//printf("Calibration failed for user %d\n", nId);
		if (g_bNeedPose)
		{
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		}
		else
		{
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
		}
	}
}

// this function is called each frame
void glutDisplay (void)
{
  ros::spinOnce();
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Setup the OpenGL viewpoint
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	xn::SceneMetaData sceneMD;
	xn::DepthMetaData depthMD;
	xn::ImageMetaData imageMD;
	g_DepthGenerator.GetMetaData(depthMD);
	
	glOrtho(0, depthMD.XRes(), depthMD.YRes(), 0, -1.0, 1.0);

	glDisable(GL_TEXTURE_2D);

	if (!g_bPause)
	{
		// Read next available data
		g_Context.WaitAndUpdateAll();
	}

		// Process the data
		g_DepthGenerator.GetMetaData(depthMD);
		g_UserGenerator.GetUserPixels(0, sceneMD);
		//g_ImageGenerator.GetMetaData(imageMD);
		DrawDepthMap(depthMD, sceneMD);

	glutSwapBuffers();
}

void glutIdle (void)
{
	if (g_bQuit) {
		CleanupExit();
	}

	// Display the frame
	glutPostRedisplay();
}

void glutKeyboard (unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
		CleanupExit();
	case 'b':
		// Draw background?
		g_bDrawBackground = !g_bDrawBackground;
		break;
	case 'x':
		// Draw pixels at all?
		g_bDrawPixels = !g_bDrawPixels;
		break;
	case 's':
		// Draw Skeleton?
		g_bDrawSkeleton = !g_bDrawSkeleton;
		break;
	case 'i':
		// Print label?
		g_bPrintID = !g_bPrintID;
		break;
	case 'l':
		// Print ID & state as label, or only ID?
		g_bPrintState = !g_bPrintState;
		break;
	case'p':
		g_bPause = !g_bPause;
		break;
	}
}
void glInit (int * pargc, char ** argv)
{
	glutInit(pargc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow ("Prime Sense User Tracker Viewer");
	//glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);

	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
}

#define SAMPLE_XML_PATH "config/pointing_gesture.xml"

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}

void rawImageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{
  //IplImage *cv_image = NULL;
  try
	    {
	      global_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
	
	      cvShowImage("Image window", global_image);
	      cvWaitKey(2);

	  }
  catch (sensor_msgs::CvBridgeException error)
    {
      ROS_ERROR("error");
    }
  
}


void commandCallback  (const std_msgs::String::ConstPtr& msg)
{
  ros::NodeHandle nh;
  //std::cout<<"Got command msg!"<<msg->data<<std::endl;
	char key0 = msg->data.c_str()[0];

	switch (key0)
	{

	case 'l':
	  flag_label_type=0;
	  boolGestureWindowOpen=true;
	  std::cout<<"Gesture Window Opened"<<std::endl;
	  pointing_label_string=msg->data.substr(1);
	  std::cout<<"Label: "<<pointing_label_string<<std::endl;
	  
	  timer = nh.createTimer(ros::Duration(60.0), timerCallback,true);				  timer.start();

	case 'o':
	  flag_label_type=1;
	  boolGestureWindowOpen=true;
	  std::cout<<"Gesture Window Opened"<<std::endl;
	  pointing_label_string=msg->data.substr(1);
	  std::cout<<"Label: "<<pointing_label_string<<std::endl;
	  
	  timer = nh.createTimer(ros::Duration(60.0), timerCallback,true);				  timer.start();



	case 'g': //message is to gesture module.
		if(msg->data.length()>2)
		{
			char key2 = msg->data.c_str()[2];
			switch (key2)
			{
			case 's':
			  boolGestureWindowOpen=true;
			  std::cout<<"Gesture Window Opened"<<std::endl;
			  pointing_label_string=msg->data.substr(3);
			  std::cout<<"Label: "<<pointing_label_string<<std::endl;
			  timer = nh.createTimer(ros::Duration(60.0), timerCallback,true);				  timer.start();
			  break;
			case 'q':
			  //exit(0);
				break;
			default:
				break;
			}
		}

		break;
	default:
		break;
	}
}


int main(int argc, char **argv)
{
  ros::init (argc, argv, "pointing_gesture");
 
  ros::NodeHandle nh;
  sc = new sound_play::SoundClient();
  rgbimage_sub_ = nh.subscribe("camera/rgb/image_color", 1, rawImageCallback);
  command_subscriber_=nh.subscribe("/korus_control_topic",1,commandCallback);
  person_pub_=nh.advertise<person_following_msgs::initFollow>("person_following_topic",1);
  pointing_pub_=nh.advertise<omnimapper_msgs::LabeledPointingGesture>("pointing_gesture_topic",1);


  /* timer = nh.createTimer(ros::Duration(60.0), timerCallback,true);		        
     timer.stop();*/

	XnStatus nRetVal = XN_STATUS_OK;
	if (argc > 1)
	{
		nRetVal = g_Context.Init();
		CHECK_RC(nRetVal, "Init");
		nRetVal = g_Context.OpenFileRecording(argv[1]);
		if (nRetVal != XN_STATUS_OK)
		{
			printf("Can't open recording %s: %s\n", argv[1], xnGetStatusString(nRetVal));
			return 1;
		}
	}
	else
	{

	  string configFilename = ros::package::getPath ("openni_CogRos") + "/config/pointing_gesture.xml";
	  nRetVal = g_Context.InitFromXmlFile (configFilename.c_str ());
	  CHECK_RC (nRetVal, "InitFromXml");

		xn::EnumerationErrors errors;
		if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
		{
			XnChar strError[1024];
			errors.ToString(strError, 1024);
			printf("%s\n", strError);
			return (nRetVal);
		}
		else if (nRetVal != XN_STATUS_OK)
		{
			printf("Open failed: %s\n", xnGetStatusString(nRetVal));
			return (nRetVal);
		}
	}

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	CHECK_RC(nRetVal, "Find depth generator");
	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK)
	{
		nRetVal = g_UserGenerator.Create(g_Context);
		CHECK_RC(nRetVal, "Find user generator");
	}

	XnCallbackHandle hUserCallbacks, hCalibrationCallbacks, hPoseCallbacks;
	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
	{
		printf("Supplied user generator doesn't support skeleton\n");
		return 1;
	}
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);
		
	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
	{
	  g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
		{
			printf("Pose required, but not supported\n");
			return 1;
		}
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);
		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}
	
	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_UPPER); //ALL

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

	glInit(&argc, argv);
	ros::spinOnce();

	pthread_mutex_lock(&mutexSc); 
	sc->say("Gestures Active");
	pthread_mutex_unlock(&mutexSc);

	glutMainLoop();

}

#define MAX_DEPTH 10000
#define DIST_THRES_SQUARED 235.0
#define Y_ANGLE_SINE_THRESH 0.75
#define CONE_HEIGHT_SQUARED_MM 490000 //h=70cm
#define CYLINDER_RADIUS_SQUARED_MM 250000 //r=50cm
#define GESTURE_FRAME_DURATION 24

float g_pDepthHist[MAX_DEPTH];

extern ros::Publisher person_pub_;
extern ros::Publisher pointing_pub_;

unsigned int getClosestPowerOfTwo(unsigned int n)
{
	unsigned int m = 2;
	while(m < n) m<<=1;

	return m;
}
GLuint initTexture(void** buf, int& width, int& height)
{
	GLuint texID = 0;
	glGenTextures(1,&texID);

	width = getClosestPowerOfTwo(width);
	height = getClosestPowerOfTwo(height); 
	*buf = new unsigned char[width*height*4];
	glBindTexture(GL_TEXTURE_2D,texID);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	return texID;
}

GLfloat texcoords[8];
void DrawRectangle(float topLeftX, float topLeftY, float bottomRightX, float bottomRightY)
{
	GLfloat verts[8] = {	topLeftX, topLeftY,
		topLeftX, bottomRightY,
		bottomRightX, bottomRightY,
		bottomRightX, topLeftY
	};
	glVertexPointer(2, GL_FLOAT, 0, verts);
	glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

	//TODO: Maybe glFinish needed here instead - if there's some bad graphics crap
	glFlush();
}
void DrawTexture(float topLeftX, float topLeftY, float bottomRightX, float bottomRightY)
{
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glTexCoordPointer(2, GL_FLOAT, 0, texcoords);

	DrawRectangle(topLeftX, topLeftY, bottomRightX, bottomRightY);

	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
}

XnFloat Colors[][3] =
{
	{0,1,1},
	{0,0,1},
	{0,1,0},
	{1,1,0},
	{1,0,0},
	{1,.5,0},
	{.5,1,0},
	{0,.5,1},
	{.5,0,1},
	{1,1,.5},
	{1,1,1}
};
XnUInt32 nColors = 10;

void glPrintString(void *font, char *str)
{
	int i,l = strlen(str);

	for(i=0; i<l; i++)
	{
		glutBitmapCharacter(font,*str++);
	}
}

void DrawLimb(XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2)
{
	if (!g_UserGenerator.GetSkeletonCap().IsTracking(player))
	{
		printf("not tracked!\n");
		return;
	}

	XnSkeletonJointPosition joint1, joint2;
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint1, joint1);
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint2, joint2);

	if (joint1.fConfidence < 0.5 || joint2.fConfidence < 0.5)
	{
		return;
	}
	XnPoint3D pt[2];
	
	pt[0] = joint1.position;
	pt[1] = joint2.position;

	g_DepthGenerator.ConvertRealWorldToProjective(2, pt, pt);
	glVertex3i(pt[0].X, pt[0].Y, 0);
	glVertex3i(pt[1].X, pt[1].Y, 0);
}
struct Vec3
{
	float x;
	float y;
	float z;
};

float CylTest_CapsFirst(XnPoint3D  &pt1,XnPoint3D & pt2, float lengthsq, float radius_sq, XnPoint3D &testpt )
{
	float dx, dy, dz;	// vector d  from line segment point 1 to point 2
	float pdx, pdy, pdz;	// vector pd from point 1 to test point
	float dot, dsq;

	dx = pt2.X - pt1.X;	// translate so pt1 is origin.  Make vector from
	dy = pt2.Y - pt1.Y;     // pt1 to pt2.  Need for this is easily eliminated
	dz = pt2.Z - pt1.Z;

	pdx = testpt.X - pt1.X;		// vector from pt1 to test point.
	pdy = testpt.Y - pt1.Y;
	pdz = testpt.Z - pt1.Z;

	// Dot the d and pd vectors to see if point lies behind the 
	// cylinder cap at pt1.x, pt1.y, pt1.z

	dot = pdx * dx + pdy * dy + pdz * dz;

	// If dot is less than zero the point is behind the pt1 cap.
	// If greater than the cylinder axis line segment length squared
	// then the point is outside the other end cap at pt2.

	//	if( dot < 0.0f || dot > lengthsq )
	//std::cout<<"dot: "<<dot<<std::endl;	
	if( dot < 0.0f)
	  {
	    	    return( -1.0f );
	}
	else 
	{
		// Point lies within the parallel caps, so find
		// distance squared from point to line, using the fact that sin^2 + cos^2 = 1
		// the dot = cos() * |d||pd|, and cross*cross = sin^2 * |d|^2 * |pd|^2
		// Carefull: '*' means mult for scalars and dotproduct for vectors
		// In short, where dist is pt distance to cyl axis: 
		// dist = sin( pd to d ) * |pd|
		// distsq = dsq = (1 - cos^2( pd to d)) * |pd|^2
		// dsq = ( 1 - (pd * d)^2 / (|pd|^2 * |d|^2) ) * |pd|^2
		// dsq = pd * pd - dot * dot / lengthsq
		//  where lengthsq is d*d or |d|^2 that is passed into this function 

		// distance squared to the cylinder axis:

	  float pdsq= (pdx*pdx + pdy*pdy + pdz*pdz);
	  dsq = pdsq- dot*dot/lengthsq;
	  float hsq=pdsq-dsq;

	  //std::cout<<"h: "<<sqrt(hsq)/1000.0<<"m"<<std::endl;
	  //std::cout<<"d: "<<sqrt(dsq)/1000.0<<"m"<<std::endl;
	  
	  float radiusThresholdSq;
	  if(hsq<CONE_HEIGHT_SQUARED_MM) //in cone zone
	    {
	      radiusThresholdSq=radius_sq*(hsq/CONE_HEIGHT_SQUARED_MM);
	      //std::cout<<"IN CONE ZONE"<<std::endl;
	    }
	  else //in cylinder zone
	    {
	      radiusThresholdSq=radius_sq;
	      //std::cout<<"IN CYCLONE ZONE"<<std::endl;
	    }
	  
	  if( dsq > radiusThresholdSq )
	    {
	      return( -1.0f );
	    }
	  else
	    {
	      return( dsq );		// return distance squared to axis
	    }
	  
	}
}


bool DetectLeftArmPointingGesture(XnUserID player,XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2, bool resetGestureWhenDetected, bool drawLine, XnPoint3D &handPt, XnPoint3D &extensionPt)
{
  static float prevHandX=0.0;
  static float prevHandY=0.0;
  static float prevHandZ=0.0;
  static float prevElbowX=0.0;
  static float prevElbowY=0.0;
  static float prevElbowZ=0.0;
  static int counter=0; 

  if(!boolGestureWindowOpen)
    {
      counter=0;
      return false;
    }

  bool gestureContinued=false;

	if (!g_UserGenerator.GetSkeletonCap().IsTracking(player))
	  {
	    printf("not tracked!\n");
	    counter=0;
	    return false;
	  }
	
	XnSkeletonJointPosition joint1, joint2;
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint1, joint1);
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint2, joint2);
	
	if (joint1.fConfidence < 0.5 || joint2.fConfidence < 0.5)
	  {
	  counter=0;
	  return false;
	  }


	XnPoint3D pt[2];
	pt[0] = joint1.position; //elbow
	pt[1] = joint2.position; //hand
	
	
	
	float elbowDistDiff=(pt[0].X-prevElbowX)*(pt[0].X-prevElbowX)+(pt[0].Y-prevElbowY)*(pt[0].Y-prevElbowY)+(pt[0].Z-prevElbowZ)*(pt[0].Z-prevElbowZ);
	
	if(elbowDistDiff<DIST_THRES_SQUARED)
	  {
	    float handDistDiff=(pt[1].X-prevHandX)*(pt[1].X-prevHandX)+(pt[1].Y-prevHandY)*(pt[1].Y-prevHandY)+(pt[1].Z-prevHandZ)*(pt[1].Z-prevHandZ);

	    if(handDistDiff<DIST_THRES_SQUARED)
	    {
	      gestureContinued=true;
	    }
	  }  

	   
	float xIncCoeff=0.0;
	float yIncCoeff=0.0;
	float zIncCoeff=0.0;
	if(gestureContinued)
	  {   
	    float frontArmDist=sqrt((pt[0].X-pt[1].X)*(pt[0].X-pt[1].X)+(pt[0].Y-pt[1].Y)*(pt[0].Y-pt[1].Y)+(pt[0].Z-pt[1].Z)*(pt[0].Z-pt[1].Z));
	    
	    xIncCoeff=(pt[1].X-pt[0].X)/frontArmDist;
	    yIncCoeff=(pt[1].Y-pt[0].Y)/frontArmDist;
	    zIncCoeff=(pt[1].Z-pt[0].Z)/frontArmDist;	
	    
	    //std::cout<<xIncCoeff+yIncCoeff+zIncCoeff<<std::endl;
	    //std::cout<<"yCoeff: "<<abs(yIncCoeff)<<std::endl;
	   
	    if(yIncCoeff>0)
	      {
		if(yIncCoeff>Y_ANGLE_SINE_THRESH)	
		  {
		    gestureContinued=false;
		  }
	      }
	    else
	      {
		if(yIncCoeff<-Y_ANGLE_SINE_THRESH)	
		  {
		    gestureContinued=false;
		  }
	      }
	    
	  }

	prevElbowX=pt[0].X;
	prevElbowY=pt[0].Y;
	prevElbowZ=pt[0].Z;
	prevHandX=pt[1].X;
	prevHandY=pt[1].Y;
	prevHandZ=pt[1].Z;
	
	if(gestureContinued)
	  {
	    counter++;    
	    if(counter>GESTURE_FRAME_DURATION) //was 16
	      {
		std::cout<<counter<<std::endl;
		if(resetGestureWhenDetected)
		  {
		    counter=0;
		  }
		
		handPt=pt[1];
		float increment=500.0;
		for(int i=0;i<1;i++)
		  {
		    pt[1].X=pt[1].X+increment*xIncCoeff;
		    pt[1].Y=pt[1].Y+increment*yIncCoeff;
		    pt[1].Z=pt[1].Z+increment*zIncCoeff;
		  }	
		extensionPt=pt[1];	    
		
		//std::cout<<"elbow X: "<<pt[0].X<<" ,Y: "<<pt[0].Y<<" ,Z: "<<pt[0].Z<<std::endl;
		//std::cout<<"far X: "<<pt[1].X<<" ,Y: "<<pt[1].Y<<" ,Z: "<<pt[1].Z<<std::endl;
		/*
		if(drawLine)
		  {
		    pt[0]=handPt;
		    g_DepthGenerator.ConvertRealWorldToProjective(2, pt, pt);
		    
		    glColor3f(0.0f, 1.0f, 0.0f);
		    glVertex3i(pt[0].X, pt[0].Y, 0);
		    glVertex3i(pt[1].X, pt[1].Y, 0);
		  }
		*/


		omnimapper_msgs::LabeledPointingGesture msg;
		msg.header.frame_id = "/camera_depth_optical_frame";
		msg.header.stamp = ros::Time::now();
		msg.hand.translation.x = joint2.position.X*0.001;
		msg.hand.translation.y = -joint2.position.Y*0.001;
		msg.hand.translation.z = joint2.position.Z*0.001;
		msg.hand.rotation.x = 0.0;
		msg.hand.rotation.y = 0.0;
		msg.hand.rotation.z = 0.0;
		msg.hand.rotation.w = 1.0;

		msg.elbow.translation.x = joint1.position.X*0.001;
		msg.elbow.translation.y = -joint1.position.Y*0.001;
		msg.elbow.translation.z = joint1.position.Z*0.001;
		msg.elbow.rotation.x = 0.0;
		msg.elbow.rotation.y = 0.0;
		msg.elbow.rotation.z = 0.0;
		msg.elbow.rotation.w = 1.0;

		msg.label = pointing_label_string;
		msg.label_type=flag_label_type;
		printf ("PUBLISHING GESTURE MESSAGE!\n");
		pointing_pub_.publish(msg);
		boolGestureWindowOpen=false;
		
		pthread_mutex_lock(&mutexSc);
		sc->say("OK");
		pthread_mutex_unlock(&mutexSc);

		return true;
	      }  
	  }
	else
	  {
	    counter=0;
	    return false;
	  }
	return false;
}


bool DetectRightArmPointingGesture(XnUserID player,XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2, bool resetGestureWhenDetected, bool drawLine, XnPoint3D &handPt, XnPoint3D &extensionPt)
{
    static float prevHandX=0.0;
  static float prevHandY=0.0;
  static float prevHandZ=0.0;
  static float prevElbowX=0.0;
  static float prevElbowY=0.0;
  static float prevElbowZ=0.0;
  static int counter=0; 
 
if(!boolGestureWindowOpen)
    {
      counter=0;
      return false;
    }
 bool gestureContinued=false;

	if (!g_UserGenerator.GetSkeletonCap().IsTracking(player))
	  {
	    printf("not tracked!\n");
	    counter=0;
	    return false;
	  }
	
	XnSkeletonJointPosition joint1, joint2;
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint1, joint1);
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint2, joint2);
	
	if (joint1.fConfidence < 0.5 || joint2.fConfidence < 0.5)
	  {
	  counter=0;
	  return false;
	  }

	XnPoint3D pt[2];
	pt[0] = joint1.position; //elbow
	pt[1] = joint2.position; //hand
	
	//	std::cout<<"Hand X: "<<pt[1].X<<" ,Y: "<<pt[1].Y<<" ,Z: "<<pt[1].Z<<std::endl;	
	
	float elbowDistDiff=(pt[0].X-prevElbowX)*(pt[0].X-prevElbowX)+(pt[0].Y-prevElbowY)*(pt[0].Y-prevElbowY)+(pt[0].Z-prevElbowZ)*(pt[0].Z-prevElbowZ);
	
	if(elbowDistDiff<DIST_THRES_SQUARED)
	  {
	    float handDistDiff=(pt[1].X-prevHandX)*(pt[1].X-prevHandX)+(pt[1].Y-prevHandY)*(pt[1].Y-prevHandY)+(pt[1].Z-prevHandZ)*(pt[1].Z-prevHandZ);

	    if(handDistDiff<DIST_THRES_SQUARED)
	    {
	      gestureContinued=true;
	    }
	  }  

	   
	float xIncCoeff=0.0;
	float yIncCoeff=0.0;
	float zIncCoeff=0.0;
	if(gestureContinued)
	  {   
	    float frontArmDist=sqrt((pt[0].X-pt[1].X)*(pt[0].X-pt[1].X)+(pt[0].Y-pt[1].Y)*(pt[0].Y-pt[1].Y)+(pt[0].Z-pt[1].Z)*(pt[0].Z-pt[1].Z));
	    
	    xIncCoeff=(pt[1].X-pt[0].X)/frontArmDist;
	    yIncCoeff=(pt[1].Y-pt[0].Y)/frontArmDist;
	    zIncCoeff=(pt[1].Z-pt[0].Z)/frontArmDist;	
	    
	    //std::cout<<xIncCoeff+yIncCoeff+zIncCoeff<<std::endl;
	    //std::cout<<"yCoeff: "<<abs(yIncCoeff)<<std::endl;
	   
	    if(yIncCoeff>0)
	      {
		if(yIncCoeff>Y_ANGLE_SINE_THRESH)	
		  {
		    gestureContinued=false;
		  }
	      }
	    else
	      {
		if(yIncCoeff<-Y_ANGLE_SINE_THRESH)	
		  {
		    gestureContinued=false;
		  }
	      }
	    
	  }

	prevElbowX=pt[0].X;
	prevElbowY=pt[0].Y;
	prevElbowZ=pt[0].Z;
	prevHandX=pt[1].X;
	prevHandY=pt[1].Y;
	prevHandZ=pt[1].Z;
	
	if(gestureContinued)
	  {
	    counter++;    
	    if(counter>GESTURE_FRAME_DURATION) // was 16
	      {
		std::cout<<counter<<std::endl;
		if(resetGestureWhenDetected)
		  {
		    counter=0;
		  }
		

		omnimapper_msgs::LabeledPointingGesture msg;
		msg.header.frame_id = "/camera_depth_optical_frame";
		msg.header.stamp = ros::Time::now();
		msg.hand.translation.x = joint2.position.X*0.001;
		msg.hand.translation.y = -joint2.position.Y*0.001;
		msg.hand.translation.z = joint2.position.Z*0.001;
		msg.hand.rotation.x = 0.0;
		msg.hand.rotation.y = 0.0;
		msg.hand.rotation.z = 0.0;
		msg.hand.rotation.w = 1.0;

		msg.elbow.translation.x = joint1.position.X*0.001;
		msg.elbow.translation.y = -joint1.position.Y*0.001;
		msg.elbow.translation.z = joint1.position.Z*0.001;
		msg.elbow.rotation.x = 0.0;
		msg.elbow.rotation.y = 0.0;
		msg.elbow.rotation.z = 0.0;
		msg.elbow.rotation.w = 1.0;

		msg.label = pointing_label_string;
		msg.label_type=flag_label_type;
		printf("Publishing right arm gesture!\n");
		pointing_pub_.publish(msg);
		boolGestureWindowOpen=false;

		pthread_mutex_lock(&mutexSc);
		sc->say("OK");
		pthread_mutex_unlock(&mutexSc);

		handPt=pt[1];
		float increment=500.0;
		for(int i=0;i<1;i++)
		  {
		    pt[1].X=pt[1].X+increment*xIncCoeff;
		    pt[1].Y=pt[1].Y+increment*yIncCoeff;
		    pt[1].Z=pt[1].Z+increment*zIncCoeff;
		  }	
		extensionPt=pt[1];	    
		
		//std::cout<<"elbow X: "<<pt[0].X<<" ,Y: "<<pt[0].Y<<" ,Z: "<<pt[0].Z<<std::endl;
		//std::cout<<"far X: "<<pt[1].X<<" ,Y: "<<pt[1].Y<<" ,Z: "<<pt[1].Z<<std::endl;
		
		return true;
	      }
	    
	  }
	else
	  {
	    counter=0;
	    return false;
	  }
	return false;
}



void DrawPointingDirection(XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2)
{
	if (!g_UserGenerator.GetSkeletonCap().IsTracking(player))
	{
		printf("not tracked!\n");
		return;
	}

	XnSkeletonJointPosition joint1, joint2;
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint1, joint1);
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint2, joint2);

	if (joint1.fConfidence < 0.5 || joint2.fConfidence < 0.5)
	{
		return;
	}

	XnPoint3D pt[2];
	pt[0] = joint1.position; //elbow
	pt[1] = joint2.position; //hand
	
	
	float frontArmDist=sqrt((pt[0].X-pt[1].X)*(pt[0].X-pt[1].X)+(pt[0].Y-pt[1].Y)*(pt[0].Y-pt[1].Y)+(pt[0].Z-pt[1].Z)*(pt[0].Z-pt[1].Z));
	float xIncCoeff=(pt[1].X-pt[0].X)/frontArmDist;
	float yIncCoeff=(pt[1].Y-pt[0].Y)/frontArmDist;
	float zIncCoeff=(pt[1].Z-pt[0].Z)/frontArmDist;	
	float increment=500.0;
	for(int i=0;i<1;i++)
	  {
	    pt[1].X=pt[1].X+increment*xIncCoeff;
	    pt[1].Y=pt[1].Y+increment*yIncCoeff;
	    pt[1].Z=pt[1].Z+increment*zIncCoeff;
	  }
	
	//std::cout<<"X: "<<pt[1].X<<"Y: "<<pt[1].Y<<"Z: "<<pt[1].Z<<std::endl;

	g_DepthGenerator.ConvertRealWorldToProjective(2, pt, pt);
 el:

	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3i(pt[0].X, pt[0].Y, 0);
	glVertex3i(pt[1].X, pt[1].Y, 0);
}



void DrawDepthMap(const xn::DepthMetaData& dmd, const xn::SceneMetaData& smd)
{
	static bool bInitialized = false;	
	static GLuint depthTexID;
	static unsigned char* pDepthTexBuf;
	static int texWidth, texHeight;

	 float topLeftX;
	 float topLeftY;
	 float bottomRightY;
	 float bottomRightX;
	float texXpos;
	float texYpos;

	if(!bInitialized)
	{

		texWidth =  getClosestPowerOfTwo(dmd.XRes());
		texHeight = getClosestPowerOfTwo(dmd.YRes());

//		printf("Initializing depth texture: width = %d, height = %d\n", texWidth, texHeight);
		depthTexID = initTexture((void**)&pDepthTexBuf,texWidth, texHeight) ;

//		printf("Initialized depth texture: width = %d, height = %d\n", texWidth, texHeight);
		bInitialized = true;

		topLeftX = dmd.XRes();
		topLeftY = 0;
		bottomRightY = dmd.YRes();
		bottomRightX = 0;
		texXpos =(float)dmd.XRes()/texWidth;
		texYpos  =(float)dmd.YRes()/texHeight;

		memset(texcoords, 0, 8*sizeof(float));
		texcoords[0] = texXpos, texcoords[1] = texYpos, texcoords[2] = texXpos, texcoords[7] = texYpos;

	}
	unsigned int nValue = 0;
	unsigned int nHistValue = 0;
	unsigned int nIndex = 0;
	unsigned int nX = 0;
	unsigned int nY = 0;
	unsigned int nNumberOfPoints = 0;
	XnUInt16 g_nXRes = dmd.XRes();
	XnUInt16 g_nYRes = dmd.YRes();

	unsigned char* pDestImage = pDepthTexBuf;

	const XnDepthPixel* pDepth = dmd.Data();
	const XnLabel* pLabels = smd.Data();

	// Calculate the accumulative histogram
	memset(g_pDepthHist, 0, MAX_DEPTH*sizeof(float));
	for (nY=0; nY<g_nYRes; nY++)
	{
		for (nX=0; nX<g_nXRes; nX++)
		{
			nValue = *pDepth;

			if (nValue != 0)
			{
				g_pDepthHist[nValue]++;
				nNumberOfPoints++;
			}

			pDepth++;
		}
	}

	for (nIndex=1; nIndex<MAX_DEPTH; nIndex++)
	{
		g_pDepthHist[nIndex] += g_pDepthHist[nIndex-1];
	}
	if (nNumberOfPoints)
	{
		for (nIndex=1; nIndex<MAX_DEPTH; nIndex++)
		{
			g_pDepthHist[nIndex] = (unsigned int)(256 * (1.0f - (g_pDepthHist[nIndex] / nNumberOfPoints)));
		}
	}

	pDepth = dmd.Data();
	if (g_bDrawPixels)
	{
		XnUInt32 nIndex = 0;
		// Prepare the texture map
		for (nY=0; nY<g_nYRes; nY++)
		{
			for (nX=0; nX < g_nXRes; nX++, nIndex++)
			{

				pDestImage[0] = 0;
				pDestImage[1] = 0;
				pDestImage[2] = 0;
				if (g_bDrawBackground || *pLabels != 0)
				{
					nValue = *pDepth;
					XnLabel label = *pLabels;
					XnUInt32 nColorID = label % nColors;
					if (label == 0)
					{
						nColorID = nColors;
					}

					if (nValue != 0)
					{
						nHistValue = g_pDepthHist[nValue];
						
						pDestImage[0] = nHistValue * Colors[nColorID][0]; 
						pDestImage[1] = nHistValue * Colors[nColorID][1];
						pDestImage[2] = nHistValue * Colors[nColorID][2];
					}
				}

				pDepth++;
				pLabels++;
				pDestImage+=3;
			}

			pDestImage += (texWidth - g_nXRes) *3;
		}
	}
	else
	{
		xnOSMemSet(pDepthTexBuf, 0, 3*2*g_nXRes*g_nYRes);
	}

	glBindTexture(GL_TEXTURE_2D, depthTexID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texWidth, texHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, pDepthTexBuf);

	// Display the OpenGL texture map
	glColor4f(0.75,0.75,0.75,1);

	glEnable(GL_TEXTURE_2D);
	DrawTexture(dmd.XRes(),dmd.YRes(),0,0);	
	glDisable(GL_TEXTURE_2D);

	char strLabel[50] = "";
	XnUserID aUsers[15];
	XnUInt16 nUsers = 15;
	XnPoint3D coms[15];
	g_UserGenerator.GetUsers(aUsers, nUsers);
	//std::cout<<"Num Users: "<<nUsers<<std::endl;

	for (int j = 0; j < nUsers; ++j)
	{
	  XnPoint3D com;
	  g_UserGenerator.GetCoM(aUsers[j], com);
	  coms[j]=com;
	  //std::cout<<"User "<<j<<"=("<<com.X<<","<<com.Y<<","<<com.Z<<")"<<std::endl;
	  /*
	  if(com.X==0 && com.Y==0 && com.Z ==0) // point is null
	    {
	      
	    }
	  else
	    {
	      person_following_msgs::initFollow msg;
	      msg.command="Follow";
	      msg.definedFrame="kinect";
	      msg.x=-com.X/1000;
	      msg.y=com.Y/1000;
	      msg.z=com.Z/1000;
	      person_pub_.publish(msg);
	    }
	  */
	  
	}

	for (int i = 0; i < nUsers; ++i)
	{
		if (g_bPrintID)
		{
			XnPoint3D com;
			g_UserGenerator.GetCoM(aUsers[i], com);
			g_DepthGenerator.ConvertRealWorldToProjective(1, &com, &com);

			xnOSMemSet(strLabel, 0, sizeof(strLabel));
			if (!g_bPrintState)
			{
				// Tracking
				sprintf(strLabel, "%d", aUsers[i]);
			}
			else if (g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
			{
				// Tracking
				sprintf(strLabel, "%d - Tracking", aUsers[i]);
			}
			else if (g_UserGenerator.GetSkeletonCap().IsCalibrating(aUsers[i]))
			{
				// Calibrating
				sprintf(strLabel, "%d - Calibrating...", aUsers[i]);
			}
			else
			{
				// Nothing
				sprintf(strLabel, "%d - Looking for pose", aUsers[i]);
			}


			glColor4f(1-Colors[i%nColors][0], 1-Colors[i%nColors][1], 1-Colors[i%nColors][2], 1);

			glRasterPos2i(com.X, com.Y);
			glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);
		}

		if (g_bDrawSkeleton && g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
		{
		  glLineWidth(33.0f);
			glBegin(GL_LINES);
			glColor4f(1-Colors[aUsers[i]%nColors][0], 1-Colors[aUsers[i]%nColors][1], 1-Colors[aUsers[i]%nColors][2], 1);
			DrawLimb(aUsers[i], XN_SKEL_HEAD, XN_SKEL_NECK);

			DrawLimb(aUsers[i], XN_SKEL_NECK, XN_SKEL_LEFT_SHOULDER);
			DrawLimb(aUsers[i], XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW);
			DrawLimb(aUsers[i], XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND);

			DrawLimb(aUsers[i], XN_SKEL_NECK, XN_SKEL_RIGHT_SHOULDER);
			DrawLimb(aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW);
			DrawLimb(aUsers[i], XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND);

			DrawLimb(aUsers[i], XN_SKEL_LEFT_SHOULDER, XN_SKEL_TORSO);
			DrawLimb(aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_TORSO);

			DrawLimb(aUsers[i], XN_SKEL_TORSO, XN_SKEL_LEFT_HIP);
			DrawLimb(aUsers[i], XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE);
			DrawLimb(aUsers[i], XN_SKEL_LEFT_KNEE, XN_SKEL_LEFT_FOOT);

			DrawLimb(aUsers[i], XN_SKEL_TORSO, XN_SKEL_RIGHT_HIP);
			DrawLimb(aUsers[i], XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE);
			DrawLimb(aUsers[i], XN_SKEL_RIGHT_KNEE, XN_SKEL_RIGHT_FOOT);

			DrawLimb(aUsers[i], XN_SKEL_LEFT_HIP, XN_SKEL_RIGHT_HIP);

			glLineWidth(25.0);

			XnPoint3D leftExtensionPt;
			XnPoint3D rightExtensionPt;
			XnPoint3D leftHandPt;
			XnPoint3D rightHandPt;
			bool drawLine=true;

			bool rightPointingSuccess=DetectRightArmPointingGesture(aUsers[i],XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND,true,true,rightHandPt,rightExtensionPt);
			if(false && rightPointingSuccess)
			  {
			    std::vector<int> candidLabels;
			    std::vector<XnPoint3D> candidCOMs;
			    std::vector<float> candidDistances;
			    
			    for (int j = 0; j < nUsers; ++j)
			      {
				std::cout<<"MASTER: "<<masterUserID<<std::endl;				
				if(aUsers[j]!=masterUserID) //do if it's not master
				  {
				    XnPoint3D com;
				    com=coms[j];
				    
				    if(com.X==0 && com.Y==0 && com.Z ==0) // point is null
				      {
					
				      } 
				    else
				      {
					com.Y=com.Y+430;
					//std::cout<<"User"<<j<<" COM=("<<com.X<<","<<com.Y<<","<<com.Z<<")"<<std::endl;
					//std::cout<<"Elbow: "<<leftElbowPt.X<<","<<leftElbowPt.Y<<","<<leftElbowPt.Z<<std::endl;
					//std::cout<<"Hand: "<<leftHandPt.X<<","<<leftHandPt.Y<<","<<leftHandPt.Z<<std::endl;
					
					float lengthsq=(rightExtensionPt.X-rightHandPt.X)*(rightExtensionPt.X-rightHandPt.X)+(rightExtensionPt.Y-rightHandPt.Y)*(rightExtensionPt.Y-rightHandPt.Y)+(rightExtensionPt.Z-rightHandPt.Z)*(rightExtensionPt.Z-rightHandPt.Z);
					float dist=CylTest_CapsFirst(rightHandPt,rightExtensionPt,lengthsq,CYLINDER_RADIUS_SQUARED_MM,com);
					//std::cout<<"User "<<j<<"dist: "<<dist<<std::endl;
					//float CylTest_CapsFirst(Vec3  &pt1, const Vec3 & pt2, float lengthsq, float radius_sq, const Vec3 & testpt )
					if(dist>-1)
					  {
					    candidLabels.push_back(aUsers[j]);
					    candidCOMs.push_back(com);
					    candidDistances.push_back(dist);
					  }
				      }
				  }				
			      }			    
			    
			    int bestLabel;
			    XnPoint3D bestCOM;
			    float minDist;
			    for(unsigned int a=0;a<candidLabels.size();a++)
			      {
				if(a==0)
				  {
				    bestLabel=candidLabels[0];
				    bestCOM=candidCOMs[0];
				    minDist=candidDistances[0];
				  }
				else
				  {
				    if(candidDistances[a]<minDist)
				      {
					bestLabel=candidLabels[a];
					bestCOM=candidCOMs[a];
					minDist=candidDistances[a];
				      }
				  }
			      }
			    
			    if(!candidLabels.empty())
			      {
				std::cout<<"LEFT ARM to: "<<bestLabel<<std::endl;
				if(drawLine)
				  {
				    XnPoint3D pt[2];
				    pt[1]=bestCOM;
				    pt[0]=rightHandPt;
				    g_DepthGenerator.ConvertRealWorldToProjective(2, pt, pt);
				    glColor3f(0.0f, 1.0f, 0.0f);
				    glVertex3i(pt[0].X, pt[0].Y, 0);
				    glVertex3i(pt[1].X, pt[1].Y, 0);
				  }

				person_following_msgs::initFollow msg;
				msg.command="Follow";
				msg.definedFrame="kinect";
				msg.x=-bestCOM.X/1000;
				msg.y=bestCOM.Y/1000;
				msg.z=bestCOM.Z/1000;
			        person_pub_.publish(msg);

			      }

			  }
			


			bool leftPointingSuccess=DetectLeftArmPointingGesture(aUsers[i],XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND,true,true,leftHandPt,leftExtensionPt);						
			

			if(false && leftPointingSuccess)
			  {
			    std::vector<int> candidLabels;
			    std::vector<XnPoint3D> candidCOMs;
			    std::vector<float> candidDistances;


			    for (int j = 0; j < nUsers; ++j)
			      {
				std::cout<<"MASTER: "<<masterUserID<<std::endl;				
				if(aUsers[j]!=masterUserID) //do if it's not master
				  {
				    XnPoint3D com;
				    com=coms[j];
				    
				    if(com.X==0 && com.Y==0 && com.Z ==0) // point is null
				      {
					
				      } 
				    else
				      {
					com.Y=com.Y+430;
					//std::cout<<"User"<<j<<" COM=("<<com.X<<","<<com.Y<<","<<com.Z<<")"<<std::endl;
					//std::cout<<"Elbow: "<<leftElbowPt.X<<","<<leftElbowPt.Y<<","<<leftElbowPt.Z<<std::endl;
					//std::cout<<"Hand: "<<leftHandPt.X<<","<<leftHandPt.Y<<","<<leftHandPt.Z<<std::endl;
					
					float lengthsq=(leftExtensionPt.X-leftHandPt.X)*(leftExtensionPt.X-leftHandPt.X)+(leftExtensionPt.Y-leftHandPt.Y)*(leftExtensionPt.Y-leftHandPt.Y)+(leftExtensionPt.Z-leftHandPt.Z)*(leftExtensionPt.Z-leftHandPt.Z);
					float dist=CylTest_CapsFirst(leftHandPt,leftExtensionPt,lengthsq,CYLINDER_RADIUS_SQUARED_MM,com);
					//std::cout<<"User "<<j<<"dist: "<<dist<<std::endl;
					//float CylTest_CapsFirst(Vec3  &pt1, const Vec3 & pt2, float lengthsq, float radius_sq, const Vec3 & testpt )
					if(dist>-1)
					  {
					    candidLabels.push_back(aUsers[j]);
					    candidCOMs.push_back(com);
					    candidDistances.push_back(dist);
					  }
				      }
				  }				
			      }			    


			    int bestLabel;
			    XnPoint3D bestCOM;
			    float minDist;
			    for(unsigned int a=0;a<candidLabels.size();a++)
			      {
				if(a==0)
				  {
				    bestLabel=candidLabels[0];
				    bestCOM=candidCOMs[0];
				    minDist=candidDistances[0];
				  }
				else
				  {
				    if(candidDistances[a]<minDist)
				      {
					bestLabel=candidLabels[a];
					bestCOM=candidCOMs[a];
					minDist=candidDistances[a];
				      }
				  }
			      }
			    
			    if(!candidLabels.empty())
			      {
				std::cout<<"RIGHT ARM to: "<<bestLabel<<std::endl;
				//TODO:send person location msg
			
				person_following_msgs::initFollow msg;
				msg.command="Follow";
				msg.definedFrame="kinect";
				msg.x=-bestCOM.X/1000;
				msg.y=bestCOM.Y/1000;
				msg.z=bestCOM.Z/1000;
			        person_pub_.publish(msg);

				if(drawLine)
				  {
				    XnPoint3D pt[2];
				    pt[1]=bestCOM;
				    pt[0]=leftHandPt;
				    g_DepthGenerator.ConvertRealWorldToProjective(2, pt, pt);
				    glColor3f(0.0f, 1.0f, 0.0f);
				    glVertex3i(pt[0].X, pt[0].Y, 0);
				    glVertex3i(pt[1].X, pt[1].Y, 0);
				  }
			      }

			    
			  }
			
			glEnd();
		}
	}
	
}
