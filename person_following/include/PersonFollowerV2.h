#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h> //tf
#include <tf/transform_listener.h> //tf
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <visualization_msgs/Marker.h>

#include <ptu_46/PtuCmd.h>
#include <ptu_46/PtuLookAt.h>

//#include <opencv.h>
//#include <opencv/cv.h>
//#include <opencv2/cv.h>
//#include <opencv/highgui.h>
#include "/usr/include/opencv2/opencv.hpp"
#include "/usr/include/opencv2/highgui/highgui.hpp"
#include <sensor_msgs/Joy.h>

//#include <laser_geometry/laser_geometry.h>

#include <string>
using std::string;

//#include <person_recognition/PersonIdentifier.h>
//#include <person_recognition/PersonRecControl.h>
#include <person_following_msgs/initFollow.h>
#include <sound_play/sound_play.h>


//#include <actionlib/client/simple_action_client.h>
//#include <actionlib/client/terminal_state.h>
//
//#include <jeeves_actions/Listen_FollowMeAction.h>
//#include <jeeves_actions/Listen_WaitAction.h>
//#include <jeeves_actions/Listen_StopAction.h>

using namespace std;
//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PersonFollower
{
	typedef vector <int> Segment;
 protected:
	ros::NodeHandle nh_;
	ros::Subscriber command_subscriber_;
	ros::Subscriber laser_subscriber_;
	ros::Subscriber joy_sub_;
	ros::Publisher commander_pub;
	//ros::Publisher person_control_publisher_;
	ros::Publisher vis_pub;
	//ros::Subscriber person_subscriber_;
	ros::Timer timer10sec;
	ros::Timer timer4sec;
	ros::Timer periodicTimer1sec;
	ros::Subscriber person_following_msgs_subscriber_;
	ros::ServiceClient PTUClient;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;
	tf::TransformListener listener3;
	geometry_msgs::Twist cmd;

	ros::Publisher move_base_pub;

	sound_play::SoundClient sc;
	FILE* logfile;

	//actionlib::SimpleActionClient<jeeves_actions::Listen_FollowMeAction> follow_ac;
	//actionlib::SimpleActionClient<jeeves_actions::Listen_WaitAction> wait_ac;
	//actionlib::SimpleActionClient<jeeves_actions::Listen_StopAction> stop_ac;

	enum State{IDLE,NO_TARGET_TO_TRACK_MODE,ONE_LEG_TRACK_MODE,PERSON_DETECTION};
	State currentState;
	enum motionState{REGULAR_FOLLOW,GO_NEARBY_PERSON};
	motionState currentMotionState;
	CvFont font;
	CvSize imgSize;
	IplConvKernel* meineKernel;
	CvKalman* kalman;
	CvKalman* kalmanOdomFrame;
	IplImage* imgGridMap;
	IplImage* imgGridMapPrevious;
	IplImage* imgGridMapIncludingSegmentNumbers;
	IplImage* imgGridMapPreviousNegated;
	IplImage* cleanedGridMapIncludingSegmentNumbers;
	IplImage* DifferenceMap;
	IplImage* cleanedGridMap;
	IplImage* outImg;	

	int plotParam1;
	int plotParam2;
    float floatMaxRadius;
    float thres0,thres1,thres2,thres3,thres4;
    bool plotLaser;
    int intDataPoints;
    float maxDistanceToMatchPrediction;
    bool checkForPersonToStop;
    double extraVelocityBeCareful; //for peopleBot
    double distanceToKeepWithPerson;
    double tetaNearby;
    unsigned int velocityCounter;
int noDetectionCount;
    bool rvizMarkerDisplay;    
    bool followMeCommandReceptionEnable;
    bool stopCommandReceptionEnable;
    bool waitCommandReceptionEnable;

    int featureUpdateWindowSize;
    bool boolUseFixedFeatureParams;
    bool useSpeechRec;
    bool offlineTest;
    bool useOdomForSpeed;
    bool useVoice;
    bool logLaser;
    bool usePTU;
    bool inverted;
    bool isMasterLearntByRecognizer;
    bool waitForRecognitionCompleteFlag;
    bool useOdometry;
    bool useMoveBase;
    bool useVision;
    bool inhibitLinearMovement;
    bool inhibitAngularMovement;
    bool inhibitNavigationMovement;
    bool permanentInhibitNavigationMovement;
    bool enableComeNearbyBehavior;
    bool verbose;
    bool personrecognizerCallbackReception;
    bool searchTwoSegmentsEnable;
    bool decreaseRecognitionThresholdEnable;
    double defaultStartingRecognitionThreshold;
    double periodicRecThreshold;
    bool virgin;
    float floatGridLength;
    float floatUncertainty;
    int intNumOfSegments;
    float distanceTreshold;
    unsigned int numberOfPointsTreshold;
    bool boolFilterSegmentsAccordingToLengthInSegmentation;
    float maxEuclidianDistOfSegmentsThatCanBePersonCandids;
    float minEuclidianDistOfSegmentsThatCanBePersonCandids;
    double operatingFrequency;
    float deltaT;
    int minPixelsToCountAllowedForMovementDetection;
    float floatHitRatioTresholdForMovementDetection;
    double angularResolution;
    int fillingForImages;
    CvScalar cvScalarTemp;
    double coeff1;
    double coeff2;
	float qUncertainty;
	float rUncertainty;
	bool featWindowInitPhase;
	enum { valsFsize = 16 };
	enum { originSize = 2 };
	enum { featureSize = 3 };
	double originPtt[originSize];
	float valsF[valsFsize];
	int featWindowCounter;
	double featureMeans[featureSize];
	double featureVariances[featureSize];
	double featureTotals[featureSize];
	int targetNotMovingCounter;
	double VTotal;
    double rectangularBoxXover2sizeForMovDetection;
    double rectangularBoxYsizeForMovDetection;
    

    vector<double> doubleLaserDataXprevious;
	vector<double> doubleLaserDataYprevious;
	vector<double> doubleLaserDataX;
	vector<double> doubleLaserDataY;
	Segment segTemp;
	Segment bestSegment;
	vector<Segment> SegmentIndexesVectorOfVectors;
    
 public:

	PersonFollower();
	void searchForMovement();
	void joyCallback ( const sensor_msgs::Joy::ConstPtr& msg);
	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
	void commandCallback  (const std_msgs::String::ConstPtr& msg);
	void timer10secCallback(const ros::TimerEvent& e);
	void timer4secCallback(const ros::TimerEvent& e);
	void periodicTimer1secCallback(const ros::TimerEvent& e);
	void commandRobotForReactiveFollowing(double personAngle,double distToPerson,double yValue);
	void followPerson(double checkedCenterPointLaserFrame[]);
	bool checkVelocityAndStopRobotIfTargetStationary();
	void cvKalmanNoObservation(CvKalman* kal);
	bool detectPeople(double mahalanobisDistanceThreshold,bool useFixedLegParams,double centerPointToSearchInMyCoordinates[],double searchRadiusFromCenterPoint,bool takeDistanceIntoAccountInMahDist,vector<Segment>& RefinedSegmentIndexesVectorOfVectors,vector<double>& RefinedSegmentsDistanceTotals);

	void initVectors();
	void printMat(CvMat *A);
	double findDistanceBetweenTwoPoints(const double pt1[], const double pt2[]);
	void findSegmentsAndCreateCurrentGridMap(const vector<double> &xArray, const vector<double> &yArray, IplImage* imgGridMap, IplImage* imgGridMapIncludingSegmentNumbers, int &intNumOfSegments, vector <Segment>& SegmentIndexesVectorOfVectors);
	void calculateCandidateSegments(const IplImage* cleanedGridMapIncludingSegmentNumbers,  const vector <Segment> SegmentIndexesVectorOfVectors , const int intNumOfSegments, CvSize imgSize, Segment &validSegments);
	void pointTransformer(const tf::TransformListener& listener, double inX, double inY, double &outX, double &outY, bool verbose, string from, string to);
	//void personrecognizerCallback(person_recognition::PersonIdentifierConstPtr msg);
	void person_following_msgs_callback(const person_following_msgs::initFollowConstPtr& msg);
	void stopMotors();
	double calculateIAV(Segment segTemp);
	double calculateCircCriterion(Segment segTemp,double currentSegmentWidth);
	double calculateSegmentDistance(double currentSegmentWidth,double currentSegmentIAV,double currentSegmentCircCriterion,bool boolIncludePhysicalDistance, double physicalDist);
	void updateFeatureParameters(double currentSegmentWidth,double currentSegmentIAV,double currentSegmentCircCriterion,int featureUpdateWindowSize);
};
