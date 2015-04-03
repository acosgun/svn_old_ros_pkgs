#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <person_recognition/PersonIdentifier.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>

//#include "/home/akan/ros/pkgs/vision_opencv/opencv2/opencv/include/opencv/cv.h"
//#include "/home/akan/ros/pkgs/vision_opencv/opencv2/opencv/include/opencv/highgui.h"

#include <Person.h>

#include <fstream>
using std::ifstream;

using namespace std;


class PersonRecognizer
{

 protected:
  ros::NodeHandle nh_;
  ros::Subscriber image_subscriber_;
  ros::Subscriber command_subscriber_;
  ros::Publisher person_publisher_;
  ros::Publisher command_generator_;
  const char* cascade_name;
  CvFont font;
  CvHaarClassifierCascade* cascade;
  enum State{IDLE,LEARN_PERSON,RECOGNIZE_PEOPLE_FROM_FACES,QUIT,FACE_DETECTION_TEST,GRAB_NEXT_IMAGE,CONTINIOUS_CAPTURE_MODE};
  State currentState;
  IplImage* global_image;
  float faceToTorsoLengthFactor;
  vector<Person> personList;
  double learningRate;
  double scoringBiasTowardsFaceRecog;
  float tresholdForRatioTest;
  float limitRatioForFaceDetectionInTraining;
  Person candidatePerson;
  bool trainForJustOneImage;
  double THRESHOLD_FOR_PERSON_RECOGNITION;
  double THRESHOLD_FOR_RATIO_TEST;
  string searchedPersonsName; //1."SEARCH_EVERYBODY"=searches known&unknown ppl.For unknown, name is tagged "UNKNOWN" 2."SEARCH_ONLY_KNOWN"=searches only known ppl. 3. "someName"=only searches the person called "someName"
  string learnedPersonsName;
  string everyoneStr;
  string knownStr;
  string unknownStr;


//  bool useProsilica;
  bool verbose;
  bool showImages;
  int trainHowManyImages;
  bool enableVarianceNormalization;
  bool learnStateVirgin;
  bool isNameNeeded;
  int maxNumberOfFaceTrainImages;
  int currentNumberOfFaceImagesSaved;
  int trainingImageSize;
  int nEigens;
  IplImage** faceImgArray;
  IplImage** eigenVectArray;
  IplImage* pAvgTrainImg;
  CvMat* eigenValMat;
  CvSize faceImgSize;
  string packageDir;
  char * packageDirectory;
int faceDetectionQualityWhenLearning;
int faceDetectionQualityWhenRecognition;




 public:
  PersonRecognizer();
  IplImage* grabNextImage();
  IplImage* grabLatestImage();
  State getState();
  void setState(State newState);

  void printMat(CvMat *A);
  void addPerson(Person newPerson);
  void deletePerson(string name);
  vector<Person> getPersonList();
  void printPersonList();

  void setSearchedPersonsName(string name);
  string getSearchedPersonsName();

  void commandCallback  (const std_msgs::String::ConstPtr& msg);
  void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
  void detect_and_draw( IplImage* image );
  void findFaces(IplImage* img, int flags, int min_neighbors, vector<CvRect>& faceRectanglesVector);
  void findHistogramOfRectangle(IplImage* img1, CvRect rectangle, CvHistogram* hist);
  double compareTwoHistograms(CvHistogram* hist1,CvHistogram* hist2);
  void calculateNormalizedRGB(IplImage* srcImg,IplImage* dstImg);
  void loadFaceImageArray();
  void loadPersonListFromDatabase();
  void savePersonToDatabase(Person newPerson);
  void doPCA();
  double findMahalanobisDistance(float* projectedTestFace,vector<double> meanArr,vector<double> varArray);
  double getRecognitionThreshold();
  void setRecognitionThreshold(double setValue);
  void increaseRecognitionThreshold(double addedValue);
//  bool ReceiveImage(const sensor_msgs::ImageConstPtr& img,const sensor_msgs::CameraInfoConstPtr& cam_info);
  int h_bins;
  int s_bins;
  int h_max;
  int s_max;

  void timerCallback(const ros::TimerEvent& e);

};
