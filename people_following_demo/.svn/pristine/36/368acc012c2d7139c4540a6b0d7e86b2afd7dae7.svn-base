#ifndef PERSONFOLLOWERPROJECT_H_
#define PERSONFOLLOWERPROJECT_H_



//#include <cv.h> //older
//#include <opencv/cv.h> //old

#include <opencv/cv.h> //new
#include <sensor_msgs/LaserScan.h> //new

#include <vector>
using namespace std;
typedef vector <int> Segment;

void findSegmentsAndCreateCurrentGridMap(const vector<double> &xArray, const vector<double> &yArray, IplImage* imgGridMap, IplImage* imgGridMapIncludingSegmentNumbers, int &intNumOfSegments, vector <Segment>& SegmentIndexesVectorOfVectors);
double findDistanceBetweenTwoPoints(const double pt1[], const double pt2[]);
void calculateCandidateSegments(const IplImage* cleanedGridMapIncludingSegmentNumbers,  const vector <Segment> SegmentIndexesVectorOfVectors , const int intNumOfSegments, CvSize imgSize, Segment &validSegments);
void PrintMat(CvMat *A);

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in); //new
//void commandCallback(const std_msgs::StringConstPtr& msg);

#endif
