#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cstdlib>
#include <vector>

using std::rand;
using std::srand;
using std::string;
using std::vector;

class Person
{
private:
	string personName;
	CvHistogram* shirtHistogram;
	CvScalar randomColor;
	vector<double> eigValAvgArr;
	vector<double> eigValVarArr;

public:
	Person();
	Person(string name);
	void setPersonName(string name);
	string getPersonName();
	void displayPersonName();
	void setShirtHistogram(CvHistogram* newHistogram);
	CvHistogram* getShirtHistogram();
	CvScalar getPersonColor();
	void initShirtHistogram();
	void updateShirtHistogram(CvHistogram* detectedHist,float updateRate);
	void createRandomCvScalar();
	void setEigenValueAveragesArray(const vector<double> &newVector);
	void setEigenValueVarianceArray(vector<double>& newVector);
	vector<double> getEigenValueAveragesArray();
	vector<double> getEigenValueVarianceArray();

	bool loadedFromDatabase;
	bool shirtHistogramInitialized;
	int h_bins;
	int s_bins;

};
