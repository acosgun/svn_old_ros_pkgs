#include <iostream>
using std::cout;
using std::endl;
#include <Person.h>


Person::Person()
{
	vector<double> eigValAvgArr(1);
	vector<double> eigValVarArr(1);
    h_bins = 40;
    s_bins = 40;
    loadedFromDatabase=false;
    shirtHistogramInitialized=false;
	setPersonName("user");
	createRandomCvScalar();

}
Person::Person(string name)
{
    h_bins = 40;
    s_bins = 40;
    loadedFromDatabase=false;
    shirtHistogramInitialized=false;
	setPersonName(name);
	displayPersonName();
	createRandomCvScalar();
}
void Person::setPersonName(string name)
{
	personName=name;
}
string Person::getPersonName()
{
	return personName;
}
void Person::displayPersonName()
{
	cout<<"This person's name: "<<getPersonName()<<endl;
}
void Person::setShirtHistogram(CvHistogram* newHistogram)
{
	cvCopyHist(newHistogram,&shirtHistogram);

	//shirtHistogram=newHistogram;
}
CvHistogram* Person::getShirtHistogram()
{
	return shirtHistogram;
}
void Person::updateShirtHistogram(CvHistogram* detectedHist,float updateRate)
{
	//shirtHistogram=setShirtHistogram(detectedHist);
	//cvCopyHist(detectedHist,&shirtHistogram);
/*	float max_value = 0;
	cvGetMinMaxHistValue( detectedHist, 0, &max_value, 0, 0 );
	cout<<""*/

	for (int h = 0; h < detectedHist->mat.dim[0].size; h++)
	{
		for (int w = 0; w < detectedHist->mat.dim[1].size; w++)
		{
			float* detectedValPtr=cvGetHistValue_2D(detectedHist,h,w);
			float* shirtValPtr=cvGetHistValue_2D(shirtHistogram,h,w);
			float detectedVal=*detectedValPtr;
			float shirtVal=*shirtValPtr;

			*shirtValPtr=(float)((1-updateRate)*(shirtVal)+updateRate*detectedVal);

		}
	}
	cvNormalizeHist( shirtHistogram, 1 ); // Normalize it!
}
void Person::initShirtHistogram()
{
		int hist_size[] = { h_bins, s_bins };
		float h_ranges[] = { 0, 255.0};
		float s_ranges[] = { 0, 255.0};
		float* ranges[] = { h_ranges, s_ranges };
		shirtHistogram = cvCreateHist( 2, hist_size, CV_HIST_ARRAY, ranges, 1 );
		shirtHistogramInitialized=true;
}
void Person::createRandomCvScalar()
{
    srand(time(0));
	randomColor=cvScalar(rand()%255,rand()%255,rand()%255);
}
CvScalar Person::getPersonColor()
{
	return randomColor;
}

void Person::setEigenValueAveragesArray(const vector<double> &newVector)
{
	eigValAvgArr.clear();
	eigValAvgArr=newVector;

}
void Person::setEigenValueVarianceArray(vector<double>& newVector)
{
	eigValVarArr.clear();
	eigValVarArr=newVector;
}
vector<double> Person::getEigenValueAveragesArray()
{
	return eigValAvgArr;
}
vector<double> Person::getEigenValueVarianceArray()
{
	return eigValVarArr;
}
