/*
 * Copyright (c) 2006-2009 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "FeatureExtraction.hpp"
#include "FeatureExtractionWorkerThreadInfo.h"

#ifdef USE_MARSS
#warning "Using MARSS"
#include "ptlcalls.h"
#endif
#ifdef USE_GEM5
#warning "Using GEM5"
extern "C"
{
	#include "m5op.h"
}
#endif


//#define TSC_TIMING
#ifdef TSC_TIMING
#include "tsc_class.hpp"

#endif
//#define CLOCK_GETTIME_TIMING
#ifdef CLOCK_GETTIME_TIMING
#include "time.h"
#ifndef GET_TIME_WRAPPER
#define GET_TIME_WRAPPER(dataStruct) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &dataStruct)
#endif
#endif



#ifdef TSC_TIMING
vector<TSC_VAL_w> fe_timingVector;
#endif

#ifdef CLOCK_GETTIME_TIMING
vector<struct timespec> fe_timeStructVector;
#endif


//#define DEBUG_OUTPUT
static const double pi = 3.14159265358979323846;

vector<clock_t> timingArray(12);


struct HoughBin
{
public:
	vector<int> keypointIndices;
	int scaleIndex;
	int orientationIndex;
	int xIndex;
	int yIndex;


	int scaleCenter;
	int orientationCenter;
	int xCenter;
	int yCenter;


	int count;

	friend ostream& operator<<(ostream& stream, const HoughBin bin);
};

ostream & operator<<(ostream& stream, const HoughBin bin)
{

	stream <<"X Index: "<<bin.xIndex <<", Y Index: "<< bin.yIndex <<", Scale Index: "<< bin.scaleIndex <<", Orientation Index: "<<bin.orientationIndex <<
		", Count: " << bin.count<< endl;
	stream << "Key Point Indices: (";
	for(unsigned int i=0; i<bin.keypointIndices.size(); i++)
	{
		if(i!=0)
		{
			stream << ",";

		}
		stream << bin.keypointIndices[i];
		
	}
	stream << ")" << endl;
	return stream;
}

void drawTextOnImage(string newText, Point location, Scalar color, Mat & image);
int getNextImages(vector<Mat>&  images,FeatureExtractionConfig & featureExtractionConfig);
Ptr<FeatureDetector> setupLocalization(FeatureExtractionConfig & featureExtractionConfig);
void LocalizeFeaturePoints(Mat & inputImage, vector<KeyPoint> & keyPoints,FeatureExtractionConfig & featureExtractionConfig,Ptr<FeatureDetector> detector );
void LocalizeFeaturePoints(Mat & inputImage, vector<KeyPoint> & keyPoints, vector<Rect>& bboxes,FeatureExtractionConfig & featureExtractionConfig);
void LocalizeFeaturePoints(Mat & inputImage, vector<KeyPoint> & keyPoints, FeatureExtractionConfig & featureExtractionConfig, Mat & descriptors);
void LocalizeFeaturePoints(Mat & inputImage, vector<KeyPoint> & keyPoints, vector<Rect>& bboxes,FeatureExtractionConfig & featureExtractionConfig,FeatureExtractionData & featureExtractionData);

Ptr<DescriptorMatcher> setupMatcher(FeatureExtractionConfig & featureExtractionConfig);
Ptr<DescriptorMatcher> setupMatcher(FeatureExtractionConfig & featureExtractionConfig, vector<Mat>& trainedDescriptors);
Ptr<DescriptorExtractor> setupDescriptor(FeatureExtractionConfig & featureExtractionConfig);
bool loadFeatureDescriptorsAndOrKeypoints( string filename,  string &descTypeString,string &localTypeString ,FeaturesDescriptorAlgos& descriptorType,FeaturesLocalizationAlgos& localizationMethod,Ptr<FeatureDetector>& detector,vector<KeyPoint> * keypoints,Ptr<DescriptorExtractor> & descriptorExtractor, Mat * descriptors);
bool loadFeatureDescriptorsAndOrKeypoints( string filename,  string &descTypeString,string &localTypeString ,FeaturesDescriptorAlgos& descriptorType,FeaturesLocalizationAlgos& localizationMethod,Ptr<FeatureDetector> &detector,vector<KeyPoint> * keypoints,HOGDescriptor*  &descriptorExtractor, Mat * descriptors);

bool saveFeatureDescriptorsAndOrKeypoints(string filename, FeaturesDescriptorAlgos descriptorType,FeaturesLocalizationAlgos localizationMethod, Ptr<FeatureDetector> detector,vector<KeyPoint> & keypoints,Ptr<DescriptorExtractor> descriptorExtractor, Mat & descriptors);
bool saveFeatureDescriptorsAndOrKeypoints( string filename,  FeaturesDescriptorAlgos descriptorType,FeaturesLocalizationAlgos localizationMethod,Ptr<FeatureDetector> detector,vector<KeyPoint> & keypoints,HOGDescriptor* descriptorExtractor, Mat & descriptors);
Mat buildFeatureDescriptors(Mat & inputImage, vector<KeyPoint> & keyPoints,FeatureExtractionConfig & featureExtractionConfig,Ptr<DescriptorExtractor> descriptorExtractor);
Mat buildFeatureDescriptors(Mat & inputImage, vector<KeyPoint> & points,vector<Rect> & bboxes,FeatureExtractionConfig & featureExtractionConfig,HOGDescriptor * descriptorExtractor);

KeyPoint calculateHogCenterLocation(Mat & inputImage, HOGDescriptor * hogDescPtr);
vector<DMatch> performFeatureMatching(Ptr<DescriptorMatcher> descriptorMatcher, Mat & descriptors, Mat & trainedDescriptors,vector<KeyPoint> & keyPoints, vector<KeyPoint> &trainedKeyPoints);


void paintCenterOnImage(Mat & image, vector<KeyPoint> & keypoints, Mat & descriptors);
void filterMatchesAndGetTransforms(vector<Mat> & transformsList, vector<Point2f> &refPoints,const vector<DMatch> & matches,  const vector<KeyPoint> queryKeypoints,const vector<KeyPoint> trainedKeypoints, const vector<Vec2f> trainedRefVectors, const Point2f & trainedRefPoint);
Point2f computeCenterPointOfKeyPoints(vector<KeyPoint> & keypoints);
vector<Vec2f> computeRelativeCenterPointVectorsForKeypoints(vector<KeyPoint> & keypoints);





void readFileListFromFile(string fileListFilename, string filesBaseDir,vector<string> &fileList)
{
	ifstream fileListFile;

	fileListFile.open(fileListFilename.c_str(),ios::in);


	if(fileListFile.is_open())
	{
		while(fileListFile.good())
		{
			string currentLine;

			getline(fileListFile,currentLine);
			if(currentLine != "")
			{
				fileList.push_back(filesBaseDir+currentLine);
			}
		}

		fileListFile.close();
	}
	cout << "Number of images: " << fileList.size() << endl;

}

bool loadFeatureExtractionConfigFile(vector<string> &commandArgs, string filename)
{

	std::ifstream configFile;
	int i = 3;
	string tmpString;
	bool returnVal = false;


	configFile.open(filename.c_str());


	if(configFile.good())
	{

		while(!configFile.eof())
		{
			getline(configFile,tmpString);
			commandArgs.push_back(tmpString);

		}
		returnVal = true;

	}
	else
	{

		cout << "WARNING: Unable to open classification params config file" << endl;

	}
	return returnVal;

}



void parseClassificationConfigCommandVector(FeatureExtractionConfig & featureExtractionConfig, vector<string> & commandStringVector)
{



	featureExtractionConfig.loadTrainedDescriptors = false;
	featureExtractionConfig.trainedDescriptorsLoaded = false;
	featureExtractionConfig.trainedDescriptorsAvailable = false;
	featureExtractionConfig.runSiftLocal=false;
	featureExtractionConfig.runSiftDesc=false;

	featureExtractionConfig.runFastLocal=false;
	featureExtractionConfig.runFastDesc=false;

	featureExtractionConfig.runSlidingWindowLocal=false;
	featureExtractionConfig.runHogLocal=false;
	featureExtractionConfig.runHogDesc=false;

	featureExtractionConfig.runSurfLocal=false;
	featureExtractionConfig.runSurfDesc=false;

	featureExtractionConfig.runBriefDesc=false;

	featureExtractionConfig.snapShotMode = false;

	featureExtractionConfig.accumulateDesc = false;
	featureExtractionConfig.saveAllDescToSingleFile = false;
	featureExtractionConfig.redistribute = false;

	featureExtractionConfig.leftImageCapture = NULL;
	featureExtractionConfig.rightImageCapture = NULL;
	featureExtractionConfig.kinectImageCapture = NULL;

	featureExtractionConfig.numberOfImages = -1;
	featureExtractionConfig.imageNameBase = EXTRACT_CONFIG_DEFAULT_FILENAME_BASE;
	featureExtractionConfig.imageId = EXTRACT_CONFIG_DEFAULT_FILENAME_INITIAL_ID;
	featureExtractionConfig.imageExtension = EXTRACT_CONFIG_DEFAULT_FILENAME_EXT;
	featureExtractionConfig.videoFileName = "";
	featureExtractionConfig.currentImageId = 0;
	featureExtractionConfig.currentOutputImageNumber = 0;

	featureExtractionConfig.imageListBaseDir = "";
	featureExtractionConfig.imageListFilename = "";

	featureExtractionConfig.descOutputFilename = "defaultDescFilename.yml";

	featureExtractionConfig.leftCameraId = EXTRACT_CONFIG_DEFAULT_LEFT_CAMERA_ID;
	featureExtractionConfig.rightCameraId = EXTRACT_CONFIG_DEFAULT_RIGHT_CAMERA_ID;
	featureExtractionConfig.inputMethod = EXTRACT_CONFIG_DEFAULT_INPUT_METHOD;
	featureExtractionConfig.currentFeatureMatchingAlgo = FEATURE_MATCHER_FLANN;

	featureExtractionConfig.hogMultiScale = false;
	featureExtractionConfig.addCenterLocation = false;
	featureExtractionConfig.descriptorsFilename = "";
	featureExtractionConfig.descriptorsImageFilename = "";

	featureExtractionConfig.singleCopyOfConfigStruct = true;
	featureExtractionConfig.numberOfHorizontalProcs = 1;
	featureExtractionConfig.numberOfVerticalProcs = 1;

	featureExtractionConfig.showWindows = true;//false;
	featureExtractionConfig.noWaitKey = false;//true;

	featureExtractionConfig.loopCount = 0;
	featureExtractionConfig.maxLoops =10;
	featureExtractionConfig.outOfImages = false;

	featureExtractionConfig.sameLocalAndDesc = true;
	stringstream stringBuffer("");



	//Process the command line arguments
	for(int i = 0; i < (int)commandStringVector.size(); i++)
	{
		if(!commandStringVector[i].compare("-inputMethod") || !commandStringVector[i].compare("-InputMethod"))
		{
			if(!commandStringVector[i+1].compare("monoStream"))
			{
				featureExtractionConfig.inputMethod=FEATURE_DESC_SINGLE_STREAM;
			}
			if(!commandStringVector[i+1].compare("stereoStream"))
			{
				featureExtractionConfig.inputMethod=FEATURE_DESC_STEREO_STREAM;
			}
			if(!commandStringVector[i+1].compare("imageFile"))
			{
				featureExtractionConfig.inputMethod=FEATURE_DESC_IMAGE_FILE;
			}
			if(!commandStringVector[i+1].compare("imageFileList"))
			{
				featureExtractionConfig.useImageList = true;
				featureExtractionConfig.inputMethod=FEATURE_DESC_IMAGE_FILE_LIST;
			}


			if(!commandStringVector[i+1].compare("videoFile"))
			{
				featureExtractionConfig.inputMethod=FEATURE_DESC_VIDEO_FILE;
			}

		}


		if(!commandStringVector[i].compare("-loadDescriptors") || !commandStringVector[i].compare("-LoadDescriptors"))
		{

			featureExtractionConfig.loadTrainedDescriptors = true;
			featureExtractionConfig.descriptorsFilename = commandStringVector[i+1];


		}

		if(!commandStringVector[i].compare("-loadDescriptorsImage") || !commandStringVector[i].compare("-LoadDescriptorsImage"))
		{

			featureExtractionConfig.descriptorsImageFilename = commandStringVector[i+1];

		}

		if(!commandStringVector[i].compare("-descOutFile") || !commandStringVector[i].compare("-DescOutFile"))
		{

			//featureExtractionConfig.loadTrainedDescriptors = true;
			featureExtractionConfig.descOutputFilename = commandStringVector[i+1];


		}
		if(!commandStringVector[i].compare("-imageList") || !commandStringVector[i].compare("-ImageList"))
		{


			featureExtractionConfig.imageListFilename = commandStringVector[i+1];


		}

		if(!commandStringVector[i].compare("-saveAllDesc") || !commandStringVector[i].compare("-SaveAllDesc"))
		{

			featureExtractionConfig.accumulateDesc = true;
			featureExtractionConfig.saveAllDescToSingleFile = true;




		}


		if(!commandStringVector[i].compare("-imageListBaseDir") || !commandStringVector[i].compare("-ImageListBaseDir"))
		{

			//featureExtractionConfig.loadTrainedDescriptors = true;
			featureExtractionConfig.imageListBaseDir = commandStringVector[i+1];


		}


		if(!commandStringVector[i].compare("-noShowWindows") || !commandStringVector[i].compare("-NoShowWindows"))
		{

			//featureExtractionConfig.loadTrainedDescriptors = true;
			featureExtractionConfig.showWindows = false;


		}

		if(!commandStringVector[i].compare("-noWaitKey") || !commandStringVector[i].compare("-NoWaitKey"))
		{

			//featureExtractionConfig.loadTrainedDescriptors = true;
			featureExtractionConfig.noWaitKey = true;


		}

		if(!commandStringVector[i].compare("-nImages"))
		{
			stringBuffer.str("");
			stringBuffer << commandStringVector[i+1];
			if((stringBuffer >> featureExtractionConfig.numberOfImages).fail())
			{
				cout << "Number of images could not be parsed." << endl;
			}

		}

		if(!commandStringVector[i].compare("-videoFile") || !commandStringVector[i].compare("-VideoFile"))
		{
			featureExtractionConfig.videoFileName = commandStringVector[i+1];

		}
		if(!commandStringVector[i].compare("-leftCamera") || !commandStringVector[i].compare("-LeftCamera"))
		{
			stringBuffer.str("");
			stringBuffer << commandStringVector[i+1];
			if((stringBuffer >> featureExtractionConfig.leftCameraId).fail())
			{
				cout << "Left Camera Id could not be parsed." << endl;
			}

		}
		if(!commandStringVector[i].compare("-rightCamera") || !commandStringVector[i].compare("-RightCamera"))
		{
			stringBuffer.str("");
			stringBuffer << commandStringVector[i+1];
			if((stringBuffer >> featureExtractionConfig.rightCameraId).fail())
			{
				cout << "Right Camera Id could not be parsed." << endl;
			}

		}

		if(!commandStringVector[i].compare("-snapShotMode") || !commandStringVector[i].compare("-SnapShotMode"))
		{
			featureExtractionConfig.snapShotMode = true;
		}
		if(!commandStringVector[i].compare("-imageStartId") || !commandStringVector[i].compare("-ImageStartId"))
		{
			stringBuffer.str("");
			stringBuffer << commandStringVector[i+1];
			if((stringBuffer >> featureExtractionConfig.imageId).fail())
			{
				cout << "Image start id could not be parsed." << endl;
			}

		}

		if(!commandStringVector[i].compare("-imagePath") || !commandStringVector[i].compare("-ImagePath"))
		{
			featureExtractionConfig.imageNameBase = commandStringVector[i+1];
		}

		if(!commandStringVector[i].compare("-imageExt") || !commandStringVector[i].compare("-ImageExt"))
		{
			featureExtractionConfig.imageExtension = commandStringVector[i+1];
		}



		if(!commandStringVector[i].compare("-local") || !commandStringVector[i].compare("-Local"))
		{
			if(!commandStringVector[i+1].compare("sift"))
			{
				featureExtractionConfig.runSiftLocal=true;
				featureExtractionConfig.currentLocalizationAlgo = FEATURE_LOCALIZATION_SIFT;
			}
			if(!commandStringVector[i+1].compare("surf"))
			{
				featureExtractionConfig.runSurfLocal=true;
				featureExtractionConfig.currentLocalizationAlgo = FEATURE_LOCALIZATION_SURF;
				
			}
			if(!commandStringVector[i+1].compare("fast"))
			{
				featureExtractionConfig.runFastLocal=true;
				featureExtractionConfig.currentLocalizationAlgo = FEATURE_LOCALIZATION_FAST;
			}
			if(!commandStringVector[i+1].compare("orb"))
			{
				featureExtractionConfig.runOrbLocal=true;
				featureExtractionConfig.currentLocalizationAlgo = FEATURE_LOCALIZATION_ORB;
			}

			if(!commandStringVector[i+1].compare("hog"))
			{
				featureExtractionConfig.runHogLocal=true;
				featureExtractionConfig.currentLocalizationAlgo = FEATURE_LOCALIZATION_HoG;
			}
			if(!commandStringVector[i+1].compare("slidingWin"))
			{
				featureExtractionConfig.runSlidingWindowLocal=true;
				featureExtractionConfig.currentLocalizationAlgo = FEATURE_LOCALIZATION_SLIDING_WINDOW;
			}
			if(!commandStringVector[i+1].compare("centerPoint"))
			{
				featureExtractionConfig.addCenterLocation=true;
				featureExtractionConfig.currentLocalizationAlgo = FEATURE_LOCALIZATION_CENTER_POINT;
			}


		}

		if(!commandStringVector[i].compare("-hogAddCenter") || !commandStringVector[i].compare("-HogAddCenter"))
		{
			featureExtractionConfig.addCenterLocation = true;
		}

		if(!commandStringVector[i].compare("-hogMultiScale") || !commandStringVector[i].compare("-HogMultiScale"))
		{
			featureExtractionConfig.hogMultiScale = true;
		}
		if(!commandStringVector[i].compare("-hogSlidingWin") || !commandStringVector[i].compare("-HogSlidingWin"))
		{
			featureExtractionConfig.runSlidingWindowLocal=true;
		}



		if(!commandStringVector[i].compare("-desc") || !commandStringVector[i].compare("-Desc"))
		{
			if(!commandStringVector[i+1].compare("sift"))
			{
				featureExtractionConfig.runSiftDesc=true;
				featureExtractionConfig.currentDescriptorAlgo = FEATURE_DESC_SIFT;

			}
			if(!commandStringVector[i+1].compare("surf"))
			{
				featureExtractionConfig.runSurfDesc=true;
				featureExtractionConfig.currentDescriptorAlgo = FEATURE_DESC_SURF;
			}
			if(!commandStringVector[i+1].compare("fast"))
			{
				featureExtractionConfig.runFastDesc=true;
				featureExtractionConfig.currentDescriptorAlgo = FEATURE_DESC_FAST;
			}
			if(!commandStringVector[i+1].compare("orb"))
			{
				featureExtractionConfig.runOrbDesc=true;
				featureExtractionConfig.currentDescriptorAlgo = FEATURE_DESC_ORB;
			}

			if(!commandStringVector[i+1].compare("hog"))
			{
				featureExtractionConfig.runHogDesc=true;
				featureExtractionConfig.currentDescriptorAlgo = FEATURE_DESC_HoG;
			}
			if(!commandStringVector[i+1].compare("brief"))
			{
				featureExtractionConfig.runBriefDesc=true;
				featureExtractionConfig.currentDescriptorAlgo = FEATURE_DESC_BRIEF;
			}

		}
		if(commandStringVector[i].compare("-nVertCores") == 0)
		{
			stringBuffer.str("");
			stringBuffer.clear();
			stringBuffer << commandStringVector[i+1];
			cout << commandStringVector[i+1] << endl;
			if((stringBuffer >> featureExtractionConfig.numberOfVerticalProcs).fail())
			{
				cout << "Number of vertical cores could not be parsed." << endl;
			}

		}

		if(commandStringVector[i].compare("-nHoriCores")==0)
		{
			stringBuffer.str("");
			stringBuffer.clear();
			stringBuffer << commandStringVector[i+1];
			cout << commandStringVector[i+1] << endl;
			if((stringBuffer >> featureExtractionConfig.numberOfHorizontalProcs).fail())
			{
				cout << "Number of horizontal cores could not be parsed." << endl;
			}

		}
		if((commandStringVector[i].compare("-sameLocalAndDesc")==0 )  ||(commandStringVector[i].compare("-SameLocalAndDesc")==0 ))
		{
			featureExtractionConfig.sameLocalAndDesc = true;

		}
		if((commandStringVector[i].compare("-redistribute")==0 )  ||(commandStringVector[i].compare("-Redistribute")==0 ))
		{
			featureExtractionConfig.redistribute = true;

		}


	}

	if(featureExtractionConfig.numberOfVerticalProcs ==1 && featureExtractionConfig.numberOfHorizontalProcs ==1)
	{
		featureExtractionConfig.singleThreaded = true;


	}
	else
	{
		featureExtractionConfig.singleThreaded = false;

	}

	if(!(((featureExtractionConfig.currentLocalizationAlgo == FEATURE_LOCALIZATION_SURF)&&(featureExtractionConfig.currentDescriptorAlgo == FEATURE_DESC_SURF))||
		((featureExtractionConfig.currentLocalizationAlgo == FEATURE_LOCALIZATION_SIFT)&&(featureExtractionConfig.currentDescriptorAlgo == FEATURE_DESC_SIFT)) ||
		((featureExtractionConfig.currentLocalizationAlgo == FEATURE_LOCALIZATION_ORB)&&(featureExtractionConfig.currentDescriptorAlgo == FEATURE_DESC_ORB))))
	{

		featureExtractionConfig.sameLocalAndDesc = false;
	}

	if(featureExtractionConfig.redistribute)
	{

		featureExtractionConfig.sameLocalAndDesc = false;
	}


}

void featureExtractionSingleThreadedMain(FeatureExtractionConfig & featureExtractionConfig,vector<Mat> & inputImagesPerFrame)
{

	Mat mainWindowImage(featureExtractionConfig.frameSize,CV_8UC3);
	Mat currentFeaturePointsImage(featureExtractionConfig.frameSize,CV_8UC3);
	Mat MatchedFeaturePointsImage(featureExtractionConfig.frameSize,CV_8UC3);

	if(featureExtractionConfig.showWindows)
	{
		if(featureExtractionConfig.showMainWindow)
		{
			namedWindow("Feature Extraction Window",1);
		}

		if(featureExtractionConfig.showCurrentFeaturePointsWindow)
		{
			namedWindow("Current Feature Points Window",1);
		}
		if(featureExtractionConfig.showMatchWindow)
		{
			namedWindow("Matched Feature Points Window",1);

		}
	}

	int lastKey = -1;
	int key = -1;
	int frameRateWait = 33;
	if(featureExtractionConfig.snapShotMode)
	{
		frameRateWait = 0;

	}
	bool done = false;
	vector<KeyPoint> currentKeyPoints;
	Ptr<FeatureDetector> featureDetector = NULL;
	Ptr<DescriptorExtractor> descriptorExtractor = NULL;
	Ptr<DescriptorMatcher> matcher = NULL;
	Mat trainedImage;
	Mat trainedDescriptors;
	Point2f trainedRefPoint;
	vector<KeyPoint> trainedKeypoints;
	vector<Vec2f> trainedRefVectors;
	Mat descriptors;
//	vector<KeyPoint> keypoints;
	double matchRadius = .1;
	double matchRadiusDecrement = 0;//.001;
	vector<Rect> currentBboxes;
	Mat allDescriptors;
	//vector<KeyPoint> allKeyPoints;

	if(featureExtractionConfig.loadTrainedDescriptors)
	{
		string descTypeString, localTypeString;
		loadFeatureDescriptorsAndOrKeypoints( featureExtractionConfig.descriptorsFilename,  descTypeString,localTypeString ,featureExtractionConfig.currentDescriptorAlgo,featureExtractionConfig.currentLocalizationAlgo,featureDetector,&trainedKeypoints,descriptorExtractor, &trainedDescriptors);
		featureExtractionConfig.sift= new SIFT;
		featureExtractionConfig.surf= new SURF(100.00,4,2,false);
		trainedRefPoint = computeCenterPointOfKeyPoints(trainedKeypoints);
		trainedRefVectors = computeRelativeCenterPointVectorsForKeypoints( trainedKeypoints);
		vector<Mat> trainDescriptorsVec;
		trainDescriptorsVec.push_back(trainedDescriptors);
		matcher= setupMatcher(featureExtractionConfig,trainDescriptorsVec);
		cout << "Loaded:   Desc: " << descTypeString << "Local: " << localTypeString << endl;


		featureExtractionConfig.trainedDescriptorsLoaded =true;

		featureExtractionConfig.trainedDescriptorsAvailable = true;
		if(featureExtractionConfig.descriptorsImageFilename.compare(""))
		{
			trainedImage = imread(featureExtractionConfig.descriptorsImageFilename);
			//paintCenterOnImage(trainedImage, trainedKeypoints, trainedDescriptors);
			imshow("Testing",trainedImage);
			waitKey(0);

		}

	}
	else
	{
		featureDetector = setupLocalization(featureExtractionConfig);
		descriptorExtractor = setupDescriptor(featureExtractionConfig);

	}
	if(matcher ==NULL)
	{
		matcher = setupMatcher( featureExtractionConfig);

	}


	if(((featureExtractionConfig.currentLocalizationAlgo == FEATURE_LOCALIZATION_SURF)&&(featureExtractionConfig.currentDescriptorAlgo == FEATURE_DESC_SURF))||
		((featureExtractionConfig.currentLocalizationAlgo == FEATURE_LOCALIZATION_SIFT)&&(featureExtractionConfig.currentDescriptorAlgo == FEATURE_DESC_SIFT))||
		((featureExtractionConfig.currentLocalizationAlgo == FEATURE_LOCALIZATION_ORB)&&(featureExtractionConfig.currentDescriptorAlgo == FEATURE_DESC_ORB)))
	{

		featureExtractionConfig.sameLocalAndDesc = true;
	}

	cout << "Feature Extraction Beginning Main Loop" << endl;
#ifdef USE_MARSS
	cout << "Switching to simulation in Feature Extraction." << endl;
	ptlcall_switch_to_sim();

	switch(featureExtractionConfig.currentDescriptorAlgo)
	{
	case FEATURE_DESC_SIFT:
		//ptlcall_single_enqueue("-logfile featureExtract_sift.log");
		ptlcall_single_enqueue("-stats featureExtract_sift.stats");
		//ptlcall_single_enqueue("-loglevel 0");
		break;
	case FEATURE_DESC_SURF:
		//ptlcall_single_enqueue("-logfile featureExtract_surf.log");
		ptlcall_single_enqueue("-stats featureExtract_surf.stats");
		//ptlcall_single_enqueue("-loglevel 0");
		break;
	case FEATURE_DESC_ORB:
		//ptlcall_single_enqueue("-logfile featureExtract_surf.log");
		ptlcall_single_enqueue("-stats featureExtract_orb.stats");
		//ptlcall_single_enqueue("-loglevel 0");
		break;
	case FEATURE_DESC_FAST:
		//ptlcall_single_enqueue("-logfile featureExtract_fast.log");
		ptlcall_single_enqueue("-stats featureExtract_fast.stats");
		//ptlcall_single_enqueue("-loglevel 0");
		break;
	case FEATURE_DESC_HoG:
		//ptlcall_single_enqueue("-logfile featureExtract_hog.log");
		ptlcall_single_enqueue("-stats featureExtract_hog.stats");
		//ptlcall_single_enqueue("-loglevel 0");
		break;
	case FEATURE_DESC_BRIEF:
		//ptlcall_single_enqueue("-logfile featureExtract_brief.log");
		ptlcall_single_enqueue("-stats featureExtract_brief.stats");
		//ptlcall_single_enqueue("-loglevel 0");
		break;
	default:
		//ptlcall_single_enqueue("-logfile featureExtract.log");
		ptlcall_single_enqueue("-stats featureExtract.stats");
		//ptlcall_single_enqueue("-loglevel 0");
		break;

	};
	//ptlcall_single_enqueue("-logfile featureExtract.log");
	//ptlcall_single_enqueue("-stats featureExtract.stats");
	//ptlcall_single_enqueue("-loglevel 0");
#endif

#ifdef USE_GEM5
	#ifdef GEM5_CHECKPOINT_WORK
		m5_checkpoint(0, 0);
	#endif

	#ifdef GEM5_SWITCHCPU_AT_WORK
		m5_switchcpu();
	#endif 
	m5_dumpreset_stats(0, 0);
#endif


#ifdef TSC_TIMING
	READ_TIMESTAMP_WITH_WRAPPER( fe_timingVector[0] );
#endif
#ifdef CLOCK_GETTIME_TIMING
	GET_TIME_WRAPPER(fe_timeStructVector[0]);
#endif

	while(!done)
	{
		int numberOfImagesRemaining;
		numberOfImagesRemaining = getNextImages(inputImagesPerFrame,featureExtractionConfig);


		if(numberOfImagesRemaining <=1 )
		{
			done = true;
		}
		if(inputImagesPerFrame[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA].empty())
		{

			break;
		}

		if((featureExtractionConfig.showWindows == false) || featureExtractionConfig.noWaitKey == true)
		{
			featureExtractionConfig.loopCount++;
		}
		if((featureExtractionConfig.loopCount >= featureExtractionConfig.maxLoops) || (featureExtractionConfig.outOfImages))
		{


			break;
		}
//		timingArray[0] = clock();


		if(featureExtractionConfig.currentLocalizationAlgo == FEATURE_LOCALIZATION_HoG)
		{
			//cout << "Running Localize" << endl;
			LocalizeFeaturePoints(inputImagesPerFrame[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA],  currentKeyPoints, currentBboxes,featureExtractionConfig);
		}
		else
		{
				

			if(	!featureExtractionConfig.sameLocalAndDesc)
			{

				//cout << "Running Desc" << endl;

				LocalizeFeaturePoints(inputImagesPerFrame[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA],  currentKeyPoints, featureExtractionConfig,featureDetector );

			}
			else
			{


				//cout << "Running Desc" << endl;
				LocalizeFeaturePoints(inputImagesPerFrame[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA],  currentKeyPoints, featureExtractionConfig, descriptors);







			}

		}
//		timingArray[1] = clock();
//		timingArray[2] = timingArray[1] - timingArray[0];


		#ifdef DEBUG_OUTPUT
			cout << "Number of Feature Found" << currentKeyPoints.size() << endl;
		#endif
		if(featureExtractionConfig.showWindows)
		{

			if(featureExtractionConfig.showCurrentFeaturePointsWindow)
			{

				Mat tmpImage;
				if(inputImagesPerFrame[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA].channels() ==3)
				{
					cvtColor(inputImagesPerFrame[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA],tmpImage,CV_BGR2GRAY);
				}
				else
				{
					inputImagesPerFrame[0].copyTo(tmpImage);
				}

				drawKeypoints(tmpImage,currentKeyPoints,currentFeaturePointsImage,Scalar(255.0,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
				imshow("Current Feature Points Window",currentFeaturePointsImage);

				/*
				Mat testSplit = inputImagesPerFrame[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA].clone();

				Mat pieces[4];
				Rect origPieces[4];

				Mat piecesDisp[4];
				vector<KeyPoint> pieceKeys[4];


				Mat pieceDesc[4];
				pieces[0] = inputImagesPerFrame[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA](Range(0,testSplit.rows/2),Range(0,testSplit.cols/2));
				origPieces[0].x = 0;
				origPieces[0].y =0;
				origPieces[0].height = testSplit.rows/2;
				origPieces[0].width =testSplit.cols/2;
				Size testSize;

				Point offsets[4];

				int i=0;

				for (i=0; i< 4; i++)
				{
					piecesDisp[i].create(pieces[0].rows,pieces[0].cols,CV_8UC3);


				}
				int bufferAmount = 16;
				pieces[0].adjustROI(0,bufferAmount,0,bufferAmount);


				pieces[0].locateROI(testSize,offsets[0]);

				i=0;
				LocalizeFeaturePoints(pieces[i],  pieceKeys[i], featureExtractionConfig, pieceDesc[i]);

				i++;

				pieces[1] = inputImagesPerFrame[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA](Range(0,testSplit.rows/2),Range(testSplit.cols/2,testSplit.cols));
				origPieces[1].x = testSplit.cols/2;
				origPieces[1].y =0;
				origPieces[1].height = testSplit.rows/2;
				origPieces[1].width =testSplit.cols/2;


				pieces[1].adjustROI(0,bufferAmount,bufferAmount,0);
				pieces[1].locateROI(testSize,offsets[1]);
				LocalizeFeaturePoints(pieces[i],  pieceKeys[i], featureExtractionConfig, pieceDesc[i]);
				i++;

				pieces[2] = inputImagesPerFrame[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA](Range(testSplit.rows/2,testSplit.rows),Range(0,testSplit.cols/2));
				origPieces[2].x =0;
				origPieces[2].y =testSplit.rows/2;
				origPieces[2].height = testSplit.rows/2;
				origPieces[2].width =testSplit.cols/2;


				pieces[2].adjustROI(bufferAmount,0,0,bufferAmount);
				pieces[2].locateROI(testSize,offsets[2]);

				LocalizeFeaturePoints(pieces[i],  pieceKeys[i], featureExtractionConfig, pieceDesc[i]);
				i++;


				pieces[3] = inputImagesPerFrame[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA](Range(testSplit.rows/2,testSplit.rows),Range(testSplit.cols/2,testSplit.cols));
				origPieces[3].x =testSplit.cols/2;
				origPieces[3].y =testSplit.rows/2;
				origPieces[3].height = testSplit.rows/2;
				origPieces[3].width =testSplit.cols/2;


				pieces[3].adjustROI(bufferAmount,0,bufferAmount,0);
				pieces[3].locateROI(testSize,offsets[3]);

				LocalizeFeaturePoints(pieces[i],  pieceKeys[i], featureExtractionConfig, pieceDesc[i]);

				int totalSplit = 0;
				for(i = 0; i < 4; i++)
				{

					Mat tmpImage;
					cvtColor(pieces[i],tmpImage,CV_BGR2GRAY);
					drawKeypoints(tmpImage,pieceKeys[i],piecesDisp[i],Scalar(255.0,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
					cout <<"Piece: "  <<i<< "  Number Of Keys: "<<pieceKeys[i].size()<< endl;
					 totalSplit+=pieceKeys[i].size();

				}





				cout <<" Total Number Of Keys: "<<totalSplit<< endl;


				vector<KeyPoint> allKeys;
				for(i = 0; i < 4; i++)
				{
					for(int j= 0; j<pieceKeys[i].size(); j++)
					{

						KeyPoint currentKeyPoint = pieceKeys[i][j];


						currentKeyPoint.pt.x += offsets[i].x;
						currentKeyPoint.pt.y += offsets[i].y;
						bool found = false;

						if(origPieces[i].contains(Point(currentKeyPoint.pt.x,currentKeyPoint.pt.y)))
						{

							cout << "KeyPoint: [ x:" << currentKeyPoint.pt.x << " y:" << currentKeyPoint.pt.y <<" ang:"<< currentKeyPoint.angle << " o:"<<currentKeyPoint.octave << " rp:"<< currentKeyPoint.response << " sz:"<< currentKeyPoint.size << " ]"<<endl;
							for(int k=0; k< allKeys.size(); k++)
							{
								if((abs(currentKeyPoint.pt.x -allKeys[k].pt.x)  <= .5)&& (abs(currentKeyPoint.pt.y -allKeys[k].pt.y)  <= .5)&&((abs(currentKeyPoint.angle -allKeys[k].angle))<= 2.0)&& (abs(currentKeyPoint.size- allKeys[k].size) <= .5)&&(currentKeyPoint.octave == allKeys[k].octave))//  && (currentKeyPoint.response == allKeys[k].response) )
								//if((currentKeyPoint.size == allKeys[k].size) && (currentKeyPoint.pt == allKeys[k].pt)&&(currentKeyPoint.angle == allKeys[k].angle)) //&& (currentKeyPoint.octave == allKeys[k].octave) && (currentKeyPoint.response == allKeys[k].response) )
								{

									found =true;

								}

							}


							if(!found)
							{
								allKeys.push_back(currentKeyPoint);


							}
						}
					}



				}


				vector<KeyPoint> sortedPoints;
				int insertIndex;
				for(int j = 0; j < allKeys.size(); j++)
				{


					KeyPoint currentAll = allKeys[j];
					insertIndex = sortedPoints.size();
					vector<KeyPoint>::iterator it;


					for(int p = 0; p< sortedPoints.size(); p++)
					{
						if(currentAll.pt.x < sortedPoints[p].pt.x)
						{
							insertIndex = p;
							break;

						}
					}
					it = sortedPoints.begin();
					sortedPoints.insert(it+insertIndex,currentAll);


				}

				cout << "Sorted Count: " << sortedPoints.size()<<endl;
				for(int j = 0; j < sortedPoints.size(); j++)
				{
					KeyPoint currentKeyPoint = sortedPoints[j];
					cout << "KeyPoint: [ x:" << currentKeyPoint.pt.x << " y:" << currentKeyPoint.pt.y <<" ang:"<< currentKeyPoint.angle << " o:"<<currentKeyPoint.octave << " rp:"<< currentKeyPoint.response << " sz:"<< currentKeyPoint.size << " ]"<<endl;


				}


				cout << "All Keys Count: " << allKeys.size() << endl;
				vector<KeyPoint> notFoundPoints;
				namedWindow("Doh");
				waitKey(0);
				int notFoundInNewCount = 0;
				int sameCount = 0;
				for(i=0; i< currentKeyPoints.size(); i++)
				{
					KeyPoint currentKeyPoint = currentKeyPoints[i];
					//cout << "KeyPoint: [ x:" << currentKeyPoint.pt.x << " y:" << currentKeyPoint.pt.y <<" ang:"<< currentKeyPoint.angle << " o:"<<currentKeyPoint.octave << " rp:"<< currentKeyPoint.response << " sz:"<< currentKeyPoint.size << " ]"<<endl;

					bool found = false;
					for(int k=0; k< allKeys.size(); k++)
					{

						Point currentPoint(currentKeyPoint.pt.x,currentKeyPoint.pt.y);
						Point allPoint(allKeys[k].pt.x,allKeys[k].pt.y);

						if((abs(currentKeyPoint.pt.x -allKeys[k].pt.x)  <= .5)&& (abs(currentKeyPoint.pt.y -allKeys[k].pt.y)  <= .5)&&((abs(currentKeyPoint.angle -allKeys[k].angle))<= 2.0)&& (abs(currentKeyPoint.size- allKeys[k].size) <= .5)&&(currentKeyPoint.octave == allKeys[k].octave))//  && (currentKeyPoint.response == allKeys[k].response) )
						{

							found =true;

						}

					}

					if(found ==false)
					{

						cout << "Not Found KeyPoint: [ x:" << currentKeyPoint.pt.x << " y:" << currentKeyPoint.pt.y <<" ang:"<< currentKeyPoint.angle << " o:"<<currentKeyPoint.octave << " rp:"<< currentKeyPoint.response << " sz:"<< currentKeyPoint.size << " ]"<<endl;
						notFoundInNewCount ++;
					}
					else
					{
						sameCount++;

					}

				}

				cout << "Diffs:" << notFoundInNewCount << " Same:" << sameCount << endl;
				waitKey(0);






				namedWindow("Split 1");
				namedWindow("Split 2");
				namedWindow("Split 3");
				namedWindow("Split 4");
				namedWindow("Full");

				i=0;
				imshow("Split 1", piecesDisp[i]);
				i++;
				imshow("Split 2", piecesDisp[i]);
				i++;

				imshow("Split 3", piecesDisp[i]);
				i++;
				imshow("Split 4", piecesDisp[i]);
				imshow("Full", testSplit);
				waitKey(0);
				*/
			}
		}
		//timingArray[3] = clock();

		//currentKeyPoints.clear();

#ifdef USE_GEM5
	m5_dumpreset_stats(0, 0);
#endif

		if(featureExtractionConfig.currentDescriptorAlgo == FEATURE_DESC_HoG)
		{
			//cout << "Desc Build" << endl;
			descriptors =  buildFeatureDescriptors(inputImagesPerFrame[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA], currentKeyPoints,currentBboxes,featureExtractionConfig,featureExtractionConfig.descHogPtr);

		}
		else
		{
			if(!featureExtractionConfig.sameLocalAndDesc)
			{
				descriptors =  buildFeatureDescriptors(inputImagesPerFrame[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA], currentKeyPoints,featureExtractionConfig,descriptorExtractor);

			}
		}
		//timingArray[4] = clock();
		//timingArray[5] = timingArray[4] - timingArray[3];
		//cout << "Build Desc Done" << endl;

		if(featureExtractionConfig.accumulateDesc)
		{
			cout << "Accum" << endl;
			if(allDescriptors.empty())
			{
				allDescriptors = descriptors.clone();


			}
			else
			{
				if(descriptors.rows >0)
				{
					allDescriptors.push_back(descriptors);
				}

			}
		}

		if(featureExtractionConfig.showWindows)
		{

			if(featureExtractionConfig.showMainWindow)
			{
				if(inputImagesPerFrame[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA].channels() ==1)
				{
					cvtColor(inputImagesPerFrame[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA],mainWindowImage,CV_GRAY2BGR);
				}
				else
				{
					inputImagesPerFrame[0].copyTo(mainWindowImage);
				}



				imshow("Feature Extraction Window",mainWindowImage);
			}
		}
		if(!featureExtractionConfig.snapShotMode && !featureExtractionConfig.noWaitKey)
		{
			cout << "WaitKey" << endl;
			key = waitKey(frameRateWait+10000) & 0xFF;
		}

		if(key == 27 || key =='q' || key =='d')
		{
			done = true;

		}
		if(featureExtractionConfig.snapShotMode)
		{

			if(featureExtractionConfig.trainedDescriptorsAvailable)
			{

				vector<DMatch> filteredMatches;
//				vector<vector<DMatch>> foundMatches;
//				vector<vector<DMatch>> foundMatches2;
//				matcher->knnMatch(descriptors,trainedDescriptors,foundMatches,1);

//				matchRadius -= matchRadiusDecrement;
//				if(matchRadius <.001)
//				{
//					matchRadius = .001;

//				}

//				cout << "Match Radius: " << matchRadius << endl;
//				matcher->radiusMatch(descriptors,trainedDescriptors,foundMatches,(float)matchRadius);
//				matcher->knnMatch(descriptors,trainedDescriptors,foundMatches,1);
//				matcher->match(descriptors,trainedDescriptors,filteredMatches);


				vector<KeyPoint> filteredKeyPoints;


//				for(int i = 0; i < (int)foundMatches.size(); i++)
//				{
//					if(foundMatches[i].size() >0)
//					{
//						filteredMatches.push_back(foundMatches[i][0]);
//						filteredKeyPoints.push_back(keypoints[i]);
//
//					}



//				}

//				timingArray[6] = clock();


//				cout << "Starting Matching" << endl;
				Mat tmpDescriptors;
				filteredMatches = performFeatureMatching(matcher, descriptors,/*trainedDescriptors*/tmpDescriptors,currentKeyPoints, trainedKeypoints);

//				timingArray[7] = clock();
//				timingArray[8] = timingArray[7] - timingArray[6];


//				timingArray[9] = clock();


				vector<Mat> transformsList;
				vector<Point2f> refPoints;
				filterMatchesAndGetTransforms(transformsList, refPoints,filteredMatches, currentKeyPoints,trainedKeypoints,trainedRefVectors, trainedRefPoint);


//				timingArray[10] = clock();
//				timingArray[11] = timingArray[10] - timingArray[9];

				if(filteredMatches.size() ==0)
				{

					cout << "Found no matches" << endl;
				}


				Mat centerDisplay = inputImagesPerFrame[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA].clone();
				for(int i =0; i < (int)refPoints.size(); i++)
				{

					circle(centerDisplay,refPoints[i],5,Scalar(0,255,255),CV_FILLED);

				}
				imshow("Center Display",centerDisplay);

				drawMatches(inputImagesPerFrame[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA],currentKeyPoints,trainedImage,trainedKeypoints,filteredMatches,MatchedFeaturePointsImage,Scalar(0,0,255.0),Scalar(-1,-1,-1,-1), vector<char> (),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

				imshow("Matched Feature Points Window",MatchedFeaturePointsImage);
				waitKey(100);

			}



			if(!featureExtractionConfig.trainedDescriptorsLoaded && !featureExtractionConfig.trainedDescriptorsAvailable)
			{
				trainedDescriptors = descriptors.clone();
				trainedKeypoints = currentKeyPoints;
				inputImagesPerFrame[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA].copyTo(trainedImage);
				featureExtractionConfig.trainedDescriptorsAvailable = true;
				trainedRefPoint = computeCenterPointOfKeyPoints(trainedKeypoints);
				trainedRefVectors = computeRelativeCenterPointVectorsForKeypoints( trainedKeypoints);
				vector<Mat> trainDescriptorsVec;
				trainDescriptorsVec.push_back(trainedDescriptors);
				matcher->add(trainDescriptorsVec);
				matcher->train();


			}

			cout << "Waiting for user input:" << endl;



			key = waitKey(30) & 0xFF;

			cout << "User key id recieved: " << (char)key << endl;

			if(key == 's')
			{
				stringstream stringBuffer;
				stringBuffer.str("");
				stringBuffer << featureExtractionConfig.currentOutputImageNumber++;

				string saveFileName = featureExtractionConfig.descOutputFilename + stringBuffer.str() + ".yml";
				saveFeatureDescriptorsAndOrKeypoints( saveFileName,  featureExtractionConfig.currentDescriptorAlgo,featureExtractionConfig.currentLocalizationAlgo, featureDetector,currentKeyPoints,descriptorExtractor, descriptors);
				cout << "Saved File to: "<< saveFileName << endl;

				saveFileName = featureExtractionConfig.descOutputFilename + stringBuffer.str() + ".png";
				imwrite(saveFileName,inputImagesPerFrame[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA]);

			}
			else if(key == 27 || key =='q' || key =='d')
			{
				done = true;

			}




		}


		//cout << "Timing ";
		//for(int i=0; i < timingArray.size(); i++)
		//{

		//	cout << "Entry " << i << ": " << timingArray[i] << " ";

		//}

		//cout << endl;
	}
#ifdef USE_MARSS
	ptlcall_kill();
#endif

#ifdef USE_GEM5

	m5_dumpreset_stats(0, 0);
	#ifdef GEM5_EXIT_AFTER_WORK
		m5_exit(0);
	#endif


#endif



#ifdef TSC_TIMING
	READ_TIMESTAMP_WITH_WRAPPER( fe_timingVector[ 0 + fe_timingVector.size() /2] );
#endif
#ifdef CLOCK_GETTIME_TIMING
	GET_TIME_WRAPPER(fe_timeStructVector[0+ fe_timeStructVector.size()/2]);
#endif
	cout << "Timing Done" << endl;
	if(featureExtractionConfig.saveAllDescToSingleFile)
	{
		vector<KeyPoint> keypoints;
		if(featureExtractionConfig.currentDescriptorAlgo == FEATURE_DESC_HoG)
		{

			saveFeatureDescriptorsAndOrKeypoints( featureExtractionConfig.descOutputFilename,  featureExtractionConfig.currentDescriptorAlgo,featureExtractionConfig.currentLocalizationAlgo,featureExtractionConfig.featureDetector,keypoints,featureExtractionConfig.descHogPtr, allDescriptors);

		}
		else
		{
			saveFeatureDescriptorsAndOrKeypoints( featureExtractionConfig.descOutputFilename,  featureExtractionConfig.currentDescriptorAlgo,featureExtractionConfig.currentLocalizationAlgo, featureDetector,keypoints,descriptorExtractor, allDescriptors);
		}

	}


	//waitKey(0);
	if(featureExtractionConfig.showWindows)
	{

		if(featureExtractionConfig.showMainWindow)
		{
			destroyWindow("Feature Extraction Window");
		}
	}
	switch(featureExtractionConfig.inputMethod)
	{

		case FEATURE_DESC_STEREO_STREAM:
			{
				delete featureExtractionConfig.leftImageCapture;
				delete featureExtractionConfig.rightImageCapture;

			}
			break;
		case FEATURE_DESC_IMAGE_FILE:
			{


			}
			break;
		case FEATURE_DESC_VIDEO_FILE:
			{

			}
			break;
		case FEATURE_DESC_SINGLE_STREAM:
		default:
			{
				delete featureExtractionConfig.leftImageCapture;



			}
	}



}

void featureExtractionHandleLoadingTrainDescriptorsData(FeatureExtractionConfig & featureExtractionConfig, FeatureExtractionData & featureExtractionData)
{



}



void featureExtractionSetupFeatureExtractionData(FeatureExtractionConfig & featureExtractionConfig, FeatureExtractionData & featureExtractionData)
{

	// TODO  Take from here

	featureExtractionData.inputImagesPerFrame = new vector<Mat>;
	//int numberOfInputImagesPerFrame;

	//Setting up input method
	switch(featureExtractionConfig.inputMethod)
	{

		case FEATURE_DESC_STEREO_STREAM:
			{
				featureExtractionConfig.leftImageCapture = new VideoCapture(featureExtractionConfig.leftCameraId);
				featureExtractionConfig.rightImageCapture = new VideoCapture(featureExtractionConfig.rightCameraId);

				Mat rightFrame;
				Mat leftFrame;
				Mat vectorLeftFrame;
				Mat vectorRightFrame;
				(*featureExtractionConfig.leftImageCapture)>>leftFrame;
				(*featureExtractionConfig.rightImageCapture)>>rightFrame;
				leftFrame.copyTo(vectorLeftFrame);
				rightFrame.copyTo(vectorRightFrame);

				featureExtractionData.inputImagesPerFrame->push_back(vectorLeftFrame);
				featureExtractionData.inputImagesPerFrame->push_back(vectorRightFrame);
				featureExtractionConfig.frameSize = leftFrame.size();
				featureExtractionData.frameSize = leftFrame.size();
			}
			break;
		case FEATURE_DESC_IMAGE_FILE:
			{


			}
			break;
		case FEATURE_DESC_IMAGE_FILE_LIST:
			{
				if(featureExtractionConfig.imageListFilename.compare("") == 0)
				{
					cout << "Image File list name is NULL. " << endl;
					exit(0);
				}
				else
				{
					readFileListFromFile(featureExtractionConfig.imageListBaseDir + featureExtractionConfig.imageListFilename, featureExtractionConfig.imageListBaseDir,featureExtractionConfig.imageList);
					if(featureExtractionConfig.imageList.size() == 0)
					{

						cout << "Image File List empty"<< endl;
						exit(0);
					}
				}

				Mat leftFrame = imread(featureExtractionConfig.imageList[0]);
				featureExtractionConfig.frameSize = leftFrame.size();
				featureExtractionData.frameSize = leftFrame.size();

				break;
			}
		case FEATURE_DESC_VIDEO_FILE:
			{

			}
			break;
		case FEATURE_DESC_SINGLE_STREAM:
		default:
			{
				featureExtractionConfig.leftImageCapture = new VideoCapture(featureExtractionConfig.leftCameraId);

				//featureExtractionConfig.leftImageCapture->set(CV_CAP_PROP_FRAME_WIDTH,320);//640);//1024);//320);
				//featureExtractionConfig.leftImageCapture->set(CV_CAP_PROP_FRAME_HEIGHT,240);//480);//768);//240);
				Mat leftFrame;
				Mat vectorLeftFrame;

				(*featureExtractionConfig.leftImageCapture)>>leftFrame;

				leftFrame.copyTo(vectorLeftFrame);


				featureExtractionData.inputImagesPerFrame->push_back(vectorLeftFrame);

				featureExtractionData.frameSize = leftFrame.size();
				featureExtractionConfig.frameSize = leftFrame.size();

			}
	}


	if(featureExtractionConfig.currentDescriptorAlgo == FEATURE_DESC_ORB  || featureExtractionConfig.currentLocalizationAlgo == FEATURE_LOCALIZATION_ORB )
	{

		featureExtractionData.xBufferAmount = 31;
		featureExtractionData.yBufferAmount = 31;

	}
	else
	{
		featureExtractionData.xBufferAmount = 16;
		featureExtractionData.yBufferAmount = 16;

	}

	featureExtractionData.descriptorExtractor = NULL;
	featureExtractionData.featureDetector = NULL;
	featureExtractionData.matcher = NULL;
	featureExtractionData.sift = NULL;
	featureExtractionData.surf = NULL;
	featureExtractionData.orb = NULL;
	featureExtractionConfig.sift = NULL;
	featureExtractionConfig.surf = NULL;
	featureExtractionConfig.orb = NULL;
	featureExtractionData.trainedDescriptors = new Mat;
	featureExtractionData.trainedRefPoint = new Point2f;
	featureExtractionData.trainedKeypoints = new vector<KeyPoint>;
	featureExtractionData.trainedRefVectors = new vector<Vec2f>;

	featureExtractionData.trainImagesPerFrame = new vector<Mat>;

	if(featureExtractionConfig.loadTrainedDescriptors)
	{
		string descTypeString, localTypeString;
		loadFeatureDescriptorsAndOrKeypoints( featureExtractionConfig.descriptorsFilename,  descTypeString,localTypeString ,featureExtractionConfig.currentDescriptorAlgo,featureExtractionConfig.currentLocalizationAlgo,featureExtractionData.featureDetector,&(*(featureExtractionData.trainedKeypoints)),featureExtractionData.descriptorExtractor, &(*(featureExtractionData.trainedDescriptors)));
		featureExtractionData.sift= new SIFT;
		featureExtractionData.surf= new SURF;
		featureExtractionData.orb= new ORB;

		*(featureExtractionData.trainedRefPoint) = computeCenterPointOfKeyPoints(*(featureExtractionData.trainedKeypoints));
		*(featureExtractionData.trainedRefVectors) = computeRelativeCenterPointVectorsForKeypoints( *(featureExtractionData.trainedKeypoints));
		vector<Mat> trainDescriptorsVec;
		trainDescriptorsVec.push_back(*(featureExtractionData.trainedDescriptors));
		featureExtractionData.matcher= setupMatcher(featureExtractionConfig,trainDescriptorsVec);
		cout << "Loaded:   Desc: " << descTypeString << "Local: " << localTypeString << endl;


		featureExtractionConfig.trainedDescriptorsLoaded =true;

		featureExtractionConfig.trainedDescriptorsAvailable = true;
		if(featureExtractionConfig.descriptorsImageFilename.compare(""))
		{

			featureExtractionData.trainImagesPerFrame->push_back(Mat());
			(*(featureExtractionData.trainImagesPerFrame))[0] = imread(featureExtractionConfig.descriptorsImageFilename);
			//paintCenterOnImage(trainedImage, trainedKeypoints, trainedDescriptors);
			imshow("Testing",(*(featureExtractionData.trainImagesPerFrame))[0]);
			waitKey(0);

		}

	}
	else
	{
		featureExtractionData.featureDetector = setupLocalization(featureExtractionConfig);
		featureExtractionData.localizationHogPtr = featureExtractionConfig.localizationHogPtr;
		featureExtractionData.descriptorExtractor = setupDescriptor(featureExtractionConfig);
		featureExtractionData.descHogPtr = featureExtractionConfig.descHogPtr;
	}
	if(featureExtractionData.matcher ==NULL)
	{
		featureExtractionData.matcher = setupMatcher( featureExtractionConfig);

	}








}

void setupFeatureExtractionConfigFromFile(string filename, FeatureExtractionConfig & featureExtractionConfig)
{
	vector<string> configArgs;
	loadFeatureExtractionConfigFile(configArgs, filename);






	parseClassificationConfigCommandVector(featureExtractionConfig, configArgs);



}


void  featureExtractionCoordinatorThreadSetupFunctionStandAlone(struct GeneralWorkerThreadData * genData, void * workerThreadStruct)
{
	struct FeatureExtractionWorkerThreadInfo * workerThreadInfo = reinterpret_cast<FeatureExtractionWorkerThreadInfo *> (workerThreadStruct);
	Thread * myThread = genData->myThread;
	struct FeatureExtractionMultiThreadData * myMultiThreadExtractionData = reinterpret_cast<FeatureExtractionMultiThreadData *>(workerThreadInfo->multiThreadAlgorithmData);
	FeatureExtractionData * myFeatureExtractionData = workerThreadInfo->myFeatureExtractionData;


	Size imageSize = workerThreadInfo->myFeatureExtractionData->frameSize;

	int nonPaddedRegionWidth = imageSize.width/workerThreadInfo->numberOfHorizontalCores;
	int nonPaddedRegionHeight = imageSize.height/workerThreadInfo->numberOfVerticalCores;

	//Set the x and y location of the unpadded
	workerThreadInfo->myNonPaddedImageRegionRect.x = workerThreadInfo->horizontalThreadId *	nonPaddedRegionWidth;
	workerThreadInfo->myNonPaddedImageRegionRect.y = workerThreadInfo->verticalThreadId *	nonPaddedRegionHeight;

	//NonPadded
	if(workerThreadInfo->horizontalThreadId == 0)
	{



	}

	if(workerThreadInfo->horizontalThreadId == workerThreadInfo->numberOfHorizontalCores-1)
	{
		nonPaddedRegionWidth = imageSize.width - (workerThreadInfo->numberOfHorizontalCores-1)*nonPaddedRegionWidth;


	}


	if(workerThreadInfo->verticalThreadId == 0)
	{



	}

	if(workerThreadInfo->verticalThreadId == workerThreadInfo->numberOfVerticalCores-1)
	{
		nonPaddedRegionHeight = imageSize.height - (workerThreadInfo->numberOfVerticalCores-1)*nonPaddedRegionHeight;


	}



	//Set the height and width of the unpadded
	workerThreadInfo->myNonPaddedImageRegionRect.height =nonPaddedRegionHeight;
	workerThreadInfo->myNonPaddedImageRegionRect.width =nonPaddedRegionWidth;


	//First start the padded with the unpadded Info
	workerThreadInfo->myPaddedImageRegionRect =workerThreadInfo->myNonPaddedImageRegionRect;

	//Now check the edges to determine what to modify
	//Left most edge check
	if(workerThreadInfo->horizontalThreadId == 0)
	{



	}
	else
	{

		int overFlowAmount = 0;
		workerThreadInfo->myPaddedImageRegionRect.x= workerThreadInfo->myPaddedImageRegionRect.x-  myFeatureExtractionData->xBufferAmount;
		if(workerThreadInfo->myPaddedImageRegionRect.x<0)
		{

			overFlowAmount = workerThreadInfo->myPaddedImageRegionRect.x;
			workerThreadInfo->myPaddedImageRegionRect.x =0;
		}
		workerThreadInfo->myPaddedImageRegionRect.width = workerThreadInfo->myPaddedImageRegionRect.width+myFeatureExtractionData->xBufferAmount + overFlowAmount;


	}


	//right most edge check
	if(workerThreadInfo->horizontalThreadId == workerThreadInfo->numberOfHorizontalCores-1)
	{



	}
	else
	{
		int overFlowAmount = 0;
		int regionEndXLocation = workerThreadInfo->myPaddedImageRegionRect.x+  myFeatureExtractionData->xBufferAmount;
		if(regionEndXLocation> imageSize.width)
		{

			overFlowAmount = imageSize.width- regionEndXLocation;

		}
		workerThreadInfo->myPaddedImageRegionRect.width = workerThreadInfo->myPaddedImageRegionRect.width+myFeatureExtractionData->xBufferAmount + overFlowAmount;
	}


	//Top most edge check
	if(workerThreadInfo->verticalThreadId == 0)
	{



	}
	else
	{
		int overFlowAmount = 0;
		workerThreadInfo->myPaddedImageRegionRect.y= workerThreadInfo->myPaddedImageRegionRect.y-  myFeatureExtractionData->yBufferAmount;
		if(workerThreadInfo->myPaddedImageRegionRect.y<0)
		{

			overFlowAmount = workerThreadInfo->myPaddedImageRegionRect.y;
			workerThreadInfo->myPaddedImageRegionRect.y =0;
		}
		workerThreadInfo->myPaddedImageRegionRect.height = workerThreadInfo->myPaddedImageRegionRect.height+myFeatureExtractionData->yBufferAmount + overFlowAmount;




	}


	//Bottom most edge
	if(workerThreadInfo->verticalThreadId == workerThreadInfo->numberOfVerticalCores-1)
	{



	}
	else
	{
		int overFlowAmount = 0;
		int regionEndLocation = workerThreadInfo->myPaddedImageRegionRect.y+  myFeatureExtractionData->yBufferAmount;
		if(regionEndLocation> imageSize.height)
		{

			overFlowAmount = imageSize.height- regionEndLocation;

		}
		workerThreadInfo->myPaddedImageRegionRect.height = workerThreadInfo->myPaddedImageRegionRect.height+myFeatureExtractionData->yBufferAmount + overFlowAmount;



	}

	//Regions: Padded and NonPadded Are now officially computed


}







void  featureExtractionWorkerThreadSetupFunctionStandAlone(struct GeneralWorkerThreadData * genData,void * workerThreadStruct)
{
	struct FeatureExtractionWorkerThreadInfo * workerThreadInfo = reinterpret_cast<FeatureExtractionWorkerThreadInfo *> (workerThreadStruct);
	Thread * myThread = genData->myThread;
	struct FeatureExtractionMultiThreadData * myMultiThreadExtractionData = reinterpret_cast<FeatureExtractionMultiThreadData *>(workerThreadInfo->multiThreadAlgorithmData);
	FeatureExtractionData * myFeatureExtractionData = workerThreadInfo->myFeatureExtractionData;


	Size imageSize = workerThreadInfo->myFeatureExtractionData->frameSize;

	int nonPaddedRegionWidth = imageSize.width/workerThreadInfo->numberOfHorizontalCores;
	int nonPaddedRegionHeight = imageSize.height/workerThreadInfo->numberOfVerticalCores;

	//Set the x and y location of the unpadded
	workerThreadInfo->myNonPaddedImageRegionRect.x = workerThreadInfo->horizontalThreadId *	nonPaddedRegionWidth;
	workerThreadInfo->myNonPaddedImageRegionRect.y = workerThreadInfo->verticalThreadId *	nonPaddedRegionHeight;

	//NonPadded
	if(workerThreadInfo->horizontalThreadId == 0)
	{



	}

	if(workerThreadInfo->horizontalThreadId == workerThreadInfo->numberOfHorizontalCores-1)
	{
		nonPaddedRegionWidth = imageSize.width - (workerThreadInfo->numberOfHorizontalCores-1)*nonPaddedRegionWidth;


	}


	if(workerThreadInfo->verticalThreadId == 0)
	{



	}

	if(workerThreadInfo->verticalThreadId == workerThreadInfo->numberOfVerticalCores-1)
	{
		nonPaddedRegionHeight = imageSize.height - (workerThreadInfo->numberOfVerticalCores-1)*nonPaddedRegionHeight;


	}



	//Set the height and width of the unpadded
	workerThreadInfo->myNonPaddedImageRegionRect.height =nonPaddedRegionHeight;
	workerThreadInfo->myNonPaddedImageRegionRect.width =nonPaddedRegionWidth;


	//First start the padded with the unpadded Info
	workerThreadInfo->myPaddedImageRegionRect =workerThreadInfo->myNonPaddedImageRegionRect;

	//Now check the edges to determine what to modify
	//Left most edge check
	if(workerThreadInfo->horizontalThreadId == 0)
	{



	}
	else
	{

		int overFlowAmount = 0;
		workerThreadInfo->myPaddedImageRegionRect.x= workerThreadInfo->myPaddedImageRegionRect.x-  myFeatureExtractionData->xBufferAmount;
		if(workerThreadInfo->myPaddedImageRegionRect.x<0)
		{

			overFlowAmount = workerThreadInfo->myPaddedImageRegionRect.x;
			workerThreadInfo->myPaddedImageRegionRect.x =0;
		}
		workerThreadInfo->myPaddedImageRegionRect.width = workerThreadInfo->myPaddedImageRegionRect.width+myFeatureExtractionData->xBufferAmount + overFlowAmount;


	}


	//right most edge check
	if(workerThreadInfo->horizontalThreadId == workerThreadInfo->numberOfHorizontalCores-1)
	{



	}
	else
	{
		int overFlowAmount = 0;
		int regionEndXLocation = workerThreadInfo->myPaddedImageRegionRect.x+  myFeatureExtractionData->xBufferAmount;
		if(regionEndXLocation> imageSize.width)
		{

			overFlowAmount = imageSize.width- regionEndXLocation;

		}
		workerThreadInfo->myPaddedImageRegionRect.width = workerThreadInfo->myPaddedImageRegionRect.width+myFeatureExtractionData->xBufferAmount + overFlowAmount;
	}


	//Top most edge check
	if(workerThreadInfo->verticalThreadId == 0)
	{



	}
	else
	{
		int overFlowAmount = 0;
		workerThreadInfo->myPaddedImageRegionRect.y= workerThreadInfo->myPaddedImageRegionRect.y-  myFeatureExtractionData->yBufferAmount;
		if(workerThreadInfo->myPaddedImageRegionRect.y<0)
		{

			overFlowAmount = workerThreadInfo->myPaddedImageRegionRect.y;
			workerThreadInfo->myPaddedImageRegionRect.y =0;
		}
		workerThreadInfo->myPaddedImageRegionRect.height = workerThreadInfo->myPaddedImageRegionRect.height+myFeatureExtractionData->yBufferAmount + overFlowAmount;




	}


	//Bottom most edge
	if(workerThreadInfo->verticalThreadId == workerThreadInfo->numberOfVerticalCores-1)
	{



	}
	else
	{
		int overFlowAmount = 0;
		int regionEndLocation = workerThreadInfo->myPaddedImageRegionRect.y+  myFeatureExtractionData->yBufferAmount;
		if(regionEndLocation> imageSize.height)
		{

			overFlowAmount = imageSize.height- regionEndLocation;

		}
		workerThreadInfo->myPaddedImageRegionRect.height = workerThreadInfo->myPaddedImageRegionRect.height+myFeatureExtractionData->yBufferAmount + overFlowAmount;



	}

	//Regions: Padded and NonPadded Are now officially computed




}
void featureExtractionHandleKeypointRedistribution(struct GeneralWorkerThreadData * genData,FeatureExtractionWorkerThreadInfo * workerThreadInfo)
{
	Thread * myThread = genData->myThread;
	struct FeatureExtractionMultiThreadData * myMultiThreadExtractionData = reinterpret_cast<FeatureExtractionMultiThreadData *>(workerThreadInfo->multiThreadAlgorithmData);
	FeatureExtractionData * myFeatureExtractionData = workerThreadInfo->myFeatureExtractionData;
	FeatureExtractionConfig * myFeatureExtractionConfig = workerThreadInfo->myFeatureExtractionInfo;




	//Put your data in memory
	for(int i = 0 ; i<myMultiThreadExtractionData->myKeypoints.size(); i++)
	{
		(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].pt = myMultiThreadExtractionData->myKeypoints[i].pt;
		(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].angle = myMultiThreadExtractionData->myKeypoints[i].angle;
		(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].class_id = myMultiThreadExtractionData->myKeypoints[i].class_id;
		(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].octave = myMultiThreadExtractionData->myKeypoints[i].octave;
		(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].response = myMultiThreadExtractionData->myKeypoints[i].response;
		(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].size = myMultiThreadExtractionData->myKeypoints[i].size;
	}

	myThread->waitAtBarrier();

	//Coordinator work out how to grab the data
	if(workerThreadInfo->coordinator)
	{


		int totalKeyPoints=myMultiThreadExtractionData->keypoints->size();
		int keypointsPerCore = totalKeyPoints/workerThreadInfo->numberOfCores;
		int currentCoreIndex = 0;


		for(int i = 0; i< workerThreadInfo->numberOfCores; i++)
		{

			Thread * workerThread = workerThreadInfo->threadManager->getThread(i);
			struct GeneralWorkerThreadData * currentGenData = reinterpret_cast<struct GeneralWorkerThreadData *>(workerThread->getThreadParam());
			struct FeatureExtractionWorkerThreadInfo * currentWorkerThreadInfo = reinterpret_cast<FeatureExtractionWorkerThreadInfo *> (currentGenData->featureExtractionData  );
			struct FeatureExtractionMultiThreadData * currentThreadMultiThreadData =  reinterpret_cast<FeatureExtractionMultiThreadData *>(currentWorkerThreadInfo->multiThreadAlgorithmData);

			if(i< workerThreadInfo->numberOfCores-1)
			{
				currentThreadMultiThreadData->myKeyPointsCount = keypointsPerCore;
				currentThreadMultiThreadData->myKeyPointsIndex = currentCoreIndex;



			}
			else
			{
				currentThreadMultiThreadData->myKeyPointsCount = totalKeyPoints-currentCoreIndex;
				currentThreadMultiThreadData->myKeyPointsIndex = currentCoreIndex;
			}

				currentCoreIndex+=keypointsPerCore;

		}

	}


	myThread->waitAtBarrier();

	//Get your assigned keypoints from memory
	myMultiThreadExtractionData->myKeypoints.resize(myMultiThreadExtractionData->myKeyPointsCount);
	for(int i =0; i < myMultiThreadExtractionData->myKeypoints.size(); i++)
	{
		myMultiThreadExtractionData->myKeypoints[i].pt=(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].pt;
		myMultiThreadExtractionData->myKeypoints[i].angle=(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].angle;
		myMultiThreadExtractionData->myKeypoints[i].class_id = (*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].class_id;
		myMultiThreadExtractionData->myKeypoints[i].octave=(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].octave;
		myMultiThreadExtractionData->myKeypoints[i].response=(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].response;
		myMultiThreadExtractionData->myKeypoints[i].size=(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].size;
	}


}
void featureExtractionSetThreadImageRegion(struct GeneralWorkerThreadData * genData,FeatureExtractionWorkerThreadInfo * workerThreadInfo)
{
	workerThreadInfo->myNonPaddedImageRegion = (*(workerThreadInfo->featureExtractionData->inputImagesPerFrame))[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA](workerThreadInfo->myNonPaddedImageRegionRect);
	workerThreadInfo->myPaddedImageRegion = (*(workerThreadInfo->featureExtractionData->inputImagesPerFrame))[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA](workerThreadInfo->myPaddedImageRegionRect);



}


void featureExtractionHandleMultiThreadLocalizeFeaturePoints(struct GeneralWorkerThreadData * genData,FeatureExtractionWorkerThreadInfo * workerThreadInfo)
{
	Thread * myThread = genData->myThread;
	struct FeatureExtractionMultiThreadData * myMultiThreadExtractionData = reinterpret_cast<FeatureExtractionMultiThreadData *>(workerThreadInfo->multiThreadAlgorithmData);
	FeatureExtractionData * myFeatureExtractionData = workerThreadInfo->myFeatureExtractionData;
	FeatureExtractionConfig * myFeatureExtractionConfig = workerThreadInfo->myFeatureExtractionInfo;


	if(myFeatureExtractionConfig->currentLocalizationAlgo == FEATURE_LOCALIZATION_HoG)
	{

		LocalizeFeaturePoints(workerThreadInfo->myPaddedImageRegion,  myMultiThreadExtractionData->myKeypoints, myMultiThreadExtractionData->myBboxes,*myFeatureExtractionConfig,*myFeatureExtractionData);
	}




	if(!workerThreadInfo->myFeatureExtractionInfo->sameLocalAndDesc  || workerThreadInfo->myFeatureExtractionInfo->redistribute)
	{

		LocalizeFeaturePoints(workerThreadInfo->myPaddedImageRegion,  myMultiThreadExtractionData->myKeypoints, *myFeatureExtractionConfig,myFeatureExtractionData->featureDetector );


	}
	else
	{
		LocalizeFeaturePoints(workerThreadInfo->myPaddedImageRegion,  myMultiThreadExtractionData->myKeypoints, *myFeatureExtractionConfig,myMultiThreadExtractionData->myDescriptors );



	}


#ifdef VERBOSE_DEBUG
	cout << "KeyPoints Count " << myMultiThreadExtractionData->myKeypoints.size() <<endl;
#endif


	//If I am to redistribute them then I had better put them back to universal coordinates before I am done
	//And put them in the keypoints repository
	if(workerThreadInfo->myFeatureExtractionInfo->redistribute)
	{
		vector<KeyPoint> newMyKeypoints;
		newMyKeypoints.reserve(myMultiThreadExtractionData->myKeypoints.size());
		KeyPoint tmpKeypoint;
		Rect myRegion = workerThreadInfo->myNonPaddedImageRegionRect;


		//Adjust the points to fit the whole image
		//Filter the points
		for(int i = 0; i < myMultiThreadExtractionData->myKeypoints.size(); i++)
		{
			tmpKeypoint = myMultiThreadExtractionData->myKeypoints[i];
			tmpKeypoint.pt.x += workerThreadInfo->myPaddedImageRegionRect.x;
			tmpKeypoint.pt.y += workerThreadInfo->myPaddedImageRegionRect.y;

			if(myRegion.contains(tmpKeypoint.pt))
			{
				newMyKeypoints.push_back(tmpKeypoint);

			}
		}

		myMultiThreadExtractionData->myKeypoints = newMyKeypoints;
		myMultiThreadExtractionData->myKeyPointsCount = myMultiThreadExtractionData->myKeypoints.size();


#ifdef VERBOSE_DEBUG
		cout << "Filtered KeyPoints Count " << myMultiThreadExtractionData->myKeypoints.size() <<endl;
#endif

		myThread->waitAtBarrier();


		if(workerThreadInfo->coordinator)
		{
			vector<struct FeatureExtractionWorkerThreadInfo *> allThreadsWorkerInfos;
			vector<struct FeatureExtractionMultiThreadData *> allThreadsMultiThreadData;


			int totalKeyPoints=0;
			for(int i = 0; i< workerThreadInfo->numberOfCores; i++)
			{

				Thread * workerThread = workerThreadInfo->threadManager->getThread(i);
				struct GeneralWorkerThreadData * currentGenData = reinterpret_cast<struct GeneralWorkerThreadData *>(workerThread->getThreadParam());
				struct FeatureExtractionWorkerThreadInfo * currentWorkerThreadInfo = reinterpret_cast<FeatureExtractionWorkerThreadInfo *> (currentGenData->featureExtractionData  );
				struct FeatureExtractionMultiThreadData * currentThreadMultiThreadData =  reinterpret_cast<FeatureExtractionMultiThreadData *>(currentWorkerThreadInfo->multiThreadAlgorithmData);
				allThreadsWorkerInfos.push_back(currentWorkerThreadInfo);

				allThreadsMultiThreadData.push_back(currentThreadMultiThreadData);


				currentThreadMultiThreadData->myKeyPointsIndex = totalKeyPoints;

				totalKeyPoints += currentThreadMultiThreadData->myKeyPointsCount;


			}
#ifdef VERBOSE_DEBUG
			cout << "As per your wish there are " << totalKeyPoints << " for us.  We shall share them equally" << endl;
#endif

			myMultiThreadExtractionData->keypoints->clear();
			myMultiThreadExtractionData->keypoints->resize(totalKeyPoints);

		}


		myThread->waitAtBarrier();
		featureExtractionHandleKeypointRedistribution(genData,workerThreadInfo);

	}
	else
	{

		vector<KeyPoint> newMyKeypoints;
		newMyKeypoints.reserve(myMultiThreadExtractionData->myKeypoints.size());
		KeyPoint tmpKeypoint;
		Rect myRegion = workerThreadInfo->myNonPaddedImageRegionRect;
		Mat newDescriptors;

		newDescriptors.create(0,myMultiThreadExtractionData->myDescriptors.cols,myMultiThreadExtractionData->myDescriptors.type());

		//Adjust the points to fit the whole image
		//Filter the points
		for(int i = 0; i < myMultiThreadExtractionData->myKeypoints.size(); i++)
		{
			tmpKeypoint = myMultiThreadExtractionData->myKeypoints[i];
			tmpKeypoint.pt.x += workerThreadInfo->myPaddedImageRegionRect.x;
			tmpKeypoint.pt.y += workerThreadInfo->myPaddedImageRegionRect.y;

			if(myRegion.contains(tmpKeypoint.pt))
			{
				newMyKeypoints.push_back(myMultiThreadExtractionData->myKeypoints[i]);

				if(myFeatureExtractionConfig->sameLocalAndDesc)
				{
					newDescriptors.push_back(myMultiThreadExtractionData->myDescriptors.row(i));

				}


			}
		}


		myMultiThreadExtractionData->myDescriptors = newDescriptors;
		myMultiThreadExtractionData->myKeypoints = newMyKeypoints;
		myMultiThreadExtractionData->myKeyPointsCount = myMultiThreadExtractionData->myKeypoints.size();

#ifdef VERBOSE_DEBUG
		cout << "Filtered KeyPoints Count " << myMultiThreadExtractionData->myKeypoints.size() <<endl;
#endif


		myMultiThreadExtractionData->myKeyPointsCount = myMultiThreadExtractionData->myKeypoints.size();

	}

}

void featureExtractionHandleMultiThreadBuildFeatureDescriptors(struct GeneralWorkerThreadData * genData,FeatureExtractionWorkerThreadInfo * workerThreadInfo)
{
	//struct FeatureExtractionWorkerThreadInfo * workerThreadInfo = reinterpret_cast<FeatureExtractionWorkerThreadInfo *> (workerThreadStruct);
	Thread * myThread = genData->myThread;
	struct FeatureExtractionMultiThreadData * myMultiThreadExtractionData = reinterpret_cast<FeatureExtractionMultiThreadData *>(workerThreadInfo->multiThreadAlgorithmData);
	FeatureExtractionData * myFeatureExtractionData = workerThreadInfo->myFeatureExtractionData;
	FeatureExtractionConfig * myFeatureExtractionConfig = workerThreadInfo->myFeatureExtractionInfo;


	Mat regionToUseForComputation;

	if(myFeatureExtractionConfig->redistribute)
	{
		regionToUseForComputation = (*(myFeatureExtractionData->inputImagesPerFrame))[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA];



	}
	else
	{
		regionToUseForComputation = workerThreadInfo->myPaddedImageRegion;

	}


	if(myFeatureExtractionConfig->currentDescriptorAlgo == FEATURE_DESC_HoG)
	{
		//cout << "Build Feature Desc" << endl;
		myMultiThreadExtractionData->myDescriptors =  buildFeatureDescriptors(regionToUseForComputation, myMultiThreadExtractionData->myKeypoints,myMultiThreadExtractionData->myBboxes,*myFeatureExtractionConfig,myFeatureExtractionData->descHogPtr);
		//cout << "Boxes: "<<myMultiThreadExtractionData->myBboxes.size() << endl;
	}
	else
	{
		if(!myFeatureExtractionConfig->sameLocalAndDesc)
		{
			myMultiThreadExtractionData->myDescriptors =  buildFeatureDescriptors(regionToUseForComputation, myMultiThreadExtractionData->myKeypoints,*myFeatureExtractionConfig,myFeatureExtractionData->descriptorExtractor);

		}
	}


	myMultiThreadExtractionData->myDescriptorsCount = myMultiThreadExtractionData->myDescriptors.rows;
	if(myMultiThreadExtractionData->myDescriptorsCount != myMultiThreadExtractionData->myKeyPointsCount)
	{
		myMultiThreadExtractionData->myKeyPointsCount = myMultiThreadExtractionData->myKeypoints.size();
	}
	if(myMultiThreadExtractionData->myDescriptorsCount != myMultiThreadExtractionData->myKeypoints.size())
	{
		if(!(myFeatureExtractionConfig->runSlidingWindowLocal && myFeatureExtractionConfig->currentDescriptorAlgo==FEATURE_DESC_HoG))
			{
				//SHould not happen but just in case
				cout << "WARNING: Keypoint and descriptor missmatch.";

			}
	}



	myThread->waitAtBarrier();


	// TODO Add in code for putting descriptors together
	//Suggest add the total of descriptors and then have each one grab the rows they need based on index and then they can copy to it
	//Remembered that I need the descriptors to match the keypoint so I need to put them back in order the keypoints are stored
	//I will do this outside this function


}

void featureExtractionHandleFeatureExtractionWorker(struct GeneralWorkerThreadData * genData,FeatureExtractionWorkerThreadInfo * workerThreadInfo)
{
	//struct FeatureExtractionWorkerThreadInfo * workerThreadInfo = reinterpret_cast<FeatureExtractionWorkerThreadInfo *> (workerThreadStruct);
	Thread * myThread = genData->myThread;
	struct FeatureExtractionMultiThreadData * myMultiThreadExtractionData = reinterpret_cast<FeatureExtractionMultiThreadData *>(workerThreadInfo->multiThreadAlgorithmData);
	FeatureExtractionData * myFeatureExtractionData = workerThreadInfo->myFeatureExtractionData;
	FeatureExtractionConfig * myFeatureExtractionConfig = workerThreadInfo->myFeatureExtractionInfo;


	featureExtractionHandleMultiThreadLocalizeFeaturePoints(genData,workerThreadInfo);
//	if(workerThreadInfo->featureExtractionInfo->redistribute)
//	{
//		myThread->waitAtBarrier();
//	}

	// TODO Add in descriptor building right here --->
	featureExtractionHandleMultiThreadBuildFeatureDescriptors(genData,workerThreadInfo);

	int totalKeyPoints=0;
	//We did not redistribute or organize before now so I will go ahead and get how many things we found now
	//And adjust their location and store the keypoints
	if(!workerThreadInfo->featureExtractionInfo->redistribute)
	{
		myThread->waitAtBarrier();

		for(int i = 0 ; i<myMultiThreadExtractionData->myKeypoints.size(); i++)
		{

			//Adjust the Key point back to global coordinates
			Point2f tmpPoint = myMultiThreadExtractionData->myKeypoints[i].pt;
			tmpPoint.x += workerThreadInfo->myPaddedImageRegionRect.x;
			tmpPoint.y += workerThreadInfo->myPaddedImageRegionRect.y;

			(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].pt = tmpPoint;
			(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].angle = myMultiThreadExtractionData->myKeypoints[i].angle;
			(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].class_id = myMultiThreadExtractionData->myKeypoints[i].class_id;
			(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].octave = myMultiThreadExtractionData->myKeypoints[i].octave;
			(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].response = myMultiThreadExtractionData->myKeypoints[i].response;
			(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].size = myMultiThreadExtractionData->myKeypoints[i].size;
		}



	}





	myThread->waitAtBarrier();

	myMultiThreadExtractionData->myDescriptorsCount = myMultiThreadExtractionData->myKeyPointsCount;
	myMultiThreadExtractionData->myDescriptorsIndex = myMultiThreadExtractionData->myKeyPointsIndex;

	Mat myDescriptorRows= myMultiThreadExtractionData->descriptors->rowRange(myMultiThreadExtractionData->myDescriptorsIndex,myMultiThreadExtractionData->myDescriptorsIndex+myMultiThreadExtractionData->myDescriptorsCount);

	myMultiThreadExtractionData->myDescriptors.copyTo(myDescriptorRows);


	myThread->waitAtBarrier();
}

void featureExtractionHandleFeatureExtractionCoordinator(struct GeneralWorkerThreadData * genData,FeatureExtractionWorkerThreadInfo * workerThreadInfo)
{
	//struct FeatureExtractionWorkerThreadInfo * workerThreadInfo = reinterpret_cast<FeatureExtractionWorkerThreadInfo *> (workerThreadStruct);
	Thread * myThread = genData->myThread;
	struct FeatureExtractionMultiThreadData * myMultiThreadExtractionData = reinterpret_cast<FeatureExtractionMultiThreadData *>(workerThreadInfo->multiThreadAlgorithmData);
	FeatureExtractionData * myFeatureExtractionData = workerThreadInfo->myFeatureExtractionData;
	FeatureExtractionConfig * myFeatureExtractionConfig = workerThreadInfo->myFeatureExtractionInfo;


	featureExtractionHandleMultiThreadLocalizeFeaturePoints(genData,workerThreadInfo);
//	if(workerThreadInfo->featureExtractionInfo->redistribute)
//	{
//		myThread->waitAtBarrier();
//	}

#ifdef USE_GEM5
	m5_dumpreset_stats(0, 0);
#endif

	// TODO Add in descriptor building right here --->
	featureExtractionHandleMultiThreadBuildFeatureDescriptors(genData,workerThreadInfo);


	int totalKeyPoints=0;
	//We did not redistribute or organize before now so I will go ahead and get how many things we found now
	if(!workerThreadInfo->featureExtractionInfo->redistribute)
	{

		for(int i = 0; i< workerThreadInfo->numberOfCores; i++)
		{

			Thread * workerThread = workerThreadInfo->threadManager->getThread(i);
			struct GeneralWorkerThreadData * currentGenData = reinterpret_cast<struct GeneralWorkerThreadData *>(workerThread->getThreadParam());
			struct FeatureExtractionWorkerThreadInfo * currentWorkerThreadInfo = reinterpret_cast<FeatureExtractionWorkerThreadInfo *> (currentGenData->featureExtractionData  );
			struct FeatureExtractionMultiThreadData * currentThreadMultiThreadData =  reinterpret_cast<FeatureExtractionMultiThreadData *>(currentWorkerThreadInfo->multiThreadAlgorithmData);


			currentThreadMultiThreadData->myKeyPointsIndex = totalKeyPoints;

			if((currentThreadMultiThreadData->myDescriptorsCount != currentThreadMultiThreadData->myKeyPointsCount))
			{
				if(!(currentWorkerThreadInfo->myFeatureExtractionInfo->runSlidingWindowLocal && currentWorkerThreadInfo->myFeatureExtractionInfo->currentDescriptorAlgo==FEATURE_DESC_HoG))
				{
					//SHould not happen but just in case
					cout << "WARNING: Keypoint and descriptor missmatch.";

				}
				else if (currentThreadMultiThreadData->myKeyPointsCount == 0)
				{
					currentThreadMultiThreadData->myKeyPointsCount = currentThreadMultiThreadData->myDescriptorsCount;

				}
			}

			totalKeyPoints += currentThreadMultiThreadData->myKeyPointsCount;


		}



		myMultiThreadExtractionData->keypoints->resize(totalKeyPoints);


		myThread->waitAtBarrier();

		for(int i = 0 ; i<myMultiThreadExtractionData->myKeypoints.size(); i++)
		{
			(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].pt = myMultiThreadExtractionData->myKeypoints[i].pt;
			(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].angle = myMultiThreadExtractionData->myKeypoints[i].angle;
			(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].class_id = myMultiThreadExtractionData->myKeypoints[i].class_id;
			(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].octave = myMultiThreadExtractionData->myKeypoints[i].octave;
			(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].response = myMultiThreadExtractionData->myKeypoints[i].response;
			(*(myMultiThreadExtractionData->keypoints))[i+myMultiThreadExtractionData->myKeyPointsIndex].size = myMultiThreadExtractionData->myKeypoints[i].size;
		}



	}
	totalKeyPoints = myMultiThreadExtractionData->keypoints->size();



	myMultiThreadExtractionData->descriptors->create(totalKeyPoints,myMultiThreadExtractionData->myDescriptors.cols,myMultiThreadExtractionData->myDescriptors.type());
	myThread->waitAtBarrier();

	myMultiThreadExtractionData->myDescriptorsCount = myMultiThreadExtractionData->myKeyPointsCount;
	myMultiThreadExtractionData->myDescriptorsIndex = myMultiThreadExtractionData->myKeyPointsIndex;

	Mat myDescriptorRows= myMultiThreadExtractionData->descriptors->rowRange(myMultiThreadExtractionData->myDescriptorsIndex,myMultiThreadExtractionData->myDescriptorsIndex+myMultiThreadExtractionData->myDescriptorsCount);

	myMultiThreadExtractionData->myDescriptors.copyTo(myDescriptorRows);

	myThread->waitAtBarrier();
}



void  featureExtractionCoordinatorThreadFunctionStandAlone(struct GeneralWorkerThreadData * genData,void * workerThreadStruct)
{
	struct FeatureExtractionWorkerThreadInfo * workerThreadInfo = reinterpret_cast<FeatureExtractionWorkerThreadInfo *> (workerThreadStruct);
	Thread * myThread = genData->myThread;
	struct FeatureExtractionMultiThreadData * myMultiThreadExtractionData = reinterpret_cast<FeatureExtractionMultiThreadData *>(workerThreadInfo->multiThreadAlgorithmData);

	myMultiThreadExtractionData->clearNonShared();
	myMultiThreadExtractionData->clear();
	int imageCountRemaining = getNextImages(*(workerThreadInfo->featureExtractionData->inputImagesPerFrame),*(workerThreadInfo->featureExtractionInfo));
	if(imageCountRemaining ==1)
	{
		workerThreadInfo->done = true;

	}
	myThread->waitAtBarrier();
	if(workerThreadInfo->featureExtractionInfo->outOfImages || workerThreadInfo->myFeatureExtractionInfo->outOfImages)
	{

		return;
	}

	featureExtractionSetThreadImageRegion(genData,workerThreadInfo);

#ifdef VERBOSE_DEBUG
	cout << "I am the very model " << genData->myThread->getThreadLogicalId() << " ."<<endl;
#endif

	featureExtractionHandleFeatureExtractionCoordinator(genData,workerThreadInfo);

	myThread->waitAtBarrier();
#ifdef VERBOSE_DEBUG
	cout<< "Feature Extraction Complete" << endl;
	//featureExtractionHandleFeatureExtractionCoordinator(genData,workerThreadInfo);
#endif

}


void  featureExtractionWorkerThreadFunctionStandAlone(struct GeneralWorkerThreadData * genData,void * workerThreadStruct)
{
	struct FeatureExtractionWorkerThreadInfo * workerThreadInfo = reinterpret_cast<FeatureExtractionWorkerThreadInfo *> (workerThreadStruct);
	Thread * myThread = genData->myThread;
	struct FeatureExtractionMultiThreadData * myMultiThreadExtractionData = reinterpret_cast<FeatureExtractionMultiThreadData *>(workerThreadInfo->multiThreadAlgorithmData);
	myMultiThreadExtractionData->clearNonShared();
	myThread->waitAtBarrier();
	if(workerThreadInfo->featureExtractionInfo->outOfImages || workerThreadInfo->myFeatureExtractionInfo->outOfImages)
	{

		return;
	}



	featureExtractionSetThreadImageRegion(genData,workerThreadInfo);
#ifdef VERBOSE_DEBUG
	cout << "of a cartoon individual " << genData->myThread->getThreadLogicalId() << " ."<<endl;
#endif

	featureExtractionHandleFeatureExtractionWorker(genData,workerThreadInfo);

	myThread->waitAtBarrier();

	//featureExtractionHandleFeatureExtractionWorker(genData,workerThreadInfo);




}

void * featureExtraction_testCoordinatorThreadStandAlone(void * threadParam)
{
	struct GeneralWorkerThreadData * genData = reinterpret_cast<struct GeneralWorkerThreadData *>(threadParam);


	featureExtractionCoordinatorThreadSetupFunctionStandAlone(genData,genData->featureExtractionData);
#ifdef USE_MARSS

	struct FeatureExtractionWorkerThreadInfo * workerThreadInfo = reinterpret_cast<FeatureExtractionWorkerThreadInfo *> (genData->featureExtractionData);
	cout << "Switching to simulation in Feature Extraction." << endl;
	ptlcall_switch_to_sim();
	switch(workerThreadInfo->featureExtractionInfo->currentDescriptorAlgo)
	{
	case FEATURE_DESC_SIFT:
		//ptlcall_single_enqueue("-logfile featureExtract_sift.log");
		ptlcall_single_enqueue("-stats featureExtract_sift.stats");
		//ptlcall_single_enqueue("-loglevel 0");
		break;
	case FEATURE_DESC_SURF:
		//ptlcall_single_enqueue("-logfile featureExtract_surf.log");
		ptlcall_single_enqueue("-stats featureExtract_surf.stats");
		//ptlcall_single_enqueue("-loglevel 0");
		break;
	case FEATURE_DESC_ORB:
		//ptlcall_single_enqueue("-logfile featureExtract_surf.log");
		ptlcall_single_enqueue("-stats featureExtract_orb.stats");
		//ptlcall_single_enqueue("-loglevel 0");
		break;
	case FEATURE_DESC_FAST:
		//ptlcall_single_enqueue("-logfile featureExtract_fast.log");
		ptlcall_single_enqueue("-stats featureExtract_fast.stats");
		//ptlcall_single_enqueue("-loglevel 0");
		break;
	case FEATURE_DESC_HoG:
		//ptlcall_single_enqueue("-logfile featureExtract_hog.log");
		ptlcall_single_enqueue("-stats featureExtract_hog.stats");
		//ptlcall_single_enqueue("-loglevel 0");
		break;
	case FEATURE_DESC_BRIEF:
		//ptlcall_single_enqueue("-logfile featureExtract_brief.log");
		ptlcall_single_enqueue("-stats featureExtract_brief.stats");
		//ptlcall_single_enqueue("-loglevel 0");
		break;
	default:
		//ptlcall_single_enqueue("-logfile featureExtract.log");
		ptlcall_single_enqueue("-stats featureExtract.stats");
		//ptlcall_single_enqueue("-loglevel 0");
		break;

	};
	//ptlcall_single_enqueue("-logfile featureExtract.log");
	//ptlcall_single_enqueue("-stats featureExtract.stats");
	//ptlcall_single_enqueue("-loglevel 0");
#endif

#ifdef USE_GEM5
	#ifdef GEM5_CHECKPOINT_WORK
		m5_checkpoint(0, 0);
	#endif

	#ifdef GEM5_SWITCHCPU_AT_WORK
		m5_switchcpu();
	#endif 
	m5_dumpreset_stats(0, 0);
#endif


#ifdef TSC_TIMING
	READ_TIMESTAMP_WITH_WRAPPER( fe_timingVector[genData->myThread->getThreadLogicalId()] );
#endif
#ifdef CLOCK_GETTIME_TIMING
	GET_TIME_WRAPPER(fe_timeStructVector[genData->myThread->getThreadLogicalId()]);
#endif
	featureExtractionCoordinatorThreadFunctionStandAlone(genData,genData->featureExtractionData);
#ifdef USE_MARSS
	ptlcall_kill();
#endif
#ifdef USE_GEM5

	m5_dumpreset_stats(0, 0);
	#ifdef GEM5_EXIT_AFTER_WORK
		m5_exit(0);
	#endif


#endif


#ifdef TSC_TIMING
	READ_TIMESTAMP_WITH_WRAPPER( fe_timingVector[(genData->myThread->getThreadLogicalId() + fe_timingVector.size()/2)] );
#endif
#ifdef CLOCK_GETTIME_TIMING
	GET_TIME_WRAPPER(fe_timeStructVector[genData->myThread->getThreadLogicalId()+ fe_timeStructVector.size()/2]);
#endif


	return NULL;
}


void * featureExtraction_testWorkerThreadStandAlone(void * threadParam)
{
	struct GeneralWorkerThreadData * genData = reinterpret_cast<struct GeneralWorkerThreadData *>(threadParam);
	featureExtractionWorkerThreadSetupFunctionStandAlone(genData,genData->featureExtractionData);
#ifdef TSC_TIMING
	READ_TIMESTAMP_WITH_WRAPPER( fe_timingVector[genData->myThread->getThreadLogicalId()] );
#endif
#ifdef CLOCK_GETTIME_TIMING
	GET_TIME_WRAPPER(fe_timeStructVector[genData->myThread->getThreadLogicalId()]);
#endif
	featureExtractionWorkerThreadFunctionStandAlone(genData,genData->featureExtractionData);
#ifdef TSC_TIMING
	READ_TIMESTAMP_WITH_WRAPPER( fe_timingVector[(genData->myThread->getThreadLogicalId() + fe_timingVector.size()/2)] );
#endif
#ifdef CLOCK_GETTIME_TIMING
	GET_TIME_WRAPPER(fe_timeStructVector[genData->myThread->getThreadLogicalId()+ fe_timeStructVector.size()/2]);
#endif
	return NULL;
}




void featureExtractionCreateThreadManager(ThreadManager * &threadManager,int verticalThreads, int horizontalThreads)
{
	int numberOfThreads = verticalThreads*horizontalThreads;


	cout << "Creating Thread Manager and basic thread structs" << endl;
	threadManager = new ThreadManager(numberOfThreads);

	for(int i = 0; i<numberOfThreads;i++)
	{


		Thread * currentThreadPtr = (threadManager->getThread(i));


		currentThreadPtr->setThreadLogicalId(i);
		threadManager->setThreadBarrier(i);
	}

}

void setFeatureExtractionWorkingThreadDataInGeneralWorkingData(vector<GeneralWorkerThreadData> &genData, vector<struct FeatureExtractionWorkerThreadInfo> & workingThreads)
{

	for(int i = 0; i< genData.size() && i<workingThreads.size(); i++)
	{

		genData[i].featureExtractionData = (void *)&workingThreads[i];



	}


}


void getFeatureExtractionConfigForThread(FeatureExtractionConfig * &threadFeatureExtractionConfig,FeatureExtractionConfig &featureExtractionConfig,struct FeatureExtractionWorkerThreadInfo * workerInfo)
{
	if(featureExtractionConfig.singleCopyOfConfigStruct)
	{
		threadFeatureExtractionConfig = &featureExtractionConfig;

	}
	else if(featureExtractionConfig.partialCopyOfConfigStruct)
	{
		//Partial






	}
	else
	{
		//Individual pass the main and let them duplicate within or I could make it here.  Still up for thought
		threadFeatureExtractionConfig = &featureExtractionConfig;


	}

	return;
}


void getFeatureExtractionDataForThread(FeatureExtractionData * &threadFeatureExtractionData,FeatureExtractionData & featureExtractionData,FeatureExtractionConfig &featureExtractionConfig,struct FeatureExtractionWorkerThreadInfo * workerInfo)
{
	// TODO Change to setup single Feauture ExtractionData
	threadFeatureExtractionData = new FeatureExtractionData;


	//We can have the same frames
	threadFeatureExtractionData->inputImagesPerFrame =featureExtractionData.inputImagesPerFrame;

	//Train images same
	threadFeatureExtractionData->trainImagesPerFrame = featureExtractionData.trainImagesPerFrame;

	//We have the same frame size
	threadFeatureExtractionData->frameSize = featureExtractionData.frameSize;


	threadFeatureExtractionData->xBufferAmount = featureExtractionData.xBufferAmount;

	threadFeatureExtractionData->yBufferAmount = featureExtractionData.yBufferAmount;


	threadFeatureExtractionData->trainedDescriptors = featureExtractionData.trainedDescriptors;
	threadFeatureExtractionData->trainedRefPoint = featureExtractionData.trainedRefPoint;
	threadFeatureExtractionData->trainedKeypoints = featureExtractionData.trainedKeypoints;
	threadFeatureExtractionData->trainedRefVectors = featureExtractionData.trainedRefVectors;



	if(featureExtractionConfig.loadTrainedDescriptors)
	{
		string descTypeString, localTypeString;
		Mat tmpMat;
		vector<KeyPoint> tmpKeyPoint;


		loadFeatureDescriptorsAndOrKeypoints( featureExtractionConfig.descriptorsFilename,  descTypeString,localTypeString ,featureExtractionConfig.currentDescriptorAlgo,featureExtractionConfig.currentLocalizationAlgo,featureExtractionData.featureDetector,&tmpKeyPoint,featureExtractionData.descriptorExtractor, &tmpMat);
		threadFeatureExtractionData->sift= new SIFT;
		threadFeatureExtractionData->surf= new SURF;
		vector<Mat> trainDescriptorsVec;
		trainDescriptorsVec.push_back(*(featureExtractionData.trainedDescriptors));

		threadFeatureExtractionData->matcher= setupMatcher(featureExtractionConfig,trainDescriptorsVec);

	}
	else
	{
		threadFeatureExtractionData->featureDetector = setupLocalization(featureExtractionConfig);
		threadFeatureExtractionData->localizationHogPtr = featureExtractionConfig.localizationHogPtr;
		threadFeatureExtractionData->descriptorExtractor = setupDescriptor(featureExtractionConfig);
		threadFeatureExtractionData->descHogPtr = featureExtractionConfig.descHogPtr;
	}
	if(threadFeatureExtractionData->matcher ==NULL)
	{
		threadFeatureExtractionData->matcher = setupMatcher( featureExtractionConfig);

	}

	return;
}


MultiThreadAlgorithmData * setupFeatureExtractionMultiThreadedData(FeatureExtractionConfig featureExtractionConfig)
{
	MultiThreadAlgorithmData * returnVal = NULL;

	switch(featureExtractionConfig.currentLocalizationAlgo)
	{
		case FEATURE_LOCALIZATION_SIFT:
			{

				FeatureExtractionMultiThreadData * multiThreadData = new FeatureExtractionMultiThreadData;


				multiThreadData->keypoints = new vector<KeyPoint>;
				multiThreadData->descriptors = new Mat;


				returnVal = multiThreadData;



			}
			break;

		case FEATURE_LOCALIZATION_SURF:
			{

				FeatureExtractionMultiThreadData * multiThreadData = new FeatureExtractionMultiThreadData;



				multiThreadData->keypoints = new vector<KeyPoint>;
				multiThreadData->descriptors = new Mat;



				returnVal = multiThreadData;



			}
			break;
		case FEATURE_LOCALIZATION_ORB:
			{

				FeatureExtractionMultiThreadData * multiThreadData = new FeatureExtractionMultiThreadData;



				multiThreadData->keypoints = new vector<KeyPoint>;
				multiThreadData->descriptors = new Mat;



				returnVal = multiThreadData;



			}
			break;

		case FEATURE_LOCALIZATION_FAST:
			{

				FeatureExtractionMultiThreadData * multiThreadData = new FeatureExtractionMultiThreadData;

				multiThreadData->keypoints = new vector<KeyPoint>;
				multiThreadData->descriptors = new Mat;





				returnVal = multiThreadData;




			}
			break;
		case FEATURE_LOCALIZATION_HoG:
			{

				FeatureExtractionMultiThreadData * multiThreadData = new FeatureExtractionMultiThreadData;


				multiThreadData->keypoints = new vector<KeyPoint>;
				multiThreadData->descriptors = new Mat;




				returnVal = multiThreadData;



			break;
			}
		case FEATURE_LOCALIZATION_SLIDING_WINDOW:
			{
				FeatureExtractionMultiThreadData * multiThreadData = new FeatureExtractionMultiThreadData;

				multiThreadData->keypoints = new vector<KeyPoint>;
				multiThreadData->descriptors = new Mat;





				returnVal = multiThreadData;


			break;
			}

		default:
			FeatureExtractionMultiThreadData * multiThreadData = new FeatureExtractionMultiThreadData;



			multiThreadData->keypoints = new vector<KeyPoint>;
			multiThreadData->descriptors = new Mat;



			returnVal = multiThreadData;


			break;
	}

	switch(featureExtractionConfig.currentDescriptorAlgo)
	{
		case FEATURE_DESC_SIFT:
			{

			}
			break;

		case FEATURE_DESC_SURF:
			{


			}
			break;
		case FEATURE_DESC_ORB:
			{


			}
			break;
		case FEATURE_DESC_FAST:
			{


			}
			break;
		case FEATURE_DESC_HoG:
			{



			}
			break;
		case FEATURE_DESC_BRIEF:
			{
			}
			break;

		default:

			break;
	}




	return returnVal;



}

MultiThreadAlgorithmData * cloneFeatureExtractionMultiThreadedData(FeatureExtractionConfig featureExtractionConfig,MultiThreadAlgorithmData * baseData)
{

	MultiThreadAlgorithmData * returnVal = NULL;

	switch(featureExtractionConfig.currentLocalizationAlgo)
	{
		case FEATURE_LOCALIZATION_SIFT:
			{
				FeatureExtractionMultiThreadData * baseMultiThreadData = reinterpret_cast<FeatureExtractionMultiThreadData *>(baseData);
				FeatureExtractionMultiThreadData * multiThreadData = new FeatureExtractionMultiThreadData;
				*multiThreadData = *baseMultiThreadData;

				returnVal = multiThreadData;



			}
			break;

		case FEATURE_LOCALIZATION_SURF:
			{

				FeatureExtractionMultiThreadData * baseMultiThreadData = reinterpret_cast<FeatureExtractionMultiThreadData *>(baseData);
				FeatureExtractionMultiThreadData * multiThreadData = new FeatureExtractionMultiThreadData;
				*multiThreadData = *baseMultiThreadData;

				returnVal = multiThreadData;

			}
		case FEATURE_LOCALIZATION_ORB:
			{

				FeatureExtractionMultiThreadData * baseMultiThreadData = reinterpret_cast<FeatureExtractionMultiThreadData *>(baseData);
				FeatureExtractionMultiThreadData * multiThreadData = new FeatureExtractionMultiThreadData;
				*multiThreadData = *baseMultiThreadData;

				returnVal = multiThreadData;

			}
			break;
		case FEATURE_LOCALIZATION_FAST:
			{

				FeatureExtractionMultiThreadData * baseMultiThreadData = reinterpret_cast<FeatureExtractionMultiThreadData *>(baseData);
				FeatureExtractionMultiThreadData * multiThreadData = new FeatureExtractionMultiThreadData;
				*multiThreadData = *baseMultiThreadData;

				returnVal = multiThreadData;

			}
			break;
		case FEATURE_LOCALIZATION_HoG:
			{

				FeatureExtractionMultiThreadData * baseMultiThreadData = reinterpret_cast<FeatureExtractionMultiThreadData *>(baseData);
				FeatureExtractionMultiThreadData * multiThreadData = new FeatureExtractionMultiThreadData;
				*multiThreadData = *baseMultiThreadData;

				returnVal = multiThreadData;


			break;
			}
		case FEATURE_LOCALIZATION_SLIDING_WINDOW:
			{
				FeatureExtractionMultiThreadData * baseMultiThreadData = reinterpret_cast<FeatureExtractionMultiThreadData *>(baseData);
				FeatureExtractionMultiThreadData * multiThreadData = new FeatureExtractionMultiThreadData;
				*multiThreadData = *baseMultiThreadData;

				returnVal = multiThreadData;


			break;
			}

		default:
			FeatureExtractionMultiThreadData * baseMultiThreadData = reinterpret_cast<FeatureExtractionMultiThreadData *>(baseData);
			FeatureExtractionMultiThreadData * multiThreadData = new FeatureExtractionMultiThreadData;
			*multiThreadData = *baseMultiThreadData;

			returnVal = multiThreadData;

			break;
	}

	switch(featureExtractionConfig.currentDescriptorAlgo)
	{
		case FEATURE_DESC_SIFT:
			{

			}
			break;

		case FEATURE_DESC_SURF:
			{


			}
			break;
		case FEATURE_DESC_ORB:
			{


			}
			break;
		case FEATURE_DESC_FAST:
			{


			}
			break;
		case FEATURE_DESC_HoG:
			{



			}
			break;
		case FEATURE_DESC_BRIEF:
			{
			}
			break;

		default:

			break;
	}




	return returnVal;






}
void setupFeatureExtractionThreadsData(FeatureExtractionConfig & featureExtractionConfig, FeatureExtractionData & featureExtractionData, ThreadManager * &threadManager,vector<struct GeneralWorkerThreadData> &genData,int verticalThreads, int horizontalThreads)
{



	int numberOfThreads = verticalThreads*horizontalThreads;



	cout << "Creating woking threads structs" << endl;
	//workingThreads.clear();
	//workingThreads.resize(numberOfThreads);

	//Work on this one.....
	//multiThreadResultPtr = new MultiThreadedMatResult(64);
	//multiThreadResultPtr -> createResultsMat(queryDescriptors.rows,1);

	MultiThreadAlgorithmData * multiThreadAlgorithmDataBasePtr = setupFeatureExtractionMultiThreadedData(featureExtractionConfig);

	cout << "Filling in Data structs and creating worker threads" << endl;

	for(int i = 0; i<numberOfThreads;i++)
	{

		Thread * currentThreadPtr = (threadManager->getThread(i));
		struct FeatureExtractionWorkerThreadInfo * currentWorkerThreadInfo = (struct FeatureExtractionWorkerThreadInfo *)genData[i].featureExtractionData;
				//&workingThreads[i];
		genData[i].myThread = currentThreadPtr;
		genData[i].threadManager = threadManager;

		currentWorkerThreadInfo->myThread =currentThreadPtr;
		currentWorkerThreadInfo->threadManager = threadManager;
		currentWorkerThreadInfo->verticalThreadId = i/horizontalThreads;
		currentWorkerThreadInfo->horizontalThreadId = i%horizontalThreads;
		currentWorkerThreadInfo->coordinator = (i==0? true: false);
		currentWorkerThreadInfo->standAlone = true;
		currentWorkerThreadInfo->done= false;
		currentWorkerThreadInfo->numberOfCores = numberOfThreads;
		currentWorkerThreadInfo->numberOfHorizontalCores = horizontalThreads;
		currentWorkerThreadInfo->numberOfVerticalCores = verticalThreads;
		currentWorkerThreadInfo->multiThreadAlgorithmData = cloneFeatureExtractionMultiThreadedData(featureExtractionConfig,multiThreadAlgorithmDataBasePtr );

		//Get the config Data
		getFeatureExtractionConfigForThread(currentWorkerThreadInfo->myFeatureExtractionInfo,featureExtractionConfig,currentWorkerThreadInfo);
		//getFeatureClassificationInfoForThread(currentWorkerThreadInfo->myFeatureClassificationInfo,featureClassificationConfig,currentWorkerThreadInfo);
		currentWorkerThreadInfo->featureExtractionInfo = &featureExtractionConfig;


		getFeatureExtractionDataForThread(currentWorkerThreadInfo->myFeatureExtractionData,featureExtractionData,featureExtractionConfig,currentWorkerThreadInfo);
		currentWorkerThreadInfo->featureExtractionData = &featureExtractionData;
#ifdef VERBOSE_DEBUG
		cout << "Test" << endl;
#endif
/******************************Jason continue here with breaking image into regions  or write this in the setup function**************/
//Jason you decided to write this in setup


	}

}

void featureExtractionMultiThreadedSetupOnly(FeatureExtractionConfig & featureExtractionConfig, FeatureExtractionData & featureExtractionData, ThreadManager * &threadManager,vector<struct GeneralWorkerThreadData> &genData)
{
	bool setupResult = false;




	setupFeatureExtractionThreadsData(featureExtractionConfig,featureExtractionData,threadManager,genData,featureExtractionConfig.numberOfVerticalProcs, featureExtractionConfig.numberOfHorizontalProcs);



}


void FeatureExtractionMultiThreadData::clear()
{
	this->keypoints->clear();
	this->descriptors->release();


}
void FeatureExtractionMultiThreadData::clearNonShared()
{
	this->myDescriptorsCount =0;
	this->myDescriptorsIndex=0;
	this->myDescriptors.release();
	this->myKeyPointsCount = 0;
	this->myKeyPointsIndex = 0;
	this->myKeypoints.clear();
	this->myBboxes.clear();
}

FeatureExtractionMultiThreadData::FeatureExtractionMultiThreadData()
{

	this->algorithmName = "Feature Extraction Multithread Data General";


}



void featureExtractionSetupThreadManager(ThreadManager * &threadManager,FeatureExtractionConfig & featureExtractionConfig,vector<thread_fptr> threadFunctions, vector<void*> threadArgs,int verticalThreads, int horizontalThreads)
{




	int numberOfThreads = threadManager->getNumberOfThreads();

	cout << "Filling in Base Thread Data structs" << endl;

	for(int i = 0; i<numberOfThreads;i++)
	{

		Thread * currentThreadPtr = (threadManager->getThread(i));


		currentThreadPtr->setThreadParam(threadArgs[i]);
		currentThreadPtr->setThreadFunction(threadFunctions[i]);//(currentWorkerThreadInfo->coordinator? classificationCoordinatorThreadFunction: classificationWorkerThreadFunction));

	}







}

void featureExtractionLaunchThreads(ThreadManager * &threadManager)
{
	//Done so we can have the work waiting for them once they are spawned
	for(int i = 0; i<threadManager->getNumberOfThreads();i++)
	{
		Thread * currentThreadPtr = (threadManager->getThread(i));

		int errcode = pthread_create(currentThreadPtr->getThreadIdPtr(),NULL,currentThreadPtr->getThreadFunction(),currentThreadPtr->getThreadParam());
		if(errcode != 0)
		{
			cout << "Error creating thread: " << strerror(errcode) << endl;
		}
	}
}

void multiThreadFeatureExtraction_StandAloneTest(int argc, const char * argv[])
{

	//Lets grab some commnad line arguments and convert them to strings
	//I just prefer them in a vector and I know it is inefficient.
	vector<string> commandLineArgs;
	for(int i = 0; i < argc; i++)
	{
		string currentString = argv[i];
		commandLineArgs.push_back(currentString);
	}

	FeatureExtractionConfig featureExtractionConfig;

	// TODO Here
	if((commandLineArgs[1].compare("-configFile") ==0) || (commandLineArgs[1].compare("-ConfigFile") ==0))
	{

		setupFeatureExtractionConfigFromFile(commandLineArgs[2],featureExtractionConfig );
	}


	FeatureExtractionData featureExtractionData;

	featureExtractionSetupFeatureExtractionData(featureExtractionConfig, featureExtractionData);


	ThreadManager * threadManager;
	featureExtractionCreateThreadManager(threadManager,featureExtractionConfig.numberOfVerticalProcs, featureExtractionConfig.numberOfHorizontalProcs);

	vector<GeneralWorkerThreadData> genData(featureExtractionConfig.numberOfVerticalProcs *featureExtractionConfig.numberOfHorizontalProcs);
	vector<struct FeatureExtractionWorkerThreadInfo> workingThreads(featureExtractionConfig.numberOfVerticalProcs *featureExtractionConfig.numberOfHorizontalProcs);

	setFeatureExtractionWorkingThreadDataInGeneralWorkingData(genData,workingThreads);


	featureExtractionMultiThreadedSetupOnly(featureExtractionConfig, featureExtractionData, threadManager,genData);

	vector<void * > threadArgs(genData.size());
	for(int i =0; i< threadArgs.size(); i++)
	{
		threadArgs[i] = (void *)&genData[i];


	}
	vector<thread_fptr> threadFunctions(genData.size());
	threadFunctions[0] = featureExtraction_testCoordinatorThreadStandAlone;
	for(int i = 1; i < threadArgs.size(); i++)
	{

		threadFunctions[i] = featureExtraction_testWorkerThreadStandAlone;



	}

	//Place the data in the Thread manager
	featureExtractionSetupThreadManager(threadManager,featureExtractionConfig, threadFunctions, threadArgs,featureExtractionConfig.numberOfVerticalProcs, featureExtractionConfig.numberOfHorizontalProcs);


	featureExtractionLaunchThreads(threadManager);


	cout << "Waiting for Threads." << endl;
	for(int i=0 ; i<threadManager->getNumberOfThreads() ; i++)
	{
		pthread_join(threadManager->getThread(i)->getThreadId(), NULL);
	}


	cout << "Multithreaded App complete" << endl;


}



//HoG save to File from Inria neg lst: -inputMethod imageFileList -Local centerPoint -Desc hog -nImages 2 -_snapShotMode -imageList neg.lst -imageListBaseDir C:/Users/jlclemon/Downloads/INRIAPersonDataSet/INRIAPerson/INRIAPerson/train_64x128_H96/ -descOutFile inriaNeg.yml -saveAllDesc
//HoG save to File from Inria pos lst: -inputMethod imageFileList -Local centerPoint -Desc hog -nImages 2 -_snapShotMode -imageList pos.lst -imageListBaseDir C:/Users/jlclemon/Downloads/INRIAPersonDataSet/INRIAPerson/INRIAPerson/train_64x128_H96/ -descOutFile inriaPos.yml -saveAllDesc
//HoG save to File from Inria test lst: -inputMethod imageFileList -Local centerPoint -Desc hog -nImages 2 -_snapShotMode -imageList Test/neg.lst -imageListBaseDir C:/Users/jlclemon/Downloads/INRIAPersonDataSet/INRIAPerson/INRIAPerson/ -descOutFile inriaTestNeg.yml -saveAllDesc
//-inputMethod monoStream -Local sift -Desc sift -nImages 2 -snapShotMode  -descOutFile monkeySift.yml
//-inputMethod monoStream -Local sift -Desc sift -nImages 2 -snapShotMode -loadDescriptors C:/Users/jlclemon/Documents/VisionData/SiftLocalSiftDesc/ProbBookAloneWithImages/probBookSift0.yml -loadDescriptorsImage C:/Users/jlclemon/Documents/VisionData/SiftLocalSiftDesc/ProbBookAloneWithImages/probBookSift0.png
//Linux -inputMethod monoStream -Local sift -Desc sift -nImages 2 -snapShotMode -loadDescriptors /home/jlclemon/Documents/visionData/visionData/SiftLocalSiftDesc/ProbBookAloneWithImages/probBookSift0.yml -loadDescriptorsImage /home/jlclemon/Documents/visionData/visionData/SiftLocalSiftDesc/ProbBookAloneWithImages/probBookSift0.png
#ifdef TSC_TIMING
		void fe_writeTimingToFile(vector<TSC_VAL_w> timingVector)
		{

			ofstream outputFile;
			outputFile.open("timing.csv");
			outputFile << "Thread,Start,Finish" << endl;
			for(int i = 0; i<(timingVector.size()/2); i++)
			{
				outputFile << i << "," << timingVector[i] << "," << timingVector[i+timingVector.size()/2] << endl;

			}
			outputFile.close();
		}
#endif
#ifdef CLOCK_GETTIME_TIMING
		void fe_writeTimingToFile(vector<struct timespec> timingVector)
		{

			ofstream outputFile;
			outputFile.open("timing.csv");
			outputFile << "Thread,Start sec, Start nsec,Finish sec, Finish nsec" << endl;
			for(int i = 0; i<(timingVector.size()/2); i++)
			{
				outputFile << i << "," << timingVector[i].tv_sec<<","<< timingVector[i].tv_nsec << "," << timingVector[i+timingVector.size()/2].tv_sec<<","<< timingVector[i+timingVector.size()/2].tv_nsec <<endl;

			}
			outputFile.close();
		}
#endif



#ifndef FEATURE_EXTRACTION_MODULE
int main(int argc, const char * argv[])
{
	FeatureExtractionConfig featureExtractionConfig;

	std::cout << "Feature Extraction Mechanism" << std::endl;

	//Load a splash screen
//	namedWindow("Splash Screen");

//	Mat splashImage = imread("C:/Users/jlclemon/Documents/VisionData/splashscreen/splash.jpg",1);

//	if(!splashImage.data)
//	{
//		splashImage.create(480,640,CV_8UC3);
//		splashImage.setTo(Scalar(0.0));
//	}
//	else
//	{

//	}
//	Mat smallSplashImage(240,320,CV_8UC3);
//	resize(splashImage,smallSplashImage,Size(),0.5,0.5,INTER_LINEAR);

//	drawTextOnImage(string("Feature Extraction Demo"), Point(250,50), Scalar(0,255,0), smallSplashImage);
//	imshow("Splash Screen", smallSplashImage);


	//Lets grab some commnad line arguments and convert them to strings
	//I just prefer them in a vector and I know it is inefficient.
	vector<string> commandLineArgs;
	for(int i = 0; i < argc; i++)
	{
		string currentString = argv[i];
		commandLineArgs.push_back(currentString);
	}


	if((commandLineArgs[1].compare("-configFile") ==0) || (commandLineArgs[1].compare("-ConfigFile") ==0))
	{
		loadFeatureExtractionConfigFile(commandLineArgs, commandLineArgs[2]);



	}


	parseClassificationConfigCommandVector(featureExtractionConfig, commandLineArgs);


#ifdef TSC_TIMING
	fe_timingVector.resize(16);
#endif
#ifdef CLOCK_GETTIME_TIMING
	fe_timeStructVector.resize(16);

#endif
	if(featureExtractionConfig.singleThreaded ==false)
	{

		multiThreadFeatureExtraction_StandAloneTest(argc, argv);
#ifdef TSC_TIMING
		fe_writeTimingToFile(fe_timingVector);
#endif
#ifdef CLOCK_GETTIME_TIMING
		fe_writeTimingToFile(fe_timeStructVector);
#endif
		return 0;
	}

	vector<Mat> inputImagesPerFrame;
	//int numberOfInputImagesPerFrame;

	//Setting up input method
	switch(featureExtractionConfig.inputMethod)
	{

		case FEATURE_DESC_STEREO_STREAM:
			{
				featureExtractionConfig.leftImageCapture = new VideoCapture(featureExtractionConfig.leftCameraId);
				featureExtractionConfig.rightImageCapture = new VideoCapture(featureExtractionConfig.rightCameraId);

				Mat rightFrame;
				Mat leftFrame;
				Mat vectorLeftFrame;
				Mat vectorRightFrame;
				(*featureExtractionConfig.leftImageCapture)>>leftFrame;
				(*featureExtractionConfig.rightImageCapture)>>rightFrame;
				leftFrame.copyTo(vectorLeftFrame);
				rightFrame.copyTo(vectorRightFrame);

				inputImagesPerFrame.push_back(vectorLeftFrame);
				inputImagesPerFrame.push_back(vectorRightFrame);
				featureExtractionConfig.frameSize = leftFrame.size();

			}
			break;
		case FEATURE_DESC_IMAGE_FILE:
			{


			}
			break;
		case FEATURE_DESC_IMAGE_FILE_LIST:
			{
				if(featureExtractionConfig.imageListFilename.compare("") == 0)
				{
					cout << "Image File list name is NULL. " << endl;
					return 0;
				}
				else
				{
					readFileListFromFile(featureExtractionConfig.imageListBaseDir + featureExtractionConfig.imageListFilename, featureExtractionConfig.imageListBaseDir,featureExtractionConfig.imageList);
					if(featureExtractionConfig.imageList.size() == 0)
					{

						cout << "Image File List empty"<< endl;
						return 0;
					}
				}
				Mat leftFrame = imread(featureExtractionConfig.imageList[0]);
				featureExtractionConfig.frameSize = leftFrame.size();

				break;
			}
		case FEATURE_DESC_VIDEO_FILE:
			{

			}
			break;
		case FEATURE_DESC_SINGLE_STREAM:
		default:
			{
				featureExtractionConfig.leftImageCapture = new VideoCapture(featureExtractionConfig.leftCameraId);

				//featureExtractionConfig.leftImageCapture->set(CV_CAP_PROP_FRAME_WIDTH,320);//640);//1024);//320);
				//featureExtractionConfig.leftImageCapture->set(CV_CAP_PROP_FRAME_HEIGHT,240);//480);//768);//240);
				Mat leftFrame;
				Mat vectorLeftFrame;

				(*featureExtractionConfig.leftImageCapture)>>leftFrame;

				leftFrame.copyTo(vectorLeftFrame);


				inputImagesPerFrame.push_back(vectorLeftFrame);


				featureExtractionConfig.frameSize = leftFrame.size();

			}
	}

	featureExtractionConfig.showMainWindow = true;
	featureExtractionConfig.showCurrentFeaturePointsWindow = true;
	featureExtractionConfig.showMatchWindow = false;


//	destroyWindow("Splash Screen");

	if(featureExtractionConfig.singleThreaded)
	{
		featureExtractionSingleThreadedMain(featureExtractionConfig,inputImagesPerFrame);
#ifdef TSC_TIMING
		fe_writeTimingToFile(fe_timingVector);
#endif
#ifdef CLOCK_GETTIME_TIMING
		fe_writeTimingToFile(fe_timeStructVector);
#endif
	}
	else
	{


	}



return 0;
}

#endif


int getNextImages(vector<Mat>&  images,FeatureExtractionConfig & featureExtractionConfig)
{
	static Mat imageBuffers[5];
	int returnVal = 2;

	switch(featureExtractionConfig.inputMethod)
	{

		case FEATURE_DESC_STEREO_STREAM:
			{

				(*featureExtractionConfig.leftImageCapture)>>imageBuffers[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA];
				(*featureExtractionConfig.rightImageCapture)>>imageBuffers[FEATURE_DESC_IMAGE_SOURCE_LOCATION_RIGHT_CAMERA];
				if(images.size() >= 2)
				{
					imageBuffers[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA].copyTo(images[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA]);
					imageBuffers[FEATURE_DESC_IMAGE_SOURCE_LOCATION_RIGHT_CAMERA].copyTo(images[FEATURE_DESC_IMAGE_SOURCE_LOCATION_RIGHT_CAMERA]);

				}
				else
				{
					Mat frames[2];


					imageBuffers[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA].copyTo(frames[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA]);
					imageBuffers[FEATURE_DESC_IMAGE_SOURCE_LOCATION_RIGHT_CAMERA].copyTo(frames[FEATURE_DESC_IMAGE_SOURCE_LOCATION_RIGHT_CAMERA]);


					images.push_back(frames[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA]);
					images.push_back(frames[FEATURE_DESC_IMAGE_SOURCE_LOCATION_RIGHT_CAMERA]);
				}
			}
			break;
		case FEATURE_DESC_IMAGE_FILE:
			{
				stringstream stringBuffer;
				string imageFullName;
				stringBuffer.str("");
				stringBuffer << featureExtractionConfig.imageId;

				imageFullName = featureExtractionConfig.imageNameBase + stringBuffer.str() + featureExtractionConfig.imageExtension;
#ifdef VERBOSE_DEBUG
				cout << "Loaded image: "<<imageFullName << endl;
#endif
				imageBuffers[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA] = imread(imageFullName,1);
				if(images.size() >= 1)
				{
					imageBuffers[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA].copyTo(images[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA]);

				}
				else
				{
					Mat frame;


					imageBuffers[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA].copyTo(frame);



					images.push_back(frame);

				}


			}
			break;
		case FEATURE_DESC_IMAGE_FILE_LIST:
			{

				string imageFullName;

				imageFullName = featureExtractionConfig.imageList[featureExtractionConfig.currentImageId];
#ifdef VERBOSE_DEBUG
				cout << "Loaded image: "<<imageFullName << endl;
#endif
				imageBuffers[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA] = imread(imageFullName,1);
				if(images.size() >= 1)
				{
					imageBuffers[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA].copyTo(images[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA]);

				}
				else
				{
					Mat frame;


					imageBuffers[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA].copyTo(frame);



					images.push_back(frame);

				}

				featureExtractionConfig.frameSize = imageBuffers[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA].size();
//				featureExtractionData.frameSize = leftFrame.size();

				featureExtractionConfig.currentImageId++;
				returnVal = featureExtractionConfig.imageList.size() - featureExtractionConfig.currentImageId;
			}
			break;
		case FEATURE_DESC_VIDEO_FILE:
			{

			}
			break;
		case FEATURE_DESC_SINGLE_STREAM:
		default:
			{
				(*featureExtractionConfig.leftImageCapture)>>imageBuffers[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA];
				if(images.size() >= 1)
				{
					imageBuffers[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA].copyTo(images[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA]);

				}
				else
				{
					Mat frame;


					imageBuffers[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA].copyTo(frame);



					images.push_back(frame);

				}




			}
	}


	if(images[0].empty())
	{

		featureExtractionConfig.outOfImages = true;
	}
	return returnVal;
}


void localizeFeaturesSingleTime(Mat & inputImage, vector<KeyPoint> & keyPoints, FeatureExtractionConfig featureExtractionConfig)
{
	switch(featureExtractionConfig.currentLocalizationAlgo)
	{
		case FEATURE_LOCALIZATION_SIFT:
			{
				//SIFT::DetectorParams detectorParams;
				//SIFT::CommonParams commonParams;

				//Keep all, 3 octaves. contrast .04, edge 10, sigma 1.6
				SiftFeatureDetector detector( 0, 3,0.04,10,1.6);
				detector.detect(inputImage,keyPoints);
			}
			break;

		case FEATURE_LOCALIZATION_SURF:
			{
				//SURF::CvSURFParams detectorParams;
								
				//detectorParams.hessianThreshold = 400.0;
				SurfFeatureDetector detector(400.0);
				detector.detect(inputImage,keyPoints);


			}
			break;
		case FEATURE_LOCALIZATION_ORB:
			{
				int numberOfFeatures = 5000;
				float scaleFactor = 1.2f;
				int levels = 8;


				#ifdef OPENCV_VER_2_3
					ORB::CommonParams orbParams = ORB::CommonParams();
					orbParams.scale_factor_ = scaleFactor;
					orbParams.n_levels_ = levels;

					OrbFeatureDetector detector(numberOfFeatures,orbParams);
				#else
					OrbFeatureDetector detector(numberOfFeatures,scaleFactor,levels);

				#endif

				detector.detect(inputImage,keyPoints);


			}
			break;
		case FEATURE_LOCALIZATION_FAST:
			{


				FastFeatureDetector detector(50,true);
				detector.detect(inputImage,keyPoints);
			}
			break;
		case FEATURE_LOCALIZATION_HoG:

			break;
		default:

			break;
	}



}



Point2f computeCenterPointOfKeyPoints(vector<KeyPoint> & keypoints)
{
	Point2f centerPoint(0,0);



	//Passing in points uses a different method which throws off the numbers
	//Better to just calculate the average my self
	//Moments currentMoments = moments(pointsMat,false);
	//centerPoint.x = currentMoments.m10/currentMoments.m00;

	//centerPoint.y = currentMoments.m01/currentMoments.m00;


	for(int i = 0; i< (int)keypoints.size(); i++)
	{
		centerPoint.x += keypoints[i].pt.x;
		centerPoint.y += keypoints[i].pt.y;

	}
	centerPoint.x /= keypoints.size();
	centerPoint.y /= keypoints.size();



	return centerPoint;
}

float calcDistanceBetweenPoints(Point2f start, Point2f end)
{
	float xDiff=0.0, yDiff = 0.0;
	
	xDiff = end.x -start.x;
	yDiff = end.y-start.y;	
	return sqrt(xDiff*xDiff + yDiff*yDiff);
}

float makeVectorAngleRelativeToFeatureAngle(float vectorAngle, float featureAngle)
{
	float newAngle;
	newAngle = vectorAngle-featureAngle; 
	if(newAngle>=360.0f)
	{
		newAngle-= 360.0f;

	}

	if(newAngle<0.0f)
	{
		newAngle+= 360.0f;

	}

	return newAngle;
}

float computeVectorAngleRelativeToImageAxis(float vectorAngle, float featureAngle)
{

	float newAngle;
	newAngle = vectorAngle+featureAngle; 
	if(newAngle>=360.0f)
	{
		newAngle-= 360.0f;

	}

	if(newAngle<0.0f)
	{
		newAngle+= 360.0f;

	}

	return newAngle;


}

float calcVectorAngleBetweenPoints(Point2f start, Point2f end)
{
	float xDiff=0.0, yDiff = 0.0;
	
	xDiff = end.x -start.x;
	yDiff = end.y-start.y;	
	
	float angle = (float)((180.0f*atan2(yDiff,xDiff))/pi);
	if(angle < 0)
	{
		angle+= 360.0f;
	}
	if(angle >=360)
	{
		angle -= 360.0f;

	}

	return angle;

}

Point2f computeReferenceInCurrentImageFrame(Vec2f rAndRelativeTheta, KeyPoint keypoint)
{


	Point2f rotatedPoint;


	//Move the theta to the current image reference frame
	float adjustedTheta = computeVectorAngleRelativeToImageAxis(rAndRelativeTheta[1],keypoint.angle);

	//cout << "Adjusted Theta: " << adjustedTheta << endl;

	//Negative adjust Theta because CCW is positive in get RotationMatrix but angle are with CW positive
	Mat rotMatrix = getRotationMatrix2D(Point2f(0,0),-adjustedTheta,1);

	//We need the rotation matrix to be floating point
	rotMatrix.convertTo(rotMatrix,CV_32FC1);

	//So the point was at angle 0, and distance of R
	//THen we move it to where the center should be relative
	//To this feature point.  This is merely a theta to align 
	//The feature point.
	//So in cartesian it is x=r, y = 0
	//Then it is rotated to the proper place
	Point3f pointInZeroFrame(rAndRelativeTheta[0],0,1);

	//Get the point read to apply the rotation matrix
	Mat pointMat = Mat(pointInZeroFrame).reshape(1); // convert vector to Mat, O(1) operation
                   // make Nx3 1-channel matrix out of Nx1 3-channel.
                              // Also, an O(1) operation
	//Apply the rotation matrix	                     
	Mat rotMat = rotMatrix*pointMat;

	//Take the rotated point and translate based on where in image
	rotatedPoint.x= rotMat.at<float>(0,0)+keypoint.pt.x;
	rotatedPoint.y= rotMat.at<float>(1,0)+keypoint.pt.y;

	return rotatedPoint;

}

Vec2f computeVectorToCenterPoint(KeyPoint & keypoint, Point2f centerPoint)
{
	Vec2f rAndTheta;
	Point2f start;
	Point2f end;
	start.x = keypoint.pt.x;
	start.y = keypoint.pt.y;
	end.x = centerPoint.x;
	end.y = centerPoint.y;

	rAndTheta[0] = 	calcDistanceBetweenPoints(start, end);
	rAndTheta[1] = 	calcVectorAngleBetweenPoints(start, end);

	return rAndTheta;
}

Vec2f computeRelativeVectorToCenterPoint(KeyPoint & keypoint, Point2f centerPoint)
{
	Vec2f rAndTheta;
	Point2f start;
	Point2f end;
	start.x = keypoint.pt.x;
	start.y = keypoint.pt.y;
	end.x = centerPoint.x;
	end.y = centerPoint.y;

	rAndTheta[0] = 	calcDistanceBetweenPoints(start, end);
	rAndTheta[1] = 	calcVectorAngleBetweenPoints(start, end);
	rAndTheta[1] = makeVectorAngleRelativeToFeatureAngle(rAndTheta[1], keypoint.angle);
	return rAndTheta;
}

void convertMatchesToSubListVectors(vector<DMatch> matches, const vector<Vec2f>& centerPointVectors, const vector<KeyPoint> & originalKeypoints, const vector<KeyPoint> & newKeypoints, vector<Vec2f>& matchedCenterPointVectors, vector<KeyPoint> & matchedOriginalKeypoints, vector<KeyPoint> & matchedNewKeypoints)
{
	matchedCenterPointVectors.clear();
	matchedOriginalKeypoints.clear(); 
	matchedNewKeypoints.clear();

	for(vector<DMatch>::iterator it = matches.begin(); it != matches.end(); ++it)
	{
		int currentNewKeypointsIndex = (*it).queryIdx;
		int currentOrigKeypointsIndex = (*it).trainIdx; 
		int currentCenterPointIndex = (*it).trainIdx;  //Centerpoints are still relative to originals
		matchedCenterPointVectors.push_back(centerPointVectors[currentCenterPointIndex]);
		matchedOriginalKeypoints.push_back(originalKeypoints[currentOrigKeypointsIndex]); 
		matchedNewKeypoints.push_back(newKeypoints[currentNewKeypointsIndex]);

	}


}


vector<Vec2f> computeScaleAdjustedCenterPointVectors(const vector<Vec2f>& centerPointVectors, const vector<KeyPoint> & originalKeypoints, const vector<KeyPoint> & newKeypoints, vector<Vec2f> & scaleAndAngleDiff)
{

	vector<Vec2f> scaledCenterPointVectors;
/*
	for(int i = 0; i < (int)centerPointVectors.size(); i++)
	{

		float currentScaling = newKeypoints[i].size/originalKeypoints[i].size;
		Vec2f currentScaleAndAngleDiff;
		scaledCenterPointVectors.push_back(centerPointVectors[i]);
		scaledCenterPointVectors[scaledCenterPointVectors.size() -1][0] = scaledCenterPointVectors[scaledCenterPointVectors.size() -1][0] * currentScaling;

		currentScaleAndAngleDiff[0] = currentScaling;
		currentScaleAndAngleDiff[1] =newKeypoints[i].angle-originalKeypoints[i].angle;
		scaleAndAngleDiff.push_back(currentScaleAndAngleDiff);
	}
	*/
	for(int i = 0; i < (int)centerPointVectors.size(); i++)
	{

		float currentScaling = newKeypoints[i].size/originalKeypoints[i].size;
		Vec2f currentScaleAndAngleDiff;
		scaledCenterPointVectors.push_back(centerPointVectors[i]);
		scaledCenterPointVectors[scaledCenterPointVectors.size() -1][0] = scaledCenterPointVectors[scaledCenterPointVectors.size() -1][0] * currentScaling;

		currentScaleAndAngleDiff[0] = currentScaling;
		currentScaleAndAngleDiff[1] =newKeypoints[i].angle-originalKeypoints[i].angle;
		
		if(currentScaleAndAngleDiff[1] < 0)
		{
			currentScaleAndAngleDiff[1] += 360;
		}
		if(currentScaleAndAngleDiff[1] >= 360)
		{
			currentScaleAndAngleDiff[1] -= 360;
		}
		

		scaleAndAngleDiff.push_back(currentScaleAndAngleDiff);
	}


	return scaledCenterPointVectors;
}


//Computed in form of r,theta in the vector
vector<Vec2f> computeRelativeCenterPointVectorsForKeypoints(vector<KeyPoint> & keypoints)
{
	vector<Vec2f> rAndRelativeTheta;

	Point2f centerFloat = computeCenterPointOfKeyPoints(keypoints);

	for(int i = 0; i < (int)keypoints.size(); i++)
	{
		Vec2f currentRAndThetaRelative = computeRelativeVectorToCenterPoint(keypoints[i], centerFloat);
		rAndRelativeTheta.push_back(currentRAndThetaRelative);

	}

	return rAndRelativeTheta;
}

vector<Point2f> computeReferencePointsInCurrentImageFrame(const vector<Vec2f> &relativeCenterVectors, const vector<KeyPoint> &keypoints)
{
	vector<Point2f> refPoints;
	for(int i = 0; i < (int)relativeCenterVectors.size(); i++)
	{


		Point2f calculatedPoint = computeReferenceInCurrentImageFrame(relativeCenterVectors[i], keypoints[i]);

		refPoints.push_back(calculatedPoint);



	}
	return refPoints;
}

vector<int> addToHistogramBin(vector<int> xIndex, vector<int> yIndex, vector<int> scaleIndex,vector<int> orientationIndex,int keypointIndex, vector<HoughBin> & histogramBins)
{
	vector<int> binCount;
	//binCount.push_back(-1);				
	bool binFound = false;

	if(xIndex[0] == -1 &&  yIndex[0] == -1 && orientationIndex[0] == -1 && scaleIndex[0] == -1)
	{

		return binCount;
	}

	/*
		for(unsigned int i=0; i< histogramBins.size(); i++)
		{
			if(histogramBins[i].xIndex == xIndex &&
				histogramBins[i].yIndex == yIndex &&
				histogramBins[i].scaleIndex == scaleIndex &&
				histogramBins[i].orientationIndex == orientationIndex)
			{
				histogramBins[i].count++;
				histogramBins[i].keypointIndices.push_back(keypointIndex);
				binCount = histogramBins[i].count;
			}


		}
	*/

	int currentXIndex;
	int currentYIndex;
	int currentScaleIndex;
	int currentOrientationIndex;
	for(int x = 0; x < (int)xIndex.size(); x++)
	{
		currentXIndex = xIndex[x];
		if(currentXIndex ==-1)
		{
			continue;
		}

		for(int y = 0; y < (int)yIndex.size(); y++)
		{
			currentYIndex = yIndex[y];
			if(currentYIndex ==-1)
			{
				continue;
			}


			for(int s = 0; s < (int)scaleIndex.size(); s++)
			{
				currentScaleIndex = scaleIndex[s];
				if(currentScaleIndex ==-1)
				{
					continue;
				}


				for(int o = 0; o < (int)orientationIndex.size(); o++)
				{
					currentOrientationIndex = orientationIndex[o];
					if(currentOrientationIndex ==-1)
					{
						continue;
					}

					binFound =false;
					for(unsigned int i=0; i< histogramBins.size(); i++)
					{
						if(histogramBins[i].xIndex == currentXIndex &&
							histogramBins[i].yIndex == currentYIndex &&
							histogramBins[i].scaleIndex == currentScaleIndex &&
							histogramBins[i].orientationIndex == currentOrientationIndex)
						{
							histogramBins[i].count++;
							histogramBins[i].keypointIndices.push_back(keypointIndex);
							binCount.push_back(histogramBins[i].count);
							binFound = true;
						}


					}


					if(binFound ==false)
					{
						histogramBins.push_back(HoughBin());
						int currentBinIndex = histogramBins.size()-1;

						histogramBins[currentBinIndex].count = 1;
						histogramBins[currentBinIndex].keypointIndices.push_back(keypointIndex);
						histogramBins[currentBinIndex].xIndex = currentXIndex;
						histogramBins[currentBinIndex].yIndex = currentYIndex;
						histogramBins[currentBinIndex].scaleIndex = currentScaleIndex;
						histogramBins[currentBinIndex].orientationIndex = currentOrientationIndex;
						binCount.push_back(histogramBins[currentBinIndex].count);

					}
				}
			}
		}
	}

	return binCount;
}

void computeBinIndices(Point2f location, Vec2f scaleAndOrientation, vector<int> & xIndex, vector<int> & yIndex, vector<int> &scaleIndex, vector<int> &orientationIndex)
{
	int numberOfXbins = 960/10;//320/10;//640/10;   //10 pixel boxes (64 bins)
	int numberOfYbins = 720/10;//240/10;//480/10;	  //10 pixel boxes (48 bins)
	float xBinWidth = 10;
	float yBinWidth = 10;
	int numberOfScaleBins = 10;   //Factor of 2 bins defined below
	int numberOfOrientationBins = 360/30;  //30 degree bins (12 bins)
	float orientationBinWidth = 30;
	float xBinRanges[97] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300,310,320,330,340,350,360,370,380,390,400,410,420,430,440,450,460,470,480,490,500,510,520,530,540,550,560,570,580,590,600,610,620,630,640,650,660,670,680,690,700,710,720,730,740,750,760,770,780,790,800,810,820,830,840,850,860,870,880,890,900,910,920,930,940,950,960};
	float yBinRanges[73] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300,310,320,330,340,350,360,370,380,390,400,410,420,430,440,450,460,470,480,490,500,510,520,530,540,550,560,570,580,590,600,610,620,630,640,650,660,670,680,690,700,710,720};


//	float xBinRanges[65] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300,310,320,330,340,350,360,370,380,390,400,410,420,430,440,450,460,470,480,490,500,510,520,530,540,550,560,570,580,590,600,610,620,630,640};
//	float yBinRanges[49] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300,310,320,330,340,350,360,370,380,390,400,410,420,430,440,450,460,470,480};
//	float xBinRanges[33] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300,310,320};
//	float yBinRanges[25] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240};

	float scaleBinRanges[] = {0.0f, .0078125f, .015625f, .03125f,.0625f,.125f,.25f,.50f, 1.0f,2.0f,4.0f};
	float orientationBinRanges[] = {0.0, 30, 60, 90,120,150,180,210, 240,270,300,330,360};

/*
	if(location.x >= xBinRanges[0])
	{
	
		for(int i = 1; i < numberOfXbins+1; i++)
		{
			if(location.x < xBinRanges[i])
			{
				xIndex = i-1;

				break;
			}
		}
	}
*/
	xIndex.resize(2);
	yIndex.resize(2);
	orientationIndex.resize(2); 
	scaleIndex.resize(2);

	xIndex[0] = -1;
	yIndex[0] = -1; 
	orientationIndex[0] = -1; 
	scaleIndex[0] = -1;
	xIndex[1] = -1; 
	yIndex[1] = -1;
	orientationIndex[1] = -1;
	scaleIndex[1] = -1;


	if(location.x < xBinRanges[numberOfXbins] && location.x >= xBinRanges[0])
	{
		xIndex[0] = (int) (location.x/xBinWidth);
		if(location.x/xBinWidth - xIndex[0] >=.5)
		{
			if(xIndex[0]+1 < numberOfXbins)
			{
				xIndex[1] = xIndex[0]+1;
			}
		}
		else
		{
			if(xIndex[0] >= 1)
			{
				xIndex[1] = xIndex[0]-1;
			}

		}

	}
	if(location.y < yBinRanges[numberOfYbins] && location.y >= yBinRanges[0])
	{

		yIndex[0] = (int) (location.y/yBinWidth);
		if(location.y/yBinWidth - yIndex[0] >=.5)
		{
			if(yIndex[0]+1 < numberOfYbins)
			{
				yIndex[1] = yIndex[0]+1;
			}
		}
		else
		{
			if(yIndex[0] >= 1)
			{
				yIndex[1] = yIndex[0]-1;
			}

		}


	}

	if(scaleAndOrientation[1] < orientationBinRanges[numberOfOrientationBins] && scaleAndOrientation[1] >= orientationBinRanges[0])
	{

		orientationIndex[0] = (int) (scaleAndOrientation[1]/orientationBinWidth);
		if(scaleAndOrientation[1]/orientationBinWidth - orientationIndex[0] >=.5)
		{
			if(orientationIndex[0]+1 < numberOfOrientationBins)
			{
				orientationIndex[1] = orientationIndex[0]+1;
			}
			else
			{
				orientationIndex[1] = 0;
			}
		}
		else
		{
			if(orientationIndex[0] >= 1)
			{
				orientationIndex[1] = orientationIndex[0]-1;
			}
			else
			{
				orientationIndex[1] = numberOfOrientationBins-1;
			}

		}



	}

	if(scaleAndOrientation[0] >= scaleBinRanges[0] && scaleAndOrientation[0] < scaleBinRanges[numberOfScaleBins])
	{
	
		for(int i = 1; i < numberOfScaleBins+1; i++)
		{
			if(scaleAndOrientation[0] < scaleBinRanges[i])
			{
				scaleIndex[0] = i-1;
				if(scaleAndOrientation[0]>=(scaleBinRanges[i-1] + .5 *(scaleBinRanges[i]-scaleBinRanges[i-1]) ))
				{
					if(i+1 < numberOfScaleBins)
					{
						scaleIndex[1] = i;
					}
				}
				else
				{
					if(i>=2)
					{
						scaleIndex[1] = i-2;
					}

				}


				break;
			}
		}
	}


	if(xIndex[0] == -1|| yIndex[0] == -1 || orientationIndex[0] == -1 || scaleIndex[0] == -1)
	{
		xIndex[0] = -1; 
		yIndex[0] = -1;
		orientationIndex[0] = -1;
		scaleIndex[0] = -1;


	}
	if(xIndex[1] == -1|| yIndex[1] == -1 || orientationIndex[1] == -1 || scaleIndex[1] == -1)
	{
		xIndex[1] = -1; 
		yIndex[1] = -1;
		orientationIndex[1] = -1;
		scaleIndex[1] = -1;


	}


	return;

}

SparseMat calcHistUsingOpenCvMethods(vector<HoughBin> & histogramBins, const vector<Vec2f> & rAndThetas, const vector<Vec2f>& scaleAndAngles, const  vector<KeyPoint> &keypoints)
{

	int numberOfXbins = 320/10;//640/10;   //10 pixel boxes (64 bins)
	int numberOfYbins = 240/10;//480/10;	  //10 pixel boxes (48 bins)
	float xBinWidth = 10;
	float yBinWidth = 10;
	int numberOfScaleBins = 10;   //Factor of 2 bins defined below
	int numberOfOrientationBins = 360/30;  //30 degree bins (12 bins)
	float orientationBinWidth = 30;
//	float xBinRanges[65] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300,310,320,330,340,350,360,370,380,390,400,410,420,430,440,450,460,470,480,490,500,510,520,530,540,550,560,570,580,590,600,610,620,630,640};
//	float yBinRanges[49] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300,310,320,330,340,350,360,370,380,390,400,410,420,430,440,450,460,470,480};
	float xBinRanges[33] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300,310,320};
	float yBinRanges[25] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240};


	float scaleBinRanges[] = {0.0f, .0078125f, .015625f, .03125f,.0625f,.125f,.25f,.50f, 1.0f,2.0f,4.0f};
	float orientationBinRanges[] = {0.0, 30, 60, 90,120,150,180,210, 240,270,300,330,360};

	int channels[] = {0,1,2,3};

	Mat matArray[2];

	vector<Point2f> centerPoints = computeReferencePointsInCurrentImageFrame(rAndThetas,keypoints);

	Mat pointsMat(centerPoints);
	Mat scaleAndAnglesMat(scaleAndAngles);
	matArray[0] = pointsMat;
	matArray[1] = scaleAndAnglesMat;

	SparseMat histogramMat;
	int histSize[] = {numberOfXbins,numberOfYbins,numberOfScaleBins,numberOfOrientationBins};
	const float * ranges[] = {xBinRanges,yBinRanges,scaleBinRanges,orientationBinRanges};


	calcHist( matArray, 2, channels, Mat(),  histogramMat, 4, histSize, ranges, false, false );

	cv::SparseMatConstIterator it = histogramMat.begin(), it_end = histogramMat.end();
#ifdef VERBOSE_DEBUG
	float totalItems= 0;

	for(; it != it_end; ++it)
	 {
		 // print element indices and the element value
		 const SparseMat::Node* n = it.node();
		 cout <<"X Index: "<<n->idx[0] <<", Y Index: "<< n->idx[1] <<", Scale Index: "<< n->idx[2] <<", Orientation Index: "<<n->idx[3] <<
			 ", Count: " << it.value<float>()<< endl;		 

		totalItems +=it.value<float>();
	 }

	cout << "Total Items: " << totalItems << endl;
#endif

	return histogramMat;

}


void houghBinTheRemappedVectors(vector<HoughBin> & histogramBins, const vector<Vec2f> & rAndThetas, const vector<Vec2f>& scaleAndAngles, const  vector<KeyPoint> &keypoints)
{
	
	vector<int> xIndex; 
	vector<int> yIndex;
	vector<int> scaleIndex;
	vector<int> orientationIndex;
	vector<int> currentBinAmount;


	vector<Point2f> centerPoints = computeReferencePointsInCurrentImageFrame(rAndThetas,keypoints);
	for(unsigned int i =0; i < rAndThetas.size(); i++)
	{

		computeBinIndices(centerPoints[i], scaleAndAngles[i], xIndex,yIndex,scaleIndex,orientationIndex);

		currentBinAmount = addToHistogramBin(xIndex, yIndex, scaleIndex, orientationIndex,i, histogramBins);


	}

/*	int totalItems = 0;
	for(vector<HoughBin>::iterator it = histogramBins.begin(); it!=histogramBins.end(); ++it)
	{
		cout << *it;
		totalItems += (*it).count;

	}
	cout << "Total Items: " << totalItems << endl;
*/		
		
	//calcHist( matArray, 2, channels, Mat(),  histogramMat, 4, histSize, ranges, false, false );



	//histogramMat.

}

float computeSqErrorForHoughBin(Mat & computedTransform,vector<int> & pointIndicesForTransform, int & numberOfValidPointsInBin,const HoughBin & bin, const vector<KeyPoint> & newKeypoints,const vector<KeyPoint> & origKeypoints)
{
	vector<int> currentBinPointIndices(bin.count);
	bool done = false;
	bool found = false;
	float maxAllowedError = ((5.0f/2.0f) *(5.0f/2.0f)) * 2;
	float totalSquareError = -1.0;
	float currentSquareError = 0.0;
	vector<Point2f> newPoints(bin.count);
	vector<Point2f> origPoints(bin.count);
	vector<Point2f> nextNewPoints(bin.count);
	vector<Point2f> nextOrigPoints(bin.count);
	vector<int> nextCurrentBinPointIndices(bin.count);
	numberOfValidPointsInBin = -1;


	for(int i = 0; i < bin.count;i++)
	{
		currentBinPointIndices[i]=bin.keypointIndices[i];
		newPoints[i].x =newKeypoints[i].pt.x;
		newPoints[i].y = newKeypoints[i].pt.y;
		origPoints[i].x =origKeypoints[i].pt.x;
		origPoints[i].y = origKeypoints[i].pt.y;

	}
#ifdef VERBOSE_DEBUG
	cout << "Bin Count: " << bin.count << endl;
#endif

	while(!done)
	{
		
		Mat origPointsMat(origPoints);
		Mat newPointsMat(newPoints);		

		Mat affineTransform = estimateRigidTransform(origPointsMat,newPointsMat,true);
		affineTransform.convertTo(affineTransform,CV_32FC1);
//		cout << "Current Affine Transform: " << affineTransform << endl;
		Mat newPointsFromXformMat;

		transform(origPointsMat,newPointsFromXformMat,affineTransform);

//		Mat diffMat;
//		subtract(newPointsFromXformMat,newPointsMat,diffMat);

		nextNewPoints.clear();
		nextOrigPoints.clear();
		nextCurrentBinPointIndices.clear();
		totalSquareError = 0.0;
		for(int i = 0; i<newPointsFromXformMat.rows;i++)
		{
			
			currentSquareError = ((newPointsFromXformMat.at<float>(i,0) - newPointsMat.at<float>(i,0)) * (newPointsFromXformMat.at<float>(i,0) - newPointsMat.at<float>(i,0))) + ((newPointsFromXformMat.at<float>(i,1) - newPointsMat.at<float>(i,1)) * (newPointsFromXformMat.at<float>(i,1) - newPointsMat.at<float>(i,1)));
			totalSquareError += currentSquareError;
			cout << "Sqr Error: " << currentSquareError << endl;
			/*if(currentSquareError > 100)
			{
				cout << "The Xfrom Points" << newPointsFromXformMat << endl;
				cout << "The Old Points" << origPointsMat << endl;
				cout << "The New Points" << newPointsMat << endl;
				cout << "How:?" << endl;
			}
			waitKey(0);
			*/
			if(currentSquareError < maxAllowedError)
			{
				nextNewPoints.push_back(newPoints[i]);
				nextOrigPoints.push_back(origPoints[i]);	
				nextCurrentBinPointIndices.push_back(currentBinPointIndices[i]);
			}

		}
		if(currentBinPointIndices.size() == nextCurrentBinPointIndices.size())
		{
			computedTransform = affineTransform;
			pointIndicesForTransform =currentBinPointIndices; 
			done = true;
			found = true;
			numberOfValidPointsInBin = nextCurrentBinPointIndices.size();
		}

		newPoints = nextNewPoints;
		origPoints = nextOrigPoints;
		currentBinPointIndices =nextCurrentBinPointIndices;

		if(currentBinPointIndices.size() <3)
		{
			totalSquareError = -1.0;
			done = true;
		}
	}


	return totalSquareError;
}

vector<Mat> computerValidTransfroms (vector<vector<int> > & pointIndicesForTransform,const vector<HoughBin> & bins, const vector<KeyPoint> & newKeypoints,const vector<KeyPoint> & origKeypoints)
{
	vector<Mat> validTransforms;
	//vector<vector<int>> validKeypointIndices;
	for(int i = 0; i < (int)bins.size()	; i++)
	{
		if(bins[i].count >= 3)
		{
			Mat currentTransform;
			vector<int> currentIndices;
			int currentNumberOfValidPoints;
			float currentSqError = computeSqErrorForHoughBin(currentTransform, currentIndices, currentNumberOfValidPoints,bins[i], newKeypoints,origKeypoints);
			if(currentSqError >=0)
			{
				validTransforms.push_back(currentTransform);
				//validKeypointIndices.push_back(currentIndices);
				pointIndicesForTransform.push_back(currentIndices);
			}
		}
	}

	return validTransforms;

}


void filterMatchesAndGetTransforms(vector<Mat> & transformsList, vector<Point2f>& refPoints,const vector<DMatch> & matches,  const vector<KeyPoint> queryKeypoints,const vector<KeyPoint> trainedKeypoints, const vector<Vec2f> trainedRefVectors, const Point2f & trainedRefPoint)
{

	vector<Vec2f> matchedCenterPointVectors; 
	vector<KeyPoint> matchedOriginalKeypoints; 
	vector<KeyPoint> matchedNewKeypoints;


	//Take the original list of all keypoints and pair them down to only the matched keypoints
	convertMatchesToSubListVectors( matches, trainedRefVectors,trainedKeypoints, queryKeypoints, matchedCenterPointVectors,  matchedOriginalKeypoints,  matchedNewKeypoints);


	//I take the center point vectors from the training set and the keypoints from the training and query set and create the center point
	//vectors for the query set based on which training set keypoint they match
	//So if the new feature is 2xScale with 30 degrees compared to the training set of 1xScale with 50 degrees
	//The vector to center is scaled by 2x/1x = 2 and the angle is adjust by 20 degrees to 
	//get the new center point vector
	vector<Vec2f> newScaleAndObjectAngles;
	vector<Vec2f> newCenterVectors = computeScaleAdjustedCenterPointVectors(matchedCenterPointVectors, matchedOriginalKeypoints, matchedNewKeypoints, newScaleAndObjectAngles);


	//Computer the Hough bins using custom code
	vector<HoughBin>  histogramBins;
	houghBinTheRemappedVectors(histogramBins, newCenterVectors, newScaleAndObjectAngles, matchedNewKeypoints);

	//Use the bins to compute possible transforms using affine error to filter
	vector<vector<int> > pointIndicesForTransform;
	transformsList = computerValidTransfroms (pointIndicesForTransform,histogramBins, matchedNewKeypoints,matchedOriginalKeypoints);

	//Setup the original Reference Point for calculating the new ref points
	Point3f oldRefPointForXForm(trainedRefPoint.x,trainedRefPoint.y,1);
	Mat refPointMat = Mat(oldRefPointForXForm).reshape(1);

	//Compute the new ref points
	for(int i=0; i < (int)transformsList.size(); i++)
	{
		Point2f relativelyCalculatedPoint;
		Mat newRefMat;//(relativelyCalculatedPoint);
		newRefMat = transformsList[i] * refPointMat;
		relativelyCalculatedPoint.x = newRefMat.at<float>(0,0);
		relativelyCalculatedPoint.y = newRefMat.at<float>(1,0);
		refPoints.push_back(relativelyCalculatedPoint);

	}



return;



}


void paintCenterOnImage(Mat & image, vector<KeyPoint> & keypoints,Mat & descriptors)
{
	Mat dispImage = image.clone();
	int upside=0, leftside = 0;	
	Point2f centerFloat = computeCenterPointOfKeyPoints(keypoints);
	Point_<int> pixel(centerFloat.x,centerFloat.y);//= Point_<int>((Point_<int>)centerFloat);



	circle(dispImage,pixel,5,Scalar(0,255,0));
	namedWindow("DispImage",1);
	imshow("DispImage",dispImage);

	Mat dispImage2 = image.clone();
	Mat tmpImage;
	if(image.channels() ==3)
	{
		cvtColor(image,tmpImage,CV_BGR2GRAY);
	}
	else
	{
		image.copyTo(tmpImage);
	}

	cout << "Key Points Total: " << keypoints.size() << endl;
	cout << "Key Points on left: "<< leftside << "KeyPoints up: " << upside <<endl;
	cout << "Center X: " << pixel.x << "Center Y: " << pixel.y << endl;

/*	
	vector<KeyPoint> testingVec;

	testingVec.push_back(keypoints[0]);
	testingVec.push_back(keypoints[1]);
	testingVec.push_back(keypoints[2]);
	testingVec.push_back(keypoints[3]);
	
	testingVec[0].pt.x = 100;
	testingVec[0].pt.y = 400;
	testingVec[0].angle = 0.0;
	
	testingVec[1].pt.x = 150;
	testingVec[1].pt.y = 400;
	testingVec[1].angle = 0.0;

	testingVec[2].pt.x = 200;
	testingVec[2].pt.y = 200;
	testingVec[2].angle = 90.0;

	testingVec[3].pt.x = 250;
	testingVec[3].pt.y = 250;
	testingVec[3].angle = 135.0;
	

	cout << "Angle 0: "<<testingVec[0].angle << endl;
	cout << "Angle 1: "<<testingVec[1].angle << endl;


	Point2f rotatedPoint;

	Vec2f rAndTheta = computeVectorToCenterPoint(testingVec[0], centerFloat);
	Vec2f rAndThetaRelative = computeRelativeVectorToCenterPoint(testingVec[0], centerFloat);
	Point2f relativelyCalculatedPoint = computeReferenceInCurrentImageFrame(rAndThetaRelative, testingVec[1]);
	cout << "Original Angle: " << rAndTheta[1] << endl;
	//testingVec[0].angle = rAndTheta[1];

	Point3f origPoint(rAndTheta[0],0,1);




	Mat rotMatrix = getRotationMatrix2D(Point2f(0,0),-rAndTheta[1],1);

	rotMatrix.convertTo(rotMatrix,CV_32FC1);
	Mat pointMat = Mat(origPoint).reshape(1); // convert vector to Mat, O(1) operation
                   // make Nx3 1-channel matrix out of Nx1 3-channel.
                              // Also, an O(1) operation
                     
	Mat rotMat = rotMatrix*pointMat;
	rotatedPoint.x= rotMat.at<float>(0,0)+testingVec[0].pt.x;
	rotatedPoint.y= rotMat.at<float>(1,0)+testingVec[0].pt.y;


	cout << "Theta 1: "<<testingVec[0].angle << endl;
	*/
	circle(tmpImage,centerFloat,3,Scalar(255,0,0));
	//circle(tmpImage,Point2f(origPoint.x+testingVec[0].pt.x,origPoint.y+testingVec[0].pt.y),7,Scalar(255,0,0));
	//circle(tmpImage,rotatedPoint,11,Scalar(255,0,0));
	//circle(tmpImage,relativelyCalculatedPoint,17,Scalar(255,0,0));

	//drawKeypoints(tmpImage,testingVec,dispImage2,Scalar(0,0,255.0),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	//Called for test image to compute initial vectors
	vector<Vec2f> centerVectors = computeRelativeCenterPointVectorsForKeypoints( keypoints);

	Ptr<DescriptorMatcher> matcher;
	Ptr<flann::IndexParams> indexParams = new flann::KDTreeIndexParams(4);
	Ptr<flann::SearchParams> searchParams = new flann::SearchParams(200);
	matcher = new FlannBasedMatcher(indexParams, searchParams);
	vector<vector<DMatch> > topNMatches;
	int numberOfNeighborsToFind = 2;
	Mat trainedDescriptors = descriptors.clone();

	matcher->knnMatch(descriptors,trainedDescriptors,topNMatches,numberOfNeighborsToFind);
	vector<DMatch> matches;
	for (unsigned int i = 0; i< topNMatches.size(); i++)
	{
		matches.push_back(topNMatches[i][0]);
	}

	vector<Vec2f> matchedCenterPointVectors; 
	vector<KeyPoint> matchedOriginalKeypoints; 
	vector<KeyPoint> matchedNewKeypoints;


	//Take the original list of all keypoints and pair them down to only the matched keypoints
	convertMatchesToSubListVectors( matches, centerVectors,keypoints, keypoints, matchedCenterPointVectors,  matchedOriginalKeypoints,  matchedNewKeypoints);


	//Now I need to modify the vectors for testing the binning code
	for(int i = 0; i< (int)matchedNewKeypoints.size(); i++)
	{
		if(i>30)
		{

			matchedNewKeypoints[i].angle += i;
			matchedNewKeypoints[i].size += i/10;
		}
		if(matchedNewKeypoints[i].angle >= 360)
		{

			matchedNewKeypoints[i].angle -= 360;
		}
		if(matchedNewKeypoints[i].angle >= 360)
		{

			matchedNewKeypoints[i].angle -= 360;
		}



	}

	//I take the center point vectors from the training set and the keypoints from the training and query set and create the center point
	//vectors for the query set based on which training set keypoint they match
	//So if the new feature is 2xScale with 30 degrees compared to the training set of 1xScale with 50 degrees
	//The vector to center is scaled by 2x/1x = 2 and the angle is adjust by 20 degrees to 
	//get the new center point vector
	vector<Vec2f> newScaleAndObjectAngles;
	vector<Vec2f> newCenterVectors = computeScaleAdjustedCenterPointVectors(matchedCenterPointVectors, matchedOriginalKeypoints, matchedNewKeypoints, newScaleAndObjectAngles);


	//Computer the Hough bins using custom code
	vector<HoughBin>  histogramBins;
	houghBinTheRemappedVectors(histogramBins, newCenterVectors, newScaleAndObjectAngles, matchedNewKeypoints);

	//Computer the bins using openCV code as a test
	vector<HoughBin>  histogramBins2;
	SparseMat sparseMat = calcHistUsingOpenCvMethods(histogramBins2,  newCenterVectors, newScaleAndObjectAngles, matchedNewKeypoints);
/*
	for(vector<HoughBin>::iterator it = histogramBins.begin(); it!=histogramBins.end(); ++it)
	{
		int x,y,s,o;
		x = (*it).xIndex;
		y = (*it).yIndex;
		s = (*it).scaleIndex;
		o = (*it).orientationIndex;

		cv::SparseMatConstIterator itInner = sparseMat.begin(), itInner_end = sparseMat.end();
		cout << *it;

		int foundInSparse = 0;
		 for(; itInner != itInner_end; ++itInner)
		 {
			 // print element indices and the element value
			 const SparseMat::Node* n = itInner.node();

			 if(n->idx[0] == x && n->idx[1] ==y&& n->idx[2]==s &&n->idx[3] ==o  && (*it).count == itInner.value<float>())
			{
				foundInSparse++;

				break;
			}



		 }
		if(foundInSparse ==0)
		{
			cout <<"Unmatched Bin Here! " << endl;
			waitKey(0);
			string input;
			cin >> input;
		}

	}

	*/
	vector<vector<int> > pointIndicesForTransform;
	vector<Mat> transformsList = computerValidTransfroms (pointIndicesForTransform,histogramBins, matchedNewKeypoints,matchedOriginalKeypoints);
	Point2f oldCenterPoint = computeCenterPointOfKeyPoints(keypoints);
	Point3f oldCenterPointForXForm(oldCenterPoint.x,oldCenterPoint.y,1);
	Mat centerPointMat = Mat(oldCenterPointForXForm).reshape(1);


	drawKeypoints(tmpImage,keypoints,dispImage2,Scalar(0,0,255.0),DrawMatchesFlags::DEFAULT);//DRAW_RICH_KEYPOINTS);
	for(int i=0; i < (int)transformsList.size(); i++)
	{
		Point2f relativelyCalculatedPoint;
		Mat newCenterMat;//(relativelyCalculatedPoint);
		newCenterMat = transformsList[i] * centerPointMat;
		relativelyCalculatedPoint.x = newCenterMat.at<float>(0,0);
		relativelyCalculatedPoint.y = newCenterMat.at<float>(1,0);

		circle(dispImage2,relativelyCalculatedPoint,15,Scalar(255,0,255));


	}


	for(int i = 0; i < (int)centerVectors.size(); i++)
	{


		Point2f relativelyCalculatedPoint = computeReferenceInCurrentImageFrame(centerVectors[i], keypoints[i]);
		circle(dispImage2,relativelyCalculatedPoint,5,Scalar(255,0,0));
		cout << "Point painted: "<<relativelyCalculatedPoint << endl;

	}



	namedWindow("DispImage2",1);
	imshow("DispImage2",dispImage2);

	waitKey(0);
	destroyWindow("DispImage");
	destroyWindow("DispImage2");
}

Ptr<DescriptorMatcher> setupMatcher(FeatureExtractionConfig & featureExtractionConfig)
{

	Ptr<DescriptorMatcher> matcher;
	switch(featureExtractionConfig.currentFeatureMatchingAlgo)
	{
	
		case FEATURE_MATCHER_FLANN:
		{

			//Yeah a memory leak here right now
			Ptr<flann::IndexParams> indexParams = new flann::KDTreeIndexParams(4);
			Ptr<flann::SearchParams> searchParams = new flann::SearchParams(200);
			matcher = new FlannBasedMatcher(indexParams, searchParams);



		}
			break;
		
		case FEATURE_MATCHER_BRUTEFORCE:
		default:
		{
		#ifdef OPENCV_VER_2_3
		matcher = new BruteForceMatcher<L2<float> >;
		#else
		matcher = new BFMatcher(NORM_L2);
		#endif


		}


			break;
	}


	return matcher;
}

Ptr<DescriptorMatcher> setupMatcher(FeatureExtractionConfig & featureExtractionConfig, 	vector<Mat> & trainedDescriptors)
{

	Ptr<DescriptorMatcher> matcher;
	switch(featureExtractionConfig.currentFeatureMatchingAlgo)
	{
	
		case FEATURE_MATCHER_FLANN:
		{

			//Yeah a memory leak here right now
			Ptr<flann::IndexParams> indexParams = new flann::KDTreeIndexParams(1);
			Ptr<flann::SearchParams> searchParams = new flann::SearchParams(32);//200
			matcher = new FlannBasedMatcher(indexParams, searchParams);
			
			matcher->add(trainedDescriptors);
			matcher->train();

		}
			break;
		
		case FEATURE_MATCHER_BRUTEFORCE:
		default:
		{


		#ifndef OPENCV_2_4
			matcher = new BruteForceMatcher<L2<float> >;
		#else
			matcher = new BFMatcher(NORM_L2);
		#endif

		matcher->add(trainedDescriptors);
		matcher->train();

		}


			break;
	}


	return matcher;
}



vector<DMatch> performFeatureMatching(Ptr<DescriptorMatcher> descriptorMatcher, Mat & descriptors, Mat & trainedDescriptors,vector<KeyPoint> & keyPoints, vector<KeyPoint> & trainedKeyPoints)
{
	vector<DMatch> featureMatches;
	vector<vector<DMatch> > topNMatches;
	int numberOfNeighborsToFind = 2;
	float nearestNeighborDistanceRatio = .40f;//.49f; //.8

	if(!trainedDescriptors.empty())
	{
		descriptorMatcher->knnMatch(descriptors,trainedDescriptors,topNMatches,numberOfNeighborsToFind);
	}
	else
	{

		descriptorMatcher->knnMatch(descriptors,topNMatches,numberOfNeighborsToFind);
	}


	for(unsigned int i = 0; i< topNMatches.size(); i++)
	{
		if(topNMatches[i].size() >=2)
		{
			float d0 = topNMatches[i][0].distance;
			float d1 = topNMatches[i][1].distance;
			if(d0< d1 * nearestNeighborDistanceRatio)
			{
				featureMatches.push_back(topNMatches[i][0]);

			}

		}
	}



	return featureMatches;
}

Ptr<FeatureDetector> setupLocalization(FeatureExtractionConfig & featureExtractionConfig)
{
    FeatureDetector* featureDetector = NULL;

	switch(featureExtractionConfig.currentLocalizationAlgo)
	{
		case FEATURE_LOCALIZATION_SIFT:
			{
				//SIFT::DetectorParams detectorParams;
				//SIFT::CommonParams commonParams;
				
				#ifdef OPENCV_VER_2_3
					featureDetector = new SiftFeatureDetector(0.04,10.0);//(detectorParams,commonParams);
					//SIFT::DescriptorParams descriptorParams;

					featureExtractionConfig.sift = new SIFT( 0.04,10.0);//(commonParams,detectorParams,descriptorParams);
					

				#else
					featureDetector = new SiftFeatureDetector( 0, 3,0.04,10,1.6);//(detectorParams,commonParams);
					//SIFT::DescriptorParams descriptorParams;

					featureExtractionConfig.sift = new SIFT( 0, 3,0.04,10,1.6);//(commonParams,detectorParams,descriptorParams);
				#endif

			}
			break;

		case FEATURE_LOCALIZATION_SURF:
			{
				//SURF::CvSURFParams detectorParams;
								
				//detectorParams.hessianThreshold = 400.0;
				featureDetector = new SurfFeatureDetector(400.0) ;//(detectorParams.hessianThreshold);
				featureExtractionConfig.surf = new SURF(400.0,4,2,false);//(detectorParams.hessianThreshold,4,2,false);
				//cout << "SURF SIZE: "<<(*(featureExtractionConfig.surf)).descriptorSize() << endl;

			}
			break;
		case FEATURE_LOCALIZATION_ORB:
			{
				int numberOfFeatures = 5000;
				float scaleFactor = 1.2f;
				int levels = 8;

				#ifdef OPENCV_VER_2_3
					ORB::CommonParams orbParams = ORB::CommonParams();
					orbParams.scale_factor_ = scaleFactor;
					orbParams.n_levels_ = levels;

					featureDetector = new OrbFeatureDetector(numberOfFeatures,orbParams);
					featureExtractionConfig.orb = new ORB(numberOfFeatures,orbParams);
				#else				
					featureDetector = new OrbFeatureDetector(numberOfFeatures,scaleFactor,levels);
					featureExtractionConfig.orb = new ORB(numberOfFeatures,scaleFactor,levels);
				#endif
				//(detectorParams.hessianThreshold,4,2,false);
				//cout << "SURF SIZE: "<<(*(featureExtractionConfig.surf)).descriptorSize() << endl;

			}
			break;
		case FEATURE_LOCALIZATION_FAST:
			{

				int fastThreshold = 10;
				featureDetector = new FastFeatureDetector(fastThreshold,true);
				


			}
			break;
		case FEATURE_LOCALIZATION_HoG:
			{

//				if(featureExtractionConfig.hogAddCenterLocation)
				{


				}
				featureExtractionConfig.localizationHogPtr = new HOGDescriptor();
			
			break;
			}
		case FEATURE_LOCALIZATION_SLIDING_WINDOW:
			{

//				if(featureExtractionConfig.hogAddCenterLocation)
				{


				}
				//featureExtractionConfig.localizationHogPtr = new HOGDescriptor();
			
			break;
			}

		default:

			break;
	}

	return featureDetector;

}

void LocalizeFeaturePoints(Mat & inputImage, vector<KeyPoint> & keyPoints,FeatureExtractionConfig & featureExtractionConfig,Ptr<FeatureDetector> detector )
{
	keyPoints.clear();
	if(featureExtractionConfig.currentLocalizationAlgo != FEATURE_LOCALIZATION_CENTER_POINT)
	{
		if(detector != NULL)
		{
			detector->detect(inputImage,keyPoints);
		}
		if(featureExtractionConfig.addCenterLocation && featureExtractionConfig.descHogPtr != NULL)
		{
			keyPoints.push_back(calculateHogCenterLocation(inputImage, featureExtractionConfig.descHogPtr));
			/*
			KeyPoint otherLoc = keyPoints[keyPoints.size()-1];
			otherLoc.pt.x = (float)featureExtractionConfig.descHogPtr->winSize.width;		
			otherLoc.pt.y = (float)featureExtractionConfig.descHogPtr->winSize.height;		
			otherLoc.size = 1.0;
			keyPoints.push_back(otherLoc);
			*/
		}


	}
	else
	{

		if(featureExtractionConfig.addCenterLocation && featureExtractionConfig.descHogPtr != NULL)
		{
			keyPoints.push_back(calculateHogCenterLocation(inputImage, featureExtractionConfig.descHogPtr));
			/*
			KeyPoint otherLoc = keyPoints[keyPoints.size()-1];
			otherLoc.pt.x += (float)featureExtractionConfig.descHogPtr->winSize.width;		
			otherLoc.pt.y += (float)featureExtractionConfig.descHogPtr->winSize.height;		
			otherLoc.size = 1.0;
			keyPoints.push_back(otherLoc);
			*/
		}

	}
	

}


KeyPoint calculateHogCenterLocation(Mat & inputImage, HOGDescriptor * hogDescPtr)
{
	Size windowSize = hogDescPtr->winSize;
	Size imageSize = inputImage.size();

	Point2f midpoint((float)imageSize.width/(float)2,(float)imageSize.height/(float)2);
	Point2f hogWindowUpperLeft((float)midpoint.x-(float)windowSize.width/(float)2,(float)midpoint.y-(float)windowSize.height/(float)2);
	if(hogWindowUpperLeft.x < 0)
	{
		hogWindowUpperLeft.x = 0;
	}
	if(hogWindowUpperLeft.y < 0)
	{
		hogWindowUpperLeft.y = 0;
	}

	KeyPoint returnVal;
	returnVal.pt.x = hogWindowUpperLeft.x;
	returnVal.pt.y = hogWindowUpperLeft.y;
	returnVal.size = 1.0;
	return returnVal;
}


void LocalizeFeaturePoints(Mat & inputImage, vector<KeyPoint> & keyPoints, FeatureExtractionConfig & featureExtractionConfig, Mat & descriptors)
{
	switch(featureExtractionConfig.currentLocalizationAlgo)
	{
		case FEATURE_LOCALIZATION_SIFT:
			{
				Mat tmp;
				if(inputImage.channels() ==3)
				{
					cvtColor(inputImage,tmp,CV_BGR2GRAY);
				}
				else
				{
					tmp.copyTo(tmp);
				}
				keyPoints.clear();
				//descriptors.
				(*(featureExtractionConfig.sift))(tmp,Mat(),keyPoints,descriptors,false);
			}
			break;

		case FEATURE_LOCALIZATION_SURF:
			{

				vector<float> descriptorsVec;
				Mat tmp;
				if(inputImage.channels() ==3)
				{
					cvtColor(inputImage,tmp,CV_BGR2GRAY);
				}
				else
				{
					tmp.copyTo(tmp);
				}
				keyPoints.clear();



				#ifdef OPENCV_VER_2_3
					//cout << "SURF Desc Count: "<<descriptors.rows << "," << descriptors.cols << endl;
					(*(featureExtractionConfig.surf))(tmp,Mat(),keyPoints,descriptorsVec,false);
					int numberOfFeatures = descriptorsVec.size()/(*(featureExtractionConfig.surf)).descriptorSize();
					//cout << "SURF SIZE: "<<(*(featureExtractionConfig.surf)).descriptorSize() << endl;
					descriptors = Mat(descriptorsVec).reshape(1,numberOfFeatures).clone();
				#else

				(*(featureExtractionConfig.surf))(tmp,Mat(),keyPoints,descriptors,false);

				#endif


			}
			break;
		case FEATURE_LOCALIZATION_ORB:
			{

				//vector<float> descriptorsVec;
				Mat tmp;
				if(inputImage.channels() ==3)
				{
					cvtColor(inputImage,tmp,CV_BGR2GRAY);
				}
				else
				{
					tmp.copyTo(tmp);
				}
				keyPoints.clear();


				(*(featureExtractionConfig.orb))(tmp,Mat(),keyPoints,descriptors,false);
				//cout << "ORB Desc Count: "<<descriptors.rows << "," << descriptors.cols << endl;
			}
			break;
		case FEATURE_LOCALIZATION_FAST:
			{

				


			}
			break;
		case FEATURE_LOCALIZATION_HoG:
			{


			
			break;
			}
		case FEATURE_LOCALIZATION_SLIDING_WINDOW:
			{

			
			break;
			}

		default:

			break;
	}


	



}

void LocalizeFeaturePoints(Mat & inputImage, vector<KeyPoint> & keyPoints, vector<Rect>& bboxes,FeatureExtractionConfig & featureExtractionConfig)
{

	vector<Point> points;
	keyPoints.clear();
	if(!featureExtractionConfig.runSlidingWindowLocal)
	{
		featureExtractionConfig.localizationHogPtr->setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
	
		if(featureExtractionConfig.hogMultiScale)
		{
			featureExtractionConfig.localizationHogPtr->detectMultiScale(inputImage, bboxes, 0, Size(8,8), Size(32,32), 1.05, 2);
			for(unsigned int i = 0; i< bboxes.size(); i++)
			{
				KeyPoint currentPoint;
				currentPoint.pt.x = (float)bboxes[i].x;
				currentPoint.pt.y = (float)bboxes[i].y;
				currentPoint.size = (float)bboxes[i].height/(float)featureExtractionConfig.localizationHogPtr->winSize.height;
				keyPoints.push_back(currentPoint);
			}

		}
		else
		{

			featureExtractionConfig.localizationHogPtr->detect(inputImage,points,0,Size(8,8),Size(32,32));
			for(unsigned int i = 0; i< points.size(); i++)
			{
				KeyPoint currentPoint;
				currentPoint.pt.x = (float)points[i].x;
				currentPoint.pt.y = (float)points[i].y;
				currentPoint.size = 1;
				keyPoints.push_back(currentPoint);
			}


		}
	//cout << "Running Sliding Windows" << endl;
	}
	//cout << "Running Localize" << endl;
	if(featureExtractionConfig.addCenterLocation && featureExtractionConfig.localizationHogPtr != NULL)
	{
		keyPoints.push_back(calculateHogCenterLocation(inputImage, featureExtractionConfig.localizationHogPtr));
		/*
		KeyPoint otherLoc = keyPoints[keyPoints.size()-1];
		otherLoc.pt.x = (float)featureExtractionConfig.descHogPtr->winSize.width;		
		otherLoc.pt.y = (float)featureExtractionConfig.descHogPtr->winSize.height;		
		keyPoints.push_back(otherLoc);
		*/
	}

}

void LocalizeFeaturePoints(Mat & inputImage, vector<KeyPoint> & keyPoints, vector<Rect>& bboxes,FeatureExtractionConfig & featureExtractionConfig,FeatureExtractionData & featureExtractionData)
{

	vector<Point> points;
	keyPoints.clear();
	if(!featureExtractionConfig.runSlidingWindowLocal)
	{
		featureExtractionData.localizationHogPtr->setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());

		if(featureExtractionConfig.hogMultiScale)
		{
			featureExtractionData.localizationHogPtr->detectMultiScale(inputImage, bboxes, 0, Size(8,8), Size(32,32), 1.05, 2);
			for(unsigned int i = 0; i< bboxes.size(); i++)
			{
				KeyPoint currentPoint;
				currentPoint.pt.x = (float)bboxes[i].x;
				currentPoint.pt.y = (float)bboxes[i].y;
				currentPoint.size = (float)bboxes[i].height/(float)featureExtractionData.localizationHogPtr->winSize.height;
				keyPoints.push_back(currentPoint);
			}

		}
		else
		{

			featureExtractionData.localizationHogPtr->detect(inputImage,points,0,Size(8,8),Size(32,32));
			for(unsigned int i = 0; i< points.size(); i++)
			{
				KeyPoint currentPoint;
				currentPoint.pt.x = (float)points[i].x;
				currentPoint.pt.y = (float)points[i].y;
				currentPoint.size = 1;
				keyPoints.push_back(currentPoint);
			}


		}
	}

	if(featureExtractionConfig.addCenterLocation && featureExtractionData.localizationHogPtr != NULL)
	{
		keyPoints.push_back(calculateHogCenterLocation(inputImage, featureExtractionData.localizationHogPtr));
		/*
		KeyPoint otherLoc = keyPoints[keyPoints.size()-1];
		otherLoc.pt.x = (float)featureExtractionConfig.descHogPtr->winSize.width;
		otherLoc.pt.y = (float)featureExtractionConfig.descHogPtr->winSize.height;
		keyPoints.push_back(otherLoc);
		*/
	}

}






#ifndef _DRAW_TEXT_ON_IMAGE_DEFINED
#define _DRAW_TEXT_ON_IMAGE_DEFINED
void drawTextOnImage(string newText, Point location, Scalar color, Mat & image)
{

        string text(newText);

        int baseLine = 0;
        Size textSize = getTextSize(text, FONT_HERSHEY_PLAIN, 1, 1, &baseLine);        
        
		if(location.x + textSize.width > image.cols)
		{
			location.x = image.cols-textSize.width -2;
		}

		if(location.y - textSize.height < 0)
		{
			location.y = textSize.height +2;
		}
		if(location.x < 0 || location.y >image.rows)
		{

			return;	
		}

		//Point textOrigin(image.cols - 2*textSize.width - 10, image.rows - 2*baseLine - 10);




        //        msg = cv::format( "%d/%d Undist", (int)imagePoints.size(), nframes );
        

        putText( image, newText, location, FONT_HERSHEY_PLAIN, 1,
                 color);



}

#endif
Ptr<DescriptorExtractor> setupDescriptor(FeatureExtractionConfig & featureExtractionConfig)
{
   Ptr<DescriptorExtractor> descriptorExtractor = NULL;

   switch(featureExtractionConfig.currentDescriptorAlgo)
	{
		case FEATURE_DESC_SIFT:
			{
				//SIFT::DescriptorParams descriptorParams;
				//SIFT::CommonParams commonParams;
				#ifdef OPENCV_VER_2_3
					descriptorExtractor = new SiftDescriptorExtractor( 0.04,10.0);//(descriptorParams,commonParams);
				#else	

					descriptorExtractor = new SiftDescriptorExtractor( 0, 3,0.04,10,1.6);//(descriptorParams,commonParams);
				#endif
			}
			break;

		case FEATURE_DESC_SURF:
			{
				//SURF::CvSURFParams detectorParams;
								
				//detectorParams.hessianThreshold = 400.0;
				//detectorParams.nOctaves = 4;
				//detectorParams.nOctaveLayers = 2;
				descriptorExtractor = new SurfDescriptorExtractor(400.0,4,2);//( detectorParams.nOctaves,detectorParams.nOctaveLayers);



			}
			break;
		case FEATURE_DESC_ORB:
			{
				int numberOfFeatures = 5000;
				float scaleFactor = 1.2f;
				int levels = 8;


				#ifdef OPENCV_VER_2_3
					ORB::CommonParams orbParams = ORB::CommonParams();
					orbParams.scale_factor_ = scaleFactor;
					orbParams.n_levels_ = levels;

					descriptorExtractor =  new OrbDescriptorExtractor(orbParams);
				#else
					descriptorExtractor =  new OrbDescriptorExtractor(numberOfFeatures,scaleFactor,levels);

				#endif
				//descriptorExtractor = new OrbDescriptorExtractor(400.0,4,2);//( detectorParams.nOctaves,detectorParams.nOctaveLayers);



			}
			break;
		case FEATURE_DESC_FAST:

			break;
		case FEATURE_DESC_HoG:
			{


//				if(featureExtractionConfig.hogAddCenterLocation)
				{


				}
				featureExtractionConfig.descHogPtr = new HOGDescriptor();

			}
			break;
		case FEATURE_DESC_BRIEF:
			{
				int briefDescriptorSize = 32;
				descriptorExtractor = new BriefDescriptorExtractor(briefDescriptorSize);
			}
			break;

		default:

			break;
	}

	return descriptorExtractor;



}

Mat buildFeatureDescriptors(Mat & inputImage, vector<KeyPoint> & keyPoints,FeatureExtractionConfig & featureExtractionConfig,Ptr<DescriptorExtractor> descriptorExtractor)
{
	Mat descriptors;
    descriptorExtractor->compute( inputImage, keyPoints, descriptors );
	return descriptors;

}

Mat buildFeatureDescriptors(Mat & inputImage, vector<KeyPoint> & keyPoints,vector<Rect> & bboxes,FeatureExtractionConfig & featureExtractionConfig,HOGDescriptor * descriptorExtractor)
{
	vector<float> descriptorsVec;
	Mat descriptors;


	vector<Point> currentPoints;
	vector<Point> points;
	for(unsigned int i = 0 ; i< keyPoints.size(); i++)
	{
		Point currentPoint;
		currentPoint.x =(int) keyPoints[i].pt.x;
		currentPoint.y = (int)keyPoints[i].pt.y;
		points.push_back(currentPoint);
	}
	//cout << "Points:" << points.size() << endl;
	if(!featureExtractionConfig.runSlidingWindowLocal)
	{
		//cout << "Sliding"<<endl;
		if(points.size() != 0)
		{

			currentPoints.resize(points.size());
			for(unsigned int i = 0; i< currentPoints.size(); i++)
			{
				currentPoints[i] = points[i];

			}


		}
		else if (bboxes.size() != 0)
		{
			currentPoints.resize(bboxes.size());
			for(unsigned int i = 0; i< currentPoints.size(); i++)
			{
				Point currentPoint;

				currentPoint.x = bboxes[i].x;
				currentPoint.y = bboxes[i].y;
				currentPoints[i] = currentPoint;

			}



		}



		descriptorExtractor->compute( inputImage,descriptorsVec,Size(),Size(),currentPoints);
		descriptors.create(currentPoints.size(),descriptorExtractor->getDescriptorSize(),CV_32FC1);
		for(unsigned int row = 0; row <currentPoints.size(); row++)
		{
			for(unsigned int col = 0; col< descriptorExtractor->getDescriptorSize(); col++)
			{
				descriptors.at<float>(row,col) = descriptorsVec[col + row*descriptorExtractor->getDescriptorSize()];

			}
		}

		//cout << "A" << endl;
	}
	else
	{
		//cout << "Desc Vec Size: " << descriptorsVec.size() << endl;
		//cout << "B" << "Rows:" <<inputImage.rows <<" Cols:"<< inputImage.cols << endl;
		descriptorExtractor->compute( inputImage,descriptorsVec,Size(8,8),Size(32,32));
		//cout << "Desc Vec Size: " << descriptorsVec.size() << endl;
		int numberOfVectors = descriptorsVec.size()/descriptorExtractor->getDescriptorSize();
		descriptors.create(numberOfVectors,descriptorExtractor->getDescriptorSize(),CV_32FC1);
		//cout << "Number of Vectors:" << numberOfVectors << endl;
		for(unsigned int row = 0; row <numberOfVectors; row++)
		{
			for(unsigned int col = 0; col< descriptorExtractor->getDescriptorSize(); col++)
			{
				descriptors.at<float>(row,col) = descriptorsVec[col + row*descriptorExtractor->getDescriptorSize()];

			}
		}


		#ifdef VERBOSE
		cout << "Desc Rows: "<<descriptors.rows  << endl;
		#endif
	}




	return descriptors;

}



string convertDescTypeToString(FeaturesDescriptorAlgos descriptorType)
{

	string retVal("");
	switch(descriptorType)
	{
		case FEATURE_DESC_SIFT:
				retVal = "Sift";
			break;
		case FEATURE_DESC_SURF:
				retVal = "Surf";
			break;
		case FEATURE_DESC_ORB:
				retVal = "Orb";
			break;

		case FEATURE_DESC_FAST:
				retVal = "Fast";
			break;

		case FEATURE_DESC_HoG:
				retVal = "Hog";
			break;
		case FEATURE_DESC_BRIEF:
				retVal = "Brief";
			break;
		default:
				retVal = "Unknown";
			break;
	}

	return retVal;


}

string convertLocalTypeToString(FeaturesLocalizationAlgos localizationMethod)
{

	string retVal("");
	switch(localizationMethod)
	{
		case FEATURE_LOCALIZATION_SIFT:
				retVal = "Sift";
			break;
		case FEATURE_LOCALIZATION_SURF:
				retVal = "Surf";
			break;
		case FEATURE_LOCALIZATION_ORB:
				retVal = "Orb";
			break;
		case FEATURE_LOCALIZATION_FAST:
				retVal = "Fast";
			break;
		case FEATURE_LOCALIZATION_HoG:
				retVal = "Hog";
			break;
		case FEATURE_LOCALIZATION_SLIDING_WINDOW:
				retVal = "SlidingWindow";
			break;
		case FEATURE_LOCALIZATION_CENTER_POINT:
				retVal = "CenterPoint";
			break;


		default:
				retVal = "Unknown";
			break;
	}

	return retVal;


}


bool saveFeatureDescriptorsAndOrKeypoints( string filename,  FeaturesDescriptorAlgos descriptorType,FeaturesLocalizationAlgos localizationMethod,Ptr<FeatureDetector> detector,vector<KeyPoint> & keypoints,Ptr<DescriptorExtractor> descriptorExtractor, Mat & descriptors)
{
	FileStorage fs(filename,FileStorage::WRITE);

	fs << "DescType" << descriptorType << "LocalAlgo" << localizationMethod;
	fs << "DescTypeString"<< convertDescTypeToString(descriptorType) << "LocalAlgoString"<<convertLocalTypeToString(localizationMethod);

	if(descriptorExtractor != NULL)
	{	
		fs << "DescStruct" << "{";
		descriptorExtractor->write(fs);
		fs << "}";
	}


	if(!descriptors.empty())
	{
		fs << "Descriptors" << descriptors;
	}
	if(keypoints.size() >0)
	{
		write(fs,"Keypoints",keypoints);
	}
	if(detector != NULL)
	{
		fs << "LocalStruct" << "{";
		detector->write(fs);
		fs << "}";
	}

	return true;

}

bool saveFeatureDescriptorsAndOrKeypoints( string filename,  FeaturesDescriptorAlgos descriptorType,FeaturesLocalizationAlgos localizationMethod,Ptr<FeatureDetector> detector,vector<KeyPoint> & keypoints,HOGDescriptor* descriptorExtractor, Mat & descriptors)
{
	FileStorage fs(filename,FileStorage::WRITE);

	fs << "DescType" << descriptorType << "LocalAlgo" << localizationMethod;
	fs << "DescTypeString"<< convertDescTypeToString(descriptorType) << "LocalAlgoString"<<convertLocalTypeToString(localizationMethod);

	if(descriptorExtractor != NULL)
	{	

		descriptorExtractor->write(fs,"DescStruct");
	}


	if(!descriptors.empty())
	{
		fs << "Descriptors" << descriptors;
	}
	if(keypoints.size() >0)
	{
		write(fs,"Keypoints",keypoints);
	}
	if(detector != NULL)
	{
		fs << "LocalStruct" << "{";
		detector->write(fs);
		fs << "}";
	}

	return true;

}


bool loadFeatureDescriptorsAndOrKeypoints( string filename,  string &descTypeString,string &localTypeString ,FeaturesDescriptorAlgos& descriptorType,FeaturesLocalizationAlgos& localizationMethod,Ptr<FeatureDetector> &detector,vector<KeyPoint> * keypoints,HOGDescriptor*  &descriptorExtractor, Mat * descriptors)
{

	FileStorage fs(filename,FileStorage::READ);

	descriptorType = (FeaturesDescriptorAlgos)((int)fs["DescType"]);
	localizationMethod = (FeaturesLocalizationAlgos)((int)fs["LocalAlgo"]);
	descTypeString = (string)fs["DescTypeString"];
	localTypeString = (string)fs["LocalAlgoString"];

	descriptorExtractor = new HOGDescriptor;
	if(descriptorExtractor != NULL)
	{	

		FileNode descStruct = fs["DescStruct"];
		descriptorExtractor->read(descStruct);
	}


	if(descriptors != NULL)
	{
		fs["Descriptors"] >> *descriptors;
	}
	if(keypoints !=NULL)
	{
		FileNode keyPointsNode = fs["Keypoints"];
		read((const FileNode)keyPointsNode, *keypoints);
	}
	switch(localizationMethod)
	{
		case FEATURE_LOCALIZATION_SIFT:
			{
				detector = new SiftFeatureDetector;

			}
			break;

		case FEATURE_LOCALIZATION_SURF:
			{
				detector = new SurfFeatureDetector;



			}
			break;
		case FEATURE_LOCALIZATION_ORB:
			{
				detector = new OrbFeatureDetector;



			}
			break;
		case FEATURE_LOCALIZATION_FAST:
			{


				detector = new FastFeatureDetector;
				
			}
			break;
		case FEATURE_LOCALIZATION_HoG:
			{


			}
			break;
		default:
			{
				detector = NULL;
			}
			break;
	}

	if(detector != NULL)
	{
		FileNode detectorNode = fs["LocalStruct"];
		detector->read(detectorNode);

	}

	return true;

}
bool loadFeatureDescriptorsAndOrKeypoints( string filename,  string &descTypeString,string &localTypeString ,FeaturesDescriptorAlgos& descriptorType,FeaturesLocalizationAlgos& localizationMethod,Ptr<FeatureDetector>& detector,vector<KeyPoint> * keypoints,Ptr<DescriptorExtractor> & descriptorExtractor, Mat * descriptors)
{
	FileStorage fs(filename,FileStorage::READ);

	descriptorType = (FeaturesDescriptorAlgos)((int)fs["DescType"]);
	localizationMethod = (FeaturesLocalizationAlgos)((int)fs["LocalAlgo"]);
	descTypeString = (string)fs["DescTypeString"];
	localTypeString = (string)fs["LocalAlgoString"];


   switch(descriptorType)
	{
		case FEATURE_DESC_SIFT:
			{
				descriptorExtractor = new SiftDescriptorExtractor;

			}
			break;

		case FEATURE_DESC_SURF:
			{

				descriptorExtractor = new SurfDescriptorExtractor;



			}
			break;
		case FEATURE_DESC_ORB:
			{

				descriptorExtractor = new OrbDescriptorExtractor;



			}
			break;

		case FEATURE_DESC_FAST:

			break;
		case FEATURE_DESC_HoG:
			break;
		case FEATURE_DESC_BRIEF:
			{

				descriptorExtractor = new BriefDescriptorExtractor;
			}
			break;

		default:
				descriptorExtractor = NULL;
			break;
	}






	if(descriptorExtractor != NULL)
	{	

		FileNode descStruct = fs["DescStruct"];
		descriptorExtractor->read(descStruct);
	}
	if(descriptors != NULL)
	{
		fs["Descriptors"] >> *descriptors;
	}
	if(keypoints !=NULL)
	{
		FileNode keyPointsNode = fs["Keypoints"];
		read((const FileNode)keyPointsNode, *keypoints);
	}


	switch(localizationMethod)
	{
		case FEATURE_LOCALIZATION_SIFT:
			{
				detector = new SiftFeatureDetector;

			}
			break;

		case FEATURE_LOCALIZATION_SURF:
			{
				detector = new SurfFeatureDetector;



			}
			break;
		case FEATURE_LOCALIZATION_ORB:
			{
				detector = new OrbFeatureDetector;



			}
			break;
		case FEATURE_LOCALIZATION_FAST:
			{


				detector = new FastFeatureDetector;
				
			}
			break;
		case FEATURE_LOCALIZATION_HoG:
			{


			}
			break;
		default:
			{
				detector = NULL;
			}
			break;
	}

	if(detector != NULL)
	{
		FileNode detectorNode = fs["LocalStruct"];
		detector->read(detectorNode);

	}


return true;


}



