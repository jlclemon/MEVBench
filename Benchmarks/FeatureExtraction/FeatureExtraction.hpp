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

/*
 * FeatureExtraction.hpp
 *
 *  Created on: May 24, 2011
 *      Author: jlclemon
 */

#ifndef FEATUREEXTRACTION_HPP_
#define FEATUREEXTRACTION_HPP_


#include <iostream>
#include <fstream>
#include <time.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#ifndef OPENCV_VER_2_2
#include <opencv2/video/tracking.hpp>
#endif

#include "MultiThreadAlgorithmData.h"
#include "MultiThreadVisionGeneral.hpp"

using namespace std;
using namespace cv;



#define EXTRACT_CONFIG_DEFAULT_FILENAME_BASE "./data/image"
#define EXTRACT_CONFIG_DEFAULT_FILENAME_INITIAL_ID 00
#define EXTRACT_CONFIG_DEFAULT_FILENAME_EXT ".jpg"
#define EXTRACT_CONFIG_DEFAULT_LEFT_CAMERA_ID 0
#define EXTRACT_CONFIG_DEFAULT_RIGHT_CAMERA_ID 1
#define EXTRACT_CONFIG_DEFAULT_INPUT_METHOD FEATURE_DESC_SINGLE_STREAM


enum FeaturesLocalizationAlgos
{
	FEATURE_LOCALIZATION_SIFT = 1,
	FEATURE_LOCALIZATION_SURF = 2,
	FEATURE_LOCALIZATION_FAST = 4,
	FEATURE_LOCALIZATION_HoG = 8,
	FEATURE_LOCALIZATION_SLIDING_WINDOW = 16,
	FEATURE_LOCALIZATION_CENTER_POINT = 32,
	NUMBER_OF_FEATURE_LOCALIZATION_ALGOS=6

};
#ifndef FEATURE_DESC_MATCHER_ENUMS_DEFINED
#define FEATURE_DESC_MATCHER_ENUMS_DEFINED
enum FeaturesDescriptorAlgos
{
	FEATURE_DESC_SIFT = 1,
	FEATURE_DESC_SURF = 2,
	FEATURE_DESC_FAST = 4,
	FEATURE_DESC_HoG = 8,
	FEATURE_DESC_BRIEF = 16,
	NUMBER_OF_FEATURE_DESC_ALGOS = 4

};

enum FeaturesMatchingAlgos
{
	FEATURE_MATCHER_BRUTEFORCE = 1,
	FEATURE_MATCHER_FLANN = 2,
	FEATURE_MATCHER_EUCL_DIST = 4,
	FEATURE_MATCHER_RADIUS_FLANN = 8,
	FEATURE_MATCHER_KNN_HESS_KDTREE = 16,
	NUMBER_OF_FEATURE_MATCHER_ALGOS = 5


};
#endif

enum FeaturesDescriptorWindows
{
	FEATURE_DESC_WINDOW_MAIN = 1,
	FEATURE_DESC_WINDOW_INPUT = 2,
	FEATURE_DESC_WINDOW_OUTPUT = 4,
	FEATURE_DESC_WINDOW_DEBUG = 8,
	NUMBER_OF_FEATURE_DESC_WINDOWS = 4

};


enum FeatureDescriptorInputMethods
{
	FEATURE_DESC_SINGLE_STREAM,
	FEATURE_DESC_STEREO_STREAM,
	FEATURE_DESC_IMAGE_FILE,
	FEATURE_DESC_IMAGE_FILE_LIST,
	FEATURE_DESC_VIDEO_FILE,
	NUMBER_OF_FEATURE_DESC_INPUT_METHODS

};

enum FeatureDescriptorImageSourceLocation
{
	FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA=0,
	FEATURE_DESC_IMAGE_SOURCE_LOCATION_RIGHT_CAMERA,
	FEATURE_DESC_IMAGE_SOURCE_LOCATION_DEPTH_CAMERA,
	NUMBER_OF_FEATURE_DESC_IMAGE_SOURCE_LOCATIONS

};


struct FeatureExtractionConfig
{
	bool runSiftLocal:1;
	bool runSurfLocal:1;
	bool runFastLocal:1;
	bool runHogLocal:1;
	bool runSlidingWindowLocal:1;

	bool runSiftDesc:1;
	bool runSurfDesc:1;
	bool runFastDesc:1;
	bool runHogDesc:1;
	bool runBriefDesc:1;

	bool sameLocalAndDesc:1;
	bool redistribute:1;

	bool snapShotMode:1;
	bool loadTrainedDescriptors:1;
	bool trainedDescriptorsLoaded:1;
	bool trainedDescriptorsAvailable:1;
	bool accumulateDesc;
	bool saveAllDescToSingleFile;

	bool useImageList:1;


	bool singleCopyOfConfigStruct:1;

	bool partialCopyOfConfigStruct:1;
	bool showWindows:1;
	bool noWaitKey:1;


	FeaturesLocalizationAlgos currentLocalizationAlgo;
	FeaturesDescriptorAlgos currentDescriptorAlgo;
	FeaturesMatchingAlgos currentFeatureMatchingAlgo;

	SIFT * sift;
	SURF * surf;

	int numberOfImages;
	int leftCameraId;
	int rightCameraId;
	VideoCapture * leftImageCapture;
	VideoCapture * rightImageCapture;
	void * kinectImageCapture;
	string imageNameBase;
	int imageId;
	string imageExtension;
	string videoFileName;


	string descriptorsFilename;
	string descriptorsImageFilename;

	string descOutputFilename;
	int currentOutputImageNumber;

	string imageListBaseDir;
	string imageListFilename;
	vector<string> imageList;
	int currentImageId;
	FeatureDescriptorInputMethods inputMethod;


	int windowsToShow;
	bool showMainWindow;
	bool showInputWindow;
	bool showOutputWindow;
	bool showDebugWindow;
	bool showCurrentFeaturePointsWindow;
	bool showMatchWindow;

	Size frameSize;

	Ptr<FeatureDetector> featureDetector;
	Ptr<DescriptorExtractor> descriptorExtractor;

	HOGDescriptor * localizationHogPtr;
	HOGDescriptor * descHogPtr;
	vector<Point> hogLocations;
	bool hogMultiScale;
	bool addCenterLocation;



	int numberOfVerticalProcs;
	int numberOfHorizontalProcs;


	bool singleThreaded;
	bool outOfImages;
	int maxLoops;
	int loopCount;



};

struct FeatureExtractionData
{
	Ptr<FeatureDetector> featureDetector;
	Ptr<DescriptorExtractor> descriptorExtractor;
	Ptr<DescriptorMatcher> matcher;
	HOGDescriptor * localizationHogPtr;
	HOGDescriptor * descHogPtr;



	Ptr<vector<Mat> >inputImagesPerFrame;
	Ptr<vector<Mat> >trainImagesPerFrame;
	Size frameSize;
	int xBufferAmount;
	int yBufferAmount;
	Ptr<SIFT> sift;
	Ptr<SURF> surf;



	Ptr<Mat> trainedDescriptors;
	Ptr<Point2f> trainedRefPoint;
	Ptr<vector<KeyPoint> >trainedKeypoints;
	Ptr<vector<Vec2f> >trainedRefVectors;



};


struct FeatureExtractionMultiThreadData : MultiThreadAlgorithmData
{

	vector<KeyPoint> * keypoints;
	Mat * descriptors;


	Mat myDescriptors;
	int myKeyPointsCount;
	int myDescriptorsCount;
	int myDescriptorsIndex;
	int myKeyPointsIndex;
	vector<KeyPoint> myKeypoints;
	vector<Rect> myBboxes;

	virtual void clear();
	virtual void clearNonShared();
	FeatureExtractionMultiThreadData();
};

void multiThreadFeatureExtraction_StandAloneTest(int argc, const char * argv[]);
void setupFeatureExtractionConfigFromFile(string filename, FeatureExtractionConfig & featureExtractionConfig);
void featureExtractionSetupFeatureExtractionData(FeatureExtractionConfig & featureExtractionConfig, FeatureExtractionData & featureExtractionData);
void featureExtractionCreateThreadManager(ThreadManager * &threadManager,int verticalThreads, int horizontalThreads);
void setFeatureExtractionWorkingThreadDataInGeneralWorkingData(vector<GeneralWorkerThreadData> &genData, vector<struct FeatureExtractionWorkerThreadInfo> & workingThreads);
void featureExtractionMultiThreadedSetupOnly(FeatureExtractionConfig & featureExtractionConfig, FeatureExtractionData & featureExtractionData, ThreadManager * &threadManager,vector<struct GeneralWorkerThreadData> &genData);
void featureExtractionSetupThreadManager(ThreadManager * &threadManager,FeatureExtractionConfig & featureExtractionConfig,vector<thread_fptr> threadFunctions, vector<void*> threadArgs,int verticalThreads, int horizontalThreads);
void featureExtractionLaunchThreads(ThreadManager * &threadManager);
void * featureExtraction_testWorkerThreadStandAlone(void * threadParam);
void * featureExtraction_testCoordinatorThreadStandAlone(void * threadParam);
void  featureExtractionCoordinatorThreadSetupFunctionStandAlone(struct GeneralWorkerThreadData * genData, void * workerThreadStruct);
void  featureExtractionWorkerThreadSetupFunctionStandAlone(struct GeneralWorkerThreadData * genData,void * workerThreadStruct);
void  featureExtractionCoordinatorThreadFunctionStandAlone(struct GeneralWorkerThreadData * genData,void * workerThreadStruct);
void  featureExtractionWorkerThreadFunctionStandAlone(struct GeneralWorkerThreadData * genData,void * workerThreadStruct);

int getNextImages(vector<Mat>&  images,FeatureExtractionConfig & featureExtractionConfig);
#endif /* FEATUREEXTRACTION_HPP_ */
