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
 * FeatureClassification.hpp
 *
 *  Created on: Apr 30, 2011
 *      Author: jlclemon
 */




#ifndef FEATURECLASSIFICATION_HPP_
#define FEATURECLASSIFICATION_HPP_

#include <string>
#include <fstream>
#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "ThreadManager.h"
#include <errno.h>
#include <pthread.h>
#include "KnnFlannClassification.h"
#include "HessKnnMatcher.h"
#include "MatcherResultsClassifier.h"
#include "SparseFeaturePointObjectClassifier.h"
#include "MultiThreadedMatResult.h"
#include "MultiThreadVisionGeneral.hpp"
#include "WorkerThreadInfo.h"

using namespace std;
using namespace cv;


#define CLASSIFICATION_DEFAULT_NUMBER_OF_ITERATIONS 1

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
	FEATURE_MATCHER_KNN_FLANN = 2,
	FEATURE_MATCHER_EUCL_DIST = 4,
	FEATURE_MATCHER_RADIUS_FLANN = 8,
	FEATURE_MATCHER_KNN_HESS_KDTREE = 16,
	NUMBER_OF_FEATURE_MATCHER_ALGOS = 5

};
#endif

enum FeaturesClassificationAlgos
{
	FEATURE_CLASSIFIER_LINEAR_SVM = 1,
	FEATURE_CLASSIFIER_BOOSTED_TREE_STUMPS = 2,
	FEATURE_CLASSIFIER_KNN_FLANN = 4,
	FEATURE_CLASSIFIER_KNN_HESS_KDTREE = 8,
	FEATURE_CLASSIFIER_GEOMETRIC_CENTER = 16,
	NUMBER_OF_FEATURE_CLASSIFIER_ALGOS = 5

};

struct FeatureDescriptorPtrs
{
	Ptr<DescriptorExtractor> descriptorExtractor;
	Ptr<HOGDescriptor> descHogPtr;
	FeatureDescriptorPtrs() : descriptorExtractor(NULL),descHogPtr(NULL)
	{


	}


};
struct FeatureDescriptorParams
{
	SIFT::CommonParams siftCommonParams;
	SIFT::DescriptorParams siftDescriptorParams;


};


struct FeatureMatcherPtrs
{
	Ptr<DescriptorMatcher> matcher;
	Ptr<HessKnnMatcher> hessKnnMatcher;
	//HessKnnMatcher * hessKnnMatcher;
	FeatureMatcherPtrs():matcher(NULL)
	{


	}

};
struct FeatureMatcherParams
{
	Ptr<flann::IndexParams> flannIndexParams;
	Ptr<flann::KDTreeIndexParams> flannKDTreeIndexParams;
	Ptr<flann::SearchParams> flannSearchParams;
	Ptr<flann::LinearIndexParams> flannLinearIndexParams;
	Ptr<flann::KMeansIndexParams> flannKmeansIndexParams;
	Ptr<flann::CompositeIndexParams> flannCompositeIndexParams;
	Ptr<flann::AutotunedIndexParams> flannAutoTunedIndexParams;
	double radiusMatchRadius;
	//L2<float> bruteForceDistanceParam;
	HessKnnMatcher::HessMatcherParams hessMatcherParams;
	int numberOfNeighbors;
	FeatureMatcherParams():flannIndexParams(NULL), flannKDTreeIndexParams(NULL),flannSearchParams(NULL),flannLinearIndexParams(NULL), flannKmeansIndexParams(NULL), flannCompositeIndexParams(NULL), flannAutoTunedIndexParams(NULL),radiusMatchRadius(0.0),numberOfNeighbors(32)
	{



	}



};



struct FeatureClassificationPtrs
{
	Ptr<CvStatModel> classifier;
	Ptr<DescriptorMatcher> knnFlannMatcher;
	FeatureClassificationPtrs():classifier(NULL), knnFlannMatcher(NULL)
	{


	}


};

struct KnnFlannClassifierParams
{
	int numberOfNeighbors;
	Ptr<flann::IndexParams> flannIndexParams;
	Ptr<flann::SearchParams> flannSearchParams;
	Ptr<flann::KDTreeIndexParams> flannKDTreeIndexParams;


	KnnFlannClassifierParams(): numberOfNeighbors(0), flannIndexParams(NULL), flannSearchParams(NULL), flannKDTreeIndexParams(NULL)
	{


	}


};

struct KnnHessClassifierParams
{
	int numberOfNeighbors;


	KnnHessClassifierParams(): numberOfNeighbors(0)
	{


	}
};





struct FeatureClassifierParams
{
	CvSVMParams svmParams;


	CvBoostParams boostParams;

	KnnFlannClassifierParams knnFlannClassifierParams;

	MatcherResultsClassifier::MatcherResultsClassifierParams matcherResultsParams;

	SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierParams sparseGeomParams;
};



struct FeatureClassificationConfig
{
	FeaturesDescriptorAlgos currentDescriptorAlgo;
	FeaturesMatchingAlgos currentMatcherAlgo;
	FeaturesClassificationAlgos currentClassificationAlgo;

	FeatureDescriptorPtrs currentDescriptorPtrs;
	FeatureDescriptorParams descriptorParams;
	string descriptorTrainFilename;
	string descriptorQueryFilename;
	string keyPointTrainFilename;
	string keyPointQueryFilename;
	string trainImageFilename;
	string queryImageFilename;




	FeatureMatcherPtrs currentMatcherPtrs;
	FeatureMatcherParams matcherParams;
	string matcherInputFilename;
	string matcherOutputFilename;


	FeatureClassificationPtrs currentClassificationPtrs;
	FeatureClassifierParams classifierParams;
	string trainDescriptorsClassesFile;
	string queryDescriptorsClassesFile;

	string classifierInputFilename;
	string classifierOutputFilename;

	int numberOfVerticalProcs;
	int numberOfHorizontalProcs;
	int numberOfIterations;

	bool singleThreaded;

	bool useTestDescriptors;
	bool matchActive;
	bool classifyActive;
	bool trainClassifier;
	bool trainMatcher;
	bool loadTrainKeypoints;
	bool loadQueryKeypoints;

	bool loadTrainImage;
	bool loadQueryImage;

	bool singleCopyOfClassificationStruct;
	bool partialCopyOfClassificationStruct;
};
typedef void * (*thread_fptr)(void *);


//Get the file config
FeatureClassificationConfig setupFeatureClassificationConfigFromFile(string filename);


//Generalized
//Create the thread manager
void classificationCreateThreadManager(ThreadManager * &classificationThreadManager,int verticalThreads, int horizontalThreads);



//Attach the two data structures
void setClassificationWorkingThreadDataInGeneralWorkingData(vector<struct GeneralWorkerThreadData> &genData, vector<struct WorkerThreadInfo> & workingThreads);



//Setup the data structures
void multiThreadedSetupOnly(Mat & trainDescriptors,Mat & trainDescriptorsClasses, vector<KeyPoint> &trainKeypoints, Mat & trainImage,Mat & queryDescriptors,Mat & queryDescriptorsClasses,vector<KeyPoint> &queryKeypoints,FeatureClassificationConfig & featureClassificationConfig, ThreadManager * &classificationThreadManager,vector<struct GeneralWorkerThreadData> &genData,MultiThreadedMatResult * &multiThreadResultPtr,MultiThreadedMatchResult * &multiThreadMatchResultPtr);




//Generalized
//Attach the functions and the params to the thread
void classificationSetupThreadManager(ThreadManager * &classificationThreadManager,FeatureClassificationConfig & featureClassificationConfig,vector<thread_fptr> threadFunctions, vector<void*> threadArgs,int verticalThreads, int horizontalThreads);



//Generalized
//Launch the threads
void classificationLaunchThreads(ThreadManager * &classificationThreadManager);



void handleDescLoading(Mat & trainDescriptors,Mat & trainDescriptorsClasses, Mat & queryDescriptors,Mat & queryDescriptorsClasses,FeatureClassificationConfig & featureClassificationConfig);
void handleKeyPointLoading(vector<KeyPoint>& trainKeyPoints,vector<KeyPoint>& queryKeyPoints,FeatureClassificationConfig & featureClassificationConfig);
void handleImageLoading(Mat &trainImage, Mat & queryImage,FeatureClassificationConfig & featureClassificationConfig);




//The actual classification
void  classificationCoordinatorThreadFunctionStandAlone(void * workerThreadStruct);
void classificationWorkerThreadFunctionStandAlone(void * workerThreadStruct);

//Setup the classification, the first time
void classificationWorkerThreadSetupFunctionStandAlone(void *  workerThreadStruct);
void classificationCoordinatorThreadSetupFunctionStandAlone(void *  workerThreadStruct);

//Internal Calls
void setupClassificationThreadsData(FeatureClassificationConfig & featureClassificationConfig,ThreadManager * &classificationThreadManager,Mat& queryDescriptors, vector<KeyPoint>& queryKeypoints,Mat &trainDescriptors, vector<KeyPoint> &trainKeypoints, vector<struct GeneralWorkerThreadData> &genData,MultiThreadedMatResult * &multiThreadResultPtr,MultiThreadedMatchResult * &multiThreadMatchResultPtr ,int verticalThreads, int horizontalThreads);


int featureClassification_testSeperate(int argc, const char * argv[]);

#endif /* FEATURECLASSIFICATION_HPP_ */
