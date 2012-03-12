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

#include "FeatureClassification.hpp"
#include "WorkerThreadInfo.h"

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
vector<TSC_VAL_w> fc_timingVector;
#endif
#ifdef CLOCK_GETTIME_TIMING
vector<struct timespec> fc_timeStructVector;
#endif

void drawTextOnImage(string newText, Point location, Scalar color, Mat & image);
void coordinatorSetAllDoneFlag(struct WorkerThreadInfo * workerInfo, int numberOfThreads);
void coordinatorThreadDistributeQueryKeypointsAndDescriptors(FeatureClassificationConfig &featureClassificationConfig,Mat & queryDescriptors,vector<KeyPoint>& queryKeypoints,struct WorkerThreadInfo * workerInfo);
void handleClassificationOperation(Mat & queryDescriptors,Mat & queryDescriptorsClasses,vector<KeyPoint> & queryKeypoints,Mat & queryImage,FeatureClassifierParams & params, FeatureClassificationPtrs & classificationPtrs, FeatureMatcherPtrs & matcherPtrs , FeatureMatcherParams & matcherParams,vector<vector <DMatch> > &matches,FeaturesClassificationAlgos & currentClassificationAlgo,FeaturesMatchingAlgos currentMatcherAlgo );
void coordinatorWakeUpOtherThreads(struct WorkerThreadInfo * workerInfo,int numberOfThreads);
void coordinatorSetAllDoneFlag(struct WorkerThreadInfo * workerInfo, int numberOfThreads);
void coordinatorThreadDistributeQueryKeypointsAndDescriptors(FeatureClassificationConfig &featureClassificationConfig,Mat & queryDescriptors,vector<KeyPoint>& queryKeypoints,struct WorkerThreadInfo * workerInfo);





void * threadTestFunction(void * threadClass)
{

	Thread*  param = reinterpret_cast<Thread *>(threadClass);

	cout << "Current Thread is "<<param->getThreadLogicalId() << endl;


	param->waitForWakeup();


	




	cout << "Thread " << param->getThreadLogicalId() << " is waiting at barrier" << endl;
	param->waitAtBarrier();
	cout << "Thread " << param->getThreadLogicalId() << " is waiting at second barrier" << endl;
	param->waitAtBarrier();
	cout << "Thread " << param->getThreadLogicalId() << " is done with test" << endl;

	return NULL;
}


void threadTestMain()
{
	int numberOfThreads = 200;
	ThreadManager classificationThreadManager;

	classificationThreadManager.setupBarrier(numberOfThreads);
	cout << "Creating Threads" << endl;
	for(int i = 0; i<numberOfThreads;i++)
	{
		Thread currentThread;
		int numberOfThreads = classificationThreadManager.addThread(currentThread);
	}

	for(int i = 0; i<numberOfThreads;i++)
	{
		Thread * currentThreadPtr = (classificationThreadManager.getThread(i));


		currentThreadPtr->setThreadLogicalId(i);
		currentThreadPtr->setThreadParam(currentThreadPtr);
		currentThreadPtr->setThreadFunction(threadTestFunction);
		classificationThreadManager.setThreadBarrier(i);
		cout << "Thread Param "<< currentThreadPtr << endl;

		int errcode = pthread_create(currentThreadPtr->getThreadIdPtr(),NULL,currentThreadPtr->getThreadFunction(),currentThreadPtr);
		if(errcode != 0)
		{
			cout << "Error creating thread: " << strerror(errcode) << endl;
		}
		cout << "Pthread Create Val: " << strerror(errcode) << endl;

		


	}



	sleep(1);
	for(int i=0 ; i<classificationThreadManager.getNumberOfThreads(); i++) 
	{
		classificationThreadManager.wakeupThread(i);
	}




	cout << "Waiting for Threads." << endl;
	for(int i=0 ; i<classificationThreadManager.getNumberOfThreads() ; i++) 
	{
		pthread_join(classificationThreadManager.getThread(i)->getThreadId(), NULL);
	}

	cout << "Threads are done." << endl;


}












Mat loadDescriptorClasses(string filename, string & descriptorClassesInfo)
{
	FileStorage fs(filename,FileStorage::READ);
	Mat descriptorClasses;


	descriptorClassesInfo = (string)fs["DescriptorsClassesInfo"];

	fs["DescriptorClasses"] >> descriptorClasses;

//	cout << "Desc Classes:" << descriptorClasses;

	return descriptorClasses;
}


void createTestDescriptors(Mat & descriptors, Mat & classes, int numberOfDescriptors, int descriptorWidth,int descriptorClass, bool reverse)
{
	float descriptorBaseAmount = 1.0f/(float)numberOfDescriptors;
	float descriptorWidthBaseAmount = 1.0f/(descriptorWidth);


	descriptors.create(numberOfDescriptors,descriptorWidth,CV_32FC1);
	classes.create(numberOfDescriptors,1,CV_32FC1);

	if(!reverse)
	{
		for(int i = 0; i < descriptors.rows; i++)
		{
			classes.at<float>(i,0) = (float)descriptorClass;

			for(int j = 0; j < descriptors.cols; j++)
			{

				float currentValeMultiplier;

				if(j<= descriptors.cols/2)
				{
					currentValeMultiplier = j*descriptorWidthBaseAmount;
				}
				else
				{
					currentValeMultiplier = ((descriptors.cols-j)) *descriptorWidthBaseAmount;
				}


				descriptors.at<float>(i,j) = (i* descriptorBaseAmount)*(currentValeMultiplier);



			}
		}

	}
	else
	{

		for(int i = 0; i < descriptors.rows; i++)
		{
			classes.at<float>(i,0) = (float)descriptorClass;

			for(int j = 0; j < descriptors.cols; j++)
			{

				float currentValeMultiplier;

				if(j<= descriptors.cols/2)
				{
					currentValeMultiplier = j*descriptorWidthBaseAmount;
				}
				else
				{
					currentValeMultiplier = ((descriptors.cols-j)) *descriptorWidthBaseAmount;
				}


				descriptors.at<float>(descriptors.rows-1-i,j) = (i* descriptorBaseAmount)*(currentValeMultiplier);



			}
		}


	}



	return;
}





Mat loadTrainDescriptorClasses(FeatureClassificationConfig & featureClassificationConfig)
{

	Mat descriptorClasses;
	string descriptorClassesInfo;
	descriptorClasses = loadDescriptorClasses(featureClassificationConfig.trainDescriptorsClassesFile, descriptorClassesInfo);
	cout << "Descriptor Classes Info: " << descriptorClassesInfo<< endl;

	return descriptorClasses;
}



Mat loadTrainDescriptors(FeatureClassificationConfig & featureClassificationConfig)
{

	FileStorage fs(featureClassificationConfig.descriptorTrainFilename,FileStorage::READ);




	featureClassificationConfig.currentDescriptorAlgo = (FeaturesDescriptorAlgos)((int)fs["DescType"]);



	string descTypeString = (string)fs["DescTypeString"];
	cout << "Loading Train Descriptors.  Descriptor Type: "  << descTypeString << endl;


	FeatureDescriptorPtrs descriptorPtrs;


	Mat descriptors;
	fs["Descriptors"] >> descriptors;
	switch(featureClassificationConfig.currentDescriptorAlgo)
	{

		case FEATURE_DESC_SIFT:
		{
			featureClassificationConfig.currentDescriptorPtrs.descriptorExtractor = new SiftDescriptorExtractor;
			FileNode descStruct = fs["DescStruct"];
			featureClassificationConfig.currentDescriptorPtrs.descriptorExtractor->read(descStruct);



			break;
		}
		case FEATURE_DESC_SURF:
		{
			featureClassificationConfig.currentDescriptorPtrs.descriptorExtractor = new SurfDescriptorExtractor;
			FileNode descStruct = fs["DescStruct"];
			featureClassificationConfig.currentDescriptorPtrs.descriptorExtractor->read(descStruct);


			break;
		}
		case FEATURE_DESC_FAST:
		{


			break;
		}
		case FEATURE_DESC_HoG:
		{

			featureClassificationConfig.currentDescriptorPtrs.descHogPtr = new HOGDescriptor;
			FileNode descStruct = fs["DescStruct"];
			featureClassificationConfig.currentDescriptorPtrs.descHogPtr->read(descStruct);


			break;
		}
		case FEATURE_DESC_BRIEF:
		{
			featureClassificationConfig.currentDescriptorPtrs.descriptorExtractor = new BriefDescriptorExtractor;
			FileNode descStruct = fs["DescStruct"];
			featureClassificationConfig.currentDescriptorPtrs.descriptorExtractor->read(descStruct);


			break;
		}
		default:
		{


			break;
		}

	}




	return descriptors;
}


Mat loadQueryDescriptorClasses(FeatureClassificationConfig & featureClassificationConfig)
{

	Mat descriptorClasses;
	string descriptorClassesInfo;
	descriptorClasses = loadDescriptorClasses(featureClassificationConfig.queryDescriptorsClassesFile, descriptorClassesInfo);
	cout << "Descriptor Classes Info: " << descriptorClassesInfo<< endl;

	return descriptorClasses;
}





Mat loadQueryDescriptors(FeatureClassificationConfig & featureClassificationConfig)
{

	FileStorage fs(featureClassificationConfig.descriptorQueryFilename,FileStorage::READ);




	FeaturesDescriptorAlgos currentDescriptorAlgo = (FeaturesDescriptorAlgos)((int)fs["DescType"]);



	string descTypeString = (string)fs["DescTypeString"];



	cout << "Loading Query Descriptors.  Descriptor Type: "  << descTypeString << endl;





	Mat descriptors;
	fs["Descriptors"] >> descriptors;
	if(currentDescriptorAlgo != featureClassificationConfig.currentDescriptorAlgo)
	{
		cout << "Error: Query and Train descriptors do not match.  Exiting..." << endl;
		exit(0);
	}



	return descriptors;
}




vector<KeyPoint> loadKeyPoints(string filename, string keyPointNodeName, string keyPointInfoName, string & keyPointInfo)
{

	vector<KeyPoint> keypoints;
	FileStorage fs(filename,FileStorage::READ);


	if(fs.isOpened())
	{
		FileNode keyPointsNode = fs[keyPointNodeName.c_str()];

		if(!keyPointsNode.empty())
		{
			read((const FileNode)keyPointsNode, keypoints);


			if(keyPointInfoName.compare("") !=0)
			{
				keyPointInfo = (string)fs[keyPointInfoName.c_str()];
			}


		}
		else
		{
			cout << "WARNING: Unable to find keypoints file node" << endl;


		}
	}
	else
	{
		cout << "WARNING: Unable to open keypoints file" << endl;


	}

	return keypoints;



}


vector<KeyPoint> loadTrainKeypoints(FeatureClassificationConfig & featureClassificationConfig)
{
	string keyPointInfo;
	string keyPointInfoName;
	return loadKeyPoints(featureClassificationConfig.keyPointTrainFilename, "Keypoints", keyPointInfoName, keyPointInfo);

}

vector<KeyPoint> loadQueryKeypoints(FeatureClassificationConfig & featureClassificationConfig)
{
	string keyPointInfo;
	string keyPointInfoName;
	return loadKeyPoints(featureClassificationConfig.keyPointQueryFilename, "Keypoints", keyPointInfoName, keyPointInfo);

}

Mat loadImage(string filename)
{

	Mat image = imread(filename);
	if(image.empty())
	{
		cout << "WARNING: Unable to load image:" << filename << endl;


	}

	return image;
}

Mat loadQueryImage(FeatureClassificationConfig & featureClassificationConfig)
{


	return loadImage(featureClassificationConfig.queryImageFilename);
}

Mat loadTrainImage(FeatureClassificationConfig & featureClassificationConfig)
{


	return loadImage(featureClassificationConfig.trainImageFilename);
}



FeatureDescriptorPtrs setupDescriptor(FeatureClassificationConfig & featureClassificationConfig, FeatureDescriptorParams & params)
{

	FeatureDescriptorPtrs descriptorPtrs;
	switch(featureClassificationConfig.currentDescriptorAlgo)
	{

		case FEATURE_DESC_SIFT:
		{


			break;
		}
		case FEATURE_DESC_SURF:
		{


			break;
		}
		case FEATURE_DESC_FAST:
		{


			break;
		}
		case FEATURE_DESC_HoG:
		{


			break;
		}
		case FEATURE_DESC_BRIEF:
		{


			break;
		}
		default:
		{


			break;
		}

	}

	return descriptorPtrs;
}

void cleanupDescriptor(FeatureClassificationConfig & featureClassificationConfig)
{

	switch(featureClassificationConfig.currentDescriptorAlgo)
	{

		case FEATURE_DESC_SIFT:
		{

			/*
			if(featureClassificationConfig.currentDescriptorPtrs.descriptorExtractor != NULL)
			{
				delete featureClassificationConfig.currentDescriptorPtrs.descriptorExtractor;
				featureClassificationConfig.currentDescriptorPtrs.descriptorExtractor = NULL;

			}

			*/
			break;


		}

		case FEATURE_DESC_SURF:
		{
			/*
			if(featureClassificationConfig.currentDescriptorPtrs.descriptorExtractor != NULL)
			{
				delete featureClassificationConfig.currentDescriptorPtrs.descriptorExtractor;
				featureClassificationConfig.currentDescriptorPtrs.descriptorExtractor = NULL;

			}
			*/

			break;
		}
		case FEATURE_DESC_FAST:
		{


			break;
		}
		case FEATURE_DESC_HoG:
		{
			/*
			if(featureClassificationConfig.currentDescriptorPtrs.descHogPtr != NULL)
			{
				delete featureClassificationConfig.currentDescriptorPtrs.descHogPtr;
				featureClassificationConfig.currentDescriptorPtrs.descHogPtr = NULL;

			}
			*/
			break;
		}
		case FEATURE_DESC_BRIEF:
		{

			/*
			if(featureClassificationConfig.currentDescriptorPtrs.descriptorExtractor != NULL)
			{
				delete featureClassificationConfig.currentDescriptorPtrs.descriptorExtractor;
				featureClassificationConfig.currentDescriptorPtrs.descriptorExtractor = NULL;

			}
			*/

			break;
		}
		default:
		{


			break;
		}

	}





}



FeatureMatcherParams setupMatcherParams(FeatureClassificationConfig & featureClassificationConfig)
{

	FeatureMatcherParams params;

	switch(featureClassificationConfig.currentMatcherAlgo)
	{

		case FEATURE_MATCHER_BRUTEFORCE:
		{
			//params.bruteForceDistanceParam;

			break;
		}
		case FEATURE_MATCHER_KNN_FLANN:
		{

			#ifdef OPENCV_VER_2_3

				//params.flannKDTreeIndexParams

				flann::KDTreeIndexParams *tmp = new flann::KDTreeIndexParams(4);
				//params.flannKDTreeIndexParams->trees = 4;
				//tmp->trees = 4;

				params.flannSearchParams = new flann::SearchParams(200);
				//params.flannSearchParams->checks = 200;
				params.flannIndexParams = &(*(tmp));//params.flannKDTreeIndexParams));
				params.numberOfNeighbors = 32;


			#else
				//params.flannKDTreeIndexParams

				flann::KDTreeIndexParams *tmp = new flann::KDTreeIndexParams;
				//params.flannKDTreeIndexParams->trees = 4;
				tmp->trees = 4;

				params.flannSearchParams = new flann::SearchParams;
				params.flannSearchParams->checks = 200;
				params.flannIndexParams = &(*(tmp));//params.flannKDTreeIndexParams));
				params.numberOfNeighbors = 32;
			#endif



			break;
		}
		case FEATURE_MATCHER_EUCL_DIST:
		{


			break;
		}
		case FEATURE_MATCHER_RADIUS_FLANN:
		{

			#ifdef OPENCV_VER_2_3
				flann::KDTreeIndexParams *tmp = new flann::KDTreeIndexParams(4);
				//params.flannKDTreeIndexParams = new flann::KDTreeIndexParams(4);
				//params.flannIndexParams  = new flann::KDTreeIndexParams(4);
				//params.flannKDTreeIndexParams->trees = 4;
				//tmp->trees = 4;


				params.flannSearchParams = new flann::SearchParams(200);
				//params.flannSearchParams->checks = 200;
				params.radiusMatchRadius = .1;
				params.flannIndexParams = &(*(tmp));//params.flannKDTreeIndexParams));


			#else
				flann::KDTreeIndexParams *tmp = new flann::KDTreeIndexParams;
				//params.flannKDTreeIndexParams = new flann::KDTreeIndexParams(4);
				//params.flannIndexParams  = new flann::KDTreeIndexParams(4);
				//params.flannKDTreeIndexParams->trees = 4;
				tmp->trees = 4;


				params.flannSearchParams = new flann::SearchParams;
				params.flannSearchParams->checks = 200;
				params.radiusMatchRadius = .1;
				params.flannIndexParams = &(*(tmp));//params.flannKDTreeIndexParams));
			#endif
			break;
		}
		case FEATURE_MATCHER_KNN_HESS_KDTREE:
		{

			params.hessMatcherParams.nearestNeighborRatio = HessKnnMatcher::HessMatcherParams::GET_DEFAULT_NEIGHBOR_RATIO();
			params.hessMatcherParams.numberOfNeighbors =HessKnnMatcher::HessMatcherParams::GET_DEFAULT_NUMBER_NEIGHBORS();
			params.hessMatcherParams.numberOfChecks = HessKnnMatcher::HessMatcherParams::GET_DEFAULT_NUMBER_CHECKS();
			break;
		}
		default:
		{


			break;
		}


	}


	return params;
}


FeatureMatcherPtrs setupMatcher(FeatureClassificationConfig & featureClassificationConfig, FeatureMatcherParams params)
{

	FeatureMatcherPtrs matcherPtrs;
	switch(featureClassificationConfig.currentMatcherAlgo)
	{

		case FEATURE_MATCHER_BRUTEFORCE:
		{
			matcherPtrs.matcher = new BruteForceMatcher<L2<float> >();

			break;
		}
		case FEATURE_MATCHER_KNN_FLANN:
		{
			matcherPtrs.matcher = new FlannBasedMatcher(params.flannIndexParams, params.flannSearchParams);

			break;
		}
		case FEATURE_MATCHER_EUCL_DIST:
		{


			break;
		}
		case FEATURE_MATCHER_RADIUS_FLANN:
		{

			matcherPtrs.matcher = new FlannBasedMatcher(params.flannIndexParams, params.flannSearchParams);
			break;
		}
		case FEATURE_MATCHER_KNN_HESS_KDTREE:
		{

			matcherPtrs.hessKnnMatcher = new HessKnnMatcher(params.hessMatcherParams);
			break;
		}
		default:
		{


			break;
		}

	}


	return matcherPtrs;


}


void cleanUpMatcher(FeatureClassificationConfig & featureClassificationConfig)
{

	FeatureMatcherPtrs matcherPtrs = featureClassificationConfig.currentMatcherPtrs;

	FeatureMatcherParams params = featureClassificationConfig.matcherParams;
	switch(featureClassificationConfig.currentMatcherAlgo)
	{

		case FEATURE_MATCHER_BRUTEFORCE:
		{
			/*
			if(matcherPtrs.matcher != NULL)
			{
				delete matcherPtrs.matcher;
				matcherPtrs.matcher = NULL;


			}

			 */


			break;
		}
		case FEATURE_MATCHER_KNN_FLANN:
		{
			/*
			if(matcherPtrs.matcher != NULL)
			{
				delete matcherPtrs.matcher;
				matcherPtrs.matcher = NULL;


			}



			if(params.flannIndexParams != NULL)
			{
				delete params.flannIndexParams;
				params.flannIndexParams = NULL;

			}

			if(params.flannSearchParams != NULL)
			{
				delete params.flannSearchParams;
				params.flannSearchParams = NULL;

			}



			*/
			break;
		}
		case FEATURE_MATCHER_EUCL_DIST:
		{


			break;
		}
		case FEATURE_MATCHER_RADIUS_FLANN:
		{
			/*
			if(matcherPtrs.matcher != NULL)
			{
				delete matcherPtrs.matcher;
				matcherPtrs.matcher = NULL;


			}

			if(params.flannIndexParams != NULL)
			{
				delete params.flannIndexParams;
				params.flannIndexParams = NULL;

			}

			if(params.flannSearchParams != NULL)
			{
				delete params.flannSearchParams;
				params.flannSearchParams = NULL;

			}

			*/
			break;
		}
		case FEATURE_MATCHER_KNN_HESS_KDTREE:
		{
			/*
			if(matcherPtrs.hessKnnMatcher != NULL)
			{
				delete matcherPtrs.hessKnnMatcher;
				matcherPtrs.hessKnnMatcher = NULL;


			}
			*/

			break;
		}
		default:
		{


			break;
		}

	}


	return;


}



FeatureClassifierParams setupClassifierParams(FeatureClassificationConfig & featureClassificationConfig)
{
	FeatureClassifierParams params;

	switch(featureClassificationConfig.currentClassificationAlgo)
	{

		case FEATURE_CLASSIFIER_LINEAR_SVM:
		{


			params.svmParams.kernel_type = CvSVM::LINEAR;
			params.svmParams.svm_type = CvSVM::C_SVC;

			params.svmParams.degree = 0;
			params.svmParams.gamma = 1;
			params.svmParams.coef0 = 0;
			params.svmParams.C = 1;
			params.svmParams.nu =0;
			params.svmParams.p = 0;
			params.svmParams.class_weights = NULL;
			params.svmParams.term_crit = cvTermCriteria( CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, FLT_EPSILON );



			break;
		}
		case FEATURE_CLASSIFIER_BOOSTED_TREE_STUMPS:
		{
			params.boostParams.boost_type = CvBoost::REAL;
			params.boostParams.weak_count = 3780;
			params.boostParams.weight_trim_rate = 0.95;
			params.boostParams.cv_folds = 0;
			params.boostParams.max_depth = 1;

			break;
		}
		case FEATURE_CLASSIFIER_KNN_FLANN:
		{

			#ifdef OPENCV_VER_2_3
				flann::KDTreeIndexParams * kdTreeIndexParams = new flann::KDTreeIndexParams(4);
				//params.knnFlannClassifierParams.flannKDTreeIndexParams =
				//kdTreeIndexParams->trees = 4;

				params.knnFlannClassifierParams.flannSearchParams = new flann::SearchParams(200);
				//params.knnFlannClassifierParams.flannSearchParams->checks = 200;

				params.knnFlannClassifierParams.numberOfNeighbors = 32;
				params.knnFlannClassifierParams.flannIndexParams = kdTreeIndexParams;

			#else
				flann::KDTreeIndexParams * kdTreeIndexParams = new flann::KDTreeIndexParams;
				//params.knnFlannClassifierParams.flannKDTreeIndexParams =
				kdTreeIndexParams->trees = 4;

				params.knnFlannClassifierParams.flannSearchParams = new flann::SearchParams;
				params.knnFlannClassifierParams.flannSearchParams->checks = 200;

				params.knnFlannClassifierParams.numberOfNeighbors = 32;
				params.knnFlannClassifierParams.flannIndexParams = kdTreeIndexParams;
			#endif
			break;
		}
		case FEATURE_CLASSIFIER_KNN_HESS_KDTREE:
		{
			params.matcherResultsParams.minClassValue = MatcherResultsClassifier::MatcherResultsClassifierParams::DEFAULT_MIN_CLASS_VALUE;
//			params.matcherResultsParams.

			break;
		}
		case FEATURE_CLASSIFIER_GEOMETRIC_CENTER:
		{

			params.sparseGeomParams.maxAveSqError = SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierParams::GET_DEFAULT_MAX_AVE_SQ_ERROR();
			params.sparseGeomParams.nearestNeighborDistanceRatio = SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierParams::GET_DEFAULT_NN_RATIO();
			params.sparseGeomParams.maxIndSqError =SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierParams::GET_DEFAULT_MAX_IND_SQ_ERROR();


			break;
		}
		default:
		{


			break;
		}

	}



	return params;
}

bool loadClassifier(FeatureClassificationPtrs&  classifierPtrs, FeatureClassificationConfig & featureClassificationConfig)
{
	bool returnVal = false;
	if(featureClassificationConfig.classifierInputFilename.compare("") != 0)
	{
		switch(featureClassificationConfig.currentClassificationAlgo)
		{

			case FEATURE_CLASSIFIER_LINEAR_SVM:
			{
				CvSVM * currentClassifier = reinterpret_cast<CvSVM *>(&(*classifierPtrs.classifier));

				FileStorage fs(featureClassificationConfig.classifierInputFilename,FileStorage::READ);

				if(!fs.isOpened())
				{
					cout << "Unable to open file: " << featureClassificationConfig.classifierInputFilename << " for reading." << endl;
					break;

				}

				CvFileNode * nodePtr;
				FileNode node = fs["svmClassifier"];

				currentClassifier->read(*fs,*node);
				if(currentClassifier->get_support_vector_count() != 0)
				{

					cout << "SVM data loaded from file: "<< featureClassificationConfig.classifierInputFilename << "." <<endl;
					returnVal = true;
				}
				else
				{

					cout << "Unable to read SVM data from file: " << featureClassificationConfig.classifierInputFilename << "." <<endl;

				}



				break;
			}
			case FEATURE_CLASSIFIER_BOOSTED_TREE_STUMPS:
			{


				CvBoost * currentClassifier = reinterpret_cast<CvBoost *>(&(*classifierPtrs.classifier));

				FileStorage fs(featureClassificationConfig.classifierInputFilename,FileStorage::READ);

				if(!fs.isOpened())
				{
					cout << "Unable to open file: " << featureClassificationConfig.classifierInputFilename << " for reading." << endl;
					break;

				}

				CvFileNode * nodePtr;
				FileNode node = fs["boostClassifier"];

				currentClassifier->read(*fs,*node);
				if(currentClassifier->get_weak_predictors() != NULL)
				{

					cout << "Boost data loaded from file: "<< featureClassificationConfig.classifierInputFilename << "." <<endl;
					returnVal = true;
				}
				else
				{

					cout << "Unable to read boost data from file: " << featureClassificationConfig.classifierInputFilename << "." <<endl;

				}





				break;
			}
			case FEATURE_CLASSIFIER_KNN_FLANN:
			{
				//classifierPtrs.classifier = new KnnFlannClassification;

				KnnFlannClassification * currentClassifier = reinterpret_cast<KnnFlannClassification *>(&(*classifierPtrs.classifier));

				FileStorage fs(featureClassificationConfig.classifierInputFilename,FileStorage::READ);

				if(!fs.isOpened())
				{
					cout << "Unable to open file: " << featureClassificationConfig.classifierInputFilename << " for reading." << endl;
					break;

				}

				CvFileNode * nodePtr;
				FileNode node = fs["knnFlannClassifier"];

				currentClassifier->read(*fs,*node);


				//if(currentClassifier->get_support_vector_count() != 0)
				//{

				//	cout << "Boost data loaded from file: "<< featureClassificationConfig.classifierInputFilename << "." <<endl;
				//	returnVal = true;
				//}
				//else
				//{

				cout << "Unable to read knn flann data from file: " << featureClassificationConfig.classifierInputFilename << "." <<endl;

				//}





				break;
			}
			case FEATURE_CLASSIFIER_KNN_HESS_KDTREE:
			{
				MatcherResultsClassifier * currentClassifier = reinterpret_cast<MatcherResultsClassifier *>(&(*classifierPtrs.classifier));

				FileStorage fs(featureClassificationConfig.classifierInputFilename,FileStorage::READ);

				if(!fs.isOpened())
				{
					cout << "Unable to open file: " << featureClassificationConfig.classifierInputFilename << " for reading." << endl;
					break;

				}

				CvFileNode * nodePtr;
				FileNode node = fs["knnHessClassifier"];

				currentClassifier->read(*fs,*node);


				//if(currentClassifier->get_support_vector_count() != 0)
				//{

				//	cout << "Boost data loaded from file: "<< featureClassificationConfig.classifierInputFilename << "." <<endl;
				//	returnVal = true;
				//}
				//else
				//{

				cout << "Unable to read knn hess data from file: " << featureClassificationConfig.classifierInputFilename << "." <<endl;

				//}




				break;
			}
			case FEATURE_CLASSIFIER_GEOMETRIC_CENTER:
			{
				SparseFeaturePointObjectClassifier * currentClassifier = reinterpret_cast<SparseFeaturePointObjectClassifier *>(&(*classifierPtrs.classifier));

				FileStorage fs(featureClassificationConfig.classifierInputFilename,FileStorage::READ);

				if(!fs.isOpened())
				{
					cout << "Unable to open file: " << featureClassificationConfig.classifierInputFilename << " for reading." << endl;
					break;

				}

				CvFileNode * nodePtr;
				FileNode node = fs["sparseGeomClassifier"];

				currentClassifier->read(*fs,*node);


				//if(currentClassifier->get_support_vector_count() != 0)
				//{

				//	cout << "Boost data loaded from file: "<< featureClassificationConfig.classifierInputFilename << "." <<endl;
				//	returnVal = true;
				//}
				//else
				//{

				cout << "Unable to read Sparse Gemo data from file: " << featureClassificationConfig.classifierInputFilename << "." <<endl;

				//}





				break;
			}
			default:
			{


				break;
			}

		}
	}


	return returnVal;


}

FeatureClassificationPtrs setupClassifier(FeatureClassificationConfig & featureClassificationConfig,FeatureClassifierParams params)
{
	FeatureClassificationPtrs classifierPtrs;



	switch(featureClassificationConfig.currentClassificationAlgo)
	{

		case FEATURE_CLASSIFIER_LINEAR_SVM:
		{
			classifierPtrs.classifier = new CvSVM;

			break;
		}
		case FEATURE_CLASSIFIER_BOOSTED_TREE_STUMPS:
		{
			classifierPtrs.classifier = new CvBoost;

			break;
		}
		case FEATURE_CLASSIFIER_KNN_FLANN:
		{
			classifierPtrs.classifier = new KnnFlannClassification;

			break;
		}
		case FEATURE_CLASSIFIER_KNN_HESS_KDTREE:
		{

			classifierPtrs.classifier = new MatcherResultsClassifier;
			break;
		}
		case FEATURE_CLASSIFIER_GEOMETRIC_CENTER:
		{
			classifierPtrs.classifier = new SparseFeaturePointObjectClassifier;

			break;
		}
		default:
		{


			break;
		}

	}



	return classifierPtrs;
}



void cleanupClassifier(FeatureClassificationConfig & featureClassificationConfig)
{

	FeatureClassifierParams params = featureClassificationConfig.classifierParams;
	FeatureClassificationPtrs classifierPtrs = featureClassificationConfig.currentClassificationPtrs;

	switch(featureClassificationConfig.currentClassificationAlgo)
	{

		case FEATURE_CLASSIFIER_LINEAR_SVM:
		{
			/*
			if(classifierPtrs.classifier != NULL)
			{
				delete classifierPtrs.classifier;
				classifierPtrs.classifier = NULL;
			}
			*/

			break;
		}
		case FEATURE_CLASSIFIER_BOOSTED_TREE_STUMPS:
		{
			/*
			if(classifierPtrs.classifier != NULL)
			{
				delete classifierPtrs.classifier;
				classifierPtrs.classifier = NULL;
			}
			*/

			break;
		}
		case FEATURE_CLASSIFIER_KNN_FLANN:
		{

			//Handled By Ptr template
/*			if(classifierPtrs.classifier != NULL)
			{
				delete classifierPtrs.classifier;
				classifierPtrs.classifier = NULL;
			}
			if(params.knnFlannClassifierParams.flannKDTreeIndexParams != NULL)
			{

				delete params.knnFlannClassifierParams.flannKDTreeIndexParams;
				params.knnFlannClassifierParams.flannKDTreeIndexParams = NULL;
			}

			if(params.knnFlannClassifierParams.flannSearchParams != NULL)
			{
				delete params.knnFlannClassifierParams.flannSearchParams;
				params.knnFlannClassifierParams.flannSearchParams = NULL;

			}

*/
			break;
		}
		case FEATURE_CLASSIFIER_KNN_HESS_KDTREE:
		{


			break;
		}
		case FEATURE_CLASSIFIER_GEOMETRIC_CENTER:
		{


			break;
		}
		default:
		{


			break;
		}

	}




}

bool trainClassifier(FeaturesClassificationAlgos & currentClassificationAlgo, FeatureClassifierParams & params, FeatureClassificationPtrs & classificationPtrs, FeatureMatcherPtrs & matcherPtrs , FeatureMatcherParams & matcherParams, Mat & trainDescriptors, Mat & descriptorClasses,vector<KeyPoint> &keypoints, Mat & trainImage ,string filename)
{

	bool result = false;


	switch(currentClassificationAlgo)
	{

		case FEATURE_CLASSIFIER_LINEAR_SVM:
		{
			CvSVM * classifierPtr = reinterpret_cast<CvSVM *>(&(*classificationPtrs.classifier));
			Mat varIdx;
			Mat sampleIdx;

			cout << "Beginning SVM auto training....";
			//result = classifierPtr->train(trainDescriptors,descriptorClasses, varIdx, sampleIdx, params.svmParams); //int k_fold = 10, CvParamGrid C_grid = get_default_grid(CvSVM::C), CvParamGrid gamma_grid = get_default_grid(CvSVM::GAMMA), CvParamGrid p_grid = get_default_grid(CvSVM::P), CvParamGrid nu_grid = get_default_grid(CvSVM::NU), CvParamGrid coef_grid = get_default_grid(CvSVM::COEF), CvParamGrid degree_grid = get_default_grid(CvSVM::DEGREE))¶
			result = classifierPtr->train_auto(trainDescriptors,descriptorClasses, varIdx, sampleIdx, params.svmParams); //int k_fold = 10, CvParamGrid C_grid = get_default_grid(CvSVM::C), CvParamGrid gamma_grid = get_default_grid(CvSVM::GAMMA), CvParamGrid p_grid = get_default_grid(CvSVM::P), CvParamGrid nu_grid = get_default_grid(CvSVM::NU), CvParamGrid coef_grid = get_default_grid(CvSVM::COEF), CvParamGrid degree_grid = get_default_grid(CvSVM::DEGREE))¶
			/*
			 *CvParamGrid Cgrid      = CvSVM::get_default_grid(CvSVM::C),
                            CvParamGrid gammaGrid  = CvSVM::get_default_grid(CvSVM::GAMMA),
                            CvParamGrid pGrid      = CvSVM::get_default_grid(CvSVM::P),
                            CvParamGrid nuGrid     = CvSVM::get_default_grid(CvSVM::NU),
                            CvParamGrid coeffGrid  = CvSVM::get_default_grid(CvSVM::COEF),
                            CvParamGrid degreeGrid = CvSVM::get_default_grid(CvSVM::DEGREE),
			 */
			cout << "SVM auto training Complete!!" << endl;
			if(filename.compare("") != 0)
			{
				cout << "Saving SVM auto training result....";
				string saveFilename = filename+"_SVM.yml";
				classifierPtr->save(saveFilename.c_str(),"svmClassifier");
				cout << "SVM auto training result saved" << endl;
			}



			break;
		}
		case FEATURE_CLASSIFIER_BOOSTED_TREE_STUMPS:
		{
			CvBoost* classifierPtr = reinterpret_cast<CvBoost *>(&(*classificationPtrs.classifier));
			Mat varIdx;
			Mat sampleIdx;
			Mat missingDataMask;
			Mat varType;

			//Define Var type

			varType.create(trainDescriptors.cols+1,1,CV_8UC1);
			varType.setTo(Scalar(CV_VAR_NUMERICAL));
			varType.at<unsigned char>(trainDescriptors.cols,0) = CV_VAR_CATEGORICAL;

			cout << "Beginning boosting auto training....";
			result = classifierPtr->train(trainDescriptors,CV_ROW_SAMPLE,descriptorClasses, varIdx, sampleIdx, varType,missingDataMask,params.boostParams); //int k_fold = 10, CvParamGrid C_grid = get_default_grid(CvSVM::C), CvParamGrid gamma_grid = get_default_grid(CvSVM::GAMMA), CvParamGrid p_grid = get_default_grid(CvSVM::P), CvParamGrid nu_grid = get_default_grid(CvSVM::NU), CvParamGrid coef_grid = get_default_grid(CvSVM::COEF), CvParamGrid degree_grid = get_default_grid(CvSVM::DEGREE))¶
			cout << " Boosting auto training Complete!!" << endl;
			if(filename.compare("") != 0)
			{
				cout << "Saving  boosting auto training result....";
				string saveFilename = filename+"_Boost.yml";
				classifierPtr->save(saveFilename.c_str(),"boostClassifier");
				cout << " Boosting auto training result saved" << endl;
			}


			break;
		}
		case FEATURE_CLASSIFIER_KNN_FLANN:
		{

			KnnFlannClassification* classifierPtr = reinterpret_cast<KnnFlannClassification*>(&(*classificationPtrs.classifier));

			Mat varIdx;
			Mat sampleIdx;
			Mat missingDataMask;
			Mat varType;

			//Define Var type

			varType.create(trainDescriptors.rows+1,1,CV_32FC1);
			varType.setTo(Scalar(CV_VAR_CATEGORICAL));

			cout << "Beginning Knn Flann training....";

			result = classifierPtr->train( trainDescriptors,descriptorClasses, sampleIdx,params.knnFlannClassifierParams.numberOfNeighbors,
					params.knnFlannClassifierParams.flannIndexParams, params.knnFlannClassifierParams.flannSearchParams,false,false);
			cout << " Knn Flann training Complete!!" << endl;
			if(filename.compare("") != 0)
			{
				cout << "Saving  Knn Flann auto training result....";
				string saveFilename = filename+"_KnnFlann.yml";
				classifierPtr->save(saveFilename.c_str(),"knnFlannClassifier");
				//cout << "Warning: Saving nearest neighbor result not allowed.  You will need to recompute every time....";
				cout << " Knn Flann training result saved" << endl;
			}




			break;
		}
		case FEATURE_CLASSIFIER_KNN_HESS_KDTREE:
		{
			MatcherResultsClassifier* classifierPtr = reinterpret_cast<MatcherResultsClassifier*>(&(*classificationPtrs.classifier));


			//Define Var type
			cout << "Beginning Knn Hess training....";

			result = classifierPtr->train( trainDescriptors,descriptorClasses, params.matcherResultsParams);
			cout << " Knn Hess training Complete!!" << endl;
			if(filename.compare("") != 0)
			{
				cout << "Saving  Knn Hess auto training result....";
				string saveFilename = filename+"_KnnHess.yml";
				classifierPtr->save(saveFilename.c_str(),"knnHessClassifier");
				//cout << "Warning: Saving nearest neighbor result not allowed.  You will need to recompute every time....";
				cout << " Knn Hess training result saved" << endl;
			}







			break;
		}
		case FEATURE_CLASSIFIER_GEOMETRIC_CENTER:
		{
			SparseFeaturePointObjectClassifier * classifierPtr = reinterpret_cast<SparseFeaturePointObjectClassifier *>(&(*classificationPtrs.classifier));

			//Define Var type
			string imageName = "singleImage";
			cout << "Beginning Geometric Center Classifier training....";
			result = classifierPtr->train( trainDescriptors,keypoints, descriptorClasses.at<float>(0,0),params.sparseGeomParams,trainImage,0,imageName, false);
			cout << " Geometric Center Classifier training Complete!!" << endl;
			if(filename.compare("") != 0)
			{
				cout << "Saving Geometric Center Classifier auto training result....";
				string saveFilename = filename+"_GeomCenter.yml";
				classifierPtr->save(saveFilename.c_str(),"GeomCenterClassifier");
				//cout << "Warning: Saving nearest neighbor result not allowed.  You will need to recompute every time....";
				cout << " Geometric Center Classifier training result saved" << endl;
			}




			break;
		}
		default:
		{


			break;
		}

	}


	return result;

}





#ifndef DRAW_TEXT_ON_IMAGE_DEFINED
#define DRAW_TEXT_ON_IMAGE_DEFINED
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


void handleMatcherTraining(Mat & trainDescriptors,Mat & trainDescriptorsClasses,FeatureMatcherPtrs matcherPtrs ,FeatureClassificationConfig & featureClassificationConfig)
{

	vector<Mat> trainDescVec;
	trainDescVec.push_back(trainDescriptors);

	switch(featureClassificationConfig.currentMatcherAlgo)
	{

		case FEATURE_MATCHER_BRUTEFORCE:
		{
			matcherPtrs.matcher->add(trainDescVec);
			matcherPtrs.matcher->train();
			break;
		}
		case FEATURE_MATCHER_KNN_FLANN:
		{
			matcherPtrs.matcher->add(trainDescVec);
			matcherPtrs.matcher->train();
			break;
		}
		case FEATURE_MATCHER_EUCL_DIST:
		{


			break;
		}
		case FEATURE_MATCHER_RADIUS_FLANN:
		{

			matcherPtrs.matcher->add(trainDescVec);
			matcherPtrs.matcher->train();
			break;
		}
		case FEATURE_MATCHER_KNN_HESS_KDTREE:
		{

			matcherPtrs.hessKnnMatcher->add(trainDescriptors);
			matcherPtrs.hessKnnMatcher->train();

			break;
		}
		default:
		{


			break;
		}

	}


	return;
}


void handleMultiThreadMatchingOperation(const Mat & descriptors, vector< vector<DMatch> > & matches,FeatureMatcherPtrs& matcherPtrs, FeatureMatcherParams& matcherParams, FeatureClassificationConfig & featureClassificationConfig)
{

	switch(featureClassificationConfig.currentMatcherAlgo)
	{

		case FEATURE_MATCHER_BRUTEFORCE:
		{
			vector< DMatch>  tmpMatches;
			matcherPtrs.matcher->match(descriptors,tmpMatches);
			matches.clear();
			matches.resize(tmpMatches.size());
			for(int i = 0; i < (int)tmpMatches.size(); i++)
			{
				matches[i].push_back(tmpMatches[i]);
			}

			break;
		}
		case FEATURE_MATCHER_KNN_FLANN:
		{
			matcherPtrs.matcher->knnMatch(descriptors,matches,matcherParams.numberOfNeighbors);
			break;
		}
		case FEATURE_MATCHER_EUCL_DIST:
		{


			break;
		}
		case FEATURE_MATCHER_RADIUS_FLANN:
		{

			matcherPtrs.matcher->radiusMatch(descriptors,matches,matcherParams.radiusMatchRadius);
			break;
		}
		case FEATURE_MATCHER_KNN_HESS_KDTREE:
		{

			matcherPtrs.hessKnnMatcher->knnMatch(descriptors,matches,matcherParams.numberOfNeighbors);

			break;
		}
		default:
		{


			break;
		}

	}


	return;




}



void handleMatchingOperation(const Mat & descriptors, vector< vector<DMatch> > & matches,FeatureMatcherPtrs& matcherPtrs, FeatureMatcherParams& matcherParams, FeatureClassificationConfig & featureClassificationConfig)
{

	switch(featureClassificationConfig.currentMatcherAlgo)
	{

		case FEATURE_MATCHER_BRUTEFORCE:
		{
			vector< DMatch>  tmpMatches;
			matcherPtrs.matcher->match(descriptors,tmpMatches);
			matches.clear();
			matches.resize(tmpMatches.size());
			for(int i = 0; i < (int)tmpMatches.size(); i++)
			{
				matches[i].push_back(tmpMatches[i]);
			}

			break;
		}
		case FEATURE_MATCHER_KNN_FLANN:
		{
			matcherPtrs.matcher->knnMatch(descriptors,matches,matcherParams.numberOfNeighbors);
			break;
		}
		case FEATURE_MATCHER_EUCL_DIST:
		{


			break;
		}
		case FEATURE_MATCHER_RADIUS_FLANN:
		{

			matcherPtrs.matcher->radiusMatch(descriptors,matches,matcherParams.radiusMatchRadius);
			break;
		}
		case FEATURE_MATCHER_KNN_HESS_KDTREE:
		{

			matcherPtrs.hessKnnMatcher->knnMatch(descriptors,matches,matcherParams.numberOfNeighbors);

			break;
		}
		default:
		{


			break;
		}

	}


	return;




}

void handleImageLoading(Mat &trainImage, Mat & queryImage,FeatureClassificationConfig & featureClassificationConfig)
{
	if(featureClassificationConfig.loadQueryImage == true)
	{
		if(featureClassificationConfig.queryImageFilename.compare("") !=0)
		{

			queryImage =  loadQueryImage(featureClassificationConfig);
		}
		else
		{
			cout << "WARNING: Invalid query image filename: NULL" << endl;


		}
	}


	if(featureClassificationConfig.loadTrainImage == true)
	{
		if(featureClassificationConfig.trainImageFilename.compare("") !=0)
		{

			trainImage =  loadTrainImage(featureClassificationConfig);
		}
		else
		{
			cout << "WARNING: Invalid train image filename: NULL" << endl;


		}
	}


}




void handleKeyPointLoading(vector<KeyPoint>& trainKeyPoints,vector<KeyPoint>& queryKeyPoints,FeatureClassificationConfig & featureClassificationConfig)
{
	if(featureClassificationConfig.loadQueryKeypoints ==true)
	{
		if(featureClassificationConfig.keyPointQueryFilename.compare("") != 0)
		{

			queryKeyPoints =  loadQueryKeypoints(featureClassificationConfig);


		}
		else
		{

			cout << "WARNING: Query Keypoints filename invalid: NULL" <<endl;
		}




	}


	if(featureClassificationConfig.loadTrainKeypoints ==true)
	{
		if(featureClassificationConfig.keyPointTrainFilename.compare("") != 0)
		{

			trainKeyPoints =  loadTrainKeypoints(featureClassificationConfig);


		}
		else
		{

			cout << "WARNING: Train Keypoints filename invalid: NULL" <<endl;
		}




	}







}
void handleDescLoading(Mat & trainDescriptors,Mat & trainDescriptorsClasses, Mat & queryDescriptors,Mat & queryDescriptorsClasses,FeatureClassificationConfig & featureClassificationConfig)
{



	if(featureClassificationConfig.useTestDescriptors)
	{
		createTestDescriptors(trainDescriptors, trainDescriptorsClasses, 10, 10,1, false);
		createTestDescriptors(queryDescriptors, queryDescriptorsClasses, 10, 10,1, true );

		Mat secondTrainDescriptors;
		Mat secondTrainDescriptorsClasses;
		Mat secondQueryDescriptors;
		Mat secondQueryDescriptorsClasses;

		createTestDescriptors(secondTrainDescriptors, secondTrainDescriptorsClasses, 10, 10,0, false);
		createTestDescriptors(secondQueryDescriptors, secondQueryDescriptorsClasses, 10, 10,0, true );

		add(secondTrainDescriptors,Scalar(1.0),secondTrainDescriptors);
		add(secondQueryDescriptors,Scalar(1.0),secondQueryDescriptors);
		trainDescriptors.push_back(secondTrainDescriptors);
		trainDescriptorsClasses.push_back(secondTrainDescriptorsClasses);
		queryDescriptors.push_back(secondQueryDescriptors);
		queryDescriptorsClasses.push_back(secondQueryDescriptorsClasses);



	}
	else
	{




		trainDescriptorsClasses = loadTrainDescriptorClasses(featureClassificationConfig);


		trainDescriptors = loadTrainDescriptors(featureClassificationConfig);


		queryDescriptorsClasses = loadQueryDescriptorClasses(featureClassificationConfig);


		queryDescriptors =  loadQueryDescriptors(featureClassificationConfig);



	}

}




bool handleClassifierTrainingOrLoading(Mat & trainDescriptors,Mat & trainDescriptorsClasses,vector<KeyPoint> &trainKeypoints, Mat &trainImage ,FeatureClassificationConfig & featureClassificationConfig)
{
	bool loadResult = false;
	bool trainResult = false;
	bool classifierReady = false;
	//Lets determine if all the information to load is here





	if(!featureClassificationConfig.trainClassifier)
	{
		//try to load

		loadResult = loadClassifier(featureClassificationConfig.currentClassificationPtrs,featureClassificationConfig);
		if(loadResult)
		{
			cout << "Classifer loading successful" << endl;
			classifierReady = true;

		}
		else
		{
			cout << "WARNING: Classifer loading failed" << endl;

		}


	}



	//if loading not possible or command given, train
	if(featureClassificationConfig.trainClassifier || !loadResult)
	{
		cout << "Training Classifier" << endl;
		trainResult = trainClassifier(featureClassificationConfig.currentClassificationAlgo, featureClassificationConfig.classifierParams, featureClassificationConfig.currentClassificationPtrs, featureClassificationConfig.currentMatcherPtrs , featureClassificationConfig.matcherParams, trainDescriptors,trainDescriptorsClasses,trainKeypoints,trainImage, featureClassificationConfig.classifierOutputFilename);
		if(trainResult)
		{
			cout << "Classifer training successful" << endl;
			classifierReady = true;
		}
		else
		{
			cout << "WARNING: Classifer training failed" << endl;

		}
	}





	return classifierReady;
}
void handleMultiThreadClassificationOperation(Mat & queryDescriptors,Mat & queryDescriptorsClasses,vector<KeyPoint> & queryKeypoints,Mat & queryImage,FeatureClassifierParams & params, FeatureClassificationPtrs & classificationPtrs, FeatureMatcherPtrs & matcherPtrs , FeatureMatcherParams & matcherParams,vector<vector <DMatch> > &matches,FeaturesClassificationAlgos & currentClassificationAlgo,FeaturesMatchingAlgos currentMatcherAlgo,struct WorkerThreadInfo *  workerInfo )
{
	switch(currentClassificationAlgo)
	{

		case FEATURE_CLASSIFIER_LINEAR_SVM:
		{
			handleClassificationOperation(queryDescriptors,queryDescriptorsClasses,queryKeypoints,queryImage,params, classificationPtrs, matcherPtrs ,matcherParams,matches,currentClassificationAlgo,currentMatcherAlgo );
			break;
		}
		case FEATURE_CLASSIFIER_BOOSTED_TREE_STUMPS:
		{
			handleClassificationOperation(queryDescriptors,queryDescriptorsClasses,queryKeypoints,queryImage,params, classificationPtrs, matcherPtrs ,matcherParams,matches,currentClassificationAlgo,currentMatcherAlgo );
			break;
		}
		case FEATURE_CLASSIFIER_KNN_FLANN:
		{

			handleClassificationOperation(queryDescriptors,queryDescriptorsClasses,queryKeypoints,queryImage,params, classificationPtrs, matcherPtrs ,matcherParams,matches,currentClassificationAlgo,currentMatcherAlgo );


			break;
		}
		case FEATURE_CLASSIFIER_KNN_HESS_KDTREE:
		{
			handleClassificationOperation(queryDescriptors,queryDescriptorsClasses,queryKeypoints,queryImage,params, classificationPtrs, matcherPtrs ,matcherParams,matches,currentClassificationAlgo,currentMatcherAlgo );
			break;
		}
		case FEATURE_CLASSIFIER_GEOMETRIC_CENTER:
		{
			//handleClassificationOperation(queryDescriptors,queryDescriptorsClasses,queryKeypoints,queryImage,params, classificationPtrs, matcherPtrs ,matcherParams,matches,currentClassificationAlgo,currentMatcherAlgo );
			vector<Point2f> estimatedRefPoints;
			vector<Mat> transformsList;
			SparseFeaturePointObjectClassifier * classifierPtr = reinterpret_cast<SparseFeaturePointObjectClassifier *>(&(*classificationPtrs.classifier));


			//cout << "Starting Geometric Center Classifier classification.....";
			float currentClass = classifierPtr->predictMultiThread(queryDescriptors,queryKeypoints,matches,transformsList, estimatedRefPoints, workerInfo);
			queryDescriptorsClasses.create(queryDescriptors.rows,1,CV_32FC1);
			queryDescriptorsClasses.setTo(Scalar(currentClass));
			if(workerInfo->coordinator)
			{
				//cout << "Geometric Center Classifier Completed: "<<estimatedRefPoints.size() <<" possible transforms found.....";
#ifdef VERBOSE_DEBUG
				cout << "Geometric Center Classifier Completed: "<<((SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierMultiThreadData *)(workerInfo->multiThreadAlgorithmData))->estimatedRefPoints->getResultsVecPtr()->size() <<" possible transforms found.....";
#endif
			}

			//cout << "This requires special calls" << endl;
			break;
		}
		default:
		{


			break;
		}

	}


}




void handleClassificationOperation(Mat & queryDescriptors,Mat & queryDescriptorsClasses,vector<KeyPoint> & queryKeypoints,Mat & queryImage,FeatureClassifierParams & params, FeatureClassificationPtrs & classificationPtrs, FeatureMatcherPtrs & matcherPtrs , FeatureMatcherParams & matcherParams,vector<vector <DMatch> > &matches,FeaturesClassificationAlgos & currentClassificationAlgo,FeaturesMatchingAlgos currentMatcherAlgo )
{


	switch(currentClassificationAlgo)
	{

		case FEATURE_CLASSIFIER_LINEAR_SVM:
		{
			CvSVM * classifierPtr = reinterpret_cast<CvSVM *>(&(*classificationPtrs.classifier));

			Mat currentSample;
			queryDescriptorsClasses.create(queryDescriptors.rows,1,CV_32FC1);
#ifdef VERBOSE_DEBUG
			cout << "Starting SVM classification.....";
#endif
			for(int currentSampleIndex =0; currentSampleIndex <queryDescriptors.rows; currentSampleIndex++)
			{

				currentSample = queryDescriptors.row(currentSampleIndex);
				queryDescriptorsClasses.at<float>(currentSampleIndex,0) = classifierPtr->predict(currentSample,false);
			//result = classifierPtr->train_auto(trainDescriptors,descriptorClasses, varIdx, sampleIdx, params.svmParams); //int k_fold = 10, CvParamGrid C_grid = get_default_grid(CvSVM::C), CvParamGrid gamma_grid = get_default_grid(CvSVM::GAMMA), CvParamGrid p_grid = get_default_grid(CvSVM::P), CvParamGrid nu_grid = get_default_grid(CvSVM::NU), CvParamGrid coef_grid = get_default_grid(CvSVM::COEF), CvParamGrid degree_grid = get_default_grid(CvSVM::DEGREE))¶

			}
#ifdef VERBOSE_DEBUG
			cout << "SVM classification Completed.....";
#endif
			break;
		}
		case FEATURE_CLASSIFIER_BOOSTED_TREE_STUMPS:
		{
			CvBoost* classifierPtr = reinterpret_cast<CvBoost *>(&(*classificationPtrs.classifier));
			Mat missingDataMask = Mat::zeros(1,queryDescriptors.cols,CV_8UC1);;
			//missingDataMask.
			Mat currentSample;
			queryDescriptorsClasses.create(queryDescriptors.rows,1,CV_32FC1);

#ifdef VERBOSE_DEBUG
			cout << "Starting Boosted tree classification.....";
#endif
			for(int currentSampleIndex =0; currentSampleIndex <queryDescriptors.rows; currentSampleIndex++)
			{

				currentSample = queryDescriptors.row(currentSampleIndex);
				queryDescriptorsClasses.at<float>(currentSampleIndex,0) = classifierPtr->predict(currentSample,missingDataMask);
			//result = classifierPtr->train_auto(trainDescriptors,descriptorClasses, varIdx, sampleIdx, params.svmParams); //int k_fold = 10, CvParamGrid C_grid = get_default_grid(CvSVM::C), CvParamGrid gamma_grid = get_default_grid(CvSVM::GAMMA), CvParamGrid p_grid = get_default_grid(CvSVM::P), CvParamGrid nu_grid = get_default_grid(CvSVM::NU), CvParamGrid coef_grid = get_default_grid(CvSVM::COEF), CvParamGrid degree_grid = get_default_grid(CvSVM::DEGREE))¶

			}
#ifdef VERBOSE_DEBUG
			cout << "Boosted tree classification Completed....";
#endif

			break;
		}
		case FEATURE_CLASSIFIER_KNN_FLANN:
		{

			KnnFlannClassification* classifierPtr = reinterpret_cast<KnnFlannClassification*>(&(*classificationPtrs.classifier));
#ifdef VERBOSE_DEBUG
			cout << "Starting KNN FLANN classification.....";
#endif
			classifierPtr->predict(queryDescriptors,queryDescriptorsClasses);
#ifdef VERBOSE_DEBUG
			cout << "KNN FLANN Completed.....";
#endif
			break;
		}
		case FEATURE_CLASSIFIER_KNN_HESS_KDTREE:
		{
			MatcherResultsClassifier * classifierPtr = reinterpret_cast<MatcherResultsClassifier*>(&(*classificationPtrs.classifier));
#ifdef VERBOSE_DEBUG
			cout << "Starting KNN Hess classification.....";
#endif
			classifierPtr->MatcherResultsClassifier::predict(queryDescriptors,queryDescriptorsClasses,matches );

#ifdef VERBOSE_DEBUG
			cout << "KNN Hess Completed.....";
#endif

			break;
		}
		case FEATURE_CLASSIFIER_GEOMETRIC_CENTER:
		{
			vector<Point2f> estimatedRefPoints;
			vector<Mat> transformsList;
			SparseFeaturePointObjectClassifier * classifierPtr = reinterpret_cast<SparseFeaturePointObjectClassifier *>(&(*classificationPtrs.classifier));
#ifdef VERBOSE_DEBUG
			cout << "Starting Geometric Center Classifier classification.....";
#endif
			float currentClass = classifierPtr->predict(queryDescriptors,queryKeypoints,matches,transformsList, estimatedRefPoints);
			queryDescriptorsClasses.create(queryDescriptors.rows,1,CV_32FC1);
			queryDescriptorsClasses.setTo(Scalar(currentClass));
#ifdef VERBOSE_DEBUG
			cout << "Geometric Center Classifier Completed: "<<estimatedRefPoints.size() <<" possible transforms found.....";
#endif


			break;
		}
		default:
		{


			break;
		}

	}




}


void singleThreadSetup(Mat & trainDescriptors,Mat & trainDescriptorsClasses, vector<KeyPoint> &trainKeypoints, Mat & trainImage,Mat & queryDescriptors,Mat & queryDescriptorsClasses,FeatureClassificationConfig & featureClassificationConfig)
{
	bool classifySetupResult = false;

	if(featureClassificationConfig.matchActive)
	{
		cout << "Matching operation active." << endl;
		featureClassificationConfig.matcherParams =  setupMatcherParams(featureClassificationConfig);
		featureClassificationConfig.currentMatcherPtrs = setupMatcher(featureClassificationConfig, featureClassificationConfig.matcherParams);

		handleMatcherTraining(trainDescriptors,trainDescriptorsClasses,featureClassificationConfig.currentMatcherPtrs ,featureClassificationConfig);
	}
	if(featureClassificationConfig.classifyActive)
	{

		cout << "Classifying operation active." << endl;
		featureClassificationConfig.classifierParams =  setupClassifierParams( featureClassificationConfig);
		featureClassificationConfig.currentClassificationPtrs = setupClassifier(featureClassificationConfig, featureClassificationConfig.classifierParams);

		classifySetupResult =  handleClassifierTrainingOrLoading(trainDescriptors,trainDescriptorsClasses,trainKeypoints,trainImage, featureClassificationConfig);
	}





}


void multiThreadMatcher(Mat & descriptors, FeatureMatcherPtrs matcherPtrs, FeatureMatcherParams matcherParams,vector <vector<DMatch> > & matches, FeatureClassificationConfig & featureClassificationConfig, struct WorkerThreadInfo * workerInfo)
{
	//vector <vector<DMatch> > matches;

	handleMultiThreadMatchingOperation(descriptors, matches,matcherPtrs, matcherParams, featureClassificationConfig);
	for(int i = 0; i <matches.size(); i++)
	{
		for(int j = 0; j < matches[i].size(); j++)
		{
			matches[i][j].queryIdx += workerInfo->descriptorStartIndex;
		}
	}


	return;

}


void multiThreadClassification(Mat & queryDescriptors,Mat & queryDescriptorsClasses,vector<KeyPoint> & queryKeypoints,Mat & queryImage,FeatureClassifierParams & params, FeatureClassificationPtrs & classificationPtrs, FeatureMatcherPtrs & matcherPtrs , FeatureMatcherParams & matcherParams,vector<vector <DMatch> > &matches,FeaturesClassificationAlgos & currentClassificationAlgo,FeaturesMatchingAlgos currentMatcherAlgo,struct WorkerThreadInfo *  workerInfo )
{
	handleMultiThreadClassificationOperation(queryDescriptors,queryDescriptorsClasses,queryKeypoints,queryImage,params, classificationPtrs, matcherPtrs ,  matcherParams,matches,currentClassificationAlgo,currentMatcherAlgo,workerInfo );
}

void singleThreadClassification(Mat & queryDescriptors,Mat & queryDescriptorsClasses,vector<KeyPoint> & queryKeypoints,Mat & queryImage,FeatureClassifierParams & params, FeatureClassificationPtrs & classificationPtrs, FeatureMatcherPtrs & matcherPtrs , FeatureMatcherParams & matcherParams,vector<vector <DMatch> > &matches,FeaturesClassificationAlgos & currentClassificationAlgo,FeaturesMatchingAlgos currentMatcherAlgo )
{

	handleClassificationOperation(queryDescriptors,queryDescriptorsClasses,queryKeypoints, queryImage,params, classificationPtrs, matcherPtrs, matcherParams,matches,currentClassificationAlgo,currentMatcherAlgo );
	return;

}

 void singleThreadMatcher(Mat & descriptors, FeatureMatcherPtrs matcherPtrs, FeatureMatcherParams matcherParams,vector <vector<DMatch> > & matches, FeatureClassificationConfig & featureClassificationConfig)
{
	//vector <vector<DMatch> > matches;

	handleMatchingOperation(descriptors, matches,matcherPtrs, matcherParams, featureClassificationConfig);



	return;

}



void singleThreadOperations_main(Mat & trainDescriptors,Mat & trainDescriptorsClasses, vector<KeyPoint>&trainKeypoints, Mat &trainImage ,Mat & queryDescriptors,Mat & queryDescriptorsClasses,vector<KeyPoint>&queryKeypoints, Mat &queryImage  ,vector <vector <DMatch > > & matches ,FeatureClassificationConfig & featureClassificationConfig)
{


	cout << "Setting up single threaded operations ..." << endl;
	singleThreadSetup(trainDescriptors,trainDescriptorsClasses,trainKeypoints,trainImage,queryDescriptors,queryDescriptorsClasses,featureClassificationConfig);

#ifdef USE_MARSS
	cout << "Switching to simulation in Feature Classification." << endl;
	ptlcall_switch_to_sim();
switch(featureClassificationConfig.currentClassificationAlgo)
{
	case FEATURE_CLASSIFIER_LINEAR_SVM:
		//ptlcall_single_enqueue("-logfile featureClassify_svm.log");
		ptlcall_single_enqueue("-stats featureClassify_svm.stats");
		//ptlcall_single_enqueue("-loglevel 0");
		break;
		case FEATURE_CLASSIFIER_BOOSTED_TREE_STUMPS:
		//ptlcall_single_enqueue("-logfile featureClassify_boost.log");
		ptlcall_single_enqueue("-stats featureClassify_boost.stats");
		//ptlcall_single_enqueue("-loglevel 0");
		break;
	case FEATURE_CLASSIFIER_KNN_FLANN:
		//ptlcall_single_enqueue("-logfile featureClassify_knn.log");
		ptlcall_single_enqueue("-stats featureClassify_knn.stats");
		//ptlcall_single_enqueue("-loglevel 0");
		break;
	case FEATURE_CLASSIFIER_KNN_HESS_KDTREE:
		//ptlcall_single_enqueue("-logfile featureClassify_hess.log");
		ptlcall_single_enqueue("-stats featureClassify_hess.stats");
		//ptlcall_single_enqueue("-loglevel 0");
		break;
	case FEATURE_CLASSIFIER_GEOMETRIC_CENTER:
		//ptlcall_single_enqueue("-logfile featureClassify_geom.log");
		ptlcall_single_enqueue("-stats featureClassify_geom.stats");
		//ptlcall_single_enqueue("-loglevel 0");
		break;
	default:
		//ptlcall_single_enqueue("-logfile featureClassify.log");
		ptlcall_single_enqueue("-stats featureClassify.stats");
		//ptlcall_single_enqueue("-loglevel 0");
		break;

};
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
	READ_TIMESTAMP_WITH_WRAPPER( fc_timingVector[0] );
#endif
#ifdef CLOCK_GETTIME_TIMING
	GET_TIME_WRAPPER(fc_timeStructVector[0]);
#endif
	for(int i = 0; i < featureClassificationConfig.numberOfIterations; i++)
	{
		if(featureClassificationConfig.matchActive)
		{
			matches.clear();
#ifdef VERBOSE_DEBUG
			cout << "Performing single threaded matching......" << endl;
#endif
			singleThreadMatcher(queryDescriptors, featureClassificationConfig.currentMatcherPtrs, featureClassificationConfig.matcherParams, matches, featureClassificationConfig);
#ifdef VERBOSE_DEBUG
			cout << "done"  << endl;
#endif
		}
		if(featureClassificationConfig.classifyActive)
		{
#ifdef VERBOSE_DEBUG
			cout << "Performing single threaded classifying......" << endl;
#endif
			singleThreadClassification(queryDescriptors,queryDescriptorsClasses,queryKeypoints, queryImage,featureClassificationConfig.classifierParams, featureClassificationConfig.currentClassificationPtrs, featureClassificationConfig.currentMatcherPtrs, featureClassificationConfig.matcherParams,matches,featureClassificationConfig.currentClassificationAlgo,featureClassificationConfig.currentMatcherAlgo );
#ifdef VERBOSE_DEBUG
			cout << "done" << endl;
#endif
		}

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
	READ_TIMESTAMP_WITH_WRAPPER( fc_timingVector[0+ fc_timingVector.size()/2] );
#endif
#ifdef CLOCK_GETTIME_TIMING
	GET_TIME_WRAPPER(fc_timeStructVector[0+ fc_timeStructVector.size()/2]);
#endif
}

void * classificationWorkerThreadFunction(void * workerThreadStruct)
{
	struct WorkerThreadInfo *  threadParams = reinterpret_cast<struct WorkerThreadInfo * >(workerThreadStruct);
	Thread * myThreadInfo =  threadParams->myThread;
	int numberOfDescriptors = threadParams->descriptorsToProcess.rows;
	Mat resultsMat;
#ifdef VERBOSE_DEBUG
	cout << "Current Worker Thread is "<< myThreadInfo->getThreadLogicalId()  << " which is ("<< threadParams->verticalThreadId << "," << threadParams->horizontalThreadId << ")" << endl;
#endif
	//Setup classification ConFig
	if(threadParams->myFeatureClassificationInfo == NULL)
	{



	}
	else
	{
#ifdef VERBOSE_DEBUG
		cout << "Using presetup classification config" << endl;
#endif
	}





	//My debug counter
	int waits = 0;

	//A debug wait
//	myThreadInfo->waitForWakeup();


	//Barrier to make sure work wait is good
//	myThreadInfo->waitAtBarrier();

#ifdef TSC_TIMING
	READ_TIMESTAMP_WITH_WRAPPER( fc_timingVector[threadParams->myThread->getThreadLogicalId()] );
#endif
#ifdef CLOCK_GETTIME_TIMING
	GET_TIME_WRAPPER(fc_timeStructVector[threadParams->myThread->getThreadLogicalId()]);
#endif

	//Wait for work to be distributed, main thread will tell me when my data is ready
	myThreadInfo->waitForWakeup();



	//Setup for work
	vector<vector <DMatch> > matches;
	Mat queryImage;


	//perform matching or classifying until told to stop
	while(!threadParams->done)
	{
		//cout <<" Worker " << myThreadInfo->getThreadLogicalId() << " been Waiting: "<<++waits << " waits" << endl;
		numberOfDescriptors = threadParams->descriptorsToProcess.rows;
		resultsMat.create(numberOfDescriptors,1,CV_32FC1);


		if(threadParams->myFeatureClassificationInfo->matchActive)
		{
			matches.clear();
			multiThreadMatcher(threadParams->descriptorsToProcess, threadParams->myFeatureClassificationInfo->currentMatcherPtrs, threadParams->myFeatureClassificationInfo->matcherParams, matches, *(threadParams->myFeatureClassificationInfo),threadParams);
			if(!threadParams->myFeatureClassificationInfo->classifyActive)
			{
				threadParams->multiThreadMatchResults->copyToReultsVec(matches,threadParams->descriptorStartIndex);
			}
		}
		if(threadParams->myFeatureClassificationInfo->classifyActive)
		{

			multiThreadClassification(threadParams->descriptorsToProcess,resultsMat,threadParams->keyPointsToProcess, queryImage,threadParams->myFeatureClassificationInfo->classifierParams, threadParams->myFeatureClassificationInfo->currentClassificationPtrs, threadParams->myFeatureClassificationInfo->currentMatcherPtrs, threadParams->myFeatureClassificationInfo->matcherParams,matches,threadParams->myFeatureClassificationInfo->currentClassificationAlgo,threadParams->myFeatureClassificationInfo->currentMatcherAlgo,threadParams );

			//Copy my answers
			threadParams->multiThreadResults->copyToReultsMat(resultsMat,threadParams->descriptorStartIndex);

		}





		//Wait after iteration of work
		myThreadInfo->waitAtBarrier();


		//Wait for work to be distributed, main thread will tell me when my data is ready
		myThreadInfo->waitForWakeup();


	}

#ifdef TSC_TIMING
	READ_TIMESTAMP_WITH_WRAPPER( fc_timingVector[threadParams->myThread->getThreadLogicalId() + fc_timingVector.size()/2] );
#endif
#ifdef CLOCK_GETTIME_TIMING
	GET_TIME_WRAPPER(fc_timeStructVector[threadParams->myThread->getThreadLogicalId()+ fc_timeStructVector.size()/2]);
#endif
	return NULL;
}

void * classificationCoordinatorThreadFunction(void * workerThreadStruct)
{

	struct WorkerThreadInfo *  threadParams = reinterpret_cast<struct WorkerThreadInfo * >(workerThreadStruct);
	Thread * myThreadInfo =  threadParams->myThread;
	int numberOfDescriptors = threadParams->descriptorsToProcess.rows;
	int numberOfThreads = threadParams->featureClassificationInfo->numberOfVerticalProcs * threadParams->featureClassificationInfo->numberOfHorizontalProcs;
	int numberOfIterations = threadParams->featureClassificationInfo->numberOfIterations;
	Mat resultsMat;




#ifdef VERBOSE_DEBUG
	cout << "Current Coordinator Thread is "<< myThreadInfo->getThreadLogicalId()  << " which is ("<< threadParams->verticalThreadId << "," << threadParams->horizontalThreadId << ")" << endl;
#endif
	resultsMat.create(numberOfDescriptors,1,CV_32FC1);



	//Setup classification ConFig
	if(threadParams->myFeatureClassificationInfo == NULL)
	{



	}
	else
	{
#ifdef VERBOSE_DEBUG
		cout << "Using presetup classification config" << endl;
#endif
	}





	//A debug wait
//	myThreadInfo->waitForWakeup();



	//Barrier to make sure fist distro works
//	myThreadInfo->waitAtBarrier();


#ifdef USE_MARSS
	cout << "Switching to simulation in Feature CLassification." << endl;
	ptlcall_switch_to_sim();
	switch(threadParams->featureClassificationInfo->currentClassificationAlgo)
	{
		case FEATURE_CLASSIFIER_LINEAR_SVM:
			//ptlcall_single_enqueue("-logfile featureClassify_svm.log");
			ptlcall_single_enqueue("-stats featureClassify_svm.stats");
			//ptlcall_single_enqueue("-loglevel 0");
			break;
		case FEATURE_CLASSIFIER_BOOSTED_TREE_STUMPS:
			//ptlcall_single_enqueue("-logfile featureClassify_boost.log");
			ptlcall_single_enqueue("-stats featureClassify_boost.stats");
			//ptlcall_single_enqueue("-loglevel 0");
			break;
		case FEATURE_CLASSIFIER_KNN_FLANN:
			//ptlcall_single_enqueue("-logfile featureClassify_knn.log");
			ptlcall_single_enqueue("-stats featureClassify_knn.stats");
			//ptlcall_single_enqueue("-loglevel 0");
			break;
		case FEATURE_CLASSIFIER_KNN_HESS_KDTREE:
			//ptlcall_single_enqueue("-logfile featureClassify_hess.log");
			ptlcall_single_enqueue("-stats featureClassify_hess.stats");
			//ptlcall_single_enqueue("-loglevel 0");
			break;
		case FEATURE_CLASSIFIER_GEOMETRIC_CENTER:
			//ptlcall_single_enqueue("-logfile featureClassify_geom.log");
			ptlcall_single_enqueue("-stats featureClassify_geom.stats");
			//ptlcall_single_enqueue("-loglevel 0");
			break;
		default:
			//ptlcall_single_enqueue("-logfile featureClassify.log");
			ptlcall_single_enqueue("-stats featureClassify.stats");
			//ptlcall_single_enqueue("-loglevel 0");
			break;

	};



//	ptlcall_single_enqueue("-logfile featureClassify.log");
//	ptlcall_single_enqueue("-stats featureClassify.stats");
//	ptlcall_single_enqueue("-loglevel 0");
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
	READ_TIMESTAMP_WITH_WRAPPER( fc_timingVector[threadParams->myThread->getThreadLogicalId()] );
#endif
#ifdef CLOCK_GETTIME_TIMING
	GET_TIME_WRAPPER(fc_timeStructVector[threadParams->myThread->getThreadLogicalId()]);
#endif

	//Give everyone their work and send them off
	coordinatorThreadDistributeQueryKeypointsAndDescriptors(*(threadParams->myFeatureClassificationInfo),*(threadParams->allQueryDescriptors),*(threadParams->allQueryKeypoints),threadParams);


	//Setup for work
	int currentIteration = 0;
	bool done = false;
	vector<vector <DMatch> > matches;
	Mat queryImage;




	//Do work until done
	while(!done)
	{

		numberOfDescriptors = threadParams->descriptorsToProcess.rows;
		resultsMat.create(numberOfDescriptors,1,CV_32FC1);
		//cout <<" Coordinator " << myThreadInfo->getThreadLogicalId() << " been Waiting: "<<++waits << " waits" << endl;

		if(threadParams->myFeatureClassificationInfo->matchActive)
		{
			matches.clear();
#ifdef VERBOSE_DEBUG
			cout << "Performing multi threaded matching......"<< endl;
#endif
			multiThreadMatcher(threadParams->descriptorsToProcess, threadParams->myFeatureClassificationInfo->currentMatcherPtrs, threadParams->myFeatureClassificationInfo->matcherParams, matches, *(threadParams->myFeatureClassificationInfo),threadParams);

			if(!threadParams->myFeatureClassificationInfo->classifyActive)
			{
				threadParams->multiThreadMatchResults->copyToReultsVec(matches,threadParams->descriptorStartIndex);
			}

#ifdef VERBOSE_DEBUG
			cout << "coordinator done"  << endl;
#endif
		}
		if(threadParams->myFeatureClassificationInfo->classifyActive)
		{
#ifdef VERBOSE_DEBUG
			cout << "Performing multi threaded Classify......"<< endl;
#endif
			multiThreadClassification(threadParams->descriptorsToProcess,resultsMat,threadParams->keyPointsToProcess, queryImage,threadParams->myFeatureClassificationInfo->classifierParams, threadParams->myFeatureClassificationInfo->currentClassificationPtrs, threadParams->myFeatureClassificationInfo->currentMatcherPtrs, threadParams->myFeatureClassificationInfo->matcherParams,matches,threadParams->myFeatureClassificationInfo->currentClassificationAlgo,threadParams->myFeatureClassificationInfo->currentMatcherAlgo,threadParams );
#ifdef VERBOSE_DEBUG
			cout << "coordinator done" << endl;
#endif
			threadParams->multiThreadResults->copyToReultsMat(resultsMat,threadParams->descriptorStartIndex);
		}



		//Wait for everyone to complete this iteration
		myThreadInfo->waitAtBarrier();

		//Increment the iteration
		currentIteration++;

		//If we have more work get it, otherwise time to finish so tell everyone
		if(currentIteration >=numberOfIterations)
		{
			done = true;
			coordinatorSetAllDoneFlag(threadParams, numberOfThreads);
			coordinatorWakeUpOtherThreads(threadParams,numberOfThreads);
		}
		else
		{
			coordinatorThreadDistributeQueryKeypointsAndDescriptors(*(threadParams->myFeatureClassificationInfo),*(threadParams->allQueryDescriptors),*(threadParams->allQueryKeypoints),threadParams);
		}
	}

	//cout << "Classification Complete." << endl;
//	myThreadInfo->waitAtBarrier();
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
	READ_TIMESTAMP_WITH_WRAPPER( fc_timingVector[threadParams->myThread->getThreadLogicalId() + fc_timingVector.size()/2] );
#endif
#ifdef CLOCK_GETTIME_TIMING
	GET_TIME_WRAPPER(fc_timeStructVector[threadParams->myThread->getThreadLogicalId()+ fc_timeStructVector.size()/2]);
#endif
	return NULL;
}

void getFeatureClassificationInfoForThread(FeatureClassificationConfig * &threadFeatureClassificationConfig,FeatureClassificationConfig &featureClassificationConfig,struct WorkerThreadInfo * workerInfo)
{
	if(featureClassificationConfig.singleCopyOfClassificationStruct)
	{
		threadFeatureClassificationConfig = &featureClassificationConfig;

	}
	else if(featureClassificationConfig.partialCopyOfClassificationStruct)
	{
		//Partial






	}
	else
	{
		//Individual pass the main and let them duplicate within or I could make it here.  Still up for thought
		threadFeatureClassificationConfig = &featureClassificationConfig;


	}

	return;
}



vector<KeyPoint> getQueryKeyPointsForThread(FeatureClassificationConfig &featureClassificationConfig,int verticalThreads, int horizontalThreads,int & keyPointStartIndex,vector<KeyPoint>& queryKeypoints,struct WorkerThreadInfo * workerInfo)
{

	int numberOfKeyPoints = queryKeypoints.size();
	int numberOfThreads = verticalThreads * horizontalThreads;
	int keyPointCountPerThread = numberOfKeyPoints/numberOfThreads;

	int currentThreadKeyPointCount =keyPointCountPerThread;
	int myThreadId = workerInfo->myThread->getThreadLogicalId();
	int startKeyPointIndex = myThreadId *currentThreadKeyPointCount;
	vector<KeyPoint> myKeyPoints;


	if(myThreadId == numberOfThreads-1)
	{
		currentThreadKeyPointCount = numberOfKeyPoints - startKeyPointIndex;


	}

	keyPointStartIndex = startKeyPointIndex;

	for(int i = startKeyPointIndex; i < currentThreadKeyPointCount; i++)
	{
		myKeyPoints.push_back(queryKeypoints[i]);





	}


	return myKeyPoints;


}
Mat getQueryDescriptorsForThread(FeatureClassificationConfig& featureClassificationConfig,int verticalThreads, int horizontalThreads,int & descStartIndex,Mat& queryDescriptors,struct WorkerThreadInfo * workerInfo)
{

	int numberOfDescriptors = queryDescriptors.rows;
	int numberOfThreads = verticalThreads * horizontalThreads;
	int descriptorCountPerThread = numberOfDescriptors/numberOfThreads;

	int currentThreadDescriptorCount =descriptorCountPerThread;
	int myThreadId = workerInfo->myThread->getThreadLogicalId();
	int startDescriptorIndex = myThreadId *currentThreadDescriptorCount;
	//int decStartIndex;
	Mat myDescriptors;


	if(myThreadId == numberOfThreads-1)
	{
		currentThreadDescriptorCount = numberOfDescriptors - startDescriptorIndex;


	}
	descStartIndex = startDescriptorIndex;
	Range descRange(startDescriptorIndex,startDescriptorIndex+currentThreadDescriptorCount);
	myDescriptors = queryDescriptors.rowRange(descRange).clone();
	//workerinfo->myDescriptors = myDescriptors;


	return myDescriptors;
}

void coordinatorWakeUpOtherThreads(struct WorkerThreadInfo * workerInfo,int numberOfThreads)
{
	if(workerInfo->coordinator)
	{
		int myThreadId = workerInfo->myThread->getThreadLogicalId();
		for(int i = 0; i< numberOfThreads; i++)
		{
			Thread * workerThread = workerInfo->threadManager->getThread(i);
			int threadId = workerThread->getThreadLogicalId();
			if(threadId != myThreadId)
			{


				workerThread->wakeup();


			}

		}

	}
	else
	{

		cout << "ERROR: Worker Thread trying tell threads wakeup" << endl;

	}

}

void coordinatorSetAllDoneFlag(struct WorkerThreadInfo * workerInfo, int numberOfThreads)
{

	if(workerInfo->coordinator)
	{

		for(int i = 0; i< numberOfThreads; i++)
		{

			Thread * workerThread = workerInfo->threadManager->getThread(i);

			struct WorkerThreadInfo * workerThreadWorkerInfo;
			if(workerInfo->standAlone)
			{
			//This line uses the thread manager to get access to the other threads worker info
				workerThreadWorkerInfo = (struct WorkerThreadInfo *)((struct GeneralWorkerThreadData *)(workerThread->getThreadParam()))->featureClassificationData;
			}
			else
			{
			//This line uses the thread manager to get access to the other threads worker info
				workerThreadWorkerInfo = (struct WorkerThreadInfo *)(workerThread->getThreadParam());
			}
			workerThreadWorkerInfo->done = true;

		}
	}
	else
	{

		cout << "ERROR: Worker Thread trying tell threads done" << endl;

	}

}


void coordinatorThreadDistributeQueryKeypointsAndDescriptors(FeatureClassificationConfig &featureClassificationConfig,Mat & queryDescriptors,vector<KeyPoint>& queryKeypoints,struct WorkerThreadInfo * workerInfo)
{
	if(workerInfo->coordinator)
	{
		int numberOfThreads = featureClassificationConfig.numberOfVerticalProcs * featureClassificationConfig.numberOfHorizontalProcs;

		if(workerInfo->multiThreadAlgorithmData != NULL)
		{
			//Clear the data
			workerInfo->multiThreadAlgorithmData->clear();
		}


		if(featureClassificationConfig.classifyActive)
		{
			workerInfo->multiThreadResults->createResultsMat(queryDescriptors.rows,1);
		}
		if(featureClassificationConfig.matchActive)
		{
			if(!featureClassificationConfig.classifyActive)
			{

				workerInfo->multiThreadMatchResults->createResultsVec(queryDescriptors.rows);


			}
		}


		//Distribute the work
		for(int i = 0; i< numberOfThreads; i++)
		{

			Thread * workerThread = workerInfo->threadManager->getThread(i);

			struct WorkerThreadInfo * workerThreadWorkerInfo;
			if(workerInfo->standAlone)
			{
			//This line uses the thread manager to get access to the other threads worker info
				workerThreadWorkerInfo = (struct WorkerThreadInfo *)((struct GeneralWorkerThreadData *)(workerThread->getThreadParam()))->featureClassificationData;
			}
			else
			{
			//This line uses the thread manager to get access to the other threads worker info
				workerThreadWorkerInfo = (struct WorkerThreadInfo *)(workerThread->getThreadParam());
			}


			if(workerThreadWorkerInfo->multiThreadAlgorithmData != NULL)
			{
				//Clear the data
				workerThreadWorkerInfo->multiThreadAlgorithmData->clearNonShared();
			}



			//Now we give him his data
			workerThreadWorkerInfo ->descriptorsToProcess =  getQueryDescriptorsForThread(featureClassificationConfig,featureClassificationConfig.numberOfVerticalProcs, featureClassificationConfig.numberOfHorizontalProcs,workerThreadWorkerInfo->descriptorStartIndex,queryDescriptors,workerThreadWorkerInfo);


			int tmpStartIndex;

			//Get the Keypoints to process
			workerThreadWorkerInfo->keyPointsToProcess = getQueryKeyPointsForThread(featureClassificationConfig,featureClassificationConfig.numberOfVerticalProcs, featureClassificationConfig.numberOfHorizontalProcs,tmpStartIndex,queryKeypoints,workerThreadWorkerInfo);

			//Reset his done flag
			workerThreadWorkerInfo->done = false;

			if(workerThreadWorkerInfo->myThread->getThreadLogicalId() != workerInfo->myThread->getThreadLogicalId())
			{
				workerThread->wakeup();


			}
			else
			{
#ifdef VERBOSE_DEBUG
				cout << "Coordinator Thread work.  No need to stop.";
#endif
			}


		}

	}
	else
	{

		cout << "ERROR:  Non coordinator trying to hand out work" << endl;

	}



}








MultiThreadAlgorithmData * setupClassificationMultiThreadedData(FeatureClassificationConfig featureClassificationConfig)
{
	MultiThreadAlgorithmData * returnVal = NULL;
	switch(featureClassificationConfig.currentClassificationAlgo)
	{

		case FEATURE_CLASSIFIER_LINEAR_SVM:
		{

			break;
		}
		case FEATURE_CLASSIFIER_BOOSTED_TREE_STUMPS:
		{

			break;
		}
		case FEATURE_CLASSIFIER_KNN_FLANN:
		{

			break;
		}
		case FEATURE_CLASSIFIER_KNN_HESS_KDTREE:
		{


			break;
		}
		case FEATURE_CLASSIFIER_GEOMETRIC_CENTER:
		{

			SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierMultiThreadData * multiThreadData = new SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierMultiThreadData;
			multiThreadData->estimatedRefPoints = new MultiThreadedPointVectorsResult;
			multiThreadData->filteredMatchesStorage = new MultiThreadedFilteredMatchResult;
			multiThreadData->houghBinVec = new SparseFeaturePointObjectClassifier::HoughBinVecMultiThread;
			multiThreadData->transformsList = new MultiThreadedMatVectorsResult;
			multiThreadData->vectorsToRef = new SparseFeaturePointObjectClassifier::MultiThreadedVectorsResult;
			multiThreadData->filteredMatchedKeypoints = new MultiThreadedQueryTrainKeypointsResult;

			returnVal = multiThreadData;
			break;
		}
		default:
		{


			break;
		}

	}



	return returnVal;



}

MultiThreadAlgorithmData * copyClassificationMultiThreadedData(FeatureClassificationConfig featureClassificationConfig,MultiThreadAlgorithmData * baseData)
{
	MultiThreadAlgorithmData * returnVal = NULL;
	switch(featureClassificationConfig.currentClassificationAlgo)
	{

		case FEATURE_CLASSIFIER_LINEAR_SVM:
		{

			break;
		}
		case FEATURE_CLASSIFIER_BOOSTED_TREE_STUMPS:
		{

			break;
		}
		case FEATURE_CLASSIFIER_KNN_FLANN:
		{

			break;
		}
		case FEATURE_CLASSIFIER_KNN_HESS_KDTREE:
		{


			break;
		}
		case FEATURE_CLASSIFIER_GEOMETRIC_CENTER:
		{

			SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierMultiThreadData * baseMultiThreadData = reinterpret_cast<SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierMultiThreadData *>(baseData);
			SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierMultiThreadData * multiThreadData = new SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierMultiThreadData;



			*multiThreadData = *baseMultiThreadData;

			returnVal = multiThreadData;
			break;
		}
		default:
		{


			break;
		}

	}



	return returnVal;



}




void setupClassificationThreads(FeatureClassificationConfig & featureClassificationConfig,ThreadManager * &classificationThreadManager,Mat& queryDescriptors, vector<KeyPoint>& queryKeypoints,Mat &trainDescriptors, vector<KeyPoint> &trainKeypoints, vector<struct WorkerThreadInfo> & workingThreads,MultiThreadedMatResult * &multiThreadResultPtr,MultiThreadedMatchResult * &multiThreadMatchResultPtr ,int verticalThreads, int horizontalThreads)
{



	int numberOfThreads = verticalThreads*horizontalThreads;


	cout << "Creating Thread Manager and basic thread structs" << endl;
	classificationThreadManager = new ThreadManager(numberOfThreads);

	cout << "Creating woking threads structs" << endl;
	workingThreads.clear();
	workingThreads.resize(numberOfThreads);


	multiThreadResultPtr = new MultiThreadedMatResult(64);
	multiThreadResultPtr -> createResultsMat(queryDescriptors.rows,1);
	multiThreadMatchResultPtr = new 	MultiThreadedMatchResult(64);
	multiThreadMatchResultPtr->createResultsVec(queryDescriptors.rows);

	MultiThreadAlgorithmData * multiThreadAlgorithmDataBasePtr = setupClassificationMultiThreadedData(featureClassificationConfig);

	cout << "Filling in Data structs and creating worker threads" << endl;

	for(int i = 0; i<numberOfThreads;i++)
	{
		Mat myDescriptors;
		Thread * currentThreadPtr = (classificationThreadManager->getThread(i));
		struct WorkerThreadInfo * currentWorkerThreadInfo = &workingThreads[i];
		currentWorkerThreadInfo->myThread =currentThreadPtr;
		currentWorkerThreadInfo->threadManager = classificationThreadManager;
		currentWorkerThreadInfo->verticalThreadId = i/horizontalThreads;
		currentWorkerThreadInfo->horizontalThreadId = i%horizontalThreads;
		currentWorkerThreadInfo->coordinator = (i==0? true: false);
		currentWorkerThreadInfo->multiThreadResults = multiThreadResultPtr;
		currentWorkerThreadInfo->multiThreadMatchResults = multiThreadMatchResultPtr;
		currentWorkerThreadInfo->multiThreadAlgorithmData = copyClassificationMultiThreadedData(featureClassificationConfig,multiThreadAlgorithmDataBasePtr);
		currentWorkerThreadInfo->standAlone = false;
		currentWorkerThreadInfo->done= false;

		currentThreadPtr->setThreadLogicalId(i);


		//Get the descriptors
		myDescriptors = getQueryDescriptorsForThread(featureClassificationConfig,verticalThreads, horizontalThreads,currentWorkerThreadInfo->descriptorStartIndex,queryDescriptors,currentWorkerThreadInfo);
		currentWorkerThreadInfo->descriptorsToProcess = myDescriptors;

		//Get the config Data
		getFeatureClassificationInfoForThread(currentWorkerThreadInfo->myFeatureClassificationInfo,featureClassificationConfig,currentWorkerThreadInfo);
		currentWorkerThreadInfo->featureClassificationInfo = &featureClassificationConfig;

		int tmpStartIndex;
		//Get the Keypoints to process
		currentWorkerThreadInfo->keyPointsToProcess = getQueryKeyPointsForThread(featureClassificationConfig,verticalThreads, horizontalThreads,tmpStartIndex,queryKeypoints,currentWorkerThreadInfo);

		currentWorkerThreadInfo->allQueryDescriptors = &queryDescriptors;
		currentWorkerThreadInfo->allQueryKeypoints = &queryKeypoints;

		currentThreadPtr->setThreadParam(currentWorkerThreadInfo);
		currentThreadPtr->setThreadFunction((currentWorkerThreadInfo->coordinator? classificationCoordinatorThreadFunction: classificationWorkerThreadFunction));
		classificationThreadManager->setThreadBarrier(i);


		//cout << "Thread Param "<< currentThreadPtr << endl;

//		int errcode = pthread_create(currentThreadPtr->getThreadIdPtr(),NULL,currentThreadPtr->getThreadFunction(),currentWorkerThreadInfo);
//		if(errcode != 0)
//		{
//			cout << "Error creating thread: " << strerror(errcode) << endl;
//		}
		//cout << "Pthread Create Val: " << strerror(errcode) << endl;




	}


	//Done so we can have the work waiting for them once they are spawned
	for(int i = 0; i<numberOfThreads;i++)
	{
		Thread * currentThreadPtr = (classificationThreadManager->getThread(i));
		struct WorkerThreadInfo * currentWorkerThreadInfo = &workingThreads[i];

		int errcode = pthread_create(currentThreadPtr->getThreadIdPtr(),NULL,currentThreadPtr->getThreadFunction(),currentWorkerThreadInfo);
		if(errcode != 0)
		{
			cout << "Error creating thread: " << strerror(errcode) << endl;
		}
	}


}


void multiThreadedSetup(Mat & trainDescriptors,Mat & trainDescriptorsClasses, vector<KeyPoint> &trainKeypoints, Mat & trainImage,Mat & queryDescriptors,Mat & queryDescriptorsClasses,vector<KeyPoint> &queryKeypoints,FeatureClassificationConfig & featureClassificationConfig, ThreadManager * &classificationThreadManager,vector<struct WorkerThreadInfo> & workingThreads,MultiThreadedMatResult * &multiThreadResultPtr,MultiThreadedMatchResult * &multiThreadMatchResultPtr)
{
	bool classifySetupResult = false;



	if(featureClassificationConfig.singleCopyOfClassificationStruct || featureClassificationConfig.partialCopyOfClassificationStruct)
	{
		if(featureClassificationConfig.matchActive)
		{
			cout << "Matching operation active." << endl;
			featureClassificationConfig.matcherParams =  setupMatcherParams(featureClassificationConfig);
			featureClassificationConfig.currentMatcherPtrs = setupMatcher(featureClassificationConfig, featureClassificationConfig.matcherParams);

			handleMatcherTraining(trainDescriptors,trainDescriptorsClasses,featureClassificationConfig.currentMatcherPtrs ,featureClassificationConfig);
		}
		if(featureClassificationConfig.classifyActive)
		{

			cout << "Classifying operation active." << endl;
			featureClassificationConfig.classifierParams =  setupClassifierParams( featureClassificationConfig);
			featureClassificationConfig.currentClassificationPtrs = setupClassifier(featureClassificationConfig, featureClassificationConfig.classifierParams);

			classifySetupResult =  handleClassifierTrainingOrLoading(trainDescriptors,trainDescriptorsClasses,trainKeypoints,trainImage, featureClassificationConfig);
		}
	}

	setupClassificationThreads(featureClassificationConfig,classificationThreadManager, queryDescriptors,  queryKeypoints,trainDescriptors, trainKeypoints, workingThreads, multiThreadResultPtr,multiThreadMatchResultPtr,featureClassificationConfig.numberOfVerticalProcs, featureClassificationConfig.numberOfHorizontalProcs);
//	setupClassificationThreads(classificationThreadManager,  workingThreads, featureClassificationConfig.numberOfVerticalProcs, featureClassificationConfig.numberOfHorizontalProcs);

}



void multiThreadedOperations_main(Mat & trainDescriptors,Mat & trainDescriptorsClasses, vector<KeyPoint>&trainKeypoints, Mat &trainImage ,Mat & queryDescriptors,Mat & queryDescriptorsClasses,vector<KeyPoint>&queryKeypoints, Mat &queryImage  ,vector <vector <DMatch > > & matches ,FeatureClassificationConfig & featureClassificationConfig)
{

	ThreadManager * classificationThreadManager;
	vector<struct WorkerThreadInfo> workingThreads;
	MultiThreadedMatResult * multiThreadResultPtr;
	MultiThreadedMatchResult * multiThreadMatchResultPtr;

	cout << "Setting up multithreaded operations ..." << endl;
	multiThreadedSetup(trainDescriptors,trainDescriptorsClasses,trainKeypoints,trainImage,queryDescriptors,queryDescriptorsClasses,queryKeypoints,featureClassificationConfig,classificationThreadManager,workingThreads, multiThreadResultPtr,multiThreadMatchResultPtr);

//Test Code
//	waitKey(0);
	//sleep(1);
//	for(int i=0 ; i<classificationThreadManager->getNumberOfThreads(); i++)
//	{
//		classificationThreadManager->wakeupThread(i);
//	}



#ifdef VERBOSE_DEBUG
	cout << "Waiting for Threads." << endl;
#endif
	for(int i=0 ; i<classificationThreadManager->getNumberOfThreads() ; i++)
	{
		pthread_join(classificationThreadManager->getThread(i)->getThreadId(), NULL);
	}
	cout << "MultiThread Iteration complete.  Copying results"<< endl;//<<multiThreadResultPtr->getResultsMat() << endl;
	queryDescriptorsClasses = multiThreadResultPtr->getResultsMat();
	matches = multiThreadMatchResultPtr->getResultsVec();
	/*
	if(featureClassificationConfig.matchActive)
	{

		cout << "Performing single threaded matching......";
		singleThreadMatcher(queryDescriptors, featureClassificationConfig.currentMatcherPtrs, featureClassificationConfig.matcherParams, matches, featureClassificationConfig);
		cout << "done"  << endl;

	}
	if(featureClassificationConfig.classifyActive)
	{
		cout << "Performing single threaded matching......";
		singleThreadClassification(queryDescriptors,queryDescriptorsClasses,queryKeypoints, queryImage,featureClassificationConfig.classifierParams, featureClassificationConfig.currentClassificationPtrs, featureClassificationConfig.currentMatcherPtrs, featureClassificationConfig.matcherParams,matches,featureClassificationConfig.currentClassificationAlgo,featureClassificationConfig.currentMatcherAlgo );
		cout << "done" << endl;
	}
	cout << "Threads are done." << endl;

*/

}


bool loadFeatureClassificationConfigFile(vector<string> &commandArgs, string filename)
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

void parseClassificationConfigCommandVector(FeatureClassificationConfig & featureClassificationConfig, vector<string> & commandStringVector)
{



	vector<string>::iterator it_start = commandStringVector.begin();
	vector<string>::iterator it_end = commandStringVector.end();
	vector<string>::iterator it_current;

	stringstream stringBuffer;

	//Set the defaults
	featureClassificationConfig.currentDescriptorAlgo = FEATURE_DESC_SIFT;
	featureClassificationConfig.currentMatcherAlgo = FEATURE_MATCHER_KNN_FLANN;
	featureClassificationConfig.currentClassificationAlgo = FEATURE_CLASSIFIER_GEOMETRIC_CENTER;


	featureClassificationConfig.matchActive = false;
	featureClassificationConfig.classifyActive = false;
	featureClassificationConfig.useTestDescriptors = false;
	featureClassificationConfig.singleThreaded = false;
	featureClassificationConfig.trainClassifier = false;


	featureClassificationConfig.numberOfVerticalProcs = 1;
	featureClassificationConfig.numberOfHorizontalProcs = 1;
	featureClassificationConfig.singleCopyOfClassificationStruct = true;
	featureClassificationConfig.partialCopyOfClassificationStruct = false;
	featureClassificationConfig.numberOfIterations = CLASSIFICATION_DEFAULT_NUMBER_OF_ITERATIONS;

	//-desc sift -match knn_flann -classify linear_svm -queryDescFile test.yml -trainDescFile test.yml -loadClassificationStruct test.yml -loadMatchStruct test.yml
	for(it_current=it_start; it_current!=it_end; ++it_current)
	{


		if(!it_current->compare("-desc") || !it_current->compare("-Desc"))
		{
			if(!(it_current+1)->compare("sift"))
			{

				featureClassificationConfig.currentDescriptorAlgo = FEATURE_DESC_SIFT;

			}
			if(!(it_current+1)->compare("surf"))
			{

				featureClassificationConfig.currentDescriptorAlgo = FEATURE_DESC_SURF;
			}
			if(!(it_current+1)->compare("fast"))
			{

				featureClassificationConfig.currentDescriptorAlgo = FEATURE_DESC_FAST;
			}
			if(!(it_current+1)->compare("hog"))
			{

				featureClassificationConfig.currentDescriptorAlgo = FEATURE_DESC_HoG;
			}
			if(!(it_current+1)->compare("brief"))
			{

				featureClassificationConfig.currentDescriptorAlgo = FEATURE_DESC_BRIEF;
			}

		}



		if(!it_current->compare("-match") || !it_current->compare("-Match"))
		{
			featureClassificationConfig.matchActive = true;
			if(!(it_current+1)->compare("brute_force"))
			{

				featureClassificationConfig.currentMatcherAlgo = FEATURE_MATCHER_BRUTEFORCE;

			}
			if(!(it_current+1)->compare("knn_flann"))
			{

				featureClassificationConfig.currentMatcherAlgo = FEATURE_MATCHER_KNN_FLANN;

			}

			if(!(it_current+1)->compare("eucl_dist"))
			{

				featureClassificationConfig.currentMatcherAlgo = FEATURE_MATCHER_EUCL_DIST;

			}

			if(!(it_current+1)->compare("radius_flann"))
			{

				featureClassificationConfig.currentMatcherAlgo = FEATURE_MATCHER_RADIUS_FLANN;

			}

			if(!(it_current+1)->compare("knn_hess"))
			{

				featureClassificationConfig.currentMatcherAlgo = FEATURE_MATCHER_KNN_HESS_KDTREE;

			}


		}



		if(!it_current->compare("-class") || !it_current->compare("-Class"))
		{
			featureClassificationConfig.classifyActive = true;
			if(!(it_current+1)->compare("linear_svm"))
			{

				featureClassificationConfig.currentClassificationAlgo = FEATURE_CLASSIFIER_LINEAR_SVM;

			}
			if(!(it_current+1)->compare("boosted_stumps"))
			{

				featureClassificationConfig.currentClassificationAlgo = FEATURE_CLASSIFIER_BOOSTED_TREE_STUMPS;

			}

			if(!(it_current+1)->compare("knn_flann"))
			{

				featureClassificationConfig.currentClassificationAlgo = FEATURE_CLASSIFIER_KNN_FLANN;

			}
			if(!(it_current+1)->compare("knn_hess"))
			{

				featureClassificationConfig.currentClassificationAlgo = FEATURE_CLASSIFIER_KNN_HESS_KDTREE;

			}

			if(!(it_current+1)->compare("geo_center"))
			{

				featureClassificationConfig.currentClassificationAlgo = FEATURE_CLASSIFIER_GEOMETRIC_CENTER;

			}


		}

		if(!it_current->compare("-queryDescFile") || !it_current->compare("-QueryDescFile"))
		{

			featureClassificationConfig.descriptorQueryFilename = *(it_current+1);

		}

		if(!it_current->compare("-trainDescFile") || !it_current->compare("-TrainDescFile"))
		{

			featureClassificationConfig.descriptorTrainFilename = *(it_current+1);

		}

		if(!it_current->compare("-queryDescClassesFile") || !it_current->compare("-QueryDescClassesFile"))
		{

			featureClassificationConfig.queryDescriptorsClassesFile = *(it_current+1);

		}

		if(!it_current->compare("-trainDescClassesFile") || !it_current->compare("-TrainDescClassesFile"))
		{

			featureClassificationConfig.trainDescriptorsClassesFile = *(it_current+1);

		}


		if(!it_current->compare("-inputMethod") || !it_current->compare("-InputMethod"))
		{
			if(!(it_current+1)->compare("monoStream"))
			{
				//featureClassificationConfig.inputMethod=FEATURE_DESC_SINGLE_STREAM;
			}

		}

		if(!it_current->compare("-nIterations"))
		{
			stringBuffer.str("");
			stringBuffer.clear();
			stringBuffer << *(it_current+1);
			cout << "Iter:"<<*(it_current+1) << endl;
			if((stringBuffer >> featureClassificationConfig.numberOfIterations).fail())
			{
				cout << "Number of iterations could not be parsed." << endl;
			}

		}

		if(!it_current->compare("-nVertCores"))
		{
			stringBuffer.str("");
			stringBuffer.clear();
			stringBuffer << *(it_current+1);
			cout << "Vcores:"<<*(it_current+1) << endl;
			if((stringBuffer >> featureClassificationConfig.numberOfVerticalProcs).fail())
			{
				cout << "Number of vertical cores could not be parsed." << endl;
			}

		}

		if(!it_current->compare("-nHoriCores"))
		{
			stringBuffer.str("");
			stringBuffer.clear();
			stringBuffer << *(it_current+1);
			cout << "Hcores:"<<*(it_current+1) << endl;
			if((stringBuffer >> featureClassificationConfig.numberOfHorizontalProcs).fail())
			{
				cout << "Number of horizontal cores could not be parsed." << endl;
			}

		}


		if( it_current->compare("-trainClassifier") == 0 || it_current->compare("-TrainClassifier") == 0)
		{
			featureClassificationConfig.trainClassifier = true;
		}

		if( it_current->compare("-singleThreaded") == 0 || it_current->compare("-SingleThreaded") == 0)
		{
			featureClassificationConfig.singleThreaded= true;
		}

		if(!it_current->compare("-testDesc"))
		{
			featureClassificationConfig.useTestDescriptors = true;
		}

		if(!it_current->compare("-loadClassificationStruct"))
		{
			featureClassificationConfig.classifierInputFilename = *(it_current+1);
		}

		if(!it_current->compare("-saveClassificationStruct"))
		{

			//featureClassificationConfig.
			featureClassificationConfig.classifierOutputFilename = *(it_current+1);
		}

		if((it_current->compare("-loadQueryKeyPoints")==0) || (it_current->compare("-LoadQueryKeyPoints")==0))
		{

			featureClassificationConfig.loadQueryKeypoints = true;
			featureClassificationConfig.keyPointQueryFilename = *(it_current+1);
		}

		if((it_current->compare("-loadTrainKeyPoints")==0) || (it_current->compare("-LoadTrainKeyPoints")==0))
		{
			featureClassificationConfig.loadTrainKeypoints = true;
			featureClassificationConfig.keyPointTrainFilename = *(it_current+1);
		}

		if((it_current->compare("-loadTrainImage")==0) || (it_current->compare("-LoadTrainImage")==0))
		{
			featureClassificationConfig.loadTrainImage = true;
			featureClassificationConfig.trainImageFilename = *(it_current+1);
		}

		if((it_current->compare("-loadQueryImage")==0) || (it_current->compare("-LoadQueryImage")==0))
		{
			featureClassificationConfig.loadQueryImage = true;
			featureClassificationConfig.queryImageFilename = *(it_current+1);
		}



	}


	if(featureClassificationConfig.numberOfVerticalProcs ==1 && featureClassificationConfig.numberOfHorizontalProcs ==1)
	{
		featureClassificationConfig.singleThreaded = true;


	}
	else
	{
		featureClassificationConfig.singleThreaded = false;

	}





	return;

}

FeatureClassificationConfig setupFeatureClassificationConfigFromFile(string filename)
{
	FeatureClassificationConfig featureClassificationConfig;

	vector<string> commandArgs;


	loadFeatureClassificationConfigFile(commandArgs, filename);

	parseClassificationConfigCommandVector(featureClassificationConfig, commandArgs);
	return featureClassificationConfig;

}


void classificationCreateThreadManager(ThreadManager * &classificationThreadManager,int verticalThreads, int horizontalThreads)
{
	int numberOfThreads = verticalThreads*horizontalThreads;


	cout << "Creating Thread Manager and basic thread structs" << endl;
	classificationThreadManager = new ThreadManager(numberOfThreads);

	for(int i = 0; i<numberOfThreads;i++)
	{


		Thread * currentThreadPtr = (classificationThreadManager->getThread(i));


		currentThreadPtr->setThreadLogicalId(i);
		classificationThreadManager->setThreadBarrier(i);
	}

}



void classificationSetupThreadManager(ThreadManager * &classificationThreadManager,FeatureClassificationConfig & featureClassificationConfig,vector<thread_fptr> threadFunctions, vector<void*> threadArgs,int verticalThreads, int horizontalThreads)
{




	int numberOfThreads = classificationThreadManager->getNumberOfThreads();

	cout << "Filling in Base Thread Data structs" << endl;

	for(int i = 0; i<numberOfThreads;i++)
	{

		Thread * currentThreadPtr = (classificationThreadManager->getThread(i));


		currentThreadPtr->setThreadParam(threadArgs[i]);
		currentThreadPtr->setThreadFunction(threadFunctions[i]);//(currentWorkerThreadInfo->coordinator? classificationCoordinatorThreadFunction: classificationWorkerThreadFunction));

	}







}

void classificationLaunchThreads(ThreadManager * &classificationThreadManager)
{
	//Done so we can have the work waiting for them once they are spawned
	for(int i = 0; i<classificationThreadManager->getNumberOfThreads();i++)
	{
		Thread * currentThreadPtr = (classificationThreadManager->getThread(i));

		int errcode = pthread_create(currentThreadPtr->getThreadIdPtr(),NULL,currentThreadPtr->getThreadFunction(),currentThreadPtr->getThreadParam());
		if(errcode != 0)
		{
			cout << "Error creating thread: " << strerror(errcode) << endl;
		}
	}
}

void setClassificationWorkingThreadDataInGeneralWorkingData(vector<GeneralWorkerThreadData> &genData, vector<struct WorkerThreadInfo> & workingThreads)
{

	for(int i = 0; i< genData.size() && i<workingThreads.size(); i++)
	{

		genData[i].featureClassificationData = (void *)&workingThreads[i];



	}


}



//This function assumes another function will create the thread manager and the thread structs
void setupClassificationThreadsData(FeatureClassificationConfig & featureClassificationConfig,ThreadManager * &classificationThreadManager,Mat& queryDescriptors, vector<KeyPoint>& queryKeypoints,Mat &trainDescriptors, vector<KeyPoint> &trainKeypoints, vector<struct GeneralWorkerThreadData> &genData,MultiThreadedMatResult * &multiThreadResultPtr,MultiThreadedMatchResult * &multiThreadMatchResultPtr ,int verticalThreads, int horizontalThreads)
{



	int numberOfThreads = verticalThreads*horizontalThreads;



	cout << "Creating woking threads structs" << endl;
	//workingThreads.clear();
	//workingThreads.resize(numberOfThreads);

	//Work on this one.....
	multiThreadResultPtr = new MultiThreadedMatResult(64);
	multiThreadResultPtr -> createResultsMat(queryDescriptors.rows,1);
	multiThreadMatchResultPtr = new 	MultiThreadedMatchResult(64);
	multiThreadMatchResultPtr->createResultsVec(queryDescriptors.rows);

	MultiThreadAlgorithmData * multiThreadAlgorithmDataBasePtr = setupClassificationMultiThreadedData(featureClassificationConfig);

	cout << "Filling in Data structs and creating worker threads" << endl;

	for(int i = 0; i<numberOfThreads;i++)
	{
		Mat myDescriptors;
		Thread * currentThreadPtr = (classificationThreadManager->getThread(i));
		struct WorkerThreadInfo * currentWorkerThreadInfo = (struct WorkerThreadInfo *)genData[i].featureClassificationData;
				//&workingThreads[i];

		genData[i].myThread = currentThreadPtr;
		genData[i].threadManager = classificationThreadManager;
		currentWorkerThreadInfo->myThread =currentThreadPtr;
		currentWorkerThreadInfo->threadManager = classificationThreadManager;
		currentWorkerThreadInfo->verticalThreadId = i/horizontalThreads;
		currentWorkerThreadInfo->horizontalThreadId = i%horizontalThreads;
		currentWorkerThreadInfo->coordinator = (i==0? true: false);
		currentWorkerThreadInfo->multiThreadResults = multiThreadResultPtr;
		currentWorkerThreadInfo->multiThreadMatchResults = multiThreadMatchResultPtr;
		currentWorkerThreadInfo->multiThreadAlgorithmData = copyClassificationMultiThreadedData(featureClassificationConfig,multiThreadAlgorithmDataBasePtr);
		currentWorkerThreadInfo->standAlone = true;
		currentWorkerThreadInfo->done= false;



		//Get the descriptors
		myDescriptors = getQueryDescriptorsForThread(featureClassificationConfig,verticalThreads, horizontalThreads,currentWorkerThreadInfo->descriptorStartIndex,queryDescriptors,currentWorkerThreadInfo);
		currentWorkerThreadInfo->descriptorsToProcess = myDescriptors;

		//Get the config Data
		getFeatureClassificationInfoForThread(currentWorkerThreadInfo->myFeatureClassificationInfo,featureClassificationConfig,currentWorkerThreadInfo);
		currentWorkerThreadInfo->featureClassificationInfo = &featureClassificationConfig;

		int tmpStartIndex;
		//Get the Keypoints to process
		currentWorkerThreadInfo->keyPointsToProcess = getQueryKeyPointsForThread(featureClassificationConfig,verticalThreads, horizontalThreads,tmpStartIndex,queryKeypoints,currentWorkerThreadInfo);

		currentWorkerThreadInfo->allQueryDescriptors = &queryDescriptors;
		currentWorkerThreadInfo->allQueryKeypoints = &queryKeypoints;

	}


	//Done so we can have the work waiting for them once they are spawned
/*	for(int i = 0; i<numberOfThreads;i++)
	{
		Thread * currentThreadPtr = (classificationThreadManager->getThread(i));
		struct WorkerThreadInfo * currentWorkerThreadInfo = &workingThreads[i];

		int errcode = pthread_create(currentThreadPtr->getThreadIdPtr(),NULL,currentThreadPtr->getThreadFunction(),currentWorkerThreadInfo);
		if(errcode != 0)
		{
			cout << "Error creating thread: " << strerror(errcode) << endl;
		}
	}
*/

}




void multiThreadedSetupOnly(Mat & trainDescriptors,Mat & trainDescriptorsClasses, vector<KeyPoint> &trainKeypoints, Mat & trainImage,Mat & queryDescriptors,Mat & queryDescriptorsClasses,vector<KeyPoint> &queryKeypoints,FeatureClassificationConfig & featureClassificationConfig, ThreadManager * &classificationThreadManager,vector<struct GeneralWorkerThreadData> &genData,MultiThreadedMatResult * &multiThreadResultPtr,MultiThreadedMatchResult * &multiThreadMatchResultPtr)
{
	bool classifySetupResult = false;



	if(featureClassificationConfig.singleCopyOfClassificationStruct || featureClassificationConfig.partialCopyOfClassificationStruct)
	{
		if(featureClassificationConfig.matchActive)
		{
			cout << "Matching operation active." << endl;
			featureClassificationConfig.matcherParams =  setupMatcherParams(featureClassificationConfig);
			featureClassificationConfig.currentMatcherPtrs = setupMatcher(featureClassificationConfig, featureClassificationConfig.matcherParams);

			handleMatcherTraining(trainDescriptors,trainDescriptorsClasses,featureClassificationConfig.currentMatcherPtrs ,featureClassificationConfig);
		}
		if(featureClassificationConfig.classifyActive)
		{

			cout << "Classifying operation active." << endl;
			featureClassificationConfig.classifierParams =  setupClassifierParams( featureClassificationConfig);
			featureClassificationConfig.currentClassificationPtrs = setupClassifier(featureClassificationConfig, featureClassificationConfig.classifierParams);

			classifySetupResult =  handleClassifierTrainingOrLoading(trainDescriptors,trainDescriptorsClasses,trainKeypoints,trainImage, featureClassificationConfig);
		}
	}

	setupClassificationThreadsData(featureClassificationConfig,classificationThreadManager, queryDescriptors,  queryKeypoints,trainDescriptors, trainKeypoints, genData, multiThreadResultPtr,multiThreadMatchResultPtr,featureClassificationConfig.numberOfVerticalProcs, featureClassificationConfig.numberOfHorizontalProcs);
//	setupClassificationThreads(classificationThreadManager,  workingThreads, featureClassificationConfig.numberOfVerticalProcs, featureClassificationConfig.numberOfHorizontalProcs);

}


void classificationCoordinatorThreadSetupFunctionStandAlone(void *  workerThreadStruct)
{

	struct WorkerThreadInfo *  threadParams = reinterpret_cast<struct WorkerThreadInfo * >(workerThreadStruct);
	Thread * myThreadInfo =  threadParams->myThread;
	int numberOfDescriptors = threadParams->descriptorsToProcess.rows;
	int numberOfThreads = threadParams->featureClassificationInfo->numberOfVerticalProcs * threadParams->featureClassificationInfo->numberOfHorizontalProcs;
	int numberOfIterations = threadParams->featureClassificationInfo->numberOfIterations;





#ifdef VERBOSE_DEBUG
	cout << "Current Coordinator Thread is "<< myThreadInfo->getThreadLogicalId()  << " which is ("<< threadParams->verticalThreadId << "," << threadParams->horizontalThreadId << ")" << endl;
#endif


	//Setup classification ConFig
	if(threadParams->myFeatureClassificationInfo == NULL)
	{



	}
	else
	{
#ifdef VERBOSE_DEBUG
		cout << "Using presetup classification config" << endl;
#endif
	}



}
void classificationWorkerThreadSetupFunctionStandAlone(void *  workerThreadStruct)
{

	struct WorkerThreadInfo *  threadParams = reinterpret_cast<struct WorkerThreadInfo * >(workerThreadStruct);
	Thread * myThreadInfo =  threadParams->myThread;
	int numberOfDescriptors = threadParams->descriptorsToProcess.rows;
	Mat resultsMat;
#ifdef VERBOSE_DEBUG
	cout << "Current Worker Thread is "<< myThreadInfo->getThreadLogicalId()  << " which is ("<< threadParams->verticalThreadId << "," << threadParams->horizontalThreadId << ")" << endl;
#endif
	//Setup classification ConFig
	if(threadParams->myFeatureClassificationInfo == NULL)
	{



	}
	else
	{
#ifdef VERBOSE_DEBUG
		cout << "Using presetup classification config" << endl;
#endif
	}


}


void classificationWorkerThreadFunctionStandAlone(void * workerThreadStruct)
{
	struct WorkerThreadInfo *  threadParams = reinterpret_cast<struct WorkerThreadInfo * >(workerThreadStruct);
	Thread * myThreadInfo =  threadParams->myThread;
	int numberOfDescriptors = threadParams->descriptorsToProcess.rows;
	Mat resultsMat;
#ifdef VERBOSE_DEBUG
	cout << "Current Worker Thread is "<< myThreadInfo->getThreadLogicalId()  << " which is ("<< threadParams->verticalThreadId << "," << threadParams->horizontalThreadId << ")" << endl;
#endif




	//My debug counter
	int waits = 0;

	//A debug wait
//	myThreadInfo->waitForWakeup();


	//Barrier to make sure work wait is good
//	myThreadInfo->waitAtBarrier();


	//Wait for work to be distributed, main thread will tell me when my data is ready
	myThreadInfo->waitForWakeup();



	//Setup for work
	vector<vector <DMatch> > matches;
	Mat queryImage;


	//perform matching or classifying until told to stop
	while(!threadParams->done)
	{
		//cout <<" Worker " << myThreadInfo->getThreadLogicalId() << " been Waiting: "<<++waits << " waits" << endl;
		numberOfDescriptors = threadParams->descriptorsToProcess.rows;
		resultsMat.create(numberOfDescriptors,1,CV_32FC1);


		if(threadParams->myFeatureClassificationInfo->matchActive)
		{
			matches.clear();
			multiThreadMatcher(threadParams->descriptorsToProcess, threadParams->myFeatureClassificationInfo->currentMatcherPtrs, threadParams->myFeatureClassificationInfo->matcherParams, matches, *(threadParams->myFeatureClassificationInfo),threadParams);
			if(!threadParams->myFeatureClassificationInfo->classifyActive)
			{
				threadParams->multiThreadMatchResults->copyToReultsVec(matches,threadParams->descriptorStartIndex);
			}
		}
		if(threadParams->myFeatureClassificationInfo->classifyActive)
		{

			multiThreadClassification(threadParams->descriptorsToProcess,resultsMat,threadParams->keyPointsToProcess, queryImage,threadParams->myFeatureClassificationInfo->classifierParams, threadParams->myFeatureClassificationInfo->currentClassificationPtrs, threadParams->myFeatureClassificationInfo->currentMatcherPtrs, threadParams->myFeatureClassificationInfo->matcherParams,matches,threadParams->myFeatureClassificationInfo->currentClassificationAlgo,threadParams->myFeatureClassificationInfo->currentMatcherAlgo,threadParams );

			//Copy my answers
			threadParams->multiThreadResults->copyToReultsMat(resultsMat,threadParams->descriptorStartIndex);

		}





		//Wait after iteration of work
		myThreadInfo->waitAtBarrier();


		//Wait for work to be distributed, main thread will tell me when my data is ready
		myThreadInfo->waitForWakeup();


	}


//	return NULL;
}

void  classificationCoordinatorThreadFunctionStandAlone(void * workerThreadStruct)
{


	struct WorkerThreadInfo *  threadParams = reinterpret_cast<struct WorkerThreadInfo * >(workerThreadStruct);
	Thread * myThreadInfo =  threadParams->myThread;
	int numberOfDescriptors = threadParams->descriptorsToProcess.rows;
	int numberOfThreads = threadParams->featureClassificationInfo->numberOfVerticalProcs * threadParams->featureClassificationInfo->numberOfHorizontalProcs;
	int numberOfIterations = threadParams->featureClassificationInfo->numberOfIterations;



	Mat resultsMat;





	//cout << "Current Coordinator Thread is "<< myThreadInfo->getThreadLogicalId()  << " which is ("<< threadParams->verticalThreadId << "," << threadParams->horizontalThreadId << ")" << endl;
	resultsMat.create(numberOfDescriptors,1,CV_32FC1);



	//Give everyone their work and send them off
	coordinatorThreadDistributeQueryKeypointsAndDescriptors(*(threadParams->myFeatureClassificationInfo),*(threadParams->allQueryDescriptors),*(threadParams->allQueryKeypoints),threadParams);


	//Setup for work
	int currentIteration = 0;
	bool done = false;
	vector<vector <DMatch> > matches;
	Mat queryImage;



	//Do work until done
	while(!done)
	{

		numberOfDescriptors = threadParams->descriptorsToProcess.rows;
		resultsMat.create(numberOfDescriptors,1,CV_32FC1);
		//cout <<" Coordinator " << myThreadInfo->getThreadLogicalId() << " been Waiting: "<<++waits << " waits" << endl;

		if(threadParams->myFeatureClassificationInfo->matchActive)
		{
			matches.clear();

#ifdef VERBOSE_DEBUG
			cout << "Performing multi threaded matching......"<< endl;
#endif

			multiThreadMatcher(threadParams->descriptorsToProcess, threadParams->myFeatureClassificationInfo->currentMatcherPtrs, threadParams->myFeatureClassificationInfo->matcherParams, matches, *(threadParams->myFeatureClassificationInfo),threadParams);

			if(!threadParams->myFeatureClassificationInfo->classifyActive)
			{
				threadParams->multiThreadMatchResults->copyToReultsVec(matches,threadParams->descriptorStartIndex);
			}

#ifdef VERBOSE_DEBUG
			cout << "coordinator done"  << endl;
#endif
		}
		if(threadParams->myFeatureClassificationInfo->classifyActive)
		{
#ifdef VERBOSE_DEBUG
			cout << "Performing multi threaded Classify......"<< endl;
#endif

			multiThreadClassification(threadParams->descriptorsToProcess,resultsMat,threadParams->keyPointsToProcess, queryImage,threadParams->myFeatureClassificationInfo->classifierParams, threadParams->myFeatureClassificationInfo->currentClassificationPtrs, threadParams->myFeatureClassificationInfo->currentMatcherPtrs, threadParams->myFeatureClassificationInfo->matcherParams,matches,threadParams->myFeatureClassificationInfo->currentClassificationAlgo,threadParams->myFeatureClassificationInfo->currentMatcherAlgo,threadParams );

#ifdef VERBOSE_DEBUG
			cout << "coordinator done" << endl;
#endif

			threadParams->multiThreadResults->copyToReultsMat(resultsMat,threadParams->descriptorStartIndex);
		}



		//Wait for everyone to complete this iteration
		myThreadInfo->waitAtBarrier();

		//Increment the iteration
		currentIteration++;

		//If we have more work get it, otherwise time to finish so tell everyone
		if(currentIteration >=numberOfIterations)
		{
			done = true;
			coordinatorSetAllDoneFlag(threadParams, numberOfThreads);
			coordinatorWakeUpOtherThreads(threadParams,numberOfThreads);
		}
		else
		{
			coordinatorThreadDistributeQueryKeypointsAndDescriptors(*(threadParams->myFeatureClassificationInfo),*(threadParams->allQueryDescriptors),*(threadParams->allQueryKeypoints),threadParams);
		}
	}


//	myThreadInfo->waitAtBarrier();


//	return NULL;
}
void * featureClassification_testCoordinatorThreadStandAlone(void * threadParam)
{
	struct GeneralWorkerThreadData * genData = reinterpret_cast<struct GeneralWorkerThreadData *>(threadParam);


	classificationCoordinatorThreadSetupFunctionStandAlone(genData->featureClassificationData);
#ifdef USE_MARSS

	struct WorkerThreadInfo *  threadParams = reinterpret_cast<struct WorkerThreadInfo * >(genData->featureClassificationData);
	Thread * myThreadInfo =  threadParams->myThread;

	cout << "Switching to simulation in Feature Classification." << endl;
	ptlcall_switch_to_sim();
	switch(threadParams->featureClassificationInfo->currentClassificationAlgo)
	{
		case FEATURE_CLASSIFIER_LINEAR_SVM:
			//ptlcall_single_enqueue("-logfile featureClassify_svm.log");
			ptlcall_single_enqueue("-stats featureClassify_svm.stats");
			//ptlcall_single_enqueue("-loglevel 0");
			break;
		case FEATURE_CLASSIFIER_BOOSTED_TREE_STUMPS:
			//ptlcall_single_enqueue("-logfile featureClassify_boost.log");
			ptlcall_single_enqueue("-stats featureClassify_boost.stats");
			//ptlcall_single_enqueue("-loglevel 0");
			break;
		case FEATURE_CLASSIFIER_KNN_FLANN:
			//ptlcall_single_enqueue("-logfile featureClassify_knn.log");
			ptlcall_single_enqueue("-stats featureClassify_knn.stats");
			//ptlcall_single_enqueue("-loglevel 0");
			break;
		case FEATURE_CLASSIFIER_KNN_HESS_KDTREE:
			//ptlcall_single_enqueue("-logfile featureClassify_hess.log");
			ptlcall_single_enqueue("-stats featureClassify_hess.stats");
			//ptlcall_single_enqueue("-loglevel 0");
			break;
		case FEATURE_CLASSIFIER_GEOMETRIC_CENTER:
			//ptlcall_single_enqueue("-logfile featureClassify_geom.log");
			ptlcall_single_enqueue("-stats featureClassify_geom.stats");
			//ptlcall_single_enqueue("-loglevel 0");
			break;
		default:
			//ptlcall_single_enqueue("-logfile featureClassify.log");
			ptlcall_single_enqueue("-stats featureClassify.stats");
			//ptlcall_single_enqueue("-loglevel 0");
			break;

	};


//	ptlcall_single_enqueue("-logfile featureClassify.log");
//	ptlcall_single_enqueue("-stats featureClassify.stats");
//	ptlcall_single_enqueue("-loglevel 0");
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


	classificationCoordinatorThreadFunctionStandAlone(genData->featureClassificationData);

#ifdef USE_MARSS
	ptlcall_kill();
#endif
#ifdef USE_GEM5

	m5_dumpreset_stats(0, 0);
	#ifdef GEM5_EXIT_AFTER_WORK
		m5_exit(0);
	#endif


#endif


	return NULL;
}


void * featureClassification_testWorkerThreadStandAlone(void * threadParam)
{
	struct GeneralWorkerThreadData * genData = reinterpret_cast<struct GeneralWorkerThreadData *>(threadParam);
	classificationWorkerThreadSetupFunctionStandAlone(genData->featureClassificationData);

	classificationWorkerThreadFunctionStandAlone(genData->featureClassificationData);

	return NULL;
}


int featureClassification_testSeperate(int argc, const char * argv[])
{

	//Lets grab some commnad line arguments and convert them to strings
	//I just prefer them in a vector and I know it is inefficient.
	vector<string> commandLineArgs;
	for(int i = 0; i < argc; i++)
	{
		string currentString = argv[i];
		commandLineArgs.push_back(currentString);
	}

	FeatureClassificationConfig featureClassificationConfig;


	if((commandLineArgs[1].compare("-configFile") ==0) || (commandLineArgs[1].compare("-ConfigFile") ==0))
	{

		featureClassificationConfig = setupFeatureClassificationConfigFromFile(commandLineArgs[2]);
	}
	Mat trainDescriptors;
	Mat trainDescriptorsClasses;
	vector<KeyPoint> trainKeypoints;
	Mat trainImage;
	Mat queryDescriptors;
	Mat queryDescriptorsClasses;
	vector<KeyPoint> queryKeypoints;
	ThreadManager * classificationThreadManager;
//	vector<struct WorkerThreadInfo> workingThreads;
	MultiThreadedMatResult * multiThreadResultPtr;
	MultiThreadedMatchResult * multiThreadMatchResultPtr;
	vector <vector<DMatch> >matches;


	Mat queryDescriptorsClassesComputed;
	Mat queryDescriptorsClassesLoaded;
	Mat queryImage;


	handleDescLoading(trainDescriptors,trainDescriptorsClasses, queryDescriptors,queryDescriptorsClassesLoaded,featureClassificationConfig);
	handleKeyPointLoading(trainKeypoints,queryKeypoints,featureClassificationConfig);
	handleImageLoading(trainImage,queryImage,featureClassificationConfig);


	classificationCreateThreadManager(classificationThreadManager,featureClassificationConfig.numberOfVerticalProcs, featureClassificationConfig.numberOfHorizontalProcs);

	vector<GeneralWorkerThreadData> genData(featureClassificationConfig.numberOfVerticalProcs *featureClassificationConfig.numberOfHorizontalProcs);
	vector<struct WorkerThreadInfo> workingThreads(featureClassificationConfig.numberOfVerticalProcs *featureClassificationConfig.numberOfHorizontalProcs);


	//Connect the general data and the working threads data
	setClassificationWorkingThreadDataInGeneralWorkingData(genData,  workingThreads);





	//Setup the Thread Structs
	multiThreadedSetupOnly(trainDescriptors, trainDescriptorsClasses,trainKeypoints,trainImage,queryDescriptors,queryDescriptorsClasses,queryKeypoints,featureClassificationConfig,classificationThreadManager,genData,multiThreadResultPtr,multiThreadMatchResultPtr);;


	vector<void * > threadArgs(genData.size());
	for(int i =0; i< threadArgs.size(); i++)
	{
		threadArgs[i] = (void *)&genData[i];


	}
	vector<thread_fptr> threadFunctions(genData.size());
	threadFunctions[0] = featureClassification_testCoordinatorThreadStandAlone;
	for(int i = 1; i < threadArgs.size(); i++)
	{

		threadFunctions[i] = featureClassification_testWorkerThreadStandAlone;

	}
	//Place the data in the Thread manager
	classificationSetupThreadManager(classificationThreadManager,featureClassificationConfig, threadFunctions, threadArgs,featureClassificationConfig.numberOfVerticalProcs, featureClassificationConfig.numberOfHorizontalProcs);


	classificationLaunchThreads(classificationThreadManager);


	cout << "Waiting for Threads." << endl;
	for(int i=0 ; i<classificationThreadManager->getNumberOfThreads() ; i++)
	{
		pthread_join(classificationThreadManager->getThread(i)->getThreadId(), NULL);
	}
	//cout << "MultiThread Results Mat"<<multiThreadResultPtr->getResultsMat() << endl;
	queryDescriptorsClasses = multiThreadResultPtr->getResultsMat();
	matches = multiThreadMatchResultPtr->getResultsVec();



	return 0;
}

#ifdef TSC_TIMING
		void fc_writeTimingToFile(vector<TSC_VAL_w> timingVector)
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
		void fc_writeTimingToFile(vector<struct timespec> timingVector)
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



int featureClassification_main(int argc, const char * argv[])
{
	FeatureClassificationConfig featureClassificationConfig;

	std::cout << "Feature Classification Mechanism" << std::endl;
#ifdef TSC_TIMING
	fc_timingVector.resize(16);
#endif
#ifdef CLOCK_GETTIME_TIMING
	fc_timeStructVector.resize(16);

#endif

#ifdef SPLASH_SCREEN
	//Load a splash screen
	namedWindow("Splash Screen");
#endif
	Mat splashImage = imread("C:/Users/jlclemon/Documents/VisionData/splashscreen/splash.jpg",1);

	if(!splashImage.data)
	{
		splashImage.create(480,640,CV_8UC3);
		splashImage.setTo(Scalar(0.0));
	}
	else
	{

	}
	Mat smallSplashImage(240,320,CV_8UC3);
	resize(splashImage,smallSplashImage,Size(),0.5,0.5,INTER_LINEAR);

	drawTextOnImage(string("Feature Classification Demo"), Point(250,50), Scalar(0,255,0), smallSplashImage);

#ifdef SPLASH_SCREEN
//	imshow("Splash Screen", smallSplashImage);
#endif

	//featureClassification_testSeperate(argc, argv);

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
		loadFeatureClassificationConfigFile(commandLineArgs, commandLineArgs[2]);



	}


	parseClassificationConfigCommandVector(featureClassificationConfig, commandLineArgs);


	Mat  trainDescriptors;
	Mat  trainDescriptorsClasses;
	Mat  queryDescriptors;
	Mat queryDescriptorsClassesComputed;
	Mat queryDescriptorsClassesLoaded;
	vector <vector <DMatch> > matches;
	vector<KeyPoint> trainKeyPoints;
	vector<KeyPoint> queryKeyPoints;
	Mat trainImage;
	Mat queryImage;

	handleDescLoading(trainDescriptors,trainDescriptorsClasses, queryDescriptors,queryDescriptorsClassesLoaded,featureClassificationConfig);
	handleKeyPointLoading(trainKeyPoints,queryKeyPoints,featureClassificationConfig);
	handleImageLoading(trainImage,queryImage,featureClassificationConfig);



	//cout << "Test Desc:" << trainDescriptors << "\n Query Desc: " << queryDescriptors << endl;

	if(featureClassificationConfig.singleThreaded)
	{
		singleThreadOperations_main(trainDescriptors,trainDescriptorsClasses,trainKeyPoints ,trainImage,queryDescriptors, queryDescriptorsClassesComputed,queryKeyPoints,queryImage, matches , featureClassificationConfig);

		cout << "Operation iteration complete. "<<endl; //Loaded Query Classes: =====>\n" << queryDescriptorsClassesLoaded << "\nComputed Query Classes: =====>\n" << queryDescriptorsClassesComputed << endl;

	}
	else
	{
		//Run as multithreaded Version
		multiThreadedOperations_main(trainDescriptors,trainDescriptorsClasses,trainKeyPoints ,trainImage,queryDescriptors, queryDescriptorsClassesComputed,queryKeyPoints,queryImage, matches , featureClassificationConfig);
		cout << "Multithreaded operation iteration complete."<<endl; //Loaded Query Classes: =====>\n" << queryDescriptorsClassesLoaded << "\nComputed Query Classes: =====>\n" << queryDescriptorsClassesComputed << endl;
	}

#ifdef TSC_TIMING
		fc_writeTimingToFile(fc_timingVector);
#endif
#ifdef CLOCK_GETTIME_TIMING
		fc_writeTimingToFile(fc_timeStructVector);
#endif

#ifdef MATCHES_VERBOSE
	cout <<"Matches: [";
	for(int i = 0; i < matches.size(); i++)
	{
		for(int j = 0; j <matches[i].size(); j++)
		{
			cout << "("<<matches[i][j].queryIdx <<"--->"<< matches[i][j].trainIdx<< ")"<<",";
		}
	}
	cout <<" ]" << endl;
#endif

#ifdef SPLASH_SCREEN
	waitKey(0);
#endif
	//threadTestMain();
	//waitKey(0);
#ifdef SPLASH_SCREEN
	destroyWindow("Splash Screen");
#endif

	cout << "Cleaning up memory...." << endl;

	//Time to clean up memory
	cleanUpMatcher(featureClassificationConfig);
	cleanupClassifier(featureClassificationConfig);
	cleanupDescriptor(featureClassificationConfig);

	cout << "Program Complete.  Exiting.  Have a nice day  :-)" << endl;
	return 0;
}



#ifndef FEATURE_CLASSIFICATION_MODULE

int main(int argc, const char * argv[])
{
	return featureClassification_main(argc, argv);

}
#endif


