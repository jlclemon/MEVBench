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
 * FeatureExtractionAndClassification.cpp
 *
 *  Created on: May 31, 2011
 *      Author: jlclemon
 */

#include <iostream>
#include <sched.h>
#include <unistd.h>
#include <opencv2/features2d/features2d.hpp>
#ifndef OPENCV_VER_2_3
	#include <opencv2/nonfree/nonfree.hpp>
#endif
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "FeatureExtraction.hpp"
#include "FeatureExtractionWorkerThreadInfo.h"
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


#ifdef USE_GEM5
#include <fenv.h>
#include <signal.h>
#include <stdio.h>
void classification_sig_handler(int signum);
#endif

#define TIMING_MAX_NUMBER_OF_THREADS 256

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
vector<TSC_VAL_w> fec_timingVector;
#endif
#ifdef CLOCK_GETTIME_TIMING
vector<struct timespec> fec_timeStructVector;
#endif

using namespace std;
using namespace cv;

bool globalDone;
bool globalDataReady;
Mat globalCurrentImage;
vector<Point2f> globalCurrentRefs;


struct FeatureExtractionAndClassificationConfig
{

	string featureExtractionConfigFile;
	string featureClassificationConfigFile;
	int configFileCount;
	bool noShowWindows;
	bool noWaitKey;

	int maxLoops;

	int currentLoop;
};


void internalCallToTestExtractionAlone(int argc, const char * argv[], bool callDirect)
{

	if(callDirect)
	{
		multiThreadFeatureExtraction_StandAloneTest(argc,argv);

	#ifdef TSC_TIMING
		//READ_TIMESTAMP_WITH_WRAPPER( fec_timingVector[genData->myThread->getThreadLogicalId()+ TIMING_MAX_NUMBER_OF_THREADS*2] );
	#endif
	#ifdef CLOCK_GETTIME_TIMING
		//GET_TIME_WRAPPER(fec_timeStructVector[genData->myThread->getThreadLogicalId()+ TIMING_MAX_NUMBER_OF_THREADS*2]);
	#endif

		featureClassification_testSeperate(argc, argv);

	}
	else
	{

		cout << "That is false sir" << endl;
	}



}


void setExtractedDescriptorsAndKeyPointsToClassificationInput(void * featureExtractionDataPtr, void * featureClassificationDataPtr)
{
	struct WorkerThreadInfo *  featureClassificationData = reinterpret_cast<struct WorkerThreadInfo * >(featureClassificationDataPtr);
	struct FeatureExtractionWorkerThreadInfo *featureExtractionData = reinterpret_cast<FeatureExtractionWorkerThreadInfo *> (featureExtractionDataPtr);
	struct FeatureExtractionMultiThreadData * myMultiThreadExtractionData = reinterpret_cast<FeatureExtractionMultiThreadData *>(featureExtractionData->multiThreadAlgorithmData);

	featureClassificationData->queryImage = &((*(featureExtractionData->myFeatureExtractionData->inputImagesPerFrame))[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA]);
	featureClassificationData->allQueryDescriptors = (myMultiThreadExtractionData->descriptors);
	featureClassificationData->allQueryKeypoints = (myMultiThreadExtractionData->keypoints);

#ifdef VERBOSE_DEBUG
	cout << "Info From Extraction" << "\n  Number Of Extracted Keypoints: "<<featureClassificationData->allQueryKeypoints->size()
			<< "\n Number of extracted descriptors: " << featureClassificationData->allQueryDescriptors->rows << endl;
#endif



}

void * coordinatorThreadFunction(void * threadParam)
{
	int currentLoop = 0;
	int maxLoops = 20;
	struct GeneralWorkerThreadData * genData = reinterpret_cast<struct GeneralWorkerThreadData *>(threadParam);
	struct FeatureExtractionWorkerThreadInfo *featureExtractionData = reinterpret_cast<FeatureExtractionWorkerThreadInfo *> (genData->featureExtractionData);
	struct WorkerThreadInfo *  featureClassificationData = reinterpret_cast<struct WorkerThreadInfo * >(genData->featureClassificationData);

	//Setup the pieces
	featureExtractionCoordinatorThreadSetupFunctionStandAlone(genData,genData->featureExtractionData);
	classificationCoordinatorThreadSetupFunctionStandAlone(genData->featureClassificationData);

#ifdef VERBOSE_DEBUG
	int counting = 0;
#endif

#ifdef USE_MARSS
	cout << "Switching to simulation in Object Recognition." << endl;
	ptlcall_switch_to_sim();
	//ptlcall_single_enqueue("-logfile objRecog.log");
	//ptlcall_single_enqueue("-stats objRecog.stats");
	ptlcall_single_flush("-stats objRecog.stats");
	//ptlcall_single_enqueue("-loglevel 0");
#endif


#ifdef USE_GEM5
	#ifdef GEM5_CHECKPOINT_WORK
		m5_checkpoint(0, 0);
	#endif

//	#ifdef GEM5_SWITCHCPU_AT_WORK
//		m5_switchcpu();
//	#endif 
	m5_dumpreset_stats(0, 0);
#endif




#ifdef TSC_TIMING
	READ_TIMESTAMP_WITH_WRAPPER( fec_timingVector[genData->myThread->getThreadLogicalId()] );
#endif
#ifdef CLOCK_GETTIME_TIMING
	GET_TIME_WRAPPER(fec_timeStructVector[genData->myThread->getThreadLogicalId()]);
#endif

	while(!globalDone)
	{
		//Call the pieces
		featureExtractionCoordinatorThreadFunctionStandAlone(genData,genData->featureExtractionData);

#ifdef VERBOSE_DEBUG
		counting ++;
		cout << "Count: " << counting << endl;

		if(counting == 11)
		{
			cout << "debug here" << endl;

		}
#endif
		#ifdef TSC_TIMING
			READ_TIMESTAMP_WITH_WRAPPER( fec_timingVector[genData->myThread->getThreadLogicalId()+ TIMING_MAX_NUMBER_OF_THREADS*2] );
		#endif
		#ifdef CLOCK_GETTIME_TIMING
			GET_TIME_WRAPPER(fec_timeStructVector[genData->myThread->getThreadLogicalId()+ TIMING_MAX_NUMBER_OF_THREADS*2]);
		#endif


		if(featureExtractionData->featureExtractionInfo->outOfImages || featureExtractionData->myFeatureExtractionInfo->outOfImages)
		{
			break;
		}
		if(featureExtractionData->done)
		{
			globalDone = true;
		}
		currentLoop++;
		if(currentLoop >=maxLoops)
		{
			globalDone = true;

		}


		//Connect the two pieces here
		setExtractedDescriptorsAndKeyPointsToClassificationInput(genData->featureExtractionData, genData->featureClassificationData);



		//Run Classification
		classificationCoordinatorThreadFunctionStandAlone(genData->featureClassificationData);
		if(globalDataReady ==false)
		{
			globalCurrentImage = ((*(featureExtractionData->myFeatureExtractionData->inputImagesPerFrame))[FEATURE_DESC_IMAGE_SOURCE_LOCATION_LEFT_CAMERA]).clone();

			globalCurrentRefs = reinterpret_cast<struct SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierMultiThreadData *>(featureClassificationData->multiThreadAlgorithmData)->estimatedRefPoints->getResultsVec();
			globalDataReady = true;

		}



	}

	globalDone = true;
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
	READ_TIMESTAMP_WITH_WRAPPER( fec_timingVector[genData->myThread->getThreadLogicalId()+ TIMING_MAX_NUMBER_OF_THREADS] );
#endif
#ifdef CLOCK_GETTIME_TIMING
	GET_TIME_WRAPPER(fec_timeStructVector[genData->myThread->getThreadLogicalId()+ TIMING_MAX_NUMBER_OF_THREADS]);
#endif


	cout << "Coordinator Complete" << endl;
	return NULL;

}

void * workerThreadFunction(void * threadParam)
{

	struct GeneralWorkerThreadData * genData = reinterpret_cast<struct GeneralWorkerThreadData *>(threadParam);
	struct FeatureExtractionWorkerThreadInfo *featureExtractionData = reinterpret_cast<FeatureExtractionWorkerThreadInfo *> (genData->featureExtractionData);
	struct WorkerThreadInfo *  featureClassificationData = reinterpret_cast<struct WorkerThreadInfo * >(genData->featureClassificationData);

	//Setup the pieces
	featureExtractionWorkerThreadSetupFunctionStandAlone(genData,genData->featureExtractionData);
	classificationWorkerThreadSetupFunctionStandAlone(genData->featureClassificationData);

#ifdef TSC_TIMING
	READ_TIMESTAMP_WITH_WRAPPER( fec_timingVector[genData->myThread->getThreadLogicalId()] );
#endif
#ifdef CLOCK_GETTIME_TIMING
	GET_TIME_WRAPPER(fec_timeStructVector[genData->myThread->getThreadLogicalId()]);
#endif


	while(!globalDone)
	{


		//Run the functions
		featureExtractionWorkerThreadFunctionStandAlone(genData,genData->featureExtractionData);


#ifdef TSC_TIMING
	READ_TIMESTAMP_WITH_WRAPPER( fec_timingVector[genData->myThread->getThreadLogicalId()+ TIMING_MAX_NUMBER_OF_THREADS*2] );
#endif
#ifdef CLOCK_GETTIME_TIMING
	GET_TIME_WRAPPER(fec_timeStructVector[genData->myThread->getThreadLogicalId()+ TIMING_MAX_NUMBER_OF_THREADS*2]);
#endif


		if(featureExtractionData->featureExtractionInfo->outOfImages || featureExtractionData->myFeatureExtractionInfo->outOfImages)
		{
			break;
		}

		classificationWorkerThreadFunctionStandAlone(genData->featureClassificationData);

	}
#ifdef TSC_TIMING
	READ_TIMESTAMP_WITH_WRAPPER( fec_timingVector[genData->myThread->getThreadLogicalId()+ TIMING_MAX_NUMBER_OF_THREADS] );
#endif
#ifdef CLOCK_GETTIME_TIMING
	GET_TIME_WRAPPER(fec_timeStructVector[genData->myThread->getThreadLogicalId()+ TIMING_MAX_NUMBER_OF_THREADS]);
#endif

	globalDone = true;
	return NULL;



}
void parseConfigCommandVector(FeatureExtractionAndClassificationConfig & config, vector<string> & commandStringVector, vector <string> &extractionExtraParams, vector<string>  &classificationExtraParams)
{



	vector<string>::iterator it_start = commandStringVector.begin();
	vector<string>::iterator it_end = commandStringVector.end();
	vector<string>::iterator it_current;

	stringstream stringBuffer;

	//Set the defaults
	config.featureClassificationConfigFile = "classificationConfig.txt";
	config.featureExtractionConfigFile = "extractionConfig.txt";
	config.configFileCount = 0;

	config.currentLoop = 0;
	config.maxLoops = 5;

	config.noShowWindows = false;
	config.noWaitKey = false;

	//-desc sift -match knn_flann -classify linear_svm -queryDescFile test.yml -trainDescFile test.yml -loadClassificationStruct test.yml -loadMatchStruct test.yml
	for(it_current=it_start; it_current!=it_end; ++it_current)
	{


		if(!it_current->compare("-extractionConfig") || !it_current->compare("-ExractionConfig"))
		{

			config.featureExtractionConfigFile =*(it_current+1);
			config.configFileCount |= 1;
		}

		if(!it_current->compare("-classificationConfig") || !it_current->compare("-ClassificationConfig"))
		{

			config.featureClassificationConfigFile =*(it_current+1);
			config.configFileCount |= 2;
		}
		if(!it_current->compare("-noShowWindows") || !it_current->compare("-NoShowWindows"))
		{

			config.noShowWindows = true;

		}

		if(!it_current->compare("-noWaitKey") || !it_current->compare("-NoWaitKey"))
		{

			config.noWaitKey = true;

		}
		if(!it_current->compare("-extractXtraParams") || !it_current->compare("-ExtractXtraParams"))
		{
			vector<string> xtraParamsVec;
			string extraParams =*(it_current+1);
			char currentString[120];
			strcpy(currentString, extraParams.c_str());
			char * currentTokPointer = strtok(currentString,",");

			while(currentTokPointer != NULL)
			{
				string currentNewString(currentTokPointer);
				xtraParamsVec.push_back(currentNewString);
				currentTokPointer = strtok(NULL,",");
			}

			extractionExtraParams = xtraParamsVec;
		}

		if(!it_current->compare("-classifyXtraParams") || !it_current->compare("-ClassifyXtraParams"))
		{
			vector<string> xtraParamsVec;
			string extraParams  =*(it_current+1);
			char currentString[120];
			strcpy(currentString, extraParams.c_str());
			char * currentTokPointer = strtok(currentString,",");

			while(currentTokPointer != NULL)
			{
				string currentNewString(currentTokPointer);
				xtraParamsVec.push_back(currentNewString);
				currentTokPointer = strtok(NULL,",");
			}

			classificationExtraParams = xtraParamsVec;


		}



	}

	if(config.configFileCount != 3)
	{

		cout << "Warning: Not enough config files given on command line" << endl;


	}





}


void runMultiThreadedFeatureExtractionAndClassification(int argc, const char * argv[])
{

	vector<string> commandLineArgs;
	for(int i = 0; i < argc; i++)
	{
		string currentString = argv[i];
		commandLineArgs.push_back(currentString);
	}


	FeatureExtractionAndClassificationConfig  config;
	vector <string> extractionExtraParams;
	vector <string> classificationExtraParams;

	parseConfigCommandVector(config, commandLineArgs,extractionExtraParams, classificationExtraParams);

	FeatureExtractionConfig featureExtractionConfig;
	FeatureClassificationConfig featureClassificationConfig;


	setupFeatureExtractionConfigFromFile(config.featureExtractionConfigFile,featureExtractionConfig,extractionExtraParams);
	featureClassificationConfig = setupFeatureClassificationConfigFromFile(config.featureClassificationConfigFile,classificationExtraParams);

	//They need to have the same number of threads
	featureClassificationConfig.numberOfHorizontalProcs = featureExtractionConfig.numberOfHorizontalProcs;
	featureClassificationConfig.numberOfVerticalProcs = featureExtractionConfig.numberOfVerticalProcs;


	//Setup Extraction Data*******************************
	FeatureExtractionData featureExtractionData;
	featureExtractionSetupFeatureExtractionData(featureExtractionConfig, featureExtractionData);

//#define WAIT_FOR_STABLE_STREAM
#ifdef WAIT_FOR_STABLE_STREAM

	if(!config.noShowWindows)
	{
		int key = -1;
		namedWindow("CurrentStream");

		while(key ==-1)
		{

			getNextImages((*(featureExtractionData.inputImagesPerFrame)),featureExtractionConfig);
			imshow("CurrentStream",(*(featureExtractionData.inputImagesPerFrame))[0]);

			key = waitKey(33);

		}
		destroyWindow("CurrentStream");
	}
#endif
	//Setup classification Data************************
	Mat trainDescriptors;
	Mat trainDescriptorsClasses;
	vector<KeyPoint> trainKeypoints;
	Mat trainImage;
	Mat queryDescriptors;
	Mat queryDescriptorsClasses;
	vector<KeyPoint> queryKeypoints;
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



	//Create thread Manager
	ThreadManager * threadManager;
	featureExtractionCreateThreadManager(threadManager,featureExtractionConfig.numberOfVerticalProcs, featureExtractionConfig.numberOfHorizontalProcs);


	//Create the general struct
	vector<GeneralWorkerThreadData> genData(featureExtractionConfig.numberOfVerticalProcs *featureExtractionConfig.numberOfHorizontalProcs);


	//Create the worker infos for extraction and classification
	vector<struct FeatureExtractionWorkerThreadInfo> extractionWorkingThreads(featureExtractionConfig.numberOfVerticalProcs *featureExtractionConfig.numberOfHorizontalProcs);
	vector<struct WorkerThreadInfo> classificationWorkingThreads(featureClassificationConfig.numberOfVerticalProcs *featureClassificationConfig.numberOfHorizontalProcs);

	//Connect the general data and the working threads data for extraction
	setFeatureExtractionWorkingThreadDataInGeneralWorkingData(genData,extractionWorkingThreads);

	//Connect the general data and the working threads data for classification
	setClassificationWorkingThreadDataInGeneralWorkingData(genData,  classificationWorkingThreads);

	//Setup the Extraction
	featureExtractionMultiThreadedSetupOnly(featureExtractionConfig, featureExtractionData, threadManager,genData);

	//Setup the Classification
	multiThreadedSetupOnly(trainDescriptors, trainDescriptorsClasses,trainKeypoints,trainImage,queryDescriptors,queryDescriptorsClasses,queryKeypoints,featureClassificationConfig,threadManager,genData,multiThreadResultPtr,multiThreadMatchResultPtr);;



	vector<void * > threadArgs(genData.size());
	for(int i =0; i< threadArgs.size(); i++)
	{
		threadArgs[i] = (void *)&genData[i];


	}
	vector<thread_fptr> threadFunctions(genData.size());
	threadFunctions[0] = coordinatorThreadFunction;
	for(int i = 1; i < threadArgs.size(); i++)
	{

		threadFunctions[i] = workerThreadFunction;



	}

	//Place the data in the Thread manager
	featureExtractionSetupThreadManager(threadManager,featureExtractionConfig, threadFunctions, threadArgs,featureExtractionConfig.numberOfVerticalProcs, featureExtractionConfig.numberOfHorizontalProcs);

	globalDone = false;
	globalDataReady = false;
	featureExtractionLaunchThreads(threadManager);

//	bool globalDone;
//	bool globalDataReady;
//	Mat globalCurrentImage;
//	vector<Point2f> globalCurrentRefs;
	if(!config.noShowWindows)
	{
		namedWindow("CurrentResults");
	}
	Mat currentResults;
	currentResults.create(480,640,CV_8UC3);

	vector<Point2f> currentRefs;
	int mainKey = -1;


	while(mainKey != 'd' && mainKey != 27 && !globalDone)
	{

		if(globalDataReady)
		{
			if(!config.noShowWindows)
			{

				currentResults = globalCurrentImage.clone();
				currentRefs = globalCurrentRefs;
			}

			globalDataReady = false;


		}
		if(!config.noShowWindows)
		{
			for(int i = 0; i < currentRefs.size(); i++)
			{
				 circle(currentResults,currentRefs[i],5,Scalar(0,255,255),CV_FILLED);


			 }
			imshow("CurrentResults", currentResults);
		}
		if(!config.noWaitKey)
		{


			mainKey = waitKey(33) & 0xff;
		}

	}
	globalDone = true;




	//cout << "Waiting for Threads." << endl;
	for(int i=0 ; i<threadManager->getNumberOfThreads() ; i++)
	{
		pthread_join(threadManager->getThread(i)->getThreadId(), NULL);
	}


//#define SHOW_FEATURE_CLASSIFICATION_OUTPUT_GEOM
#ifdef SHOW_FEATURE_CLASSIFICATION_OUTPUT_GEOM

	 namedWindow("Show Where");
	 vector<Point2f> estimatedPoints = reinterpret_cast<struct SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierMultiThreadData *>(classificationWorkingThreads[0].multiThreadAlgorithmData)->estimatedRefPoints->getResultsVec();
	 Mat where = classificationWorkingThreads[0].queryImage->clone();

	 for(int i = 0; i < estimatedPoints.size(); i++)
	 {
		 circle(where,estimatedPoints[i],5,Scalar(0,255,255),CV_FILLED);


	 }

	 imshow("Show Where", where);
	 waitKey(0);
	 destroyWindow("Show Where");
#endif
	cout << "Multithreaded App complete" << endl;




	return;
}

#ifdef TSC_TIMING
		void fec_writeTimingToFile(vector<TSC_VAL_w> timingVector)
		{

			ofstream outputFile;
			outputFile.open("timing.csv");
			outputFile << "Thread,Start,Finish,Mid" << endl;
			for(int i = 0; i<(TIMING_MAX_NUMBER_OF_THREADS); i++)
			{
				outputFile << i << "," << timingVector[i] << "," << timingVector[i+TIMING_MAX_NUMBER_OF_THREADS]<< ","<< timingVector[i+2*TIMING_MAX_NUMBER_OF_THREADS] << endl;

			}
			outputFile.close();
		}
#endif
#ifdef CLOCK_GETTIME_TIMING
		void fec_writeTimingToFile(vector<struct timespec> timingVector)
		{

			ofstream outputFile;
			outputFile.open("timing.csv");
			outputFile << "Thread,Start sec, Start nsec,Finish sec, Finish nsec,Mid sec, Mid nsec" << endl;
			for(int i = 0; i<(TIMING_MAX_NUMBER_OF_THREADS); i++)
			{
				outputFile << i << "," << timingVector[i].tv_sec<<","<< timingVector[i].tv_nsec << "," << timingVector[i+TIMING_MAX_NUMBER_OF_THREADS].tv_sec<<","<< timingVector[i+TIMING_MAX_NUMBER_OF_THREADS].tv_nsec<< "," << timingVector[i+2*TIMING_MAX_NUMBER_OF_THREADS].tv_sec<<","<< timingVector[i+2*TIMING_MAX_NUMBER_OF_THREADS].tv_nsec <<endl;

			}
			outputFile.close();
		}
#endif




int main (int argc, const char * argv[])
{





	cout << "Hello world" << endl;
#ifdef TSC_TIMING
	fec_timingVector.resize(TIMING_MAX_NUMBER_OF_THREADS*3);
#endif
#ifdef CLOCK_GETTIME_TIMING
	//fec_timeStructVector.resize(16);
	fec_timeStructVector.resize(TIMING_MAX_NUMBER_OF_THREADS*3);
#endif

	internalCallToTestExtractionAlone(argc,argv,false);
	runMultiThreadedFeatureExtractionAndClassification(argc,argv);
#ifdef TSC_TIMING
		fec_writeTimingToFile(fec_timingVector);
#endif
#ifdef CLOCK_GETTIME_TIMING
		fec_writeTimingToFile(fec_timeStructVector);
#endif
	return 0;
}
