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
 * FeatureExtractionWorkerThreadInfo.h
 *
 *  Created on: Apr 30, 2011
 *      Author: jlclemon
 */




#ifndef FEATUREEXTRACTIONWORKERTHREADINFO_H_
#define FEATUREEXTRACTIONWORKERTHREADINFO_H_


#include "ThreadManager.h"
//#include "MultiThreadedMatResult.h"
//#include "FeatureClassification.hpp"
#include "MultiThreadAlgorithmData.h"

struct FeatureExtractionConfig;
struct FeatureExtractionWorkerThreadInfo
{
	Thread * myThread;
	struct FeatureExtractionConfig * myFeatureExtractionInfo;
	int verticalThreadId;
	int horizontalThreadId;
	int numberOfCores;
	int numberOfHorizontalCores;
	int numberOfVerticalCores;
	bool standAlone;
	bool coordinator;

	bool done;



	//vector<Mat> * inputImagesPerFrame;
//	Mat inputImage;
//	Mat trainImage;
	//vector<Mat> * trainImagesPerFrame;
	Rect myPaddedImageRegionRect;
	Rect myNonPaddedImageRegionRect;
	Mat myPaddedImageRegion;
	Mat myNonPaddedImageRegion;

	struct FeatureExtractionData * featureExtractionData;
	struct FeatureExtractionData *  myFeatureExtractionData;
	vector<KeyPoint> keyPoints;
	struct FeatureExtractionConfig * featureExtractionInfo;
	ThreadManager * threadManager;
	struct MultiThreadAlgorithmData * multiThreadAlgorithmData;

};







#endif /* FEATUREEXTRACTIONWORKERTHREADINFO_H_ */
