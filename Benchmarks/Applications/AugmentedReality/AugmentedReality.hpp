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
 * AugmentedReality.hpp
 *
 *  Created on: May 25, 2011
 *      Author: jlclemon
 */

#ifndef AUGMENTEDREALITY_HPP_
#define AUGMENTEDREALITY_HPP_
#include <string>
#include <fstream>
#include <iostream>
#include <time.h>
#include <list>
#include <opencv2/features2d/features2d.hpp>
#ifndef OPENCV_VER_2_3
	#include <opencv2/nonfree/nonfree.hpp>
#endif
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/tracking.hpp>
#include "CameraCalibration.h"

using namespace cv;
using namespace std;


enum AugmentedRealityInputMethods
{

	AUGMENT_REAL_SINGLE_STREAM,
	AUGMENT_REAL_STEREO_STREAM,
	AUGMENT_REAL_IMAGE_FILE_LIST,
	AUGMENT_REAL_VIDEO_FILE,
	NUMBER_OF_AUGMENT_REAL_INPUT_METHODS




};


struct AugmentedRealityConfig
{
	AugmentedRealityInputMethods inputMethod;
	int cameraId[2];
	bool useImageList;
	int numberOfVerticalProcs;
	int numberOfHorizontalProcs;
	bool singleThreaded;
	bool bufferedImageList;
	string imageListFilename;
	string imageListBaseDir;
	string calibFilename;
	string inputVideoFilename;
	string outputRawStreamFilename;
	string outputAugmentedStreamFilename;
	int bufferedImageListBufferSize;
	string outputRawImageBaseName;
	string outputRawImageExt;
	string outputAugImageBaseName;
	string outputAugImageExt;
	int markerLostFrameCount;
	int numberOfListLoops;

	bool tracking;
	bool showWindows;
	bool noWaitKey;
	bool pauseMode;
	bool longList;
};


struct AugmentedRealityData
{
	VideoCapture capture[2];
	vector<string> imageList;
	VideoWriter rawStreamWriter;
	VideoWriter augmentedStreamWriter;
	int currentOutputRawImageId;
	int currentOutputAugImageId;
	int currentInputImageId;
	vector<string> inputImageList;
	list<Mat> imageListBuffer[2];
	Mat frameBuffer[2];
	Mat currentFrame[2];

	Mat prevFrame[2];
	vector<Point2f> currentMarkerCorners;
	vector<Point2f> prevCorners;
	int lastNormalizedOrientation;
	int normalizedOrientation;
	Mat currentAugmentedFrame[2];
	vector<Point2f> trackingCorners;
	Size frameSize;

	CameraCalibration cameraCalibration;

	Mat map1[2];
	Mat map2[2];
	Mat rVec[2];
	Mat tVec[2];

	int currentListLoop;
	bool outOfImages;
};


#endif /* AUGMENTEDREALITY_HPP_ */
