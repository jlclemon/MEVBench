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
 * FaceDetection.hpp
 *
 *  Created on: Jun 11, 2011
 *      Author: jlclemon
 */

#ifndef FACEDETECTION_HPP_
#define FACEDETECTION_HPP_
#include <string>
#include <fstream>
#include <iostream>
#include <time.h>
#include <list>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/objdetect/objdetect.hpp>


using namespace cv;
using namespace std;


enum FaceDetectionInputMethods
{

	FACE_DETECT_SINGLE_STREAM,
	FACE_DETECT_STEREO_STREAM,
	FACE_DETECT_IMAGE_FILE_LIST,
	FACE_DETECT_VIDEO_FILE,
	NUMBER_OF_FACE_DETECT_INPUT_METHODS


};

struct FaceDetectionConfig
{
	FaceDetectionInputMethods inputMethod;

	int cameraId[2];
	bool useImageList;
	int numberOfVerticalProcs;
	int numberOfHorizontalProcs;
	bool singleThreaded;
	bool bufferedImageList;
	string imageListFilename;
	string imageListBaseDir;
	string cascadeFaceDetectorFilename;


	string inputVideoFilename;
	string outputRawStreamFilename;
	string outputAugmentedStreamFilename;
	int bufferedImageListBufferSize;
	string outputRawImageBaseName;
	string outputRawImageExt;
	double faceScale;
	bool showWindows;
	bool noWaitKey;


};

struct FaceDetectionData
{
	VideoCapture capture[2];
	vector<string> imageList;
	VideoWriter rawStreamWriter;
	VideoWriter augmentedStreamWriter;
	int currentOutputRawImageId;
	int currentInputImageId;
	vector<string> inputImageList;
	list<Mat> imageListBuffer[2];
	CascadeClassifier faceCascade;

	Mat frameBuffer[2];
	Mat currentFrame[2];
	Mat currentAgumentedFrame[2];
	Size frameSize;
	bool outOfImages;


};



#endif /* FACEDETECTION_HPP_ */
