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
 * CameraCalibration.h
 *
 *  Created on: May 26, 2011
 *      Author: jlclemon
 */

#ifndef CAMERACALIBRATION_H_
#define CAMERACALIBRATION_H_
#include <string>
#include <fstream>
#include <iostream>
#include <time.h>
#include <stdio.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/objdetect/objdetect.hpp>

using namespace std;
using namespace cv;


#define CAMERA_CALIBRATION_ESCAPE_KEY_VALUE 27
#define CAMERA_CALIBRATION_RETURN_KEY_VALUE 10
#define CAMERA_CALIBRATION_DEFAULT_CAMERA_ID 0
#define CAMERA_CALIBRATION_DEFAULT_LEFT_CAMERA_ID 0
#define CAMERA_CALIBRATION_DEFAULT_RIGHT_CAMERA_ID 1
#define CAMERA_CALIBRATION_DEFAULT_BOARD_HEIGHT 6
#define CAMERA_CALIBRATION_DEFAULT_BOARD_WIDTH 8
#define CAMERA_CALIBRATION_DEFAULT_FLAGS 0
#define CAMERA_CALIBRATION_DEFAULT_ASPECT_RATIO 0.0f
#define CAMERA_CALIBRATION_DEFAULT_SQUARE_SIZE 3.0f
#define CAMERA_CALIBRATION_DEFAULT_OUTPUT_FILENAME "./cameraCalibration.yml"
#define CAMERA_CALIBRATION_DEFAULT_DELAY 1000




class CameraCalibration {
public:
	CameraCalibration();
	CameraCalibration(const string& filename, bool loadExtrinsics=false, bool loadReprojErrs=false, bool loadImagePoints=false );

	virtual ~CameraCalibration();


	void handleCalibrateCamera(vector<string> calibrationCommandString);
	void loadCameraParams( const string& filename, bool loadExtrinsics, bool loadReprojErrs, bool loadImagePoints );
	void saveCameraParams( const string& filename);
	Mat getCameraMatrix();
	Mat getCameraMatrixClone();
	Mat getDistCoeffs();
	Mat getDistCoeffsClone();
	float getSquareSize();
	Size getImageSize();


	enum CalibrationCameraType
	{
		CAMERA_TYPE_MONO,
		CAMERA_TYPE_STEREO,
		CAMERA_TYPE_KINECT,
		NUMBER_OF_CAMERA_TYPES


	};

	enum CalibrationStates
	{
		CALIBRATION_STATE_INIT,
		CALIBRATION_STATE_SHOW_IMAGE_BEFORE_CAPTURE,
		CALIBRATION_STATE_IMAGE_CAPTURE,
		CALIBRATION_STATE_RUN,
		CALIBRATION_STATE_SAVE,
		CALIBRATION_STATE_ABORT,
		CALIBRATION_STATE_COMPLETE,
		CALIBRATION_STATE_DONE,
		NUMBER_OF_CALIBRATIONS_STATES


	};


	enum CameraLocation
	{
		CAMERA_LOCATION_LEFT =0,
		CAMERA_LOCATION_RIGHT=1,
		CAMERA_LOCATION_DEPTH,
		NUMBER_OF_CAMERA_LOCATIONS


	};

	enum CameraRemapAxisTypes
	{
		CAMERA_REMAP_AXIS_X =0,
		CAMERA_REMAP_AXIS_Y=1,
		NUMBER_OF_CAMERA_REMAP_AXES


	};


	struct CalibrationConfigStruct
	{
		CalibrationCameraType e_calibrationCameraType;
		Size boardSize;
		Size imageSize;
		float squareSize;
		float aspectRatio;
		int flags;
		int delay;

		int cameraId;
		int leftCameraId;
		int rightCameraId;

		bool writeExtrinsics;
		bool writePoints;
		string outputFilename;


	};





protected:

	int parseCalibrationConfigCommandVector(struct CalibrationConfigStruct & struct_calibrationConfig, vector<string> & s_commandLineArguments);
	bool loadCalibrationConfigFile(vector<string> &commandArgs, string filename);
	bool runAndSave(const string& outputFilename, const vector<vector<Point2f> >& imagePoints, Size imageSize, Size boardSize, float squareSize,  float aspectRatio, int flags, Mat& cameraMatrix, Mat& distCoeffs, bool writeExtrinsics, bool writePoints, VideoCapture & capture, Mat image );
	bool displayAndAcceptRemappedImages(VideoCapture & capture, Mat image, Size imageSize,Mat cameraMatrix, Mat distCoeffs);
	bool runCalibration(vector<vector<Point2f> > imagePoints, Size imageSize, Size boardSize, float squareSize, float aspectRatio, int flags, Mat& cameraMatrix, Mat& distCoeffs, vector<Mat>& rvecs, vector<Mat>& tvecs, vector<float>& reprojErrs, double& totalAvgErr);
	void calcChessboardCorners(Size boardSize, float squareSize, int numberOfViews, vector<vector<Point3f> >& corners);
	double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints, const vector<vector<Point2f> >& imagePoints, const vector<Mat>& rvecs, const vector<Mat>& tvecs, const Mat& cameraMatrix, const Mat& distCoeffs, vector<float>& perViewErrors );
	int handleMonoCalibration(struct CalibrationConfigStruct struct_calibrationConfig);



    Size  imageSize;
    Size  boardSize;
    float squareSize;
    float aspectRatio;
    int  flags;

    Mat cameraMatrix;
    Mat distCoeffs;

    vector<Mat> rvecs;
    vector<Mat> tvecs;
    vector<float> reprojErrs;
    vector<vector<Point2f> > imagePoints;
    double  totalAvgErr;



};

#endif /* CAMERACALIBRATION_H_ */
