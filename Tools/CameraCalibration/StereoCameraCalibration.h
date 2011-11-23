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
 * StereoCameraCalibration.h
 *
 *  Created on: Jun 29, 2011
 *      Author: jlclemon
 */

#ifndef STEREOCAMERACALIBRATION_H_
#define STEREOCAMERACALIBRATION_H_

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

#define STEREO_CAMERA_CALIBRATION_ESCAPE_KEY_VALUE 27
#define STEREO_CAMERA_CALIBRATION_RETURN_KEY_VALUE 10
#define STEREO_CAMERA_CALIBRATION_DEFAULT_CAMERA_ID 0
#define STEREO_CAMERA_CALIBRATION_DEFAULT_LEFT_CAMERA_ID 0
#define STEREO_CAMERA_CALIBRATION_DEFAULT_RIGHT_CAMERA_ID 1
#define STEREO_CAMERA_CALIBRATION_DEFAULT_BOARD_HEIGHT 6
#define STEREO_CAMERA_CALIBRATION_DEFAULT_BOARD_WIDTH 8
#define STEREO_CAMERA_CALIBRATION_DEFAULT_FLAGS 0
#define STEREO_CAMERA_CALIBRATION_DEFAULT_ASPECT_RATIO 0.0f
#define STEREO_CAMERA_CALIBRATION_DEFAULT_SQUARE_SIZE 2.5f
#define STEREO_CAMERA_CALIBRATION_DEFAULT_OUTPUT_FILENAME "./stereoCameraCalibration.yml"
#define STEREO_CAMERA_CALIBRATION_DEFAULT_DELAY 1000


class StereoCameraCalibration {
public:
	StereoCameraCalibration();
	StereoCameraCalibration(const string& filename, bool loadImagePoints=false );

	virtual ~StereoCameraCalibration();
	bool getValidCalibration();
	void handleCalibrateCamera(vector<string> calibrationCommandString);
	void loadCameraParams( const string& filename, bool loadImagePoints= false );
	void saveCameraParams( const string& filename, bool saveImagePoints=false);
	Mat getCameraMatrix(int index);
	Mat getCameraMatrixClone(int index);
	Mat getDistCoeffs(int index);
	Mat getDistCoeffsClone(int index);
	Mat getR();
	Mat getT();
	Mat getRClone();
	Mat getTClone();
	Mat getRx(int index);
	Mat getPx(int index);
	Mat getQ();
	Mat getRxClone(int index);
	Mat getPxClone(int index);
	Mat getQClone();
	Rect getValidRoi(int index);

	double getRmsErr();
	float getTotalAvgErr();


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




	friend ostream & operator<<(ostream& stream, const StereoCameraCalibration calibration);

protected:





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
	void calcChessboardCorners(Size boardSize, float squareSize, int numberOfViews, vector<vector<Point3f> >& corners);
	void loadStereoCameraParams( const string& filename,
	                       Size & imageSize, Size & boardSize,
	                       float & squareSize, float & aspectRatio, int & flags,
	                       Mat cameraMatrix[2], Mat distCoeffs[2],
	                       Mat& R, Mat& T,Mat& R1,
	                       Mat& P1, Mat& R2, Mat& P2,Mat& Q,Rect validRoi[2],
	                       double& rmsErr, bool loadImagePoints,
	                       vector<vector<Point2f> > imagePoints[2],
	                       double & totalAvgErr );

	void saveStereoCameraParams( const string& filename,
	                       Size imageSize, Size boardSize,
	                       float squareSize, float aspectRatio, int flags,
	                       const Mat cameraMatrix[2], const Mat distCoeffs[2],
	                       const Mat& R, const Mat& T,const Mat& R1,
	                       const Mat& P1,const Mat& R2,const Mat& P2,const Mat& Q,
	                       const Rect validRoi[2], const double& rmsErr,
	                       const vector<vector<Point2f> > imagePoints[2],
	                       double totalAvgErr );


	bool loadCalibrationConfigFile(vector<string> &commandArgs, string filename);
	int parseCalibrationConfigCommandVector(struct StereoCameraCalibration::CalibrationConfigStruct & struct_calibrationConfig, vector<string> & s_commandLineArguments);
	//Stereo Calibration prototype
	int handleStereoCalibration(struct StereoCameraCalibration::CalibrationConfigStruct struct_calibrationConfig);




	bool runStereoCalibration(vector<vector<Point2f> > imagePoints[2],
	        Size imageSize, Size boardSize,
	        float squareSize, float aspectRatio,
	        int flags, Mat cameraMatrix[2], Mat distCoeffs[2],
	        Mat& R, Mat& T, Mat& R1, Mat& P1, Mat& R2, Mat& P2, Mat& Q ,
	        Rect validRoi[2],
	        double & rmsErr,
	        double& totalAvgErr);

	double computeStereoReprojectionErrors(
	        const vector<vector<Point3f> >& objectPoints,
	        const vector<vector<Point2f> > imagePoints[2],
	        const Mat F,
	        const Mat cameraMatrix[2], const Mat distCoeffs[2]);



	int displayRemappedStereoImages(VideoCapture& leftCameraCapture,VideoCapture& rightCameraCapture, const Rect validRoi[2],const Size& imageSize,const Mat rmap[2][2]);





	Size imageSize;
	Size boardSize;
	float squareSize;
	float aspectRatio;
	int flags;
	Mat cameraMatrix[2];
	Mat distCoeffs[2];
    Mat R;
    Mat T;
    Mat R1;
    Mat P1;
    Mat R2;
    Mat P2;
    Mat Q;
    Rect validRoi[2];
    double rmsErr;
    vector<vector<Point2f> > imagePoints[2];
    double totalAvgErr;





};


#endif /* STEREOCAMERACALIBRATION_H_ */
