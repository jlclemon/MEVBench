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
 * StereoCameraCalibration.cpp
 *
 *  Created on: Jun 29, 2011
 *      Author: jlclemon
 */

#include "StereoCameraCalibration.h"
#ifndef WIN32
	#ifndef SLEEP_DEFINED
		#define SLEEP_DEFINED
		#define Sleep(x) {usleep(x*1000);}
	#endif
#endif


StereoCameraCalibration::StereoCameraCalibration() {
	// TODO Auto-generated constructor stub

}

StereoCameraCalibration::StereoCameraCalibration(const string& filename, bool loadImagePoints ) {
	// TODO Auto-generated constructor stub


	this->loadCameraParams(filename,loadImagePoints);

}



StereoCameraCalibration::~StereoCameraCalibration() {
	// TODO Auto-generated destructor stub
}




bool StereoCameraCalibration::loadCalibrationConfigFile(vector<string> &commandArgs, string filename)
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


int StereoCameraCalibration::parseCalibrationConfigCommandVector(CalibrationConfigStruct & struct_calibrationConfig, vector<string> & s_commandLineArguments)
{

	int argc = s_commandLineArguments.size();


	//Setup the default configuration values in struct
	struct_calibrationConfig.e_calibrationCameraType = CAMERA_TYPE_STEREO;
	struct_calibrationConfig.cameraId = STEREO_CAMERA_CALIBRATION_DEFAULT_CAMERA_ID;
	struct_calibrationConfig.leftCameraId = STEREO_CAMERA_CALIBRATION_DEFAULT_LEFT_CAMERA_ID;
	struct_calibrationConfig.rightCameraId = STEREO_CAMERA_CALIBRATION_DEFAULT_RIGHT_CAMERA_ID;

	struct_calibrationConfig.boardSize.height = STEREO_CAMERA_CALIBRATION_DEFAULT_BOARD_HEIGHT;
	struct_calibrationConfig.boardSize.width = STEREO_CAMERA_CALIBRATION_DEFAULT_BOARD_WIDTH;
	struct_calibrationConfig.flags = STEREO_CAMERA_CALIBRATION_DEFAULT_FLAGS;
	struct_calibrationConfig.aspectRatio = STEREO_CAMERA_CALIBRATION_DEFAULT_ASPECT_RATIO;
	struct_calibrationConfig.squareSize = STEREO_CAMERA_CALIBRATION_DEFAULT_SQUARE_SIZE;
	struct_calibrationConfig.outputFilename = STEREO_CAMERA_CALIBRATION_DEFAULT_OUTPUT_FILENAME;
	struct_calibrationConfig.delay = STEREO_CAMERA_CALIBRATION_DEFAULT_DELAY;




	//List out the command line argurments
	cout << argc <<" command Line Parameters:\n"
			<< "\tExecutable: " << s_commandLineArguments[0] << endl;
	for(int i=1 ;i < argc ;i++)
	{
		cout << "\t Arg " << i <<": "<<s_commandLineArguments[i];


		//Handle the camera Type, stereo, mono, kinect?
		if(strcmp(s_commandLineArguments[i].c_str(),"-cameraType")==0)
		{
			cout << " " << s_commandLineArguments[i+1].c_str() << endl;
			cout << "\t\tCamera Type: ";
			if(strcmp(s_commandLineArguments[++i].c_str(),"mono")==0)
			{
				struct_calibrationConfig.e_calibrationCameraType = CAMERA_TYPE_MONO;
				cout << "CAMERA_TYPE_MONO" << endl;
			}
			else
			{
				struct_calibrationConfig.e_calibrationCameraType = CAMERA_TYPE_STEREO;
				cout << "CAMERA_TYPE_STEREO" << endl;

			}


		}



		if(strcmp(s_commandLineArguments[i].c_str(),"-cameraId")==0)
		{

			if(sscanf( s_commandLineArguments[++i].c_str(), "%u", &struct_calibrationConfig.cameraId ) != 1)
			{
				cout << "Camera Id value invalid......exiting calibration" << endl;
				Sleep(5);
				return 1;
			}


		}


		if(strcmp(s_commandLineArguments[i].c_str(),"-leftCameraId")==0)
		{

			if(sscanf( s_commandLineArguments[++i].c_str(), "%u", &struct_calibrationConfig.leftCameraId ) != 1)
			{
				cout << "Left Camera Id value invalid......exiting calibration" << endl;
				Sleep(5);
				return 1;
			}


		}

		if(strcmp(s_commandLineArguments[i].c_str(),"-rightCameraId")==0)
		{

			if(sscanf( s_commandLineArguments[++i].c_str(), "%u", &struct_calibrationConfig.rightCameraId ) != 1)
			{
				cout << "Right Camera Id value invalid......exiting calibration" << endl;
				Sleep(5);
				return 1;
			}


		}


		if(strcmp(s_commandLineArguments[i].c_str(),"-boardHeight")==0)
		{

			if(sscanf( s_commandLineArguments[++i].c_str(), "%u", &struct_calibrationConfig.boardSize.height ) != 1)
			{
				cout << "Board height value invalid......exiting calibration" << endl;
				Sleep(5);
				return 1;
			}


		}

		if(strcmp(s_commandLineArguments[i].c_str(),"-boardWidth")==0)
		{

			if(sscanf( s_commandLineArguments[++i].c_str(), "%u", &struct_calibrationConfig.boardSize.width ) != 1)
			{
				cout << "Board width value invalid......exiting calibration" << endl;
				Sleep(5);
				return 1;
			}


		}

		if(strcmp(s_commandLineArguments[i].c_str(),"-outputFile")==0)
		{
			struct_calibrationConfig.outputFilename = s_commandLineArguments[++i];


		}

		if(strcmp(s_commandLineArguments[i].c_str(),"-squareSize")==0)
		{

			if(sscanf( s_commandLineArguments[++i].c_str(), "%f", &struct_calibrationConfig.squareSize ) != 1)
			{
				cout << "Square size value invalid......exiting calibration" << endl;
				Sleep(5);
				return 1;
			}


		}

		if(strcmp(s_commandLineArguments[i].c_str(),"-aspectRatio")==0)
		{

			if(sscanf( s_commandLineArguments[++i].c_str(), "%f", &struct_calibrationConfig.aspectRatio ) != 1)
			{
				cout << "Aspect Ratio value invalid......exiting calibration" << endl;
				Sleep(5);
				return 1;
			}
			struct_calibrationConfig.flags |= CV_CALIB_FIX_ASPECT_RATIO;

		}

        if( strcmp( s_commandLineArguments[i].c_str(), "-zeroTangentDist" ) == 0 )
        {
        	struct_calibrationConfig.flags |= CV_CALIB_ZERO_TANGENT_DIST;
        }
        if( strcmp( s_commandLineArguments[i].c_str(), "-fixPrinciplePoint" ) == 0 )
        {
        	struct_calibrationConfig.flags |= CV_CALIB_FIX_PRINCIPAL_POINT;
        }



		cout <<endl;






	}

	if(struct_calibrationConfig.e_calibrationCameraType != CAMERA_TYPE_STEREO)
	{
		cout << "ERROR: This class is only for stereo calbiration" << endl;
		exit(0);


	}


	return 0;
}


//**************************************************************************************
//
//handleMonoCalibration function:  Calibratation for a single camera)
//	Arugments:  struct_calibrationConfig - calibration information from the command line
//
//	Return:  0 if successful, non zero otherwise
//	Author:  Jason Clemons
//	Date Created: 2-22-2011
//
//
//***************************************************************************************

int StereoCameraCalibration::handleStereoCalibration(struct StereoCameraCalibration::CalibrationConfigStruct struct_calibrationConfig)
{
	//The capture for the image
	VideoCapture leftCameraCapture;
	VideoCapture rightCameraCapture;
	//Used to signify the capture portion is done


	int quitCapture = 0;


	int lastKey = -1;
	int key = -1;

	double totalAvgErr;
	double rmsErr;

	CalibrationStates currentCalibrationState = CALIBRATION_STATE_INIT;


	//A vector to hold the points found during calibration
	vector< vector<Point2f> >calibPoints[CAMERA_LOCATION_RIGHT+1];

	//How many frames we have
	int frameCount = 0;

	clock_t prevTimeStamp;


	Mat currentLeftFrame;
	Mat currentRightFrame;
	Mat leftCaptureFrame;
	Mat rightCaptureFrame;

	Mat flippedLeftFrame;
	Mat flippedRightFrame;



	Mat cameraMatrix[2];
	Mat distCoeffs[2];
    Mat R, T, R1, R2, P1, P2, Q;
    Rect validRoi[2];
    Mat rmap[2][2];



	while(currentCalibrationState != CALIBRATION_STATE_DONE)
	{

		key = waitKey(33) & 0xff;
		switch(currentCalibrationState)
		{
			case CALIBRATION_STATE_INIT:
			{


				//Open the captures
				leftCameraCapture.open(struct_calibrationConfig.leftCameraId);
				rightCameraCapture.open(struct_calibrationConfig.rightCameraId);

				if(!(leftCameraCapture.set(CV_CAP_PROP_FPS, 30.0)))
				{
					cout << "Left frame rate set failed" << endl;
				}
				if(!(rightCameraCapture.set(CV_CAP_PROP_FPS, 30.0)))
				{
					cout << "Right frame rate set failed" << endl;


				}



				if(!(leftCameraCapture.set(CV_CAP_PROP_FRAME_WIDTH, 640)))
				{
					cout << "Left frame width set failed" << endl;
				}
				if(!(leftCameraCapture.set(CV_CAP_PROP_FRAME_HEIGHT, 480)))
				{
					cout << "Left frame height set failed" << endl;
				}


				if(!(rightCameraCapture.set(CV_CAP_PROP_FRAME_WIDTH, 640)))
				{
					cout << "Right frame width set failed" << endl;


				}
				if(!(rightCameraCapture.set(CV_CAP_PROP_FRAME_HEIGHT, 480)))
				{
					cout << "Right frame height set failed" << endl;


				}



				//Named window for calibration
				namedWindow("Current Left Calibration Image Raw", 1);
				namedWindow("Current Right Calibration Image Raw", 1);

				//Named window for calibration
				namedWindow("Current Left Calibration Image", 1);
				namedWindow("Current Right Calibration Image", 1);


				cout << "Starting calibration feature point capture." << endl;
				currentCalibrationState = CALIBRATION_STATE_SHOW_IMAGE_BEFORE_CAPTURE;

				break;
			}
			case CALIBRATION_STATE_SHOW_IMAGE_BEFORE_CAPTURE:
			{

				if(leftCameraCapture.isOpened() && rightCameraCapture.isOpened())
				{

					leftCameraCapture >> leftCaptureFrame;
					rightCameraCapture >> rightCaptureFrame;


					leftCaptureFrame.copyTo(currentLeftFrame);


					rightCaptureFrame.copyTo(currentRightFrame);


				}

				//cout << "Left Frame Size" <<currentLeftFrame.rows <<"x" << currentLeftFrame.cols << endl;
				//cout << "Right Frame Size" <<currentRightFrame.rows <<"x" << currentRightFrame.cols << endl;

				if(!currentLeftFrame.data)
				{
					cout << "No Frame Data from Left Camera" << endl;
					return 2;
				}

				if(!currentRightFrame.data)
				{
					cout << "No Frame Data from Right Camera" << endl;
					return 2;
				}



				//currentFrame.copyTo(flippedFrame);

				flip(currentLeftFrame, flippedLeftFrame,1);
				flip(currentRightFrame, flippedRightFrame,1);

				imshow("Current Left Calibration Image Raw", flippedLeftFrame);
				imshow("Current Right Calibration Image Raw", flippedRightFrame);


				if(key == 's' || key == ' ')
				{

					prevTimeStamp = clock();
					currentCalibrationState = CALIBRATION_STATE_IMAGE_CAPTURE;

				}
				if(key == 27)
				{

					currentCalibrationState =CALIBRATION_STATE_ABORT;

				}



				break;
			}
			case CALIBRATION_STATE_IMAGE_CAPTURE:
			{
				if(leftCameraCapture.isOpened() && rightCameraCapture.isOpened())
				{

					leftCameraCapture >> leftCaptureFrame;
					leftCaptureFrame.copyTo(currentLeftFrame);

					rightCameraCapture >> rightCaptureFrame;
					rightCaptureFrame.copyTo(currentRightFrame);


				}

				if(!currentLeftFrame.data)
				{
					cout << "No Frame Data from Left Camera" << endl;
					return 2;
				}

				if(!currentRightFrame.data)
				{
					cout << "No Frame Data from Right Camera" << endl;
					return 2;
				}

				Mat flippedLeftFrame;
				Mat flippedRightFrame;


				//currentFrame.copyTo(flippedFrame);

				flip(currentLeftFrame, flippedLeftFrame,1);
				flip(currentRightFrame, flippedRightFrame,1);

				imshow("Current Left Calibration Image Raw", flippedRightFrame);
				imshow("Current Right Calibration Image Raw", flippedLeftFrame);




				Mat currentFrameGray[CAMERA_LOCATION_RIGHT+1];

				cvtColor(currentLeftFrame,currentFrameGray[CAMERA_LOCATION_LEFT],CV_BGR2GRAY);
				cvtColor(currentRightFrame,currentFrameGray[CAMERA_LOCATION_RIGHT],CV_BGR2GRAY);


				vector<Point2f> currentFramePoints[CAMERA_LOCATION_RIGHT+1];

				bool foundPoints[CAMERA_LOCATION_RIGHT+1];

				//Find the corners of the Chessboard
		        foundPoints[CAMERA_LOCATION_LEFT] = findChessboardCorners( currentFrameGray[CAMERA_LOCATION_LEFT], struct_calibrationConfig.boardSize, currentFramePoints[CAMERA_LOCATION_LEFT], CV_CALIB_CB_ADAPTIVE_THRESH & CV_CALIB_CB_FAST_CHECK & CV_CALIB_CB_NORMALIZE_IMAGE);
		        foundPoints[CAMERA_LOCATION_RIGHT] = findChessboardCorners( currentFrameGray[CAMERA_LOCATION_RIGHT], struct_calibrationConfig.boardSize, currentFramePoints[CAMERA_LOCATION_RIGHT], CV_CALIB_CB_ADAPTIVE_THRESH & CV_CALIB_CB_FAST_CHECK & CV_CALIB_CB_NORMALIZE_IMAGE);



		        if(foundPoints[CAMERA_LOCATION_LEFT] || foundPoints[CAMERA_LOCATION_RIGHT])
		        {
					if(foundPoints[CAMERA_LOCATION_LEFT])
					{


						cornerSubPix( currentFrameGray[CAMERA_LOCATION_LEFT], currentFramePoints[CAMERA_LOCATION_LEFT], Size(5,5), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));


					}
					if(foundPoints[CAMERA_LOCATION_RIGHT])
					{



						cornerSubPix( currentFrameGray[CAMERA_LOCATION_RIGHT], currentFramePoints[CAMERA_LOCATION_RIGHT], Size(5,5), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));


					}


					if(foundPoints[CAMERA_LOCATION_LEFT] && foundPoints[CAMERA_LOCATION_RIGHT])
					{
						if(clock() - prevTimeStamp > struct_calibrationConfig.delay*1e-3*CLOCKS_PER_SEC)
						{


							prevTimeStamp = clock();
							//blink = capture.isOpened();
							bitwise_not(currentLeftFrame, currentLeftFrame);
							bitwise_not(currentRightFrame, currentRightFrame);


							calibPoints[CAMERA_LOCATION_LEFT].push_back(currentFramePoints[CAMERA_LOCATION_LEFT]);
							calibPoints[CAMERA_LOCATION_RIGHT].push_back(currentFramePoints[CAMERA_LOCATION_RIGHT]);
							frameCount++;


						}
					}

					if(foundPoints[CAMERA_LOCATION_LEFT])
					{


						drawChessboardCorners( currentLeftFrame, struct_calibrationConfig.boardSize, Mat(currentFramePoints[CAMERA_LOCATION_LEFT]), foundPoints[CAMERA_LOCATION_LEFT] );


					}
					if(foundPoints[CAMERA_LOCATION_RIGHT])
					{



						drawChessboardCorners( currentRightFrame, struct_calibrationConfig.boardSize, Mat(currentFramePoints[CAMERA_LOCATION_RIGHT]), foundPoints[CAMERA_LOCATION_RIGHT] );


					}



					cout << "Good Frames: " << frameCount << endl;


					imshow("Current Left Calibration Image", currentLeftFrame);
		        	imshow("Current Right Calibration Image", currentRightFrame);


					struct_calibrationConfig.imageSize = currentLeftFrame.size();



		        }
		        else
		        {
		        	imshow("Current Left Calibration Image", currentFrameGray[CAMERA_LOCATION_LEFT]);
		        	imshow("Current Right Calibration Image", currentFrameGray[CAMERA_LOCATION_RIGHT]);
		        }


		        if((key == 'd'  || key == 'c' || key == 'r' || key == ' ') && frameCount >= 15)
		        {

		        	currentCalibrationState =CALIBRATION_STATE_RUN;

		        }
		        if(key==27)
		        {

		        	currentCalibrationState =CALIBRATION_STATE_ABORT;

		        }



				break;
			}
			case CALIBRATION_STATE_RUN:
			{
			    bool ok = runStereoCalibration(calibPoints, struct_calibrationConfig.imageSize, struct_calibrationConfig.boardSize, struct_calibrationConfig.squareSize,
			    		struct_calibrationConfig.aspectRatio, struct_calibrationConfig.flags, cameraMatrix, distCoeffs,
			                   R, T, R1, P1, R2, P2, Q, validRoi, rmsErr, totalAvgErr);
			    printf("%s. avg reprojection error = %.2f\n",
			           ok ? "Calibration succeeded" : "Calibration failed",
			           totalAvgErr);

			    if(ok)
			    {
			    	cout << "Moving to save option." << endl;
			        //Precompute maps for cv::remap()
			        initUndistortRectifyMap(cameraMatrix[CAMERA_LOCATION_LEFT], distCoeffs[CAMERA_LOCATION_LEFT], R1, P1, struct_calibrationConfig.imageSize, CV_16SC2, rmap[CAMERA_LOCATION_LEFT][CAMERA_REMAP_AXIS_X], rmap[CAMERA_LOCATION_LEFT][CAMERA_REMAP_AXIS_Y]);
			        initUndistortRectifyMap(cameraMatrix[CAMERA_LOCATION_RIGHT], distCoeffs[CAMERA_LOCATION_RIGHT], R2, P2, struct_calibrationConfig.imageSize, CV_16SC2, rmap[CAMERA_LOCATION_RIGHT][CAMERA_REMAP_AXIS_X], rmap[CAMERA_LOCATION_RIGHT][CAMERA_REMAP_AXIS_Y]);



			    	currentCalibrationState =CALIBRATION_STATE_SAVE;

			    }
			    else
			    {

			    	cout << "Moving to waiting for image capture." << endl;
			    	currentCalibrationState =CALIBRATION_STATE_SHOW_IMAGE_BEFORE_CAPTURE;

			    }
				break;
			}
			case CALIBRATION_STATE_SAVE:
			{
				key = displayRemappedStereoImages(leftCameraCapture, rightCameraCapture,validRoi,struct_calibrationConfig.imageSize,rmap);

				bool ok = false;

				vector<vector<Point2f> > noPoints[2];
			    if( key == 's' )
			    {
			    	this->imageSize = struct_calibrationConfig.imageSize;

	        		this->boardSize = struct_calibrationConfig.boardSize;
	        		this->squareSize = struct_calibrationConfig.squareSize;
	        		this ->aspectRatio = struct_calibrationConfig.aspectRatio;
	        		this->flags = struct_calibrationConfig.flags;
	        		this->cameraMatrix[0] = cameraMatrix[0].clone();
	        		this->cameraMatrix[1] = cameraMatrix[1].clone();
	        		this->distCoeffs[0] = distCoeffs[0].clone();
	        		this->distCoeffs[1] = distCoeffs[1].clone();
	        		this->R = R.clone();
	        		this->T = T.clone();
	        		this->R1 = R1.clone();
	        		this->P1 = P1.clone();
	        		this->R2 = R2.clone();
	        		this->P2 = P2.clone();
	        		this->Q = Q.clone();
	        		this->validRoi[0] = validRoi[0];
	        		this->validRoi[1] = validRoi[1];
	                this->rmsErr = rmsErr;

	                this->imagePoints[0] = calibPoints[0];
	                this->imagePoints[1] = calibPoints[1];
	                this->totalAvgErr = totalAvgErr;


			    	saveStereoCameraParams( struct_calibrationConfig.outputFilename, struct_calibrationConfig.imageSize,
			        		struct_calibrationConfig.boardSize, struct_calibrationConfig.squareSize, struct_calibrationConfig.aspectRatio,
			        		struct_calibrationConfig.flags, cameraMatrix, distCoeffs,
			                         R, T, R1, P1, R2, P2, Q,validRoi,
			                         rmsErr,
			                         struct_calibrationConfig.writePoints ? calibPoints : noPoints,
			                         totalAvgErr );
			        cout << "Stereo Calibration Data Saved" << endl;
			        currentCalibrationState =CALIBRATION_STATE_COMPLETE;
			    }

			    if(key == 27)
			    {

			    	cout << "Move to abort" << endl;
			    	currentCalibrationState =CALIBRATION_STATE_ABORT;
			    }

				break;
			}
			case CALIBRATION_STATE_ABORT:
				cout << "Calibration Aborted" << endl;
				currentCalibrationState =CALIBRATION_STATE_COMPLETE;
				break;
			case CALIBRATION_STATE_COMPLETE:

				cout << "Calibration Completed" << endl;
				currentCalibrationState =CALIBRATION_STATE_DONE;
				break;
			default:
				break;

		}//switch

/*
		//We will now try to calibrate
		if(quitCapture ==1 && frameCount >= 6)
		{
			string outputFilename;
			Mat cameraMatrix;
			Mat distCoeffs;
			bool writeExtrinsics;
			bool writePoints;
			Mat image;


			bool saveResult = true;
			saveResult = runAndSave(struct_calibrationConfig.outputFilename, calibPoints, struct_calibrationConfig.imageSize,
					struct_calibrationConfig.boardSize, struct_calibrationConfig.squareSize, struct_calibrationConfig.aspectRatio, struct_calibrationConfig.flags,
					cameraMatrix, distCoeffs ,struct_calibrationConfig.writeExtrinsics,struct_calibrationConfig.writePoints, capture,image );

			if(saveResult)
			{
				cout << "Calibration Information saved " << endl;


			}
			else
			{
				cout << "Calibration rejected.  Rerunning calibration" << endl;
				quitCapture = 0;
			}


		}*/

	}//while









	return 0;
}

double StereoCameraCalibration::computeStereoReprojectionErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> > imagePoints[2],
        const Mat F,
        const Mat cameraMatrix[2], const Mat distCoeffs[2])
{


	int i, j, k;
	int nImages = imagePoints[CAMERA_LOCATION_LEFT].size();
	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	for( i = 0; i < nImages; i++ )
	{
		int npt = (int)imagePoints[0][i].size();
		Mat imgpt[2];
		for( k = 0; k < 2; k++ )
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
		}
		for( j = 0; j < npt; j++ )
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
								imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
						   fabs(imagePoints[1][i][j].x*lines[0][j][0] +
								imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	cout << "average reprojection err = " <<  err/npoints << endl;
	return err/npoints;

}
bool StereoCameraCalibration::runStereoCalibration(vector<vector<Point2f> > imagePoints[2],
        Size imageSize, Size boardSize,
        float squareSize, float aspectRatio,
        int flags, Mat cameraMatrix[2], Mat distCoeffs[2],
        Mat& R, Mat& T, Mat& R1, Mat& P1, Mat& R2, Mat& P2, Mat& Q ,
        Rect validRoi[2],
        double & rmsErr,
        double& totalAvgErr)
{
	distCoeffs[CAMERA_LOCATION_LEFT] = Mat::zeros(8, 1, CV_64F);
	distCoeffs[CAMERA_LOCATION_RIGHT] = Mat::zeros(8, 1, CV_64F);
	cameraMatrix[CAMERA_LOCATION_LEFT] = Mat::eye(3, 3, CV_64F);
	cameraMatrix[CAMERA_LOCATION_RIGHT] = Mat::eye(3, 3, CV_64F);
	Mat E, F;

    vector<vector<Point3f> > objectPoints(0);
    calcChessboardCorners(boardSize,  squareSize,imagePoints[CAMERA_LOCATION_LEFT].size(), objectPoints);


	rmsErr = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
					cameraMatrix[CAMERA_LOCATION_LEFT], distCoeffs[CAMERA_LOCATION_LEFT],
					cameraMatrix[CAMERA_LOCATION_RIGHT], distCoeffs[CAMERA_LOCATION_RIGHT],
					imageSize, R, T, E, F,
					TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
					CV_CALIB_FIX_ASPECT_RATIO +
					CV_CALIB_ZERO_TANGENT_DIST +
					CV_CALIB_SAME_FOCAL_LENGTH +
					CV_CALIB_RATIONAL_MODEL +
					CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
	cout << "done with RMS error=" << rmsErr << endl;


    totalAvgErr = computeStereoReprojectionErrors(objectPoints, imagePoints,
                F, cameraMatrix, distCoeffs);


#ifdef OPENCV_2_2
    stereoRectify(cameraMatrix[CAMERA_LOCATION_LEFT], distCoeffs[CAMERA_LOCATION_LEFT],
                  cameraMatrix[CAMERA_LOCATION_RIGHT], distCoeffs[CAMERA_LOCATION_RIGHT],
                  imageSize, R, T, R1, R2, P1, P2, Q,//CALIB_ZERO_DISPARITY,
                  1, imageSize, &validRoi[CAMERA_LOCATION_LEFT], &validRoi[CAMERA_LOCATION_RIGHT]);
#else

    stereoRectify(cameraMatrix[CAMERA_LOCATION_LEFT], distCoeffs[CAMERA_LOCATION_LEFT],
                  cameraMatrix[CAMERA_LOCATION_RIGHT], distCoeffs[CAMERA_LOCATION_RIGHT],
                  imageSize, R, T, R1, R2, P1, P2, Q,CALIB_ZERO_DISPARITY,
                  1, imageSize, &validRoi[CAMERA_LOCATION_LEFT], &validRoi[CAMERA_LOCATION_RIGHT]);




#endif


    return true;
}


void StereoCameraCalibration::saveStereoCameraParams( const string& filename,
                       Size imageSize, Size boardSize,
                       float squareSize, float aspectRatio, int flags,
                       const Mat cameraMatrix[2], const Mat distCoeffs[2],
                       const Mat& R, const Mat& T,const Mat& R1,
                       const Mat& P1,const Mat& R2,const Mat& P2,const Mat& Q,
                       const Rect validRoi[2], const double& rmsErr,
                       const vector<vector<Point2f> > imagePoints[2],
                       double totalAvgErr )
{
    FileStorage fs( filename, FileStorage::WRITE );

    time_t t;
    time( &t );
    struct tm *t2 = localtime( &t );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_time" << buf;


    fs << "nframes" << (int)imagePoints[CAMERA_LOCATION_LEFT].size();
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "square_size" << squareSize;

    if( flags & CV_CALIB_FIX_ASPECT_RATIO )
        fs << "aspectRatio" << aspectRatio;

    if( flags != 0 )
    {
        sprintf( buf, "flags: %s%s%s%s",
            flags & CV_CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
            flags & CV_CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
            flags & CV_CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
            flags & CV_CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "" );
        cvWriteComment( *fs, buf, 0 );
    }

    fs << "flags" << flags;
    fs << "M1" << cameraMatrix[CAMERA_LOCATION_LEFT] << "D1" << distCoeffs[CAMERA_LOCATION_LEFT] <<
        "M2" << cameraMatrix[CAMERA_LOCATION_RIGHT] << "D2" << distCoeffs[CAMERA_LOCATION_RIGHT];
    fs << "Valid_ROI_Left" << validRoi[CAMERA_LOCATION_LEFT];
    fs << "Valid_ROI_Right" << validRoi[CAMERA_LOCATION_RIGHT];
    fs << "avg_reprojection_error" << totalAvgErr;

    fs << "rmsErr" << rmsErr;

    fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
    if( !imagePoints[CAMERA_LOCATION_LEFT].empty() )
    {
        Mat imagePtMat((int)imagePoints[CAMERA_LOCATION_LEFT].size(), imagePoints[CAMERA_LOCATION_LEFT][0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints[CAMERA_LOCATION_LEFT].size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[CAMERA_LOCATION_LEFT][i]);
            imgpti.copyTo(r);
        }
        fs << "image_points_left" << imagePtMat;
    }
    if( !imagePoints[CAMERA_LOCATION_RIGHT].empty() )
    {
        Mat imagePtMat((int)imagePoints[CAMERA_LOCATION_RIGHT].size(), imagePoints[CAMERA_LOCATION_RIGHT][0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints[CAMERA_LOCATION_RIGHT].size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[CAMERA_LOCATION_RIGHT][i]);
            imgpti.copyTo(r);
        }
        fs << "image_points_rightt" << imagePtMat;
    }


    fs.release();
}


void StereoCameraCalibration::loadStereoCameraParams( const string& filename,
                       Size & imageSize, Size & boardSize,
                       float & squareSize, float & aspectRatio, int & flags,
                       Mat cameraMatrix[2], Mat distCoeffs[2],
                       Mat& R, Mat& T,Mat& R1,
                       Mat& P1, Mat& R2, Mat& P2,Mat& Q,Rect validRoi[2],
                       double& rmsErr, bool loadImagePoints,
                       vector<vector<Point2f> > imagePoints[2],
                       double & totalAvgErr )
{
    FileStorage fs( filename, FileStorage::READ );

    time_t t;
    time( &t );
    struct tm *t2 = localtime( &t );
    string calibrationTimeString;

    (string)fs["calibration_time"]=calibrationTimeString;

    imageSize.width = (int)fs["image_width"];
    imageSize.height = (int) fs["image_height"];
    boardSize.width = (int) fs["board_width"];
	boardSize.height = (int)fs["board_height"];
	squareSize = (float)fs["square_size"];

	flags = (int)fs["flags"];

    if( flags & CV_CALIB_FIX_ASPECT_RATIO )
    	aspectRatio = (float)fs["aspectRatio"];

    fs["M1"] >> cameraMatrix[CAMERA_LOCATION_LEFT];
    fs["D1"] >> distCoeffs[CAMERA_LOCATION_LEFT];
    fs["M2"] >> cameraMatrix[CAMERA_LOCATION_RIGHT];
    fs["D2"] >> distCoeffs[CAMERA_LOCATION_RIGHT];
    fs["R"] >> R;
    fs["T"] >> T;
    fs["R1"]>> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["Q"] >> Q;

 //   fs["Valid_ROI_Left"] >> validRoi[CAMERA_LOCATION_LEFT];
 //   fs["Valid_ROI_Right"] >> validRoi[CAMERA_LOCATION_RIGHT];



    totalAvgErr = (double)fs["avg_reprojection_error"];
    rmsErr = (double)fs["rmsErr"];

    if( loadImagePoints )
    {
        Mat imagePtMat;
        fs["image_points_left"] >> imagePtMat;
        for( int i = 0; i < (int)imagePoints[CAMERA_LOCATION_LEFT].size(); i++ )
        {
        	vector<Point2f> currentPoints = Mat_<Point2f> (imagePtMat.row(i));
        	imagePoints[CAMERA_LOCATION_LEFT].push_back(currentPoints);
        }

    }
    if( loadImagePoints )
    {
        Mat imagePtMat;
        fs["image_points_right"] >> imagePtMat;
        for( int i = 0; i < (int)imagePoints[CAMERA_LOCATION_RIGHT].size(); i++ )
        {
        	vector<Point2f> currentPoints = Mat_<Point2f> (imagePtMat.row(i));
        	imagePoints[CAMERA_LOCATION_RIGHT].push_back(currentPoints);
        }

    }



}


int StereoCameraCalibration::displayRemappedStereoImages(VideoCapture& leftCameraCapture,VideoCapture& rightCameraCapture, const Rect validRoi[2],const Size& imageSize,const Mat rmap[2][2])
{
	Mat leftCaptureFrame, currentLeftFrame, rightCaptureFrame, currentRightFrame;
	Mat rightFrameRemapped, leftFrameRemapped;
	Mat bothImages;
	namedWindow("Rectified",1);

	leftCameraCapture >> leftCaptureFrame;
	rightCameraCapture >> rightCaptureFrame;


	leftCaptureFrame.copyTo(currentLeftFrame);


	rightCaptureFrame.copyTo(currentRightFrame);


    bothImages.create(imageSize.height, imageSize.width *2, CV_8UC3);

    remap(currentLeftFrame, leftFrameRemapped, rmap[CAMERA_LOCATION_LEFT][CAMERA_REMAP_AXIS_X], rmap[CAMERA_LOCATION_LEFT][CAMERA_REMAP_AXIS_Y], CV_INTER_LINEAR);



    Mat canvasPart = bothImages(Rect(imageSize.width*CAMERA_LOCATION_LEFT, 0, imageSize.width, imageSize.height));

    leftFrameRemapped.copyTo(canvasPart);
    //resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
	Rect vroi(validRoi[CAMERA_LOCATION_LEFT].x, validRoi[CAMERA_LOCATION_LEFT].y,
			  validRoi[CAMERA_LOCATION_LEFT].width, validRoi[CAMERA_LOCATION_LEFT].height);
	rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);


    remap(currentRightFrame, rightFrameRemapped, rmap[CAMERA_LOCATION_RIGHT][CAMERA_REMAP_AXIS_X], rmap[CAMERA_LOCATION_RIGHT][CAMERA_REMAP_AXIS_Y], CV_INTER_LINEAR);

	canvasPart = bothImages(Rect(imageSize.width*CAMERA_LOCATION_RIGHT, 0, imageSize.width, imageSize.height));

	rightFrameRemapped.copyTo(canvasPart);
	//resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
	Rect vroi2(validRoi[CAMERA_LOCATION_RIGHT].x, validRoi[CAMERA_LOCATION_RIGHT].y,
			  validRoi[CAMERA_LOCATION_RIGHT].width, validRoi[CAMERA_LOCATION_RIGHT].height);
	rectangle(canvasPart, vroi2, Scalar(0,0,255), 3, 8);




    for( int j = 0; j < bothImages.cols; j += 16 )
    {
        line(bothImages, Point(j, 0), Point(j, bothImages.rows), Scalar(0, 255, 0), 1, 8);
    }
    imshow("Rectified", bothImages);


	return waitKey(10) & 0xFF;


}


Mat StereoCameraCalibration::getCameraMatrix(int index)
{


	return this->cameraMatrix[index];
}
Mat StereoCameraCalibration::getCameraMatrixClone(int index)
{


	return this->cameraMatrix[index].clone();
}


Mat StereoCameraCalibration::getDistCoeffs(int index)
{


	return this->distCoeffs[index];
}

Mat StereoCameraCalibration::getDistCoeffsClone(int index)
{


	return this->distCoeffs[index].clone();
}


Mat StereoCameraCalibration::getR()
{


	return this->R;
}





Mat StereoCameraCalibration::getT()
{


	return this->T;
}

Mat StereoCameraCalibration::getRClone()
{


	return this->R.clone();
}





Mat StereoCameraCalibration::getTClone()
{


	return this->T.clone();
}


Mat StereoCameraCalibration::getRx(int index)
{

	if(index == 0)
	{
		return this->R1;

	}
	else
	{

		return this->R2;
	}


}





Mat StereoCameraCalibration::getPx(int index)
{


	if(index == 0)
	{
		return this->P1;

	}
	else
	{

		return this->P2;
	}

}

Mat StereoCameraCalibration::getQ()
{

	return this->Q;


}

Mat StereoCameraCalibration::getRxClone(int index)
{

	if(index == 0)
	{
		return this->R1.clone();

	}
	else
	{

		return this->R2.clone();
	}


}





Mat StereoCameraCalibration::getPxClone(int index)
{


	if(index == 0)
	{
		return this->P1.clone();

	}
	else
	{

		return this->P2.clone();
	}

}

Mat StereoCameraCalibration::getQClone()
{

	return this->Q.clone();


}

Rect StereoCameraCalibration::getValidRoi(int index)
{

	return this->validRoi[index];

}

double StereoCameraCalibration::getRmsErr()
{
	return this->rmsErr;

}
float StereoCameraCalibration::getTotalAvgErr()
{
	return this->totalAvgErr;

}

bool StereoCameraCalibration::getValidCalibration()
{
	bool validCalibration = true;
	if(this->Q.empty() || this->R.empty() || this->T.empty())
	{
		validCalibration = false;

	}

	return validCalibration;

}


void StereoCameraCalibration::calcChessboardCorners(Size boardSize, float squareSize, int numberOfViews, vector<vector<Point3f> >& corners)
{
	corners.resize(numberOfViews);
    for (int k = 0; k< numberOfViews; k++)
    {
        corners[k].resize(0);
    	for( int i = 0; i < boardSize.height; i++ )
		{
			for( int j = 0; j < boardSize.width; j++ )
			{
				corners[k].push_back(Point3f(float(j*squareSize),
										  float(i*squareSize), 0));
			}
		}
    }
}



void StereoCameraCalibration::loadCameraParams( const string& filename, bool loadImagePoints)
{
	loadStereoCameraParams( filename,
	                       this->imageSize, this->boardSize,
	                       this->squareSize,this->aspectRatio,this->flags,
	                       this->cameraMatrix, this->distCoeffs,
	                       this->R, this->T,this->R1,
	                       this->P1, this->R2, this->P2,this->Q,this->validRoi,
	                       this->rmsErr,loadImagePoints,
	                       this->imagePoints,
	                       this->totalAvgErr );




}

void StereoCameraCalibration::saveCameraParams( const string& filename, bool saveImagePoints)
{
	saveStereoCameraParams( filename,
							this->imageSize, this->boardSize,
							this->squareSize, this->aspectRatio, this->flags,
							this->cameraMatrix, this->distCoeffs,
							this->R, this->T,this->R1,
							this->P1,this->R2,this->P2,this->Q,
							this->validRoi, this->rmsErr,
							this->imagePoints,
							this->totalAvgErr );



}



ostream & operator<<(ostream& stream, const StereoCameraCalibration calibration)
{

	stream <<"Calibration Total Avg Err: "<<calibration.totalAvgErr <<", Valid Rect 0: ("<< calibration.validRoi[0].x <<","<<calibration.validRoi[0].y <<","<<calibration.validRoi[0].height <<","<<calibration.validRoi[0].width <<"), Valid Rect 1: ("<< calibration.validRoi[1].x <<","<<calibration.validRoi[1].y <<","<<calibration.validRoi[1].height <<","<<calibration.validRoi[1].width <<")"<< endl;
	return stream;
}

