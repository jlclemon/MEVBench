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
 * CameraCalibration.cpp
 *
 *  Created on: May 26, 2011
 *      Author: jlclemon
 */

#include "CameraCalibration.h"
#ifndef WIN32
	#ifndef SLEEP_DEFINED
		#define SLEEP_DEFINED
		#define Sleep(x) {usleep(x*1000);}
	#endif
#endif




CameraCalibration::CameraCalibration() {
	// TODO Auto-generated constructor stub

}


CameraCalibration::CameraCalibration(const string& filename, bool loadExtrinsics, bool loadReprojErrs, bool loadImagePoints )
{

	this->loadCameraParams(filename,loadExtrinsics,loadReprojErrs,loadImagePoints);

}

CameraCalibration::~CameraCalibration() {
	// TODO Auto-generated destructor stub
}

void CameraCalibration::handleCalibrateCamera(vector<string> calibrationCommandStrings)
{

	struct CalibrationConfigStruct struct_calibrationConfig;
	if((calibrationCommandStrings[1].compare("-configFile") ==0) || (calibrationCommandStrings[1].compare("-ConfigFile") ==0))
	{
		loadCalibrationConfigFile(calibrationCommandStrings, calibrationCommandStrings[2]);



	}


	parseCalibrationConfigCommandVector(struct_calibrationConfig,  calibrationCommandStrings);

	//example command:  CameraCalibrationOpenCv -cameraType mono -boardHeight 6 -boardWidth 8 -squareSize 3.0 -outputFile camera.yml
	//example command:  CameraCalibrationOpenCv -cameraType stereo -leftCameraId 1 -rightCameraId 2 -boardHeight 6 -boardWidth 8 -squareSize 3.0 -outputFile stereocamera.yml







	if(calibrationCommandStrings.size() < 2)
	{
		cout << "Handle arguments defaults notifications" << endl;
	}




	switch(struct_calibrationConfig.e_calibrationCameraType)
	{
		case CAMERA_TYPE_MONO:
			handleMonoCalibration(struct_calibrationConfig);
			break;
		case CAMERA_TYPE_STEREO:
			cout << "Stereo running" << endl;
			//		handleStereoCalibration(struct_calibrationConfig);


			break;
		case CAMERA_TYPE_KINECT:
		default:


			cout << "Unsupported camera type." <<endl;
			break;


	}









}


Size CameraCalibration::getImageSize()
{
	return this->imageSize;


}



Mat CameraCalibration::getCameraMatrix()
{

	return this->cameraMatrix;


}
Mat CameraCalibration::getCameraMatrixClone()
{

	return this->cameraMatrix.clone();

}
Mat CameraCalibration::getDistCoeffs()
{

	return this->distCoeffs;

}
Mat CameraCalibration::getDistCoeffsClone()
{

	return this->distCoeffs.clone();


}
float CameraCalibration::getSquareSize()
{

	return this->squareSize;


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

int CameraCalibration::handleMonoCalibration(struct CalibrationConfigStruct struct_calibrationConfig)
{
	//The capture for the image
	VideoCapture capture;

	//Used to signify the capture portion is done
	int quitCapture = 0;


	int lastKey = 0;


	//Open the capture
	capture.open(struct_calibrationConfig.cameraId);



	//Named window for calibration
	namedWindow("Current Calibration Image Raw", 1);


	//Named window for calibration
	namedWindow("Current Calibration Image", 1);


	//A vector to hold the points found during calibration
	vector< vector<Point2f> >calibPoints;

	//How many frames we have
	int frameCount = 0;

	cout << "Starting calibration feature point capture." << endl;

	//Calibration feature point capture loop
	while(quitCapture == 0)
	{


		Mat currentFrame;

        vector<Point2f> currentFramePoints;

		if(capture.isOpened())
		{
			Mat captureFrame;
			capture >> captureFrame;
			captureFrame.copyTo(currentFrame);

		}

		if(!currentFrame.data)
		{
			cout << "No Frame Data" << endl;
			return 2;
		}

		Mat flippedFrame;

		//currentFrame.copyTo(flippedFrame);

		flip(currentFrame, flippedFrame,1);

		imshow("Current Calibration Image Raw", flippedFrame);


		lastKey = waitKey(33);
		//cout << "Key Value: " <<  (lastKey & 0xff ) << endl;

		int useImageForCalibration = 1;
		switch((char) lastKey)
		{
			case 'q':		//Capture phase complete
			case 'd':
				cout <<  "Last key in switch:"  << (char) lastKey << endl;
				quitCapture = 1;
				useImageForCalibration = 0;
				break;

//			case 's':     //Save calibration image
//
//				useImageForCalibration = 1;
//				break;
//
//			case ' ':	//Use image
//			case RETURN_KEY_VALUE:

//				useImageForCalibration = 1;
//				break;
			case CAMERA_CALIBRATION_ESCAPE_KEY_VALUE:	//Abort calibration


				quitCapture = 2;

				cout << "Aborting calibration" << endl;
				return 1;

				break;

			default:
				break;

		}


		if(useImageForCalibration == 1)
		{

			Mat currentFrameGray;

			cvtColor(currentFrame,currentFrameGray,CV_BGR2GRAY);


			//Find the corners of the Chessboard
	        bool foundPoints = findChessboardCorners( currentFrameGray, struct_calibrationConfig.boardSize, currentFramePoints, CV_CALIB_CB_ADAPTIVE_THRESH & CV_CALIB_CB_FAST_CHECK & CV_CALIB_CB_NORMALIZE_IMAGE);

	        imshow("Current Calibration Image", currentFrameGray);

	       //Refine the location to sub pixel if there were some points found
	        if(foundPoints)
	        {
	        	cornerSubPix( currentFrameGray, currentFramePoints, Size(5,5), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

	            drawChessboardCorners( currentFrame, struct_calibrationConfig.boardSize, Mat(currentFramePoints), foundPoints );

	        	imshow("Current Calibration Image", currentFrame);

	        	int key = waitKey(0) & 0xff;
	        	if(key == 's')
	        	{
	        		calibPoints.push_back(currentFramePoints);
	        		frameCount++;
	        	}

	        	struct_calibrationConfig.imageSize = currentFrame.size();
	        }
	        else
	        {




	        }






		}

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


		}

	}

//	cout << "Calibration images capture complete.  Moving to calibration calculation" << endl;







	namedWindow("CurrentCalibrationImageRaw2", 1);


	waitKey(0);

	destroyWindow("Current Calibration Image Raw");
	waitKey(0);

	return 0;
}


double CameraCalibration::computeReprojectionErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
        const Mat& cameraMatrix, const Mat& distCoeffs,
        vector<float>& perViewErrors )
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); i++ )
    {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err*err/n);
        totalErr += err*err;
        totalPoints += n;
    }

    return std::sqrt(totalErr/totalPoints);
}


void CameraCalibration::calcChessboardCorners(Size boardSize, float squareSize, int numberOfViews, vector<vector<Point3f> >& corners)
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

bool CameraCalibration::runCalibration(vector<vector<Point2f> > imagePoints,
        Size imageSize, Size boardSize,
        float squareSize, float aspectRatio,
        int flags, Mat& cameraMatrix, Mat& distCoeffs,
        vector<Mat>& rvecs, vector<Mat>& tvecs,
        vector<float>& reprojErrs,
        double& totalAvgErr)
{


    cameraMatrix = Mat::eye(3, 3, CV_64F);

    if( flags & CV_CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = aspectRatio;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(0);
    calcChessboardCorners(boardSize, squareSize,imagePoints.size(),  objectPoints);

    //objectPoints.resize(imagePoints.size(),objectPoints[0]);

    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, flags|CV_CALIB_RATIONAL_MODEL);
                    ///*|CV_CALIB_FIX_K3   __ |CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
    printf("RMS error reported by calibrateCamera: %g\n", rms);

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;


}


bool CameraCalibration::displayAndAcceptRemappedImages(VideoCapture & capture, Mat image, Size imageSize,Mat cameraMatrix, Mat distCoeffs)
{

	 Mat view, rview, map1, map2;
	 //initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
	 initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, imageSize, CV_16SC2, map1, map2);

	 bool displayRemappedComplete = false;

	 bool returnVal = false;

	 int key = -1;

	 Mat currentView;

	 if(capture.isOpened())
	 {
		 Mat captureView;
		 capture >> captureView;
		 captureView.copyTo(currentView);
	 }
	 else if(!image.empty())
	 {
		image.copyTo( currentView);


	 }
	 else
	 {

		 cout << "No Valid image or stream to show\n" << endl;
		 return false;

	 }

	 namedWindow("Original View");
	 namedWindow("Image View");
	 namedWindow("Undist View");
	 while(!displayRemappedComplete)
	 {

		 if(capture.isOpened())
		 {
			 Mat captureView;
			 capture >> captureView;
			 captureView.copyTo(currentView);
		 }
		 else if(!image.empty())
		 {
			image.copyTo( currentView);


		 }



        //undistort( view, rview, cameraMatrix, distCoeffs, cameraMatrix );
         remap(currentView, rview, map1, map2, INTER_LINEAR);
         Mat undistView;
         Mat temp = currentView.clone();
              undistort(temp, undistView, cameraMatrix, distCoeffs);

         imshow("Original View", currentView);
         imshow("Image View", rview);
         imshow("Undist View", undistView);

         key = 0xff & waitKey();
         if(key == 'a')
         {
        	 displayRemappedComplete = true;
        	 returnVal = true;

         }
         if( (key == 'd') || (key& 255 == 27))
         {

        	 displayRemappedComplete = true;
        	 returnVal = false;

         }
         destroyWindow("Original View");
         destroyWindow("Image View");
	 }

	 return returnVal;


}

bool CameraCalibration::runAndSave(const string& outputFilename,
                const vector<vector<Point2f> >& imagePoints,
                Size imageSize, Size boardSize, float squareSize,
                float aspectRatio, int flags, Mat& cameraMatrix,
                Mat& distCoeffs, bool writeExtrinsics, bool writePoints, VideoCapture & capture, Mat image )
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(imagePoints, imageSize, boardSize, squareSize,
                   aspectRatio, flags, cameraMatrix, distCoeffs,
                   rvecs, tvecs, reprojErrs, totalAvgErr);
    printf("%s. avg reprojection error = %.2f\n",
           ok ? "Calibration succeeded" : "Calibration failed",
           totalAvgErr);

    ok = displayAndAcceptRemappedImages(capture, image, imageSize,cameraMatrix,distCoeffs);


    if( ok )
    {
    	this->aspectRatio = aspectRatio;
    	this->boardSize = boardSize;
    	this->cameraMatrix = cameraMatrix.clone();
    	this->distCoeffs = distCoeffs.clone();
    	this->flags = flags;
    	this->imagePoints = imagePoints;
    	this->imageSize = imageSize;
    	this->reprojErrs = reprojErrs;
    	this->rvecs = rvecs;
    	this->tvecs = tvecs;
    	this->squareSize = squareSize;
    	this->totalAvgErr = totalAvgErr;



    	saveCameraParams( outputFilename );

    }
    return ok;
}




bool CameraCalibration::loadCalibrationConfigFile(vector<string> &commandArgs, string filename)
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



int CameraCalibration::parseCalibrationConfigCommandVector(struct CalibrationConfigStruct & struct_calibrationConfig, vector<string> & s_commandLineArguments)
{

	int argc = s_commandLineArguments.size();


	//Setup the default configuration values in struct
	struct_calibrationConfig.e_calibrationCameraType = CAMERA_TYPE_MONO;
	struct_calibrationConfig.cameraId = CAMERA_CALIBRATION_DEFAULT_CAMERA_ID;
	struct_calibrationConfig.leftCameraId = CAMERA_CALIBRATION_DEFAULT_LEFT_CAMERA_ID;
	struct_calibrationConfig.rightCameraId = CAMERA_CALIBRATION_DEFAULT_RIGHT_CAMERA_ID;

	struct_calibrationConfig.boardSize.height = CAMERA_CALIBRATION_DEFAULT_BOARD_HEIGHT;
	struct_calibrationConfig.boardSize.width = CAMERA_CALIBRATION_DEFAULT_BOARD_WIDTH;
	struct_calibrationConfig.flags = CAMERA_CALIBRATION_DEFAULT_FLAGS;
	struct_calibrationConfig.aspectRatio = CAMERA_CALIBRATION_DEFAULT_ASPECT_RATIO;
	struct_calibrationConfig.squareSize = CAMERA_CALIBRATION_DEFAULT_SQUARE_SIZE;
	struct_calibrationConfig.outputFilename = CAMERA_CALIBRATION_DEFAULT_OUTPUT_FILENAME;
	struct_calibrationConfig.delay = CAMERA_CALIBRATION_DEFAULT_DELAY;




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

	return 0;
}


void CameraCalibration::saveCameraParams( const string& filename)
{
    FileStorage fs( filename, FileStorage::WRITE );

    time_t t;
    time( &t );
    struct tm *t2 = localtime( &t );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
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

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

    if( !rvecs.empty() && !tvecs.empty() )
    {
        Mat bigmat((int)rvecs.size(), 6, CV_32F);
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));
            rvecs[i].copyTo(r);
            tvecs[i].copyTo(t);
        }
        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "extrinsic_parameters" << bigmat;
    }

    if( !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), imagePoints[0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }

    fs.release();
}



void CameraCalibration::loadCameraParams( const string& filename, bool loadExtrinsics, bool loadReprojErrs, bool loadImagePoints )
{
    FileStorage fs( filename, FileStorage::READ );

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



    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    totalAvgErr = (double)fs["avg_reprojection_error"];

    if( loadReprojErrs )
    {
    	Mat reprojTmp;
    	fs["per_view_reprojection_errors"] >> reprojTmp;
    	reprojErrs = Mat_<float>(reprojTmp);
    }
    if( loadExtrinsics )
    {
        Mat bigmat;
        fs["extrinsic_parameters"] >> bigmat;
        for( int i = 0; i < bigmat.rows; i++ )
        {

           	Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));

            rvecs.push_back(r);
            tvecs.push_back(t);
        }
    }

    if( loadImagePoints )
    {
        Mat imagePtMat;
        fs["image_points"] >> imagePtMat;
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
        	vector<Point2f> currentPoints = Mat_<Point2f> (imagePtMat.row(i));
        	imagePoints.push_back(currentPoints);
        }

    }



}
