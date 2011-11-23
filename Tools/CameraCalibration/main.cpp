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

//********************************************************************************************
//
//
//
// OpenCV CameraCalibration Project
//
//
//
//
//
//
//
//**********************************************************************************************



#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "CameraCalibration.h"
#include "StereoCameraCalibration.h"

#define ESCAPE_KEY_VALUE 27
#define RETURN_KEY_VALUE 10
#define DEFAULT_CAMERA_ID 0
#define DEFAULT_LEFT_CAMERA_ID 0
#define DEFAULT_RIGHT_CAMERA_ID 1
#define DEFAULT_BOARD_HEIGHT 5 //6
#define DEFAULT_BOARD_WIDTH 7 //8
#define DEFAULT_FLAGS 0
#define DEFAULT_ASPECT_RATIO 0.0f
#define DEFAULT_SQUARE_SIZE 3.0f
#define DEFAULT_OUTPUT_FILENAME "./cameraCalibration.yml"
#define DEFAULT_DELAY 5000

//CameraCalibrationOpenCv -cameraType stereo -boardHeight 7 -boardWidth 10 -leftCameraId 1 -rightCameraId 0 -squareSize 2.45 -inputSize vga -outputFile logStereoCameraHomeVGA.yml
using namespace std;
using namespace cv;

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
	CAMERA_LOCATION_MONO=0,
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

enum CameraInputSizeSettings
{
	CAMERA_INPUT_SIZE_HD, //1920x1080
	CAMERA_INPUT_SIZE_VGA, //640x480
	CAMERA_INPUT_SIZE_QVGA, //320x240
	NUMBER_OF_CAMERA_INPUT_SIZES
};


enum CameraInputTypeSettings
{
	CAMERA_INPUT_TYPE_STREAM,
	CAMERA_INPUT_TYPE_IMAGE_LIST,
	CAMERA_INPUT_TYPE_VIDEO_FILE,
	NUMBER_OF_CAMERA_INPUT_TYPES
};

typedef struct
{
	VideoCapture capture[2];

	string inputImageListFilename[2];
	string inputImageListBaseDir[2];
	vector<string> imageList[2];
	int currentInputImageIndex;


} CalibrationInputStruct;
typedef struct
{
	CalibrationCameraType e_calibrationCameraType;
	CalibrationInputStruct inputInfo;
	Size boardSize;
	Size frameSize;
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

	CameraInputSizeSettings e_inputSizeSetting;
	CameraInputTypeSettings e_inputTypeSetting;
} CalibrationConfigStruct;


//Mono calibration prototype
int handleMonoCalibration(CalibrationConfigStruct struct_calibrationConfig);

//Stereo Calibration prototype
int handleStereoCalibration(CalibrationConfigStruct struct_calibrationConfig);

//Run Calibration and save the data if accepted
bool runAndSave(const string& outputFilename,
                const vector<vector<Point2f> >& imagePoints,
                Size imageSize, Size boardSize, float squareSize,
                float aspectRatio, int flags, Mat& cameraMatrix,
                Mat& distCoeffs, bool writeExtrinsics, bool writePoints, VideoCapture& capture, Mat image );



bool runStereoCalibration(vector<vector<Point2f> > imagePoints[2],
        Size imageSize, Size boardSize,
        float squareSize, float aspectRatio,
        int flags, Mat cameraMatrix[2], Mat distCoeffs[2],
        Mat& R, Mat& T, Mat& R1, Mat& P1, Mat& R2, Mat& P2, Mat& Q ,
        Rect validRoi[2],
        double & rmsErr,
        double& totalAvgErr);




bool loadTextFileList(vector<string> &fileLines, string filename)
{

	std::ifstream configFile;
	//int i = 3;
	string tmpString;
	bool returnVal = false;


	configFile.open(filename.c_str());


	if(configFile.good())
	{

		while(!configFile.eof())
		{
			getline(configFile,tmpString);



			if(!(tmpString.compare("") == 0) && !(tmpString[0] == '#'))
			{

				if(tmpString[tmpString.size()-1] == 13)
				{

					tmpString = tmpString.erase(tmpString.size()-1);
				}
				if(tmpString[tmpString.size()-2] == 13)
				{
					tmpString = tmpString.erase(tmpString.size()-2);
				}
				if(tmpString[tmpString.size()-1] == 10)
				{
					tmpString = tmpString.erase(tmpString.size()-1);
				}
				if(tmpString[tmpString.size()-2] == 10)
				{
					tmpString = tmpString.erase(tmpString.size()-2);
				}

				fileLines.push_back(tmpString);
			}
		}
		returnVal = true;

	}
	else
	{

		cout << "WARNING: Unable to open File List file" << endl;

	}
	return returnVal;

}




bool loadCalibrationConfigFile(vector<string> &commandArgs, string filename)
{

	std::ifstream configFile;
//	int i = 3;
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



int parseCalibrationConfigCommandVector(CalibrationConfigStruct & struct_calibrationConfig, vector<string> & s_commandLineArguments)
{

	int argc = s_commandLineArguments.size();


	//Setup the default configuration values in struct
	struct_calibrationConfig.e_inputTypeSetting = CAMERA_INPUT_TYPE_STREAM;
	struct_calibrationConfig.e_calibrationCameraType = CAMERA_TYPE_MONO;
	struct_calibrationConfig.cameraId = DEFAULT_CAMERA_ID;
	struct_calibrationConfig.leftCameraId = DEFAULT_LEFT_CAMERA_ID;
	struct_calibrationConfig.rightCameraId = DEFAULT_RIGHT_CAMERA_ID;

	struct_calibrationConfig.boardSize.height = DEFAULT_BOARD_HEIGHT;
	struct_calibrationConfig.boardSize.width = DEFAULT_BOARD_WIDTH;
	struct_calibrationConfig.flags = DEFAULT_FLAGS;
	struct_calibrationConfig.aspectRatio = DEFAULT_ASPECT_RATIO;
	struct_calibrationConfig.squareSize = DEFAULT_SQUARE_SIZE;
	struct_calibrationConfig.outputFilename = DEFAULT_OUTPUT_FILENAME;
	struct_calibrationConfig.delay = DEFAULT_DELAY;




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
			if(strcmp(s_commandLineArguments[i+1].c_str(),"mono")==0)
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

		if(s_commandLineArguments[i].compare("-inputType")==0 || s_commandLineArguments[i].compare("-InputType")==0)
		{
			if(s_commandLineArguments[i+1].compare("stream")==0)
			{
				struct_calibrationConfig.e_inputTypeSetting = CAMERA_INPUT_TYPE_STREAM;
				cout << "Input Type: Stream" << endl;

			}


			if(s_commandLineArguments[i+1].compare("videoFile")==0)
			{
				struct_calibrationConfig.e_inputTypeSetting = CAMERA_INPUT_TYPE_VIDEO_FILE;
				cout << "Input Type: Video File" << endl;
			}
			if(s_commandLineArguments[i+1].compare("imageList")==0)
			{
				struct_calibrationConfig.e_inputTypeSetting = CAMERA_INPUT_TYPE_IMAGE_LIST;
				cout << "Input Type: Image List" << endl;
			}


		}
		if(s_commandLineArguments[i].compare("-leftImageList")==0 || s_commandLineArguments[i].compare("-LeftImageList")==0)
		{
			struct_calibrationConfig.inputInfo.inputImageListFilename[CAMERA_LOCATION_LEFT] = s_commandLineArguments[i+1];
		}

		if(s_commandLineArguments[i].compare("-rightImageList")==0 || s_commandLineArguments[i].compare("-RightImageList")==0)
		{

			struct_calibrationConfig.inputInfo.inputImageListFilename[CAMERA_LOCATION_RIGHT] = s_commandLineArguments[i+1];
		}

		if(s_commandLineArguments[i].compare("-leftImageListBaseDir")==0 || s_commandLineArguments[i].compare("-LeftImageListBaseDir")==0)
		{

			struct_calibrationConfig.inputInfo.inputImageListBaseDir[CAMERA_LOCATION_LEFT] = s_commandLineArguments[i+1];

		}

		if(s_commandLineArguments[i].compare("-rightImageListBaseDir")==0 || s_commandLineArguments[i].compare("-RightImageListBaseDir")==0)
		{
			struct_calibrationConfig.inputInfo.inputImageListBaseDir[CAMERA_LOCATION_RIGHT] = s_commandLineArguments[i+1];

		}




		if(strcmp(s_commandLineArguments[i].c_str(),"-inputSize")==0 || strcmp(s_commandLineArguments[i].c_str(),"-InputSize")==0)
		{
			if(strcmp(s_commandLineArguments[i+1].c_str(),"vga")==0)
			{
				struct_calibrationConfig.e_inputSizeSetting = CAMERA_INPUT_SIZE_VGA;
				cout << "VGA frame size  (640x480) set" << endl;

			}
			if(strcmp(s_commandLineArguments[i+1].c_str(),"qvga")==0)
			{
				struct_calibrationConfig.e_inputSizeSetting = CAMERA_INPUT_SIZE_QVGA;
				cout << "QVGA frame size  (320x240) set" << endl;				
			}
			if(strcmp(s_commandLineArguments[i+1].c_str(),"hd")==0)
			{
				struct_calibrationConfig.e_inputSizeSetting = CAMERA_INPUT_SIZE_HD;
				cout << "HD frame size  (1920x1080) set" << endl;

			}

		}



		if(strcmp(s_commandLineArguments[i].c_str(),"-cameraId")==0)
		{

			if(sscanf( s_commandLineArguments[++i].c_str(), "%u", &struct_calibrationConfig.cameraId ) != 1)
			{
				cout << "Camera Id value invalid......exiting calibration" << endl;
				//Sleep(5);
				return 1;
			}


		}


		if(strcmp(s_commandLineArguments[i].c_str(),"-leftCameraId")==0)
		{

			if(sscanf( s_commandLineArguments[++i].c_str(), "%u", &struct_calibrationConfig.leftCameraId ) != 1)
			{
				cout << "Left Camera Id value invalid......exiting calibration" << endl;
				//sleep(5);
				return 1;
			}


		}

		if(strcmp(s_commandLineArguments[i].c_str(),"-rightCameraId")==0)
		{

			if(sscanf( s_commandLineArguments[++i].c_str(), "%u", &struct_calibrationConfig.rightCameraId ) != 1)
			{
				cout << "Right Camera Id value invalid......exiting calibration" << endl;
				//sleep(5);
				return 1;
			}


		}


		if(strcmp(s_commandLineArguments[i].c_str(),"-boardHeight")==0)
		{

			if(sscanf( s_commandLineArguments[++i].c_str(), "%u", &struct_calibrationConfig.boardSize.height ) != 1)
			{
				cout << "Board height value invalid......exiting calibration" << endl;
				//sleep(5);
				return 1;
			}


		}

		if(strcmp(s_commandLineArguments[i].c_str(),"-boardWidth")==0)
		{

			if(sscanf( s_commandLineArguments[++i].c_str(), "%u", &struct_calibrationConfig.boardSize.width ) != 1)
			{
				cout << "Board width value invalid......exiting calibration" << endl;
				//sleep(5);
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
				//sleep(5);
				return 1;
			}


		}

		if(strcmp(s_commandLineArguments[i].c_str(),"-aspectRatio")==0)
		{

			if(sscanf( s_commandLineArguments[++i].c_str(), "%f", &struct_calibrationConfig.aspectRatio ) != 1)
			{
				cout << "Aspect Ratio value invalid......exiting calibration" << endl;
				//sleep(5);
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

//**************************************************************************************
//
//Main function:  Calibrates the camera type sepcified using the command line params as guidance
//	Arugments:  argc - command line arguments count
//				argv - command line arguments
//	Return Val: 0 - completed correctly, else error
//	Author:  Jason Clemons
//	Date Created: 2-22-2011
//
//
//***************************************************************************************

int main(int argc, const char * argv[])
{
	CalibrationConfigStruct struct_calibrationConfig;
	vector <string> s_commandLineArguments(argc);
	char c_tmp;

	//Save the command line arguments to a string array
	for(int i = 0; i < argc; i++)
	{
		s_commandLineArguments[i] = argv[i];

	}
	cout << "\n**********************OpenCv Camera Calibration Program***********************" << endl;

	if((s_commandLineArguments[1].compare("-configFile") ==0) || (s_commandLineArguments[1].compare("-ConfigFile") ==0))
	{
		loadCalibrationConfigFile(s_commandLineArguments, s_commandLineArguments[2]);



	}
	if((s_commandLineArguments[0].compare("-configFile") ==0) || (s_commandLineArguments[0].compare("-ConfigFile") ==0))
	{
		loadCalibrationConfigFile(s_commandLineArguments, s_commandLineArguments[1]);



	}


	parseCalibrationConfigCommandVector(struct_calibrationConfig,  s_commandLineArguments);

	//example command:  CameraCalibrationOpenCv -cameraType mono -boardHeight 6 -boardWidth 8 -squareSize 3.0 -outputFile camera.yml
	//example command:  CameraCalibrationOpenCv -cameraType stereo -leftCameraId 1 -rightCameraId 2 -boardHeight 6 -boardWidth 8 -squareSize 3.0 -outputFile stereocamera.yml







	if(s_commandLineArguments.size() < 2)
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
			handleStereoCalibration(struct_calibrationConfig);


			break;
		case CAMERA_TYPE_KINECT:
		default:


			cout << "Unsupported camera type." <<endl;
			break;


	}









	cin >> c_tmp;
	return 0;

}




//**************************************************************************************
//
// setupMonoCalibrationInput function:  Setup the input
//	Arugments:  struct_calibrationConfig - calibration information from the command line
//
//	Return:  0 if successful, non zero otherwise
//	Author:  Jason Clemons
//	Date Created: 11-10-2011
//
//
//***************************************************************************************

int setupMonoCalibrationInput(CalibrationConfigStruct & struct_calibrationConfig)
{
	int success = 0;

	switch(struct_calibrationConfig.e_inputTypeSetting)
	{

		case CAMERA_INPUT_TYPE_IMAGE_LIST:
		{
			break;
		}
		case CAMERA_INPUT_TYPE_VIDEO_FILE:
		{
			break;
		}
		case CAMERA_INPUT_TYPE_STREAM:
		default:
		{
			//Open the capture
			struct_calibrationConfig.inputInfo.capture[CAMERA_LOCATION_MONO].open(struct_calibrationConfig.cameraId);

			//int frameWidth;
			//int frameHeight;


			switch(struct_calibrationConfig.e_inputSizeSetting)
			{
				case CAMERA_INPUT_SIZE_QVGA:
				{
					struct_calibrationConfig.frameSize.width = 320;
					struct_calibrationConfig.frameSize.height = 240;
					break;
				}
				case CAMERA_INPUT_SIZE_HD:
				{
					struct_calibrationConfig.frameSize.width = 1920;
					struct_calibrationConfig.frameSize.height = 1080;
					break;
				}

				case CAMERA_INPUT_SIZE_VGA:
				default:
				{
					struct_calibrationConfig.frameSize.width = 640;
					struct_calibrationConfig.frameSize.height = 480;
					break;
				}


			}
			if(!(struct_calibrationConfig.inputInfo.capture[CAMERA_LOCATION_MONO].set(CV_CAP_PROP_FRAME_WIDTH, struct_calibrationConfig.frameSize.width)))//320)))//640)))
			{
				cout << "Frame width set failed" << endl;
			}
			if(!(struct_calibrationConfig.inputInfo.capture[CAMERA_LOCATION_MONO].set(CV_CAP_PROP_FRAME_HEIGHT, struct_calibrationConfig.frameSize.height)))//240)))//480)))
			{
				cout << "Frame height set failed" << endl;
			}




			break;
		}



	}

	return success;
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

Mat getNextMonoFrame(CalibrationConfigStruct & struct_calibrationConfig)
{
	Mat currentFrame;
	switch(struct_calibrationConfig.e_inputTypeSetting)
	{

		case CAMERA_INPUT_TYPE_IMAGE_LIST:
		{
			break;
		}
		case CAMERA_INPUT_TYPE_VIDEO_FILE:
		{
			break;
		}
		case CAMERA_INPUT_TYPE_STREAM:
		default:
		{
			if(struct_calibrationConfig.inputInfo.capture[CAMERA_LOCATION_MONO].isOpened())
			{
				Mat captureFrame;
				struct_calibrationConfig.inputInfo.capture[CAMERA_LOCATION_MONO] >> captureFrame;
				captureFrame.copyTo(currentFrame);

			}




			break;
		}



	}


	return currentFrame;
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

Mat getNextMonoDisplayResultFrame(CalibrationConfigStruct & struct_calibrationConfig)
{
	Mat currentFrame;
	switch(struct_calibrationConfig.e_inputTypeSetting)
	{

		case CAMERA_INPUT_TYPE_IMAGE_LIST:
		{
			break;
		}
		case CAMERA_INPUT_TYPE_VIDEO_FILE:
		{
			break;
		}
		case CAMERA_INPUT_TYPE_STREAM:
		default:
		{


			break;
		}



	}


	return currentFrame;
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

int handleMonoCalibration(CalibrationConfigStruct struct_calibrationConfig)
{
	//The capture for the image
	//VideoCapture capture;

	//Used to signify the capture portion is done
	int quitCapture = 0;


	int lastKey = 0;

	//Setup the Calibration Input
	setupMonoCalibrationInput(struct_calibrationConfig);


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

        currentFrame = getNextMonoFrame(struct_calibrationConfig);
		if(currentFrame.rows !=struct_calibrationConfig.frameSize.height || currentFrame.cols !=struct_calibrationConfig.frameSize.width)
		{

			Mat tmpCurrentFrame;
			resize(currentFrame,tmpCurrentFrame,Size(struct_calibrationConfig.frameSize.width,struct_calibrationConfig.frameSize.height));
			currentFrame = tmpCurrentFrame;
		}

		if(!currentFrame.data)
		{
			cout << "No Frame Data" << endl;
			return 2;
		}

		Mat flippedFrame;

		//currentFrame.copyTo(flippedFrame);

		flip(currentFrame, flippedFrame,1);

		Mat tmpFlippedFrame;
		resize(flippedFrame,tmpFlippedFrame,Size(640,480));
		flippedFrame = tmpFlippedFrame;
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
			case ESCAPE_KEY_VALUE:	//Abort calibration


				quitCapture = 2;

				cout << "Aborting calibration" << endl;
				return 1;

				break;

			default:
				break;

		}


		if(useImageForCalibration == 1 || struct_calibrationConfig.e_inputTypeSetting != CAMERA_INPUT_TYPE_STREAM)
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
				Mat tmpCurrentFrame;
				resize(currentFrame,tmpCurrentFrame,Size(640,480));
	        	imshow("Current Calibration Image", tmpCurrentFrame);
				cout << "Chessboard found!" << endl;
	        	int key = waitKey(0) & 0xff;
	        	if(key == 's')
	        	{
	        		calibPoints.push_back(currentFramePoints);
	        		frameCount++;
	        	}

	        	struct_calibrationConfig.frameSize = currentFrame.size();
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
			//bool writeExtrinsics;
			//bool writePoints;
			Mat image = getNextMonoDisplayResultFrame(struct_calibrationConfig);
			bool saveResult = true;
			saveResult = runAndSave(struct_calibrationConfig.outputFilename, calibPoints, struct_calibrationConfig.frameSize,
					struct_calibrationConfig.boardSize, struct_calibrationConfig.squareSize, struct_calibrationConfig.aspectRatio, struct_calibrationConfig.flags,
					cameraMatrix, distCoeffs ,struct_calibrationConfig.writeExtrinsics,struct_calibrationConfig.writePoints, struct_calibrationConfig.inputInfo.capture[CAMERA_LOCATION_MONO],image );

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


static double computeReprojectionErrors(
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


void saveCameraParams( const string& filename,
                       Size imageSize, Size boardSize,
                       float squareSize, float aspectRatio, int flags,
                       const Mat& cameraMatrix, const Mat& distCoeffs,
                       const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                       const vector<float>& reprojErrs,
                       const vector<vector<Point2f> >& imagePoints,
                       double totalAvgErr )
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

void loadCameraParams( const string& filename,
                       Size & imageSize, Size & boardSize,
                       float & squareSize, float & aspectRatio, int & flags,
                       Mat& cameraMatrix, Mat& distCoeffs,
                       bool loadExtrinsics,
                       vector<Mat>& rvecs, vector<Mat>& tvecs,
                       bool loadReprojErrs,
                       vector<float>& reprojErrs,
                       bool loadImagePoints,
                       vector<vector<Point2f> >& imagePoints,
                       double & totalAvgErr )
{
    FileStorage fs( filename, FileStorage::READ );

//    time_t t;
//   time( &t );
//    struct tm *t2 = localtime( &t );
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




static void calcChessboardCorners(Size boardSize, float squareSize, int numberOfViews, vector<vector<Point3f> >& corners)
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

bool runCalibration(vector<vector<Point2f> > imagePoints,
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

    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                    distCoeffs, rvecs, tvecs, flags|CV_CALIB_RATIONAL_MODEL);
                    ///*|CV_CALIB_FIX_K3   __ |CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
    printf("RMS error reported by calibrateCamera: %g\n", rms);

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;


}


bool displayAndAcceptRemappedImages(VideoCapture & capture, Mat image, Size imageSize,Mat cameraMatrix, Mat distCoeffs)
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
         if( (key == 'd') || (key == 27))
         {

        	 displayRemappedComplete = true;
        	 returnVal = false;

         }
         destroyWindow("Original View");
         destroyWindow("Image View");
	 }

	 return returnVal;


}

bool runAndSave(const string& outputFilename,
                const vector<vector<Point2f> >& imagePoints,
                Size imageSize, Size boardSize, float squareSize,
                float aspectRatio, int flags, Mat& cameraMatrix,
                Mat& distCoeffs, bool writeExtrinsics, bool writePoints, VideoCapture &capture, Mat image )
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
        saveCameraParams( outputFilename, imageSize,
                         boardSize, squareSize, aspectRatio,
                         flags, cameraMatrix, distCoeffs,
                         writeExtrinsics ? rvecs : vector<Mat>(),
                         writeExtrinsics ? tvecs : vector<Mat>(),
                         writeExtrinsics ? reprojErrs : vector<float>(),
                         writePoints ? imagePoints : vector<vector<Point2f> >(),
                         totalAvgErr );
    return ok;
}



void saveStereoCameraParams( const string& filename,
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


void loadStereoCameraParams( const string& filename,
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

//    time_t t;
 //   time( &t );
  //  struct tm *t2 = localtime( &t );
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

    //fs["Valid_ROI_Left"] >> validRoi[CAMERA_LOCATION_LEFT];
    //fs["Valid_ROI_Right"] >> validRoi[CAMERA_LOCATION_RIGHT];
	FileNodeIterator leftRoiIt = fs["Valid_ROI_Left"].begin();
	validRoi[CAMERA_LOCATION_LEFT].x	=((int)*leftRoiIt);
	++leftRoiIt;
	validRoi[CAMERA_LOCATION_LEFT].y	=((int)*leftRoiIt);
	++leftRoiIt;
	validRoi[CAMERA_LOCATION_LEFT].width	=((int)*leftRoiIt);
	++leftRoiIt;
	validRoi[CAMERA_LOCATION_LEFT].height	=((int)*leftRoiIt);

	FileNodeIterator rightRoiIt = fs["Valid_ROI_Right"].begin();
	validRoi[CAMERA_LOCATION_RIGHT].x	=((int)*rightRoiIt);
	++rightRoiIt;
	validRoi[CAMERA_LOCATION_RIGHT].y	=((int)*rightRoiIt);
	++rightRoiIt;
	validRoi[CAMERA_LOCATION_RIGHT].width	=((int)*rightRoiIt);
	++rightRoiIt;
	validRoi[CAMERA_LOCATION_RIGHT].height	=((int)*rightRoiIt);



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

int displayRemappedStereoImages(VideoCapture& leftCameraCapture,VideoCapture& rightCameraCapture,Mat & leftImage ,Mat & rightImage,const Rect validRoi[2],const Size& imageSize,const Mat rmap[2][2])
{
	Mat leftCaptureFrame, currentLeftFrame, rightCaptureFrame, currentRightFrame;
	Mat rightFrameRemapped, leftFrameRemapped;
	Mat bothImages;
	namedWindow("Rectified",1);


	if(leftCameraCapture.isOpened() && rightCameraCapture.isOpened())
	{
		leftCameraCapture >> leftCaptureFrame;
		rightCameraCapture >> rightCaptureFrame;
	}
	else if (!leftImage.empty() && !rightImage.empty())
	{
		leftCaptureFrame = leftImage.clone();
		rightCaptureFrame = leftImage.clone();
	}
	else
	{
		cout << "Nothing to show" << endl;
		return 's';

	}
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




    for( int j = 0; j < bothImages.rows; j += 16 )
    {
        line(bothImages, Point( 0,j), Point( bothImages.cols,j), Scalar(0, 255, 0), 1, 8);
    }
    imshow("Rectified", bothImages);


	return waitKey(10) & 0xFF;


}



//**************************************************************************************
//
// setupMonoCalibrationInput function:  Setup the input
//	Arugments:  struct_calibrationConfig - calibration information from the command line
//
//	Return:  0 if successful, non zero otherwise
//	Author:  Jason Clemons
//	Date Created: 11-10-2011
//
//
//***************************************************************************************

int setupStereoCalibrationInput(CalibrationConfigStruct & struct_calibrationConfig)
{
	int success = 0;

	switch(struct_calibrationConfig.e_inputTypeSetting)
	{

		case CAMERA_INPUT_TYPE_IMAGE_LIST:
		{
			if(struct_calibrationConfig.inputInfo.inputImageListFilename[CAMERA_LOCATION_LEFT] != "" && struct_calibrationConfig.inputInfo.inputImageListFilename[CAMERA_LOCATION_RIGHT] != "")
			{
				if(!loadTextFileList(struct_calibrationConfig.inputInfo.imageList[CAMERA_LOCATION_LEFT], struct_calibrationConfig.inputInfo.inputImageListFilename[CAMERA_LOCATION_LEFT]) || !loadTextFileList(struct_calibrationConfig.inputInfo.imageList[CAMERA_LOCATION_RIGHT], struct_calibrationConfig.inputInfo.inputImageListFilename[CAMERA_LOCATION_RIGHT]))
				{

					cout << "Unable to load Image Lists from Files" << endl;

				}
				struct_calibrationConfig.inputInfo.currentInputImageIndex = 0;
				string leftImageNameWithPath = struct_calibrationConfig.inputInfo.inputImageListBaseDir[CAMERA_LOCATION_LEFT] + struct_calibrationConfig.inputInfo.imageList[CAMERA_LOCATION_LEFT][struct_calibrationConfig.inputInfo.currentInputImageIndex];
				string rightImageNameWithPath = struct_calibrationConfig.inputInfo.inputImageListBaseDir[CAMERA_LOCATION_RIGHT] + struct_calibrationConfig.inputInfo.imageList[CAMERA_LOCATION_RIGHT][struct_calibrationConfig.inputInfo.currentInputImageIndex];

				Mat tmpLeftImage = imread(leftImageNameWithPath);
				Mat tmpRightImage = imread(rightImageNameWithPath);




				//If the stream didn't open then give error
				if(tmpLeftImage.empty() ||tmpRightImage.empty())
				{
					cout << "Error: Unable to open up image file.  Exiting...." << endl;
					string input;
					cin >> input;
					exit(0);
				}
				else
				{

					//Set the actual frame size based on what the stream says
					struct_calibrationConfig.frameSize.width = ((int)tmpLeftImage.cols + tmpRightImage.cols)/2;
					struct_calibrationConfig.frameSize.height = ((int)tmpLeftImage.rows+(int)tmpRightImage.rows)/2;



					//Set the frame size based on the config file
					int frameHeight;
					int frameWidth;

					switch(struct_calibrationConfig.e_inputSizeSetting)
					{
						case CAMERA_INPUT_SIZE_QVGA:
						{
							frameWidth = 320;
							frameHeight = 240;
							break;
						}
						case CAMERA_INPUT_SIZE_HD:
						{
							frameWidth = 1920;
							frameHeight = 1080;
							break;
						}

						case CAMERA_INPUT_SIZE_VGA:
						default:
						{
							frameWidth = 640;
							frameHeight = 480;
							break;
						}


					}




					if(struct_calibrationConfig.frameSize.width != frameWidth || struct_calibrationConfig.frameSize.height != frameHeight)
					{

						cout << "Error: frame size does not match config file.  Exiting...." << endl;
						string input;
						cin >> input;
						exit(0);


					}


				}



			}
			else
			{
				cout << "Unable to Open Image Lists" << endl;
				exit(0);
			}







			break;
		}
		case CAMERA_INPUT_TYPE_VIDEO_FILE:
		{
			break;
		}
		case CAMERA_INPUT_TYPE_STREAM:
		default:
		{
			//Open the capture
			struct_calibrationConfig.inputInfo.capture[CAMERA_LOCATION_LEFT].open(struct_calibrationConfig.leftCameraId);
			struct_calibrationConfig.inputInfo.capture[CAMERA_LOCATION_RIGHT].open(struct_calibrationConfig.rightCameraId);


			if(!(struct_calibrationConfig.inputInfo.capture[CAMERA_LOCATION_LEFT].set(CV_CAP_PROP_FPS, 30.0)))
			{
				cout << "Left frame rate set failed" << endl;
			}
			if(!(struct_calibrationConfig.inputInfo.capture[CAMERA_LOCATION_RIGHT].set(CV_CAP_PROP_FPS, 30.0)))
			{
				cout << "Right frame rate set failed" << endl;

			}

			switch(struct_calibrationConfig.e_inputSizeSetting)
			{
				case CAMERA_INPUT_SIZE_QVGA:
				{
					struct_calibrationConfig.frameSize.width = 320;
					struct_calibrationConfig.frameSize.height = 240;
					break;
				}
				case CAMERA_INPUT_SIZE_HD:
				{
					struct_calibrationConfig.frameSize.width = 1920;
					struct_calibrationConfig.frameSize.height = 1080;
					break;
				}

				case CAMERA_INPUT_SIZE_VGA:
				default:
				{
					struct_calibrationConfig.frameSize.width = 640;
					struct_calibrationConfig.frameSize.height = 480;
					break;
				}


			}

			if(!(struct_calibrationConfig.inputInfo.capture[CAMERA_LOCATION_LEFT].set(CV_CAP_PROP_FRAME_WIDTH, struct_calibrationConfig.frameSize.width)))//320)))//640)))
			{
				cout << "Left width set failed" << endl;
			}
			if(!(struct_calibrationConfig.inputInfo.capture[CAMERA_LOCATION_LEFT].set(CV_CAP_PROP_FRAME_HEIGHT, struct_calibrationConfig.frameSize.height)))//240)))//480)))
			{
				cout << "Left height set failed" << endl;
			}


			if(!(struct_calibrationConfig.inputInfo.capture[CAMERA_LOCATION_RIGHT].set(CV_CAP_PROP_FRAME_WIDTH, struct_calibrationConfig.frameSize.width)))//320)))//640)))
			{
				cout << "Right width set failed" << endl;
			}
			if(!(struct_calibrationConfig.inputInfo.capture[CAMERA_LOCATION_RIGHT].set(CV_CAP_PROP_FRAME_HEIGHT, struct_calibrationConfig.frameSize.height)))//240)))//480)))
			{
				cout << "Right height set failed" << endl;
			}



			break;
		}



	}

	return success;
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

int getNextStereoFrame(CalibrationConfigStruct & struct_calibrationConfig, Mat & leftFrame, Mat & rightFrame)
{

	int imagesRemaining = INT_MAX;

	switch(struct_calibrationConfig.e_inputTypeSetting)
	{

		case CAMERA_INPUT_TYPE_IMAGE_LIST:
		{



			string leftImageNameWithPath = struct_calibrationConfig.inputInfo.inputImageListBaseDir[CAMERA_LOCATION_LEFT] + struct_calibrationConfig.inputInfo.imageList[CAMERA_LOCATION_LEFT][struct_calibrationConfig.inputInfo.currentInputImageIndex];
			string rightImageNameWithPath = struct_calibrationConfig.inputInfo.inputImageListBaseDir[CAMERA_LOCATION_RIGHT] + struct_calibrationConfig.inputInfo.imageList[CAMERA_LOCATION_RIGHT][struct_calibrationConfig.inputInfo.currentInputImageIndex];

			//Get the new data frames
			leftFrame = imread(leftImageNameWithPath);
			rightFrame = imread(rightImageNameWithPath);


			struct_calibrationConfig.inputInfo.currentInputImageIndex++;

			int imagesRemainingLeft = struct_calibrationConfig.inputInfo.imageList[CAMERA_LOCATION_LEFT].size() - struct_calibrationConfig.inputInfo.currentInputImageIndex;
			int imagesRemainingRight = struct_calibrationConfig.inputInfo.imageList[CAMERA_LOCATION_RIGHT].size() - struct_calibrationConfig.inputInfo.currentInputImageIndex;
			imagesRemaining = (imagesRemainingLeft < imagesRemainingRight ?imagesRemainingLeft: imagesRemainingLeft);



			if(!leftFrame.data)
			{
				cout << "No Frame Data from Loaded Left Image" << endl;
				return 2;
			}

			if(!rightFrame.data)
			{
				cout << "No Frame Data from Loaded Right Image" << endl;
				return 2;
			}







			break;
		}
		case CAMERA_INPUT_TYPE_VIDEO_FILE:
		{
			break;
		}
		case CAMERA_INPUT_TYPE_STREAM:
		default:
		{

			if(struct_calibrationConfig.inputInfo.capture[CAMERA_LOCATION_LEFT].isOpened() && struct_calibrationConfig.inputInfo.capture[CAMERA_LOCATION_RIGHT].isOpened())
			{

				Mat leftCaptureFrame;
				Mat rightCaptureFrame;

				struct_calibrationConfig.inputInfo.capture[CAMERA_LOCATION_LEFT] >> leftCaptureFrame;
				struct_calibrationConfig.inputInfo.capture[CAMERA_LOCATION_RIGHT] >> rightCaptureFrame;


				leftFrame = leftCaptureFrame.clone();


				rightFrame = rightCaptureFrame.clone();


			}

				//cout << "Left Frame Size" <<currentLeftFrame.rows <<"x" << currentLeftFrame.cols << endl;
				//cout << "Right Frame Size" <<currentRightFrame.rows <<"x" << currentRightFrame.cols << endl;

				if(!leftFrame.data)
				{
					cout << "No Frame Data from Left Camera" << endl;
					return 2;
				}

				if(!rightFrame.data)
				{
					cout << "No Frame Data from Right Camera" << endl;
					return 2;
				}



			break;
		}



	}


	return imagesRemaining;
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

void getNextStereoDisplayResultFrame(CalibrationConfigStruct & struct_calibrationConfig, Mat & leftFrame, Mat & rightFrame)
{
	Mat currentFrame;
	switch(struct_calibrationConfig.e_inputTypeSetting)
	{

		case CAMERA_INPUT_TYPE_IMAGE_LIST:
		{
			string leftImageNameWithPath = struct_calibrationConfig.inputInfo.inputImageListBaseDir[CAMERA_LOCATION_LEFT] + struct_calibrationConfig.inputInfo.imageList[CAMERA_LOCATION_LEFT][0];
			string rightImageNameWithPath = struct_calibrationConfig.inputInfo.inputImageListBaseDir[CAMERA_LOCATION_RIGHT] + struct_calibrationConfig.inputInfo.imageList[CAMERA_LOCATION_RIGHT][0];

			//Get the new data frames
			leftFrame = imread(leftImageNameWithPath);
			rightFrame = imread(rightImageNameWithPath);


			break;
		}
		case CAMERA_INPUT_TYPE_VIDEO_FILE:
		{
			break;
		}
		case CAMERA_INPUT_TYPE_STREAM:
		default:
		{


			break;
		}



	}



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

int handleStereoCalibration(CalibrationConfigStruct struct_calibrationConfig)
{
	//The capture for the image
//	VideoCapture leftCameraCapture;
//	VideoCapture rightCameraCapture;


	//Used to signify the capture portion is done


//	int quitCapture = 0;


//	int lastKey = -1;
	int key = -1;

	double totalAvgErr;
	double rmsErr;

	CalibrationStates currentCalibrationState = CALIBRATION_STATE_INIT;


	//A vector to hold the points found during calibration
	vector< vector<Point2f> >calibPoints[CAMERA_LOCATION_RIGHT+1];

	//How many frames we have
	int frameCount = 0;

	clock_t prevTimeStamp =0;


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

				setupStereoCalibrationInput(struct_calibrationConfig);



				//Named window for calibration
				namedWindow("Current Left Calibration Image Raw", 1);
				namedWindow("Current Right Calibration Image Raw", 1);

				//Named window for calibration
				namedWindow("Current Left Calibration Image", 1);
				namedWindow("Current Right Calibration Image", 1);


				cout << "Starting calibration feature point capture." << endl;

				if(struct_calibrationConfig.e_inputTypeSetting == CAMERA_INPUT_TYPE_STREAM)
				{
					currentCalibrationState = CALIBRATION_STATE_SHOW_IMAGE_BEFORE_CAPTURE;
				}
				else
				{
					currentCalibrationState = CALIBRATION_STATE_IMAGE_CAPTURE;

				}
				break;
			}
			case CALIBRATION_STATE_SHOW_IMAGE_BEFORE_CAPTURE:
			{


				getNextStereoFrame(struct_calibrationConfig, currentLeftFrame, currentRightFrame);



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
				int imagesRemaining = getNextStereoFrame(struct_calibrationConfig, currentLeftFrame, currentRightFrame);
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

						if(struct_calibrationConfig.e_inputTypeSetting == CAMERA_INPUT_TYPE_STREAM)
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
						else
						{
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


					struct_calibrationConfig.frameSize = currentLeftFrame.size();



		        }
		        else
		        {
		        	imshow("Current Left Calibration Image", currentFrameGray[CAMERA_LOCATION_LEFT]);
		        	imshow("Current Right Calibration Image", currentFrameGray[CAMERA_LOCATION_RIGHT]);
		        }

		        if(imagesRemaining <= 0 )
		        {
		        	if(frameCount >= 15)
					{
		        		key ='d';
					}
		        	else
		        	{

		        		key =27;
		        	}

		        }
		        if((key == 'd'  || key == 'c' || key == 'r' || key == ' ') && frameCount >= 15)
		        {

		        	currentCalibrationState =CALIBRATION_STATE_RUN;
					cout << "Data Captured.  Running Calibration routine." << endl;
		        }
		        if(key==27)
		        {

		        	currentCalibrationState =CALIBRATION_STATE_ABORT;

		        }



				break;
			}
			case CALIBRATION_STATE_RUN:
			{
			    bool ok = runStereoCalibration(calibPoints, struct_calibrationConfig.frameSize, struct_calibrationConfig.boardSize, struct_calibrationConfig.squareSize,
			    		struct_calibrationConfig.aspectRatio, struct_calibrationConfig.flags, cameraMatrix, distCoeffs,
			                   R, T, R1, P1, R2, P2, Q, validRoi, rmsErr, totalAvgErr);
			    printf("%s. avg reprojection error = %.2f\n",
			           ok ? "Calibration succeeded" : "Calibration failed",
			           totalAvgErr);

			    if(ok)
			    {
			    	cout << "Moving to save option." << endl;
			        //Precompute maps for cv::remap()
			        initUndistortRectifyMap(cameraMatrix[CAMERA_LOCATION_LEFT], distCoeffs[CAMERA_LOCATION_LEFT], R1, P1, struct_calibrationConfig.frameSize, CV_16SC2, rmap[CAMERA_LOCATION_LEFT][CAMERA_REMAP_AXIS_X], rmap[CAMERA_LOCATION_LEFT][CAMERA_REMAP_AXIS_Y]);
			        initUndistortRectifyMap(cameraMatrix[CAMERA_LOCATION_RIGHT], distCoeffs[CAMERA_LOCATION_RIGHT], R2, P2, struct_calibrationConfig.frameSize, CV_16SC2, rmap[CAMERA_LOCATION_RIGHT][CAMERA_REMAP_AXIS_X], rmap[CAMERA_LOCATION_RIGHT][CAMERA_REMAP_AXIS_Y]);



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
				Mat leftImage, rightImage;
				getNextStereoDisplayResultFrame(struct_calibrationConfig,leftImage,rightImage);
				key = displayRemappedStereoImages(struct_calibrationConfig.inputInfo.capture[CAMERA_LOCATION_LEFT], struct_calibrationConfig.inputInfo.capture[CAMERA_LOCATION_RIGHT],leftImage, rightImage,validRoi,struct_calibrationConfig.frameSize,rmap);

				//bool ok = false;

				vector<vector<Point2f> > noPoints[2];
			    if( key == 's' )
			    {
			        saveStereoCameraParams( struct_calibrationConfig.outputFilename, struct_calibrationConfig.frameSize,
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


static double computeStereoReprojectionErrors(
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
bool runStereoCalibration(vector<vector<Point2f> > imagePoints[2],
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
					CV_CALIB_FIX_K3+ CV_CALIB_FIX_K4+CV_CALIB_FIX_K5); //CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4);//
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





/*
bool runAndSaveStereoCalibration(const string& outputFilename,
                const vector<vector<Point2f> >& imagePoints,
                Size imageSize, Size boardSize, float squareSize,
                float aspectRatio, int flags, Mat& cameraMatrix[2],
                Mat& distCoeffs[2], Mat& R, Mat& T, Mat& R1, Mat& P1,
                Mat& R2, Mat& P2, Mat& Q ,Rect & validRoi[2],bool writeExtrinsics,
                bool writePoints, VideoCapture & capture, Mat image )
{

    double rmsErr;
    double totalAvgErr = 0;

    bool ok = runStereoCalibration(imagePoints, imageSize, boardSize, squareSize,
                   aspectRatio, flags, cameraMatrix, distCoeffs,
                   R, T, R1, P1, R2, P2, Q, rmsErr, totalAvgErr);
    printf("%s. avg reprojection error = %.2f\n",
           ok ? "Calibration succeeded" : "Calibration failed",
           totalAvgErr);

    ok = displayAndAcceptRemappedStereoImages(capture, image, imageSize,cameraMatrix,distCoeffs);


    if( ok )
        saveStereoCameraParams( outputFilename, imageSize,
                         boardSize, squareSize, aspectRatio,
                         flags, cameraMatrix, distCoeffs,
                         R, T, R1, P1, R2, P2, Q,
                         writeExtrinsics ? rmsErr : 0.0,
                         writePoints ? imagePoints : vector<vector<Point2f> >(),
                         totalAvgErr );
    return ok;
}

*/
