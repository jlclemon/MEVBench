/*
 * AugmentedReality.cpp
 *
 *  Created on: May 25, 2011
 *      Author: jlclemon
 */

#include "AugmentedReality.hpp"
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



//#define TSC_TIMING
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
vector<TSC_VAL_w> aug_timingVector;
#endif
#ifdef CLOCK_GETTIME_TIMING
vector<struct timespec> aug_timeStructVector;
#endif

#ifdef TSC_TIMING
		void aug_writeTimingToFile(vector<TSC_VAL_w> timingVector)
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
		void aug_writeTimingToFile(vector<struct timespec> timingVector)
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

void readFileListFromFile(string fileListFilename, string filesBaseDir,vector<string> &fileList)
{
	ifstream fileListFile;

	fileListFile.open(fileListFilename.c_str(),ios::in);


	if(fileListFile.is_open())
	{
		while(fileListFile.good())
		{
			string currentLine;

			getline(fileListFile,currentLine);
			if(currentLine != "")
			{
				fileList.push_back(filesBaseDir+currentLine);
			}
		}

		fileListFile.close();
	}


}


bool loadAugmentedRealityConfigFile(vector<string> &commandArgs, string filename)
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
		cout << "Config file loaded!!!" << endl;
	}
	else
	{

		cout << "WARNING: Unable to open classification params config file" << endl;

	}
	return returnVal;

}


void parseAugmentedRealityConfigCommandVector(AugmentedRealityConfig & augmentedRealityConfig, vector<string> & commandStringVector)
{



	vector<string>::iterator it_start = commandStringVector.begin();
	vector<string>::iterator it_end = commandStringVector.end();
	vector<string>::iterator it_current;

	stringstream stringBuffer;



	augmentedRealityConfig.cameraId[0] = 1;
	augmentedRealityConfig.cameraId[1] = 0;

	augmentedRealityConfig.inputMethod =AUGMENT_REAL_SINGLE_STREAM;

	augmentedRealityConfig.calibFilename = "";

	augmentedRealityConfig.outputRawImageExt = ".png";
	augmentedRealityConfig.outputRawImageBaseName = "";
	augmentedRealityConfig.outputAugImageExt = ".png";
	augmentedRealityConfig.outputAugImageBaseName = "";

	augmentedRealityConfig.longList = false;

	augmentedRealityConfig.bufferedImageList = false;

	augmentedRealityConfig.showWindows = false;

	augmentedRealityConfig.noWaitKey = false;
	augmentedRealityConfig.pauseMode = false;

	augmentedRealityConfig.markerLostFrameCount = 10;

	augmentedRealityConfig.numberOfListLoops = 20;

	augmentedRealityConfig.tracking = false;

	//-desc sift -match knn_flann -classify linear_svm -queryDescFile test.yml -trainDescFile test.yml -loadClassificationStruct test.yml -loadMatchStruct test.yml
	for(it_current=it_start; it_current!=it_end; ++it_current)
	{

		if(!it_current->compare("-inputMethod") || !it_current->compare("-InputMethod"))
		{
			if(!(it_current+1)->compare("monoStream"))
			{
				augmentedRealityConfig.inputMethod=AUGMENT_REAL_SINGLE_STREAM;
			}
			if(!(it_current+1)->compare("stereoStream"))
			{
				augmentedRealityConfig.inputMethod=AUGMENT_REAL_STEREO_STREAM;
			}
			if(!(it_current+1)->compare("imageFileList"))
			{
				augmentedRealityConfig.useImageList = true;
				augmentedRealityConfig.inputMethod=AUGMENT_REAL_IMAGE_FILE_LIST;
			}


			if(!(it_current+1)->compare("videoFile"))
			{
				augmentedRealityConfig.inputMethod=AUGMENT_REAL_VIDEO_FILE;


			}

		}

		if(!it_current->compare("-camera0") || !it_current->compare("-Camera0"))
		{
			stringBuffer.str("");
			stringBuffer << *(it_current+1);
			if((stringBuffer >> augmentedRealityConfig.cameraId[0]).fail())
			{
				cout << "Camera0 Id could not be parsed." << endl;
			}

		}

		if(!it_current->compare("-camera1") || !it_current->compare("-Camera1"))
		{
			stringBuffer.str("");
			stringBuffer << *(it_current+1);
			if((stringBuffer >> augmentedRealityConfig.cameraId[1]).fail())
			{
				cout << "Camera0 Id could not be parsed." << endl;
			}

		}



		if(!it_current->compare("-imageList") || !it_current->compare("-ImageList"))
		{


			augmentedRealityConfig.imageListFilename = *(it_current+1);


		}

		if(!it_current->compare("-imageListBaseDir") || !it_current->compare("-ImageListBaseDir"))
		{


			augmentedRealityConfig.imageListBaseDir = *(it_current+1);


		}

		if(!it_current->compare("-markerLostFrameCount"))
		{
			stringBuffer.str("");
			stringBuffer.clear();
			stringBuffer << *(it_current+1);

			if((stringBuffer >> augmentedRealityConfig.markerLostFrameCount).fail())
			{
				cout << "Number of missing frames before marker considered lost could not be parsed" << endl;
			}



		}
		if(!it_current->compare("-nVertCores"))
		{
			stringBuffer.str("");
			stringBuffer.clear();
			stringBuffer << *(it_current+1);
			cout << *(it_current+1) << endl;
			if((stringBuffer >> augmentedRealityConfig.numberOfVerticalProcs).fail())
			{
				cout << "Number of vertical cores could not be parsed." << endl;
			}

		}

		if(!it_current->compare("-nHoriCores"))
		{
			stringBuffer.str("");
			stringBuffer.clear();
			stringBuffer << *(it_current+1);
			cout << *(it_current+1) << endl;
			if((stringBuffer >> augmentedRealityConfig.numberOfHorizontalProcs).fail())
			{
				cout << "Number of horizontal cores could not be parsed." << endl;
			}

		}

		if(!it_current->compare("-calibFile"))
		{
			augmentedRealityConfig.calibFilename =  *(it_current+1);

		}


		if( it_current->compare("-singleThreaded") == 0 || it_current->compare("-SingleThreaded") == 0)
		{
			augmentedRealityConfig.singleThreaded= true;
		}


		if( it_current->compare("-showWindows") == 0 || it_current->compare("-ShowWindows") == 0)
		{

			augmentedRealityConfig.showWindows = true;
		}

		if( it_current->compare("-noWaitKey") == 0 || it_current->compare("-NoWaitKey") == 0)
		{

			augmentedRealityConfig.noWaitKey = true;
		}



		if( it_current->compare("-bufferedImageList") == 0 || it_current->compare("-BufferedImageList") == 0)
		{

			augmentedRealityConfig.bufferedImageList = true;
		}
		if(!it_current->compare("-inputVideoFile") || !it_current->compare("-InputVideoFile"))
		{


			augmentedRealityConfig.inputVideoFilename = *(it_current+1);


		}
		if(!it_current->compare("-outputRawVideoFile") || !it_current->compare("-OutputRawVideoFile"))
		{


			augmentedRealityConfig.outputRawStreamFilename = *(it_current+1);


		}

		if(!it_current->compare("-outputAugmentedVideoFile") || !it_current->compare("-OutputAugmentedVideoFile"))
		{


			augmentedRealityConfig.outputAugmentedStreamFilename = *(it_current+1);


		}
		if(!it_current->compare("-outputRawImageBaseName") || !it_current->compare("-OutputRawImageBaseName"))
		{


			augmentedRealityConfig.outputRawImageBaseName = *(it_current+1);


		}
		if(!it_current->compare("-outputRawImageExt") || !it_current->compare("-OutputRawImageExt"))
		{


			augmentedRealityConfig.outputRawImageExt = *(it_current+1);


		}
		if(!it_current->compare("-outputAugImageBaseName") || !it_current->compare("-OutputAugImageBaseName"))
		{


			augmentedRealityConfig.outputAugImageBaseName = *(it_current+1);


		}
		if(!it_current->compare("-outputAugImageExt") || !it_current->compare("-OutputAugImageExt"))
		{


			augmentedRealityConfig.outputAugImageExt = *(it_current+1);


		}

		if(!it_current->compare("-pauseMode") || !it_current->compare("-PauseMode"))
		{


			augmentedRealityConfig.pauseMode = true;


		}
		if(!it_current->compare("-no-pauseMode") || !it_current->compare("-No-pauseMode"))
		{


			augmentedRealityConfig.pauseMode = false;


		}

		if(!it_current->compare("-no-tracking") || !it_current->compare("-No-tracking"))
		{


			augmentedRealityConfig.tracking = false;


		}

		if(!it_current->compare("-tracking") || !it_current->compare("-tracking"))
		{


			augmentedRealityConfig.tracking = true;


		}


		if(!it_current->compare("-longList") || !it_current->compare("-LongList"))
		{


			augmentedRealityConfig.longList = true;
			stringBuffer.str("");
			stringBuffer.clear();
			stringBuffer << *(it_current+1);
			cout << *(it_current+1) << endl;
			if((stringBuffer >> augmentedRealityConfig.numberOfListLoops).fail())
			{
				cout << "Number of list Loops could not be parsed." << endl;
			}


		}


	}


	if(augmentedRealityConfig.numberOfVerticalProcs ==1 && augmentedRealityConfig.numberOfHorizontalProcs ==1)
	{
		augmentedRealityConfig.singleThreaded = true;


	}
	else
	{
		augmentedRealityConfig.singleThreaded = false;

	}


	augmentedRealityConfig.bufferedImageListBufferSize = 5;


	return;

}

void augmentedRealitySetupOutput( AugmentedRealityConfig &augmentedRealityConfig, AugmentedRealityData &augmentedRealityData)
{
	if(augmentedRealityConfig.outputRawStreamFilename!= "")
	{

		augmentedRealityData.rawStreamWriter.open(augmentedRealityConfig.outputRawStreamFilename,CV_FOURCC('M','J','P','G'),30.0,augmentedRealityData.frameSize,true);


	}
	if(augmentedRealityConfig.outputAugmentedStreamFilename!= "")
	{

		augmentedRealityData.augmentedStreamWriter.open(augmentedRealityConfig.outputAugmentedStreamFilename,CV_FOURCC('P','I','M','1'),30.0,augmentedRealityData.frameSize, true);


	}
	if(augmentedRealityConfig.outputRawImageBaseName != "")
	{
		augmentedRealityData.currentOutputRawImageId = 0;
	}

	if(augmentedRealityConfig.outputAugImageBaseName != "")
	{
		augmentedRealityData.currentOutputAugImageId = 0;
	}



}

void augmentedRealityHandleWritingOutput( AugmentedRealityConfig &augmentedRealityConfig, AugmentedRealityData &augmentedRealityData)
{
	if(augmentedRealityData.rawStreamWriter.isOpened())
	{


		augmentedRealityData.rawStreamWriter << augmentedRealityData.currentFrame[0];

	}
	if(augmentedRealityData.augmentedStreamWriter.isOpened())
	{

		augmentedRealityData.augmentedStreamWriter << augmentedRealityData.currentAugmentedFrame[0];


	}
	if(augmentedRealityConfig.outputRawImageBaseName != "")
	{
		stringstream streamForId;
		string currentOutputId;
		streamForId << "_" <<augmentedRealityData.currentOutputRawImageId;

		streamForId >> currentOutputId;

		string imageFileName = augmentedRealityConfig.outputRawImageBaseName + currentOutputId + augmentedRealityConfig.outputRawImageExt;
		imwrite(imageFileName,augmentedRealityData.currentFrame[0]);
		augmentedRealityData.currentOutputRawImageId++;
	}
	if(augmentedRealityConfig.outputAugImageBaseName != "")
	{
		stringstream streamForId;
		string currentOutputId;
		streamForId << "_" <<augmentedRealityData.currentOutputAugImageId;

		streamForId >> currentOutputId;

		string imageFileName = augmentedRealityConfig.outputAugImageBaseName + currentOutputId + augmentedRealityConfig.outputAugImageExt;
		imwrite(imageFileName,augmentedRealityData.currentAugmentedFrame[0]);
		augmentedRealityData.currentOutputAugImageId++;
	}




}


void augmentedRealitySetupInput( AugmentedRealityConfig &augmentedRealityConfig, AugmentedRealityData &augmentedRealityData)
{
	switch(augmentedRealityConfig.inputMethod)
	{
		case AUGMENT_REAL_VIDEO_FILE:
		{

			break;
		}
		case AUGMENT_REAL_STEREO_STREAM:
		{

			break;
		}
		case AUGMENT_REAL_IMAGE_FILE_LIST:
		{

			if(augmentedRealityConfig.imageListFilename != "")
			{


				readFileListFromFile(augmentedRealityConfig.imageListFilename , augmentedRealityConfig.imageListBaseDir,augmentedRealityData.imageList);
				augmentedRealityData.currentInputImageId = 0;
				if(augmentedRealityConfig.bufferedImageList)
				{
					for(int i = augmentedRealityData.currentInputImageId; ((i < augmentedRealityConfig.bufferedImageListBufferSize)&& (i<augmentedRealityData.imageList.size())); i++)
					{
						augmentedRealityData.imageListBuffer[0].push_back(imread(augmentedRealityData.imageList[i]));
					}
				}
			}
			else
			{
				cout << "Invalid image list." << endl;
				exit(0);
			}


			Mat tmpFrameForSize = imread(augmentedRealityData.imageList[augmentedRealityData.currentInputImageId]);

			augmentedRealityData.frameSize.width = tmpFrameForSize.cols;
			augmentedRealityData.frameSize.height = tmpFrameForSize.rows;


			augmentedRealityData.currentListLoop = 0;

			break;
		}

		case AUGMENT_REAL_SINGLE_STREAM:
		default:
		{
			augmentedRealityData.capture[0].open(augmentedRealityConfig.cameraId[0]);

			if(!augmentedRealityData.capture[0].isOpened())
			{
				cout << "Error: Unable to opeb input stream" << endl;
				exit(0);

			}
			augmentedRealityData.frameSize.width = augmentedRealityData.capture[0].get(CV_CAP_PROP_FRAME_WIDTH);
			augmentedRealityData.frameSize.height = augmentedRealityData.capture[0].get(CV_CAP_PROP_FRAME_HEIGHT);
			break;
		}




	};




}


void setupAugmentedRealityCalibrationData(AugmentedRealityConfig &augmentedRealityConfig, AugmentedRealityData &augmentedRealityData)
{

	augmentedRealityData.outOfImages = false;
	if(augmentedRealityConfig.calibFilename.compare("") != 0)
	{

		augmentedRealityData.cameraCalibration.loadCameraParams(augmentedRealityConfig.calibFilename,true,true,true);

		//initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
		//initUndistortRectifyMap(augmentedRealityData.cameraCalibration.getCameraMatrix(), augmentedRealityData.cameraCalibration.getDistCoeffs(), Mat(), augmentedRealityData.cameraCalibration.getCameraMatrix(), augmentedRealityData.cameraCalibration.getImageSize(), CV_16SC2, augmentedRealityData.map1[0], augmentedRealityData.map2[0]);
		initUndistortRectifyMap(augmentedRealityData.cameraCalibration.getCameraMatrix(), augmentedRealityData.cameraCalibration.getDistCoeffs(), Mat(), augmentedRealityData.cameraCalibration.getCameraMatrix(), augmentedRealityData.frameSize, CV_16SC2, augmentedRealityData.map1[0], augmentedRealityData.map2[0]);


		cout << "Calibration file loaded" << endl;


	}
	else
	{
		cout << "Warning unable to load configuration file." << endl;


	}

}


void augmentedRealityGetNextFrames(AugmentedRealityConfig &augmentedRealityConfig, AugmentedRealityData &augmentedRealityData)
{

	switch(augmentedRealityConfig.inputMethod)
	{
		case AUGMENT_REAL_VIDEO_FILE:
		{

			break;
		}
		case AUGMENT_REAL_STEREO_STREAM:
		{

			break;
		}
		case AUGMENT_REAL_IMAGE_FILE_LIST:
		{
			if(!augmentedRealityConfig.bufferedImageList)
			{

				if(augmentedRealityData.currentInputImageId < augmentedRealityData.imageList.size())
				{
					augmentedRealityData.currentFrame[0] = imread(augmentedRealityData.imageList[augmentedRealityData.currentInputImageId]);
				}
				else
				{
					augmentedRealityData.currentFrame[0].release();

				}


			}
			else
			{
				if(augmentedRealityData.imageListBuffer[0].size() <=0)
				{
					if(augmentedRealityData.currentInputImageId <augmentedRealityData.imageList.size())
					{
						for(int i = augmentedRealityData.currentInputImageId; ((i < augmentedRealityConfig.bufferedImageListBufferSize)&& (i<augmentedRealityData.imageList.size())); i++)
						{
							augmentedRealityData.imageListBuffer[0].push_back(imread(augmentedRealityData.imageList[i]));
						}
					}
					else
					{

						if(augmentedRealityConfig.longList)
						{						

							if(augmentedRealityData.currentListLoop< augmentedRealityConfig.numberOfListLoops)
							{
								augmentedRealityData.currentInputImageId = 0;								

								for(int i = augmentedRealityData.currentInputImageId; ((i < augmentedRealityConfig.bufferedImageListBufferSize)&& (i<augmentedRealityData.imageList.size())); i++)
								{
									augmentedRealityData.imageListBuffer[0].push_back(imread(augmentedRealityData.imageList[i]));
								}



							}

						}
						else
						{

							augmentedRealityData.outOfImages = true;
						}
					}

				}
				augmentedRealityData.currentFrame[0]= augmentedRealityData.imageListBuffer[0].front();
				augmentedRealityData.imageListBuffer[0].pop_front();
			}
			augmentedRealityData.currentInputImageId++;

			if(augmentedRealityConfig.longList)
			{						
				if(augmentedRealityData.currentInputImageId >= augmentedRealityData.imageList.size())
				{
					augmentedRealityData.currentInputImageId = 0;

					augmentedRealityData.currentListLoop++;


					if(!(augmentedRealityData.currentListLoop< augmentedRealityConfig.numberOfListLoops))
					{

						augmentedRealityData.outOfImages = true;
					}


				}

			}

			


			break;
		}

		case AUGMENT_REAL_SINGLE_STREAM:
		default:
		{

			augmentedRealityData.capture[0] >> augmentedRealityData.frameBuffer[0];

			augmentedRealityData.currentFrame[0]=augmentedRealityData.frameBuffer[0].clone();



			break;
		}




	};

	if(augmentedRealityData.currentFrame[0].empty())
	{
		augmentedRealityData.outOfImages = true;

	}

}

void getObject3DPoints(vector<Point3f> & objectPoints)
{

	objectPoints.clear();
	objectPoints.push_back(Point3f(0,0,0));
	objectPoints.push_back(Point3f(0,19.6,0));
	objectPoints.push_back(Point3f(19.6,19.6,0));
	objectPoints.push_back(Point3f(19.6,0,0));



}

void getObject3DPointsWithOrientation(vector<Point3f> & objectPoints,int  normalizedOrientation)
{
	switch(normalizedOrientation)
	{




		case 90:
		{
			objectPoints.clear();
			objectPoints.push_back(Point3f(0,19.6,0));
			objectPoints.push_back(Point3f(19.6,19.6,0));
			objectPoints.push_back(Point3f(19.6,0,0));
			objectPoints.push_back(Point3f(0,0,0));

		}
			break;
		case 180:
		{
			objectPoints.clear();
			objectPoints.push_back(Point3f(19.6,19.6,0));
			objectPoints.push_back(Point3f(19.6,0,0));
			objectPoints.push_back(Point3f(0,0,0));
			objectPoints.push_back(Point3f(0,19.6,0));

		}
			break;
		case 270:
		{
			objectPoints.clear();
			objectPoints.push_back(Point3f(19.6,0,0));
			objectPoints.push_back(Point3f(0,0,0));
			objectPoints.push_back(Point3f(0,19.6,0));
			objectPoints.push_back(Point3f(19.6,19.6,0));

		}
			break;

		case 0:
		default:
		{
			objectPoints.clear();
			objectPoints.push_back(Point3f(0,0,0));
			objectPoints.push_back(Point3f(0,19.6,0));
			objectPoints.push_back(Point3f(19.6,19.6,0));
			objectPoints.push_back(Point3f(19.6,0,0));
		}
			break;


	};


}




void getMarkerNormalizationPoints(vector<Point2f> & normalizedPoints)
{
	normalizedPoints.clear();
	normalizedPoints.push_back(Point2f(0,0));
	normalizedPoints.push_back(Point2f(0,196));
	normalizedPoints.push_back(Point2f(196,196));
	normalizedPoints.push_back(Point2f(196,0));



}


int getNormalizedOrientationFromMarkerId(int markerId)
{
	int returnVal = -1;

	if(markerId == 249)
	{
		returnVal = 0;
	}
	if(markerId == 159)
	{
		returnVal = 90;

	}
	if(markerId==318)
	{
		returnVal = 180;


	}
	if(markerId ==498)
	{
		returnVal =  270;
	}


	return returnVal;
}


int calculateMarkerIdAndNormalizedOrientation(Mat binaryNormalizedImage, int & normalizedOrientation)
{

	int markerIdWidth = 3;
	int markerIdHeight = 3;

	int markerSquareIdLocationStepX = (binaryNormalizedImage.cols/2)/markerIdWidth;
	int markerSquareIdLocationStepY = (binaryNormalizedImage.rows/2)/markerIdHeight;
	int markerSquareIdLocationX = markerSquareIdLocationStepX/2 + binaryNormalizedImage.cols/4;
	int markerSquareIdLocationY = markerSquareIdLocationStepY/2 + binaryNormalizedImage.rows/4;
	Size idRegionSize(9,9);
	int markerId = 0x00;


	for(int i = 0; i <markerIdHeight; i++ )
	{
		for(int j = 0;j <markerIdWidth; j++)
		{
			Mat idRegion = binaryNormalizedImage(Range(markerSquareIdLocationY-idRegionSize.height/2,markerSquareIdLocationY+idRegionSize.height/2+1),Range(markerSquareIdLocationX-idRegionSize.width/2,markerSquareIdLocationX+idRegionSize.width/2+1));
			Scalar meanScalar = mean(idRegion);
			markerId = markerId << 1;
			if(meanScalar.val[0] <128)
			{

				markerId  = (markerId |1);

			}

			markerSquareIdLocationX+=markerSquareIdLocationStepX;
		}

		markerSquareIdLocationY+=markerSquareIdLocationStepY;
		markerSquareIdLocationX = markerSquareIdLocationStepX/2 + binaryNormalizedImage.cols/4;
	}

	normalizedOrientation = getNormalizedOrientationFromMarkerId(markerId);

	return markerId;

}

bool checkForValidMarker (int markerId)
{

	bool returnVal = false;
	if(markerId == 249 || markerId == 159 || markerId==318 || markerId ==498)
	{
		returnVal=true;
	}
	return returnVal;
}




void getAxes3DPoints(vector <Point3f> & axesPoints)
{
	//order: Origin, X limit, Y Limit, Z limit
	axesPoints.clear();
	axesPoints.push_back(Point3f(0,0,0));
	axesPoints.push_back(Point3f(19.6,0,0));
	axesPoints.push_back(Point3f(0,19.6,0));
	axesPoints.push_back(Point3f(0,0,-19.6));
}



void getBox3DPoints(vector<vector <Point3f> > & boxPoints)
{

	float cubeEdgeLength = 9.8f; //in cm
	Point3f cubeOrigin(-9.8,0,0 );
	//order: Origin, X limit, Y Limit, Z limit
	boxPoints.clear();

	vector<Point3f> boxPoints1;

	boxPoints1.push_back(Point3f(0,0,0));
	boxPoints1.push_back(Point3f(cubeEdgeLength,0,0));
	boxPoints1.push_back(Point3f(0,cubeEdgeLength,0));
	boxPoints1.push_back(Point3f(0,0,-cubeEdgeLength));

	boxPoints.push_back(boxPoints1);


	vector<Point3f> boxPoints2;

	boxPoints2.push_back(Point3f(cubeEdgeLength,cubeEdgeLength,0));
	boxPoints2.push_back(Point3f(0,cubeEdgeLength,0));
	boxPoints2.push_back(Point3f(cubeEdgeLength,0,0));
	boxPoints2.push_back(Point3f(cubeEdgeLength,cubeEdgeLength,-cubeEdgeLength));

	boxPoints.push_back(boxPoints2);


	vector<Point3f> boxPoints3;

	boxPoints3.push_back(Point3f(0,cubeEdgeLength,-cubeEdgeLength));
	boxPoints3.push_back(Point3f(cubeEdgeLength,cubeEdgeLength,-cubeEdgeLength));
	boxPoints3.push_back(Point3f(0,0,-cubeEdgeLength));
	boxPoints3.push_back(Point3f(0,cubeEdgeLength,0));

	boxPoints.push_back(boxPoints3);


	vector<Point3f> boxPoints4;

	boxPoints4.push_back(Point3f(cubeEdgeLength,0,-cubeEdgeLength));
	boxPoints4.push_back(Point3f(cubeEdgeLength,0,0));
	boxPoints4.push_back(Point3f(cubeEdgeLength,cubeEdgeLength,-cubeEdgeLength));
	boxPoints4.push_back(Point3f(0,0,-cubeEdgeLength));

	boxPoints.push_back(boxPoints4);


	for(int i = 0; i < boxPoints.size(); i++)
	{
		for(int j=0; j<boxPoints[i].size(); j++)
		{
			boxPoints[i][j].x += cubeOrigin.x;
			boxPoints[i][j].y += cubeOrigin.y;
			boxPoints[i][j].z += cubeOrigin.z;
		}
	}


}

void projectBoxToImage(AugmentedRealityConfig &augmentedRealityConfig, AugmentedRealityData &augmentedRealityData, Mat & image)
{
	vector<vector<Point3f> > boxPoints;

	vector<Point2f> currentBoxPointsInImage;

	getBox3DPoints(boxPoints);

	for(int i = 0; i < boxPoints.size(); i++)
	{
		currentBoxPointsInImage.clear();
		projectPoints(Mat(boxPoints[i]),augmentedRealityData.rVec[0],augmentedRealityData.tVec[0],augmentedRealityData.cameraCalibration.getCameraMatrix(),augmentedRealityData.cameraCalibration.getDistCoeffs(),currentBoxPointsInImage);
		for(int j =1; j< currentBoxPointsInImage.size(); j++)
		{


			//x axis
			line(image, currentBoxPointsInImage[0], currentBoxPointsInImage[j], CV_RGB(255, 0, 0), 3, CV_AA, 0);
		}
	}


}



void projectAxisToImage(AugmentedRealityConfig &augmentedRealityConfig, AugmentedRealityData &augmentedRealityData, Mat & image)
{
	vector <Point3f> axesPoints;
	getAxes3DPoints(axesPoints);

	vector<Point2f> axesPointsInImage;
	projectPoints(Mat(axesPoints),augmentedRealityData.rVec[0],augmentedRealityData.tVec[0],augmentedRealityData.cameraCalibration.getCameraMatrix(),augmentedRealityData.cameraCalibration.getDistCoeffs(),axesPointsInImage);

	//x axis
	line(image, axesPointsInImage[0], axesPointsInImage[1], CV_RGB(255, 0, 0), 3, CV_AA, 0);

	//y axis
	line(image, axesPointsInImage[0], axesPointsInImage[2], CV_RGB(0, 255, 0), 3, CV_AA, 0);

	//z axis
	line(image, axesPointsInImage[0], axesPointsInImage[3], CV_RGB(0, 0, 255), 3, CV_AA, 0);



}





int augmentedReality_main(int argc, const char * argv[])
{
	int key = -1;
	int lastKey = -1;
	AugmentedRealityConfig augmentedRealityConfig;
	AugmentedRealityData augmentedRealityData;


	cout << "Augmented Reality Application Starting" << endl;


	cout << "Beginning Setup" << endl;
	vector<string> commandLineArgs;
	for(int i = 0; i < argc; i++)
	{
		string currentString = argv[i];
		commandLineArgs.push_back(currentString);
	}

	if((commandLineArgs[1].compare("-configFile") ==0) || (commandLineArgs[1].compare("-ConfigFile") ==0))
	{
		loadAugmentedRealityConfigFile(commandLineArgs, commandLineArgs[2]);



	}


	parseAugmentedRealityConfigCommandVector(augmentedRealityConfig, commandLineArgs);



	augmentedRealitySetupInput(	augmentedRealityConfig,augmentedRealityData);
	setupAugmentedRealityCalibrationData(augmentedRealityConfig, augmentedRealityData);


	cout << "Setup Complete" << endl;

	if(augmentedRealityConfig.inputMethod ==AUGMENT_REAL_SINGLE_STREAM)
	{


		cout << "Allowing stream to setup" << endl;
		namedWindow("Stream warmup");

		key = -1;
		lastKey = -1;
		while(key != 27 && key != 'r' && key != 'd' && key != ' ')
		{
			augmentedRealityData.capture[0] >> augmentedRealityData.frameBuffer[0];
			augmentedRealityData.currentFrame[0]=augmentedRealityData.frameBuffer[0].clone();
			imshow("Stream warmup",augmentedRealityData.currentFrame[0]);

			key = waitKey(33) & 0xff;
		}

		destroyWindow("Stream warmup");

	}

	augmentedRealitySetupOutput( augmentedRealityConfig,augmentedRealityData);
	if(augmentedRealityConfig.showWindows)
	{
		cout << "Creating display windows" << endl;

		namedWindow("Current Frame Original");
		namedWindow("Current Frame Grayscale");
		namedWindow("Current Frame Thresholded");
		namedWindow("Current Frame Adaptive Thresholded");
		namedWindow("Current Frame Contours");
		namedWindow("Current Frame Approx Contours");
		namedWindow("Current Frame Corners");
		namedWindow("Current Frame Normalized");
		namedWindow("Current Frame Augmented");
		namedWindow("Tracking Corners");
	}
	cout << "Beginning Main Loop" << endl;


	key = -1;
	lastKey = -1;

	bool done = false;

	Mat tmpBuffer;

#ifdef USE_MARSS
	cout << "Switching to simulation in Augmented Reality." << endl;
	ptlcall_switch_to_sim();
	//ptlcall_single_enqueue("-logfile augReality.log");
	ptlcall_single_enqueue("-stats augReality.stats");
	//ptlcall_single_enqueue("-loglevel 0");
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
	READ_TIMESTAMP_WITH_WRAPPER( aug_timingVector[0] );
#endif
#ifdef CLOCK_GETTIME_TIMING
	GET_TIME_WRAPPER(aug_timeStructVector[0]);
#endif

	int missingCorners = 0;
	bool trackingValid = false;
	int  normalizedOrienation;
	while(!done)
	{
		augmentedRealityGetNextFrames(augmentedRealityConfig, augmentedRealityData);

		if(augmentedRealityData.outOfImages == true)
		{

			break;
		}

		Mat unmodifiedFrames[2];

		unmodifiedFrames[0] = augmentedRealityData.currentFrame[0].clone();


		if(!augmentedRealityData.cameraCalibration.getDistCoeffs().empty())
		{
			tmpBuffer = augmentedRealityData.currentFrame[0].clone();
	        remap(tmpBuffer, augmentedRealityData.currentFrame[0], augmentedRealityData.map1[0], augmentedRealityData.map2[0], INTER_LINEAR);


		}
		Mat augmentedImage[2];
		augmentedImage[0] = augmentedRealityData.currentFrame[0].clone();

		//unmodifiedFrames[1] = augmentedRealityData.currentFrame[1].clone();
		Mat grayScaleFrames[2];

		cvtColor(augmentedRealityData.currentFrame[0],grayScaleFrames[0], CV_BGR2GRAY);




		Mat adaptiveThresholdedFrames[2];
		Mat thresholdedFrames[2];
//		adaptiveThreshold(grayScaleFrames[0], adaptiveThresholdedFrames[0], 255, ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY_INV, 65, 10);
		threshold(grayScaleFrames[0], thresholdedFrames[0], 128, 255, THRESH_BINARY_INV);


		Mat contourImage[2];
		Mat approxContourImage[2];
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		Mat thresholdedFramesForContours = thresholdedFrames[0].clone();
		findContours(thresholdedFramesForContours, contours, hierarchy, CV_RETR_CCOMP,CV_CHAIN_APPROX_TC89_KCOS);


		contourImage[0].create(thresholdedFrames[0].rows,thresholdedFrames[0].cols,CV_8UC3);
		contourImage[0].setTo(Scalar(0));

		approxContourImage[0].create(thresholdedFrames[0].rows,thresholdedFrames[0].cols,CV_8UC3);
		approxContourImage[0].setTo(Scalar(0));






		int idx = 0;
		bool foundMarker = false;



	    for( ; idx >= 0; idx = hierarchy[idx][0] )
	    {
	        Scalar color( rand()&255, rand()&255, rand()&255 );
	        drawContours( contourImage[0], contours, idx, color, CV_FILLED, 8, hierarchy,0 );

			Mat curve(contours[idx]);
			vector <vector<Point> >approxCurve(1);
	        approxPolyDP(curve,  approxCurve[0], 3.0, true);

//	        drawContours( approxContourImage[0], approxCurve, 0, color, CV_FILLED, 8);

//	        cout << "Contour Points Count: "<< contours[idx].size()<< endl;
//	        cout << "Contour Points Count: "<< approxCurve[0].size()<< endl;
	        if(approxCurve[0].size() ==4)
	        {

	        	vector<Point2f> normalizedPoints;


	        	getMarkerNormalizationPoints(normalizedPoints);
	        	vector<Point2f> corners(approxCurve[0].size());
	        	for(int i = 0; i < corners.size(); i++)
	        	{
	        		corners[i] = approxCurve[0][i];
//	        		cout <<"Approx x: " <<corners[i].x << " y: "<<corners[i].y << endl;
	        	}

				Size subPixelSize(5,5);
				Size subPixelDeadZoneSize(-1,-1);

	        	cornerSubPix(grayScaleFrames[0], corners, subPixelSize, subPixelDeadZoneSize, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

				if(augmentedRealityData.prevCorners.size() > 0)
				{
					if( (abs(corners[0].x - augmentedRealityData.prevCorners[0].x) < 1) &&(abs(corners[0].y - augmentedRealityData.prevCorners[0].y) < 1)  )
					{
						corners = augmentedRealityData.prevCorners;



					}
				}

//	        	double areaVal = contourArea(Mat(corners));

//	        	cout << "Area Val: "<<areaVal << endl;

	        	Mat srcPoints(corners);
	        	Mat dstPoints(normalizedPoints);
	        	Mat normalHom = findHomography(srcPoints, dstPoints); //Mat& status, int method=0, double ransacReprojThreshold=3)
	        	Mat preWarp = thresholdedFrames[0].clone();
	        	Mat normalizedImage;
	        	normalizedImage.create(192,192,CV_8UC3);
	        	warpPerspective(preWarp, normalizedImage,normalHom, Size(192,192));

	    	    //int  normalizedOrienation;
	        	int markerId = calculateMarkerIdAndNormalizedOrientation(normalizedImage,normalizedOrienation);
	        	//cout << "Current Marker Id: "<<markerId << endl;


	        	int tmpKey = -1;//waitKey(30) & 0xff;

	        	if(((!augmentedRealityData.map1[0].empty())&& (tmpKey =='d')) || checkForValidMarker(markerId))
	        	{

	        		augmentedRealityData.normalizedOrientation = normalizedOrienation;
	        		augmentedRealityData.currentMarkerCorners = corners;
	        		foundMarker = true;
	        		missingCorners = 0;

	        		break;
	        	}
//	        	imshow ("Current Frame Normalized", normalizedImage);
//	        	waitKey(0);
	        }


//	        imshow("Current Frame Contours",contourImage[0]);
//			imshow("Current Frame Approx Contours",approxContourImage[0]);

			//waitKey(0);
	    }


	    if(!augmentedRealityData.prevFrame[0].empty() && augmentedRealityConfig.tracking)
		{
			vector<uchar> opticalFlowStatus;
			vector<float> opticalFlowErr;

			calcOpticalFlowPyrLK(augmentedRealityData.prevFrame[0], grayScaleFrames[0], augmentedRealityData.prevCorners, augmentedRealityData.trackingCorners, opticalFlowStatus,  opticalFlowErr, Size(15, 15), 3, TermCriteria( TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01), 0.5, 0);
			int cornersFound = 0;
			for(unsigned int i =0; i < opticalFlowStatus.size(); i++)
			{
				cornersFound += opticalFlowStatus[i];

			}

			Mat trackingDisplayMat = augmentedRealityData.currentFrame[0].clone();

			for(int i = 0; i< augmentedRealityData.trackingCorners.size(); i++)
			{

				if(opticalFlowStatus[i] !=0)
				{
					circle(trackingDisplayMat,augmentedRealityData.trackingCorners[i],6,Scalar(0,0,255),-1);

				}


			}


			if(augmentedRealityConfig.showWindows)
			{

				imshow("Tracking Corners", trackingDisplayMat);

			}
		    if(!foundMarker &&  cornersFound == 4)
		    {

		    	augmentedRealityData.currentMarkerCorners = augmentedRealityData.trackingCorners;
				missingCorners++;

				Size subPixelSize(5,5);
				Size subPixelDeadZoneSize(-1,-1);

				//corners = augmentedRealityData.trackingCorners;
	        	cornerSubPix(grayScaleFrames[0], augmentedRealityData.currentMarkerCorners, subPixelSize, subPixelDeadZoneSize, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
				if(augmentedRealityData.prevCorners.size() > 0)
				{
					if( (abs(augmentedRealityData.currentMarkerCorners[0].x - augmentedRealityData.prevCorners[0].x) < 1) &&(abs(augmentedRealityData.currentMarkerCorners[0].y - augmentedRealityData.prevCorners[0].y) < 1)  )
					{
						augmentedRealityData.currentMarkerCorners = augmentedRealityData.prevCorners;



					}
				}


				augmentedRealityData.normalizedOrientation = augmentedRealityData.lastNormalizedOrientation;
				if(missingCorners >augmentedRealityConfig.markerLostFrameCount)
				{
					missingCorners = augmentedRealityConfig.markerLostFrameCount+1;
					trackingValid = false;


				}
				else
				{

					trackingValid = true;
				}




		    }



		}
	    else
	    {

	    	trackingValid = false;

	    }





	    //int  normalizedOrienation;
    	//int markerId = calculateMarkerIdAndNormalizedOrientation(normalizedImage,normalizedOrienation);
    	//cout << "Current Marker Id: "<<markerId << endl;


    	//int tmpKey = -1;//waitKey(30) & 0xff;

    	//if(((!augmentedRealityData.map1[0].empty())&& (tmpKey =='d')) || checkForValidMarker(markerId))
	    if(foundMarker || trackingValid)
	    {
    		Mat rvec;
    		Mat tvec;
    		vector<Point3f> objectPoints;
    		//getObject3DPoints(objectPoints);
    		getObject3DPointsWithOrientation(objectPoints,augmentedRealityData.normalizedOrientation);


        	augmentedRealityData.prevFrame[0] = grayScaleFrames[0];
        	augmentedRealityData.prevCorners = augmentedRealityData.currentMarkerCorners;
        	augmentedRealityData.lastNormalizedOrientation = augmentedRealityData.normalizedOrientation;
    		solvePnP(Mat(objectPoints),Mat(augmentedRealityData.currentMarkerCorners),augmentedRealityData.cameraCalibration.getCameraMatrix(),augmentedRealityData.cameraCalibration.getDistCoeffs(),augmentedRealityData.rVec[0],augmentedRealityData.tVec[0],false);
//	        		cout << "RVec: "  << augmentedRealityData.rVec[0] << "\n TVec: " << augmentedRealityData.tVec[0] << endl;


    		projectAxisToImage(augmentedRealityConfig, augmentedRealityData, augmentedImage[0]);
    		projectBoxToImage(augmentedRealityConfig, augmentedRealityData, augmentedImage[0]);




    		//waitKey(0);
    	}





	    if(augmentedRealityConfig.showWindows)
		{
			imshow("Current Frame Augmented", augmentedImage[0]);
			imshow("Current Frame Original",unmodifiedFrames[0]);
//		imshow("Current Frame Grayscale",grayScaleFrames[0]);
//		imshow("Current Frame Thresholded",thresholdedFrames[0]);
//		imshow("Current Frame Adaptive Thresholded",adaptiveThresholdedFrames[0]);
//		imshow("Current Frame Contours",contourImage[0]);
//		imshow("Current Frame Corners",unmodifiedFrames[0]);


		}
		augmentedRealityData.currentAugmentedFrame[0] = augmentedImage[0];
		augmentedRealityHandleWritingOutput( augmentedRealityConfig, augmentedRealityData);


		lastKey = key;
		if(!augmentedRealityConfig.noWaitKey)
		{

			if (augmentedRealityConfig.pauseMode)
			{

				key = waitKey(0);

			}
			else
			{
				key = waitKey(10) & 0xff;
			}
		}
		else

		{
			key = -1;

		}


		if(key == 'q' || key =='d' || key == 27)
		{

			done = true;
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
	READ_TIMESTAMP_WITH_WRAPPER( aug_timingVector[0+ aug_timingVector.size()/2] );
#endif
#ifdef CLOCK_GETTIME_TIMING
	GET_TIME_WRAPPER(aug_timeStructVector[0+ aug_timeStructVector.size()/2]);
#endif
#ifdef TSC_TIMING
		aug_writeTimingToFile(aug_timingVector);
#endif
#ifdef CLOCK_GETTIME_TIMING
		aug_writeTimingToFile(aug_timeStructVector);
#endif
	cout << "Augmented Reality complete" << endl;
	return 0;

}



#ifndef AUGMENTED_REALITY_MODULE

int main(int argc, const char * argv[])
{
#ifdef TSC_TIMING
	aug_timingVector.resize(16);
#endif


	return augmentedReality_main(argc, argv);

}
#endif
