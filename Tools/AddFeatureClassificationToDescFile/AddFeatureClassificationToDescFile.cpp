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

#include <string>
#include <fstream>
#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <errno.h>
#include <pthread.h>


using namespace cv;
using namespace std;


#define MAX_NUMBER_OF_DESCRIPTORS_TO_GET 1000

enum FeaturesLocalizationAlgos
{
	FEATURE_LOCALIZATION_SIFT = 1,
	FEATURE_LOCALIZATION_SURF = 2,
	FEATURE_LOCALIZATION_FAST = 4,
	FEATURE_LOCALIZATION_HoG = 8,
	FEATURE_LOCALIZATION_SLIDING_WINDOW = 16,
	FEATURE_LOCALIZATION_CENTER_POINT = 32,
	NUMBER_OF_FEATURE_LOCALIZATION_ALGOS=6

};


enum FeaturesDescriptorAlgos
{
	FEATURE_DESC_SIFT = 1,
	FEATURE_DESC_SURF = 2,
	FEATURE_DESC_FAST = 4,
	FEATURE_DESC_HoG = 8,
	FEATURE_DESC_BRIEF = 16,
	NUMBER_OF_FEATURE_DESC_ALGOS = 4

};

enum FeaturesMatchingAlgos
{
	FEATURE_MATCHER_BRUTEFORCE = 1,
	FEATURE_MATCHER_KNN_FLANN = 2,
	FEATURE_MATCHER_EUCL_DIST = 4,
	FEATURE_MATCHER_RADIUS_FLANN = 8,
	FEATURE_MATCHER_KNN_HESS_KDTREE = 16,
	NUMBER_OF_FEATURE_MATCHER_ALGOS = 5

};


enum FeaturesClassificationAlgos
{
	FEATURE_CLASSIFIER_LINEAR_SVM = 1,
	FEATURE_CLASSIFIER_BOOSTED_TREE_STUMPS = 2,
	FEATURE_CLASSIFIER_KNN_FLANN = 4,
	FEATURE_CLASSIFIER_KNN_HESS_KDTREE = 8,
	FEATURE_CLASSIFIER_GEOMETRIC_CENTER = 16,
	NUMBER_OF_FEATURE_CLASSIFIER_ALGOS = 5

};


bool checkdescriptorHoGType(string filename)
{
	bool returnVal = false;
	FileStorage fs(filename,FileStorage::READ);
	FeaturesDescriptorAlgos descriptorType = (FeaturesDescriptorAlgos)((int)fs["DescType"]);

	if(descriptorType == FEATURE_DESC_HoG)
	{

		returnVal = true;

	}

	return returnVal;
}



bool loadFeatureDescriptorsAndOrKeypoints( string filename,  string &descTypeString,string &localTypeString ,FeaturesDescriptorAlgos& descriptorType,FeaturesLocalizationAlgos& localizationMethod,Ptr<FeatureDetector> &detector,vector<KeyPoint> * keypoints,HOGDescriptor*  &descriptorExtractor, Mat * descriptors)
{

	FileStorage fs(filename,FileStorage::READ);

	descriptorType = (FeaturesDescriptorAlgos)((int)fs["DescType"]);
	localizationMethod = (FeaturesLocalizationAlgos)((int)fs["LocalAlgo"]);
	descTypeString = (string)fs["DescTypeString"];
	localTypeString = (string)fs["LocalAlgoString"];

	descriptorExtractor = new HOGDescriptor;
	if(descriptorExtractor != NULL)
	{

		FileNode descStruct = fs["DescStruct"];
		descriptorExtractor->read(descStruct);
	}


	if(descriptors != NULL)
	{
		fs["Descriptors"] >> *descriptors;
	}
	if(keypoints !=NULL)
	{
		FileNode keyPointsNode = fs["Keypoints"];
		read((const FileNode)keyPointsNode, *keypoints);
	}
	switch(localizationMethod)
	{
		case FEATURE_LOCALIZATION_SIFT:
			{
				detector = new SiftFeatureDetector;

			}
			break;

		case FEATURE_LOCALIZATION_SURF:
			{
				detector = new SurfFeatureDetector;



			}
			break;
		case FEATURE_LOCALIZATION_FAST:
			{


				detector = new FastFeatureDetector;

			}
			break;
		case FEATURE_LOCALIZATION_HoG:
			{


			}
			break;
		default:
			{
				detector = NULL;
			}
			break;
	}

	if(detector != NULL)
	{
		FileNode detectorNode = fs["LocalStruct"];
		detector->read(detectorNode);

	}

	return true;

}
bool loadFeatureDescriptorsAndOrKeypoints( string filename,  string &descTypeString,string &localTypeString ,FeaturesDescriptorAlgos& descriptorType,FeaturesLocalizationAlgos& localizationMethod,Ptr<FeatureDetector>& detector,vector<KeyPoint> * keypoints,Ptr<DescriptorExtractor> & descriptorExtractor, Mat * descriptors)
{
	FileStorage fs(filename,FileStorage::READ);

	descriptorType = (FeaturesDescriptorAlgos)((int)fs["DescType"]);
	localizationMethod = (FeaturesLocalizationAlgos)((int)fs["LocalAlgo"]);
	descTypeString = (string)fs["DescTypeString"];
	localTypeString = (string)fs["LocalAlgoString"];


   switch(descriptorType)
	{
		case FEATURE_DESC_SIFT:
			{
				descriptorExtractor = new SiftDescriptorExtractor;

			}
			break;

		case FEATURE_DESC_SURF:
			{

				descriptorExtractor = new SurfDescriptorExtractor;



			}
			break;
		case FEATURE_DESC_FAST:

			break;
		case FEATURE_DESC_HoG:
			break;
		case FEATURE_DESC_BRIEF:
			{

				descriptorExtractor = new BriefDescriptorExtractor;
			}
			break;

		default:
				descriptorExtractor = NULL;
			break;
	}






	if(descriptorExtractor != NULL)
	{

		FileNode descStruct = fs["DescStruct"];
		descriptorExtractor->read(descStruct);
	}
	if(descriptors != NULL)
	{
		fs["Descriptors"] >> *descriptors;
	}
	if(keypoints !=NULL)
	{
		FileNode keyPointsNode = fs["Keypoints"];
		read((const FileNode)keyPointsNode, *keypoints);
	}


	switch(localizationMethod)
	{
		case FEATURE_LOCALIZATION_SIFT:
			{
				detector = new SiftFeatureDetector;

			}
			break;

		case FEATURE_LOCALIZATION_SURF:
			{
				detector = new SurfFeatureDetector;



			}
			break;
		case FEATURE_LOCALIZATION_FAST:
			{


				detector = new FastFeatureDetector;

			}
			break;
		case FEATURE_LOCALIZATION_HoG:
			{


			}
			break;
		default:
			{
				detector = NULL;
			}
			break;
	}

	if(detector != NULL)
	{
		FileNode detectorNode = fs["LocalStruct"];
		detector->read(detectorNode);

	}


return true;


}

string convertDescTypeToString(FeaturesDescriptorAlgos descriptorType)
{

	string retVal("");
	switch(descriptorType)
	{
		case FEATURE_DESC_SIFT:
				retVal = "Sift";
			break;
		case FEATURE_DESC_SURF:
				retVal = "Surf";
			break;
		case FEATURE_DESC_FAST:
				retVal = "Fast";
			break;
		case FEATURE_DESC_HoG:
				retVal = "Hog";
			break;
		case FEATURE_DESC_BRIEF:
				retVal = "Brief";
			break;
		default:
				retVal = "Unknown";
			break;
	}

	return retVal;


}

string convertLocalTypeToString(FeaturesLocalizationAlgos localizationMethod)
{

	string retVal("");
	switch(localizationMethod)
	{
		case FEATURE_LOCALIZATION_SIFT:
				retVal = "Sift";
			break;
		case FEATURE_LOCALIZATION_SURF:
				retVal = "Surf";
			break;
		case FEATURE_LOCALIZATION_FAST:
				retVal = "Fast";
			break;
		case FEATURE_LOCALIZATION_HoG:
				retVal = "Hog";
			break;
		case FEATURE_LOCALIZATION_SLIDING_WINDOW:
				retVal = "SlidingWindow";
			break;
		case FEATURE_LOCALIZATION_CENTER_POINT:
				retVal = "CenterPoint";
			break;


		default:
				retVal = "Unknown";
			break;
	}

	return retVal;


}
bool saveDescriptorClasses(FileStorage & fs, string & descriptorClassesInfo,Mat descriptorClasses)
{



	fs << "DescriptorClasses" << descriptorClasses;

	fs<< "DescriptorsClassesInfo" << descriptorClassesInfo;

	return true;
}


bool saveFeatureDescriptorsAndOrKeypoints( string filename,  FeaturesDescriptorAlgos descriptorType,FeaturesLocalizationAlgos localizationMethod,Ptr<FeatureDetector>& detector,vector<KeyPoint> & keypoints,Ptr<DescriptorExtractor>&  descriptorExtractor, Mat & descriptors, Mat & descriptorClasses)
{
	FileStorage fs(filename,FileStorage::WRITE);

	fs << "DescType" << descriptorType << "LocalAlgo" << localizationMethod;
	fs << "DescTypeString"<< convertDescTypeToString(descriptorType) << "LocalAlgoString"<<convertLocalTypeToString(localizationMethod);
	string descTypeString = convertDescTypeToString(descriptorType);
	if(descriptorExtractor != NULL)
	{
		fs << "DescStruct" << "{";
		descriptorExtractor->write(fs);
		fs << "}";
	}


	if(!descriptors.empty())
	{
		fs << "Descriptors" << descriptors;
	}

	saveDescriptorClasses(fs, descTypeString, descriptorClasses);
	if(keypoints.size() >0)
	{
		write(fs,"Keypoints",keypoints);
	}
	if(detector != NULL)
	{
		fs << "LocalStruct" << "{";
		detector->write(fs);
		fs << "}";
	}

	return true;

}

bool saveFeatureDescriptorsAndOrKeypoints( string filename,  FeaturesDescriptorAlgos descriptorType,FeaturesLocalizationAlgos localizationMethod,Ptr<FeatureDetector>& detector,vector<KeyPoint> & keypoints,HOGDescriptor* descriptorExtractor, Mat & descriptors, Mat & descriptorClasses)
{
	FileStorage fs(filename,FileStorage::WRITE);

	fs << "DescType" << descriptorType << "LocalAlgo" << localizationMethod;
	fs << "DescTypeString"<< convertDescTypeToString(descriptorType) << "LocalAlgoString"<<convertLocalTypeToString(localizationMethod);
	string descTypeString = convertDescTypeToString(descriptorType);
	if(descriptorExtractor != NULL)
	{

		descriptorExtractor->write(fs,"DescStruct");
	}


	if(!descriptors.empty())
	{
		fs << "Descriptors" << descriptors;
	}

	saveDescriptorClasses(fs, descTypeString, descriptorClasses);


	if(keypoints.size() >0)
	{
		write(fs,"Keypoints",keypoints);
	}
	if(detector != NULL)
	{
		fs << "LocalStruct" << "{";
		detector->write(fs);
		fs << "}";
	}

	return true;

}



Mat loadDescriptorClassesFromTextFile(string filename, string & descriptorClassesInfo)
{
	std::ifstream classesFile;
	float currentClass;
	Mat descriptorClasses;

	descriptorClassesInfo = "From Text File";
	classesFile.open(filename.c_str());






	if(classesFile.good())
	{


		descriptorClasses.create(1,1,CV_32FC1);
		if(classesFile >>currentClass)
		{

			descriptorClasses.at<float>(0,0) = currentClass;

		}
		while(classesFile >>currentClass)
		{

			descriptorClasses.push_back(currentClass);
		}


	}
	else
	{

		cout << "WARNING: Unable to open descriptor classes file" << endl;

	}
	return descriptorClasses;



}

Mat loadDescriptorClasses(string filename, string & descriptorClassesInfo)
{
	FileStorage fs(filename,FileStorage::READ);
	Mat descriptorClasses;


	descriptorClassesInfo = (string)fs["DescriptorsClassesInfo"];

	fs["DescriptorClasses"] >> descriptorClasses;

	return descriptorClasses;
}

Mat loadDescriptorClasses(FileStorage & fs, string & descriptorClassesInfo)
{

	Mat descriptorClasses;


	descriptorClassesInfo = (string)fs["DescriptorsClassesInfo"];

	fs["DescriptorClasses"] >> descriptorClasses;

	return descriptorClasses;
}



bool saveDescriptorClasses(string filename, string & descriptorClassesInfo, Mat descriptorClasses)
{
	FileStorage fs(filename,FileStorage::READ);



	fs << "DescriptorClasses" << descriptorClasses;

	fs<< "DescriptorsClassesInfo" << descriptorClassesInfo;

	return true ;
}


bool loadAddFeatureClassificationConfigFile(vector<string> &commandArgs)
{

	std::ifstream configFile;
	int i = 3;
	string tmpString;
	bool returnVal = false;


	configFile.open(commandArgs[2].c_str());


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

		cout << "WARNING: Unable to open add classifications params config file" << endl;

	}
	return returnVal;

}



int addFeatureClassificationToDescFile_main(int argc,const  char * argv[])
{
	vector<string> commandLineArgs;
	for(int i = 0; i < argc; i++)
	{
		string currentString = argv[i];
		commandLineArgs.push_back(currentString);
	}

	string classesTextFile;
	string descriptorInputFile;
	string descriptorInputFile2;
	string descriptorOutputFile;
	float singleClass = 0.0f;
	float singleClass2 = 1.0f;
	bool useSingleClass = true;
	bool doubleLoad = false;

	if((commandLineArgs[1].compare("-configFile") ==0) || (commandLineArgs[1].compare("-ConfigFile") ==0))
	{
		loadAddFeatureClassificationConfigFile(commandLineArgs);



	}




	vector<string> commandStringVector = commandLineArgs;



	vector<string>::iterator it_start = commandStringVector.begin();
	vector<string>::iterator it_end = commandStringVector.end();
	vector<string>::iterator it_current;

	stringstream stringBuffer;



	//-desc sift -match knn_flann -classify linear_svm -queryDescFile test.yml -trainDescFile test.yml -loadClassificationStruct test.yml -loadMatchStruct test.yml
	for(it_current=it_start; it_current!=it_end; ++it_current)
	{


		if(!it_current->compare("-classesTextFile") || !it_current->compare("-ClassesTextFile"))
		{
			useSingleClass = false;
			classesTextFile =*(it_current+1);


		}

		if(!it_current->compare("-descriptorInputFile") || !it_current->compare("-DescriptorInputFile"))
		{
			descriptorInputFile =*(it_current+1);


		}
		if(!it_current->compare("-descriptorInputFile2") || !it_current->compare("-DescriptorInputFile2"))
		{
			doubleLoad = true;
			descriptorInputFile2 =*(it_current+1);


		}


		if(!it_current->compare("-descriptorOutputFile") || !it_current->compare("-DescriptorOutputFile"))
		{
			descriptorOutputFile =*(it_current+1);


		}

		if(!it_current->compare("-singleClass") || !it_current->compare("-SingleClass"))
		{


			stringBuffer.str("");
			stringBuffer.clear();
			stringBuffer << *(it_current+1);


			useSingleClass = true;
			if((stringBuffer >> singleClass).fail())
			{

				cout << "WARNING: Could not parse singleClass"<< endl;


			}


		}

		if(!it_current->compare("-singleClass2") || !it_current->compare("-SingleClass2"))
		{


			stringBuffer.str("");
			stringBuffer.clear();
			stringBuffer << *(it_current+1);


			useSingleClass = true;
			if((stringBuffer >> singleClass2).fail())
			{

				cout << "WARNING: Could not parse singleClass"<< endl;


			}


		}


	}


	string descriptorClassesInfo;
	 string descTypeString;
	 string localTypeString;
	 FeaturesDescriptorAlgos descriptorType;
	 FeaturesLocalizationAlgos localizationMethod;
	 Ptr<FeatureDetector> detector;
	 vector<KeyPoint> keypoints;
	 HOGDescriptor* descriptorExtractor;
	 Mat descriptors;
	 Mat descriptorClasses;

	 vector<KeyPoint> keypoints2;
	 Mat descriptors2;
	 Mat descriptorClasses2;
	 int descOrigCount = 0;


	cout << "Starting conversion .......";

	if(checkdescriptorHoGType(descriptorInputFile))
	{

		loadFeatureDescriptorsAndOrKeypoints( descriptorInputFile,  descTypeString,localTypeString ,descriptorType,localizationMethod,detector, &keypoints,descriptorExtractor,&descriptors);
		if(descriptors.rows > MAX_NUMBER_OF_DESCRIPTORS_TO_GET)
		{
			Mat tmpDesc = descriptors.rowRange(0,MAX_NUMBER_OF_DESCRIPTORS_TO_GET).clone();
			descriptors = tmpDesc;
			vector<KeyPoint> tmpKeypoints;

			for(int j = 0; j<MAX_NUMBER_OF_DESCRIPTORS_TO_GET; j++)
			{
				if(j< keypoints.size())
				{
					tmpKeypoints.push_back(keypoints[j]);
				}

			}
			keypoints = tmpKeypoints;
		}

		descOrigCount = descriptors.rows;
		if(doubleLoad)
		{

			loadFeatureDescriptorsAndOrKeypoints( descriptorInputFile2,  descTypeString,localTypeString ,descriptorType,localizationMethod,detector, &keypoints2,descriptorExtractor,&descriptors2);
			if(descriptors2.rows > MAX_NUMBER_OF_DESCRIPTORS_TO_GET)
			{
				Mat tmpDesc = descriptors2.rowRange(0,MAX_NUMBER_OF_DESCRIPTORS_TO_GET).clone();
				descriptors2 = tmpDesc;
				vector<KeyPoint> tmpKeypoints2;

				for(int j = 0; j<MAX_NUMBER_OF_DESCRIPTORS_TO_GET; j++)
				{

					if(j< keypoints2.size())
					{

						tmpKeypoints2.push_back(keypoints2[j]);


					}

				}
				keypoints2 = tmpKeypoints2;
			}





			for(int i = 0; i < keypoints2.size(); i++)
			{
				keypoints.push_back(keypoints2[i]);

			}
			descriptors.push_back(descriptors2);
		}



		if(!useSingleClass)
		{
			descriptorClasses =  loadDescriptorClassesFromTextFile(classesTextFile, descriptorClassesInfo);

			if(descriptorClasses.rows != descriptors.rows)
			{
				cout << "WARNING: Number of classes and descriptors to not match" << endl;


			}
		}
		else
		{
			descriptorClasses.create(descOrigCount,1,CV_32FC1);
			descriptorClasses.setTo(Scalar(singleClass));

			if(doubleLoad)
			{

				descriptorClasses2.create(descriptors2.rows,1,CV_32FC1);
				descriptorClasses2.setTo(Scalar(singleClass2));
				descriptorClasses.push_back(descriptorClasses2);

			}


		}

		saveFeatureDescriptorsAndOrKeypoints( descriptorOutputFile, descriptorType,localizationMethod,detector, keypoints, descriptorExtractor, descriptors, descriptorClasses);
	}
	else
	{
		loadFeatureDescriptorsAndOrKeypoints(descriptorInputFile, descTypeString,localTypeString ,descriptorType,localizationMethod,detector,&keypoints,descriptorExtractor,&descriptors);
		if(doubleLoad)
		{


			loadFeatureDescriptorsAndOrKeypoints(descriptorInputFile2, descTypeString,localTypeString ,descriptorType,localizationMethod,detector,&keypoints2,descriptorExtractor,&descriptors2);
			for(int i = 0; i < keypoints2.size(); i++)
			{
				keypoints.push_back(keypoints2[i]);

			}
			descriptors.push_back(descriptors2);


		}
		if(!useSingleClass)
		{
			descriptorClasses =  loadDescriptorClassesFromTextFile(classesTextFile, descriptorClassesInfo);
			if(descriptorClasses.rows != descriptors.rows)
			{
				cout << "WARNING: Number of classes and descriptors to not match" << endl;


			}

		}
		else
		{
			descriptorClasses.create(descriptors.rows,1,CV_32FC1);
			descriptorClasses.setTo(Scalar(singleClass));
			if(doubleLoad)
			{

				descriptorClasses2.create(descriptors.rows,1,CV_32FC1);
				descriptorClasses2.setTo(Scalar(singleClass2));
				descriptorClasses.push_back(descriptorClasses2);

			}


		}


		saveFeatureDescriptorsAndOrKeypoints( descriptorOutputFile,  descriptorType,localizationMethod,detector,keypoints,descriptorExtractor, descriptors, descriptorClasses);
	}



	//Mat testMat =  loadDescriptorClassesFromTextFile("testClasses.txt", descriptorClassesInfo);
	//cout << "Test Mat:" << testMat << endl;
	cout << "All done" << endl;
	return 0;
}

#ifndef ADD_FEATURE_CLASSIFICATION_TO_DESC_MODULE

int main(int argc, const char * argv[])
{
	return addFeatureClassificationToDescFile_main(argc, argv);

}
#endif
