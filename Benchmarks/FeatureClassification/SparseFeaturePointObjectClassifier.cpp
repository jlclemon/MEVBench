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
 * SparseFeaturePointObjectClassifier.cpp
 *
 *  Created on: Apr 21, 2011
 *      Author: jlclemon
 */

#include "SparseFeaturePointObjectClassifier.h"
#include "WorkerThreadInfo.h"

using namespace cv;

SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifier() {
	// TODO Auto-generated constructor stub

}


SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifier(SparseFeaturePointObjectClassifierParams _params) {
	// TODO Auto-generated constructor stub
	params = _params;
}


SparseFeaturePointObjectClassifier::~SparseFeaturePointObjectClassifier() {
	// TODO Auto-generated destructor stub
}


SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierParams::SparseFeaturePointObjectClassifierParams(): nearestNeighborDistanceRatio(GET_DEFAULT_NN_RATIO()), maxAveSqError(GET_DEFAULT_MAX_AVE_SQ_ERROR()),maxIndSqError(GET_DEFAULT_MAX_IND_SQ_ERROR())
{

	return;
}
SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierParams::SparseFeaturePointObjectClassifierParams(float _nearestNeighborDistanceRatio,float _maxAveSqError,float _maxIndSqError) : nearestNeighborDistanceRatio(_nearestNeighborDistanceRatio), maxAveSqError(_maxAveSqError), maxIndSqError(_maxIndSqError)
{

	return;
}
SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierParams::~SparseFeaturePointObjectClassifierParams()
{


	return;
}


bool SparseFeaturePointObjectClassifier::train( const Mat& _trainDescriptors, const vector<KeyPoint> &_trainKeypoints, const float _objClass,SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierParams _params,
                    Mat &_trainImage, int _trainImageId,string &_trainImageName, bool _update_base )
{
	bool returnVal = false;
	if((_trainDescriptors.empty() == false) && (_trainKeypoints.size() >0) && !(_trainImage.empty()))
	{
		Mat trainDescCopy = _trainDescriptors.clone();
		//cout << trainDescCopy << endl;
		this->params = _params;
		this->currentDescriptorCollection.push_back(trainDescCopy);
		this->currentTrainKeypoints.push_back(_trainKeypoints);
		this->currentTrainImages.push_back(_trainImage.clone());

		this->currentTrainImageIds.push_back(_trainImageId);
		this->currentTrainImagesNames.push_back(_trainImageName);
		this->objClasses.push_back(_objClass);

		this->currentTrainedRefPoints.push_back( computeCenterPointOfKeyPoints(currentTrainKeypoints[currentTrainKeypoints.size()-1]));
		this->currentTrainedRefVectors.push_back(computeRelativeCenterPointVectorsForKeypoints( currentTrainKeypoints[currentTrainKeypoints.size()-1]));


		returnVal = true;




	}

	return returnVal;
}

float SparseFeaturePointObjectClassifier::predict(const Mat& queryDescriptors, const vector<KeyPoint> queryKeypoints,vector<vector <DMatch> > &matches, vector<Mat> & transformsList, vector<Point2f>& estimatedRefPoints)
{

	float returnVal = -1.0f;
	vector<DMatch> filteredMatches;

	estimatedRefPoints.clear();

	transformsList.clear();

	//Apply neighbor ratio filtering to matches
	filterFeatureMatches( matches, filteredMatches );

	getTransforms(transformsList, estimatedRefPoints,filteredMatches,  queryKeypoints,this->currentTrainKeypoints[0], this->currentTrainedRefVectors[0], this->currentTrainedRefPoints[0]);

	if(transformsList.size() >0)
	{

		returnVal = this->objClasses[0];
	}



	return returnVal;



}




void SparseFeaturePointObjectClassifier::clear()
{



	this->currentDescriptorCollection.clear();
	this->currentTrainKeypoints.clear();
	this->currentTrainImages.clear();

	this->currentTrainImageIds.clear();
	this->currentTrainImagesNames.clear();
	this->objClasses.clear();

	this->currentTrainedRefPoints.clear();
	this->currentTrainedRefVectors.clear();









}

void SparseFeaturePointObjectClassifier::save( const char* filename, const char* name ) const
{

	FileStorage fs(filename,FileStorage::WRITE);


	this->write(*fs,name);



}
void SparseFeaturePointObjectClassifier::load( const char* filename, const char* name )
{
	FileStorage fs(filename,FileStorage::READ);


	this->read(*fs,*fs[name]);


}

void SparseFeaturePointObjectClassifier::write( CvFileStorage* storage, const char* name ) const
{


	cout << "Warning: Saving Sparse Feature Point Object Classifier not allowed.  You will need to recompute every time....";

	return;




}



void SparseFeaturePointObjectClassifier::read( CvFileStorage* storage, CvFileNode* node )
{


	cout << "Warning: Loading Sparse Feature Point Object Classifier not allowed.  You will need to recompute every time....";
	//this->knnFlannMatcher->read(*node);


	return;
}


ostream &operator<<(ostream &stream, SparseFeaturePointObjectClassifier & ob)
{


	stream << "Sparse Feature Point Object Classifier";


	return stream;
}


ostream &operator<<(ostream& stream, const SparseFeaturePointObjectClassifier::HoughBin bin)
{

	stream <<"X Index: "<<bin.xIndex <<", Y Index: "<< bin.yIndex <<", Scale Index: "<< bin.scaleIndex <<", Orientation Index: "<<bin.orientationIndex <<
		", Count: " << bin.count<< endl;
	stream << "Key Point Indices: (";
	for(unsigned int i=0; i<bin.keypointIndices.size(); i++)
	{
		if(i!=0)
		{
			stream << ",";

		}
		stream << bin.keypointIndices[i];

	}
	stream << ")" << endl;
	return stream;
}


Point2f SparseFeaturePointObjectClassifier::computeCenterPointOfKeyPoints(vector<KeyPoint> & keypoints)
{
	Point2f centerPoint(0,0);



	//Passing in points uses a different method which throws off the numbers
	//Better to just calculate the average my self
	//Moments currentMoments = moments(pointsMat,false);
	//centerPoint.x = currentMoments.m10/currentMoments.m00;

	//centerPoint.y = currentMoments.m01/currentMoments.m00;


	for(int i = 0; i< (int)keypoints.size(); i++)
	{
		centerPoint.x += keypoints[i].pt.x;
		centerPoint.y += keypoints[i].pt.y;

	}
	centerPoint.x /= keypoints.size();
	centerPoint.y /= keypoints.size();



	return centerPoint;
}

float SparseFeaturePointObjectClassifier::calcDistanceBetweenPoints(Point2f start, Point2f end)
{
	float xDiff=0.0, yDiff = 0.0;

	xDiff = end.x -start.x;
	yDiff = end.y-start.y;
	return sqrt(xDiff*xDiff + yDiff*yDiff);
}

float SparseFeaturePointObjectClassifier::makeVectorAngleRelativeToFeatureAngle(float vectorAngle, float featureAngle)
{
	float newAngle;
	newAngle = vectorAngle-featureAngle;
	if(newAngle>=360.0f)
	{
		newAngle-= 360.0f;

	}

	if(newAngle<0.0f)
	{
		newAngle+= 360.0f;

	}

	return newAngle;
}

float SparseFeaturePointObjectClassifier::computeVectorAngleRelativeToImageAxis(float vectorAngle, float featureAngle)
{

	float newAngle;
	newAngle = vectorAngle+featureAngle;
	if(newAngle>=360.0f)
	{
		newAngle-= 360.0f;

	}

	if(newAngle<0.0f)
	{
		newAngle+= 360.0f;

	}

	return newAngle;


}

float SparseFeaturePointObjectClassifier::calcVectorAngleBetweenPoints(Point2f start, Point2f end)
{
	float xDiff=0.0, yDiff = 0.0;

	xDiff = end.x -start.x;
	yDiff = end.y-start.y;

	float angle = (float)((180.0f*atan2(yDiff,xDiff))/pi);
	if(angle < 0)
	{
		angle+= 360.0f;
	}
	if(angle >=360)
	{
		angle -= 360.0f;

	}

	return angle;

}

Point2f SparseFeaturePointObjectClassifier::computeReferenceInCurrentImageFrame(Vec2f rAndRelativeTheta, KeyPoint keypoint)
{


	Point2f rotatedPoint;


	//Move the theta to the current image reference frame
	float adjustedTheta = computeVectorAngleRelativeToImageAxis(rAndRelativeTheta[1],keypoint.angle);

	//cout << "Adjusted Theta: " << adjustedTheta << endl;

	//Negative adjust Theta because CCW is positive in get RotationMatrix but angle are with CW positive
	Mat rotMatrix = getRotationMatrix2D(Point2f(0,0),-adjustedTheta,1);

	//We need the rotation matrix to be floating point
	rotMatrix.convertTo(rotMatrix,CV_32FC1);

	//So the point was at angle 0, and distance of R
	//THen we move it to where the center should be relative
	//To this feature point.  This is merely a theta to align
	//The feature point.
	//So in cartesian it is x=r, y = 0
	//Then it is rotated to the proper place
	Point3f pointInZeroFrame(rAndRelativeTheta[0],0,1);

	//Get the point read to apply the rotation matrix
	Mat pointMat = Mat(pointInZeroFrame).reshape(1); // convert vector to Mat, O(1) operation
                   // make Nx3 1-channel matrix out of Nx1 3-channel.
                              // Also, an O(1) operation
	//Apply the rotation matrix
	Mat rotMat = rotMatrix*pointMat;

	//Take the rotated point and translate based on where in image
	rotatedPoint.x= rotMat.at<float>(0,0)+keypoint.pt.x;
	rotatedPoint.y= rotMat.at<float>(1,0)+keypoint.pt.y;

	return rotatedPoint;

}

Vec2f SparseFeaturePointObjectClassifier::computeVectorToCenterPoint(KeyPoint & keypoint, Point2f centerPoint)
{
	Vec2f rAndTheta;
	Point2f start;
	Point2f end;
	start.x = keypoint.pt.x;
	start.y = keypoint.pt.y;
	end.x = centerPoint.x;
	end.y = centerPoint.y;

	rAndTheta[0] = 	calcDistanceBetweenPoints(start, end);
	rAndTheta[1] = 	calcVectorAngleBetweenPoints(start, end);

	return rAndTheta;
}

Vec2f SparseFeaturePointObjectClassifier::computeRelativeVectorToCenterPoint(KeyPoint & keypoint, Point2f centerPoint)
{
	Vec2f rAndTheta;
	Point2f start;
	Point2f end;
	start.x = keypoint.pt.x;
	start.y = keypoint.pt.y;
	end.x = centerPoint.x;
	end.y = centerPoint.y;

	rAndTheta[0] = 	calcDistanceBetweenPoints(start, end);
	rAndTheta[1] = 	calcVectorAngleBetweenPoints(start, end);
	rAndTheta[1] = makeVectorAngleRelativeToFeatureAngle(rAndTheta[1], keypoint.angle);
	return rAndTheta;
}

void SparseFeaturePointObjectClassifier::convertMatchesToSubListVectors(vector<DMatch> matches, const vector<Vec2f>& centerPointVectors, const vector<KeyPoint> & originalKeypoints, const vector<KeyPoint> & newKeypoints, vector<Vec2f>& matchedCenterPointVectors, vector<KeyPoint> & matchedOriginalKeypoints, vector<KeyPoint> & matchedNewKeypoints)
{
	matchedCenterPointVectors.clear();
	matchedOriginalKeypoints.clear();
	matchedNewKeypoints.clear();

	for(vector<DMatch>::iterator it = matches.begin(); it != matches.end(); ++it)
	{
		int currentNewKeypointsIndex = (*it).queryIdx;
		int currentOrigKeypointsIndex = (*it).trainIdx;
		int currentCenterPointIndex = (*it).trainIdx;  //Centerpoints are still relative to originals
		matchedCenterPointVectors.push_back(centerPointVectors[currentCenterPointIndex]);
		matchedOriginalKeypoints.push_back(originalKeypoints[currentOrigKeypointsIndex]);
		matchedNewKeypoints.push_back(newKeypoints[currentNewKeypointsIndex]);

	}


}


vector<Vec2f> SparseFeaturePointObjectClassifier::computeScaleAdjustedCenterPointVectors(const vector<Vec2f>& centerPointVectors, const vector<KeyPoint> & originalKeypoints, const vector<KeyPoint> & newKeypoints, vector<Vec2f> & scaleAndAngleDiff)
{

	vector<Vec2f> scaledCenterPointVectors;
/*
	for(int i = 0; i < (int)centerPointVectors.size(); i++)
	{

		float currentScaling = newKeypoints[i].size/originalKeypoints[i].size;
		Vec2f currentScaleAndAngleDiff;
		scaledCenterPointVectors.push_back(centerPointVectors[i]);
		scaledCenterPointVectors[scaledCenterPointVectors.size() -1][0] = scaledCenterPointVectors[scaledCenterPointVectors.size() -1][0] * currentScaling;

		currentScaleAndAngleDiff[0] = currentScaling;
		currentScaleAndAngleDiff[1] =newKeypoints[i].angle-originalKeypoints[i].angle;
		scaleAndAngleDiff.push_back(currentScaleAndAngleDiff);
	}
	*/
	for(int i = 0; i < (int)centerPointVectors.size(); i++)
	{

		float currentScaling = newKeypoints[i].size/originalKeypoints[i].size;
		Vec2f currentScaleAndAngleDiff;
		scaledCenterPointVectors.push_back(centerPointVectors[i]);
		scaledCenterPointVectors[scaledCenterPointVectors.size() -1][0] = scaledCenterPointVectors[scaledCenterPointVectors.size() -1][0] * currentScaling;

		currentScaleAndAngleDiff[0] = currentScaling;
		currentScaleAndAngleDiff[1] =newKeypoints[i].angle-originalKeypoints[i].angle;

		if(currentScaleAndAngleDiff[1] < 0)
		{
			currentScaleAndAngleDiff[1] += 360;
		}
		if(currentScaleAndAngleDiff[1] >= 360)
		{
			currentScaleAndAngleDiff[1] -= 360;
		}


		scaleAndAngleDiff.push_back(currentScaleAndAngleDiff);
	}


	return scaledCenterPointVectors;
}


//Computed in form of r,theta in the vector
vector<Vec2f> SparseFeaturePointObjectClassifier::computeRelativeCenterPointVectorsForKeypoints(vector<KeyPoint> & keypoints)
{
	vector<Vec2f> rAndRelativeTheta;

	Point2f centerFloat = computeCenterPointOfKeyPoints(keypoints);

	for(int i = 0; i < (int)keypoints.size(); i++)
	{
		Vec2f currentRAndThetaRelative = computeRelativeVectorToCenterPoint(keypoints[i], centerFloat);
		rAndRelativeTheta.push_back(currentRAndThetaRelative);

	}

	return rAndRelativeTheta;
}

vector<Point2f> SparseFeaturePointObjectClassifier::computeReferencePointsInCurrentImageFrame(const vector<Vec2f> &relativeCenterVectors, const vector<KeyPoint> &keypoints)
{
	vector<Point2f> refPoints;
	for(int i = 0; i < (int)relativeCenterVectors.size(); i++)
	{


		Point2f calculatedPoint = computeReferenceInCurrentImageFrame(relativeCenterVectors[i], keypoints[i]);

		refPoints.push_back(calculatedPoint);



	}
	return refPoints;
}

vector<int> SparseFeaturePointObjectClassifier::addToHistogramBin(vector<int> xIndex, vector<int> yIndex, vector<int> scaleIndex,vector<int> orientationIndex,int keypointIndex, vector<HoughBin> & histogramBins)
{
	vector<int> binCount;
	//binCount.push_back(-1);
	bool binFound = false;

	if(xIndex[0] == -1 &&  yIndex[0] == -1 && orientationIndex[0] == -1 && scaleIndex[0] == -1)
	{

		return binCount;
	}

	/*
		for(unsigned int i=0; i< histogramBins.size(); i++)
		{
			if(histogramBins[i].xIndex == xIndex &&
				histogramBins[i].yIndex == yIndex &&
				histogramBins[i].scaleIndex == scaleIndex &&
				histogramBins[i].orientationIndex == orientationIndex)
			{
				histogramBins[i].count++;
				histogramBins[i].keypointIndices.push_back(keypointIndex);
				binCount = histogramBins[i].count;
			}


		}
	*/

	int currentXIndex;
	int currentYIndex;
	int currentScaleIndex;
	int currentOrientationIndex;
	for(int x = 0; x < (int)xIndex.size(); x++)
	{
		currentXIndex = xIndex[x];
		if(currentXIndex ==-1)
		{
			continue;
		}

		for(int y = 0; y < (int)yIndex.size(); y++)
		{
			currentYIndex = yIndex[y];
			if(currentYIndex ==-1)
			{
				continue;
			}


			for(int s = 0; s < (int)scaleIndex.size(); s++)
			{
				currentScaleIndex = scaleIndex[s];
				if(currentScaleIndex ==-1)
				{
					continue;
				}


				for(int o = 0; o < (int)orientationIndex.size(); o++)
				{
					currentOrientationIndex = orientationIndex[o];
					if(currentOrientationIndex ==-1)
					{
						continue;
					}

					binFound =false;
					for(unsigned int i=0; i< histogramBins.size(); i++)
					{
						if(histogramBins[i].xIndex == currentXIndex &&
							histogramBins[i].yIndex == currentYIndex &&
							histogramBins[i].scaleIndex == currentScaleIndex &&
							histogramBins[i].orientationIndex == currentOrientationIndex)
						{
							histogramBins[i].count++;
							histogramBins[i].keypointIndices.push_back(keypointIndex);
							binCount.push_back(histogramBins[i].count);
							binFound = true;
						}


					}


					if(binFound ==false)
					{
						histogramBins.push_back(HoughBin());
						int currentBinIndex = histogramBins.size()-1;

						histogramBins[currentBinIndex].count = 1;
						histogramBins[currentBinIndex].keypointIndices.push_back(keypointIndex);
						histogramBins[currentBinIndex].xIndex = currentXIndex;
						histogramBins[currentBinIndex].yIndex = currentYIndex;
						histogramBins[currentBinIndex].scaleIndex = currentScaleIndex;
						histogramBins[currentBinIndex].orientationIndex = currentOrientationIndex;
						binCount.push_back(histogramBins[currentBinIndex].count);

					}
				}
			}
		}
	}

	return binCount;
}

void SparseFeaturePointObjectClassifier::computeBinIndices(Point2f location, Vec2f scaleAndOrientation, vector<int> & xIndex, vector<int> & yIndex, vector<int> &scaleIndex, vector<int> &orientationIndex)
{
	int numberOfXbins = 960/10;//320/10;//640/10;   //10 pixel boxes (64 bins)
	int numberOfYbins = 720/10;//240/10;//480/10;	  //10 pixel boxes (48 bins)
	float xBinWidth = 10;
	float yBinWidth = 10;
	int numberOfScaleBins = 10;   //Factor of 2 bins defined below
	int numberOfOrientationBins = 360/30;  //30 degree bins (12 bins)
	float orientationBinWidth = 30;
	float xBinRanges[97] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300,310,320,330,340,350,360,370,380,390,400,410,420,430,440,450,460,470,480,490,500,510,520,530,540,550,560,570,580,590,600,610,620,630,640,650,660,670,680,690,700,710,720,730,740,750,760,770,780,790,800,810,820,830,840,850,860,870,880,890,900,910,920,930,940,950,960};
	float yBinRanges[73] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300,310,320,330,340,350,360,370,380,390,400,410,420,430,440,450,460,470,480,490,500,510,520,530,540,550,560,570,580,590,600,610,620,630,640,650,660,670,680,690,700,710,720};


//	float xBinRanges[65] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300,310,320,330,340,350,360,370,380,390,400,410,420,430,440,450,460,470,480,490,500,510,520,530,540,550,560,570,580,590,600,610,620,630,640};
//	float yBinRanges[49] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300,310,320,330,340,350,360,370,380,390,400,410,420,430,440,450,460,470,480};
//	float xBinRanges[33] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300,310,320};
//	float yBinRanges[25] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240};


	float scaleBinRanges[] = {0.0f, .0078125f, .015625f, .03125f,.0625f,.125f,.25f,.50f, 1.0f,2.0f,4.0f};
	float orientationBinRanges[] = {0.0, 30, 60, 90,120,150,180,210, 240,270,300,330,360};

/*
	if(location.x >= xBinRanges[0])
	{

		for(int i = 1; i < numberOfXbins+1; i++)
		{
			if(location.x < xBinRanges[i])
			{
				xIndex = i-1;

				break;
			}
		}
	}
*/
	xIndex.resize(2);
	yIndex.resize(2);
	orientationIndex.resize(2);
	scaleIndex.resize(2);

	xIndex[0] = -1;
	yIndex[0] = -1;
	orientationIndex[0] = -1;
	scaleIndex[0] = -1;
	xIndex[1] = -1;
	yIndex[1] = -1;
	orientationIndex[1] = -1;
	scaleIndex[1] = -1;


	if(location.x < xBinRanges[numberOfXbins] && location.x >= xBinRanges[0])
	{
		xIndex[0] = (int) (location.x/xBinWidth);
		if(location.x/xBinWidth - xIndex[0] >=.5)
		{
			if(xIndex[0]+1 < numberOfXbins)
			{
				xIndex[1] = xIndex[0]+1;
			}
		}
		else
		{
			if(xIndex[0] >= 1)
			{
				xIndex[1] = xIndex[0]-1;
			}

		}

	}
	if(location.y < yBinRanges[numberOfYbins] && location.y >= yBinRanges[0])
	{

		yIndex[0] = (int) (location.y/yBinWidth);
		if(location.y/yBinWidth - yIndex[0] >=.5)
		{
			if(yIndex[0]+1 < numberOfYbins)
			{
				yIndex[1] = yIndex[0]+1;
			}
		}
		else
		{
			if(yIndex[0] >= 1)
			{
				yIndex[1] = yIndex[0]-1;
			}

		}


	}

	if(scaleAndOrientation[1] < orientationBinRanges[numberOfOrientationBins] && scaleAndOrientation[1] >= orientationBinRanges[0])
	{

		orientationIndex[0] = (int) (scaleAndOrientation[1]/orientationBinWidth);
		if(scaleAndOrientation[1]/orientationBinWidth - orientationIndex[0] >=.5)
		{
			if(orientationIndex[0]+1 < numberOfOrientationBins)
			{
				orientationIndex[1] = orientationIndex[0]+1;
			}
			else
			{
				orientationIndex[1] = 0;
			}
		}
		else
		{
			if(orientationIndex[0] >= 1)
			{
				orientationIndex[1] = orientationIndex[0]-1;
			}
			else
			{
				orientationIndex[1] = numberOfOrientationBins-1;
			}

		}



	}

	if(scaleAndOrientation[0] >= scaleBinRanges[0] && scaleAndOrientation[0] < scaleBinRanges[numberOfScaleBins])
	{

		for(int i = 1; i < numberOfScaleBins+1; i++)
		{
			if(scaleAndOrientation[0] < scaleBinRanges[i])
			{
				scaleIndex[0] = i-1;
				if(scaleAndOrientation[0]>=(scaleBinRanges[i-1] + .5 *(scaleBinRanges[i]-scaleBinRanges[i-1]) ))
				{
					if(i+1 < numberOfScaleBins)
					{
						scaleIndex[1] = i;
					}
				}
				else
				{
					if(i>=2)
					{
						scaleIndex[1] = i-2;
					}

				}


				break;
			}
		}
	}


	if(xIndex[0] == -1|| yIndex[0] == -1 || orientationIndex[0] == -1 || scaleIndex[0] == -1)
	{
		xIndex[0] = -1;
		yIndex[0] = -1;
		orientationIndex[0] = -1;
		scaleIndex[0] = -1;


	}
	if(xIndex[1] == -1|| yIndex[1] == -1 || orientationIndex[1] == -1 || scaleIndex[1] == -1)
	{
		xIndex[1] = -1;
		yIndex[1] = -1;
		orientationIndex[1] = -1;
		scaleIndex[1] = -1;


	}


	return;

}

SparseMat SparseFeaturePointObjectClassifier::calcHistUsingOpenCvMethods(vector<HoughBin> & histogramBins, const vector<Vec2f> & rAndThetas, const vector<Vec2f>& scaleAndAngles, const  vector<KeyPoint> &keypoints)
{

	int numberOfXbins = 320/10;//640/10;   //10 pixel boxes (64 bins)
	int numberOfYbins = 240/10;//480/10;	  //10 pixel boxes (48 bins)
	float xBinWidth = 10;
	float yBinWidth = 10;
	int numberOfScaleBins = 10;   //Factor of 2 bins defined below
	int numberOfOrientationBins = 360/30;  //30 degree bins (12 bins)
	float orientationBinWidth = 30;
//	float xBinRanges[65] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300,310,320,330,340,350,360,370,380,390,400,410,420,430,440,450,460,470,480,490,500,510,520,530,540,550,560,570,580,590,600,610,620,630,640};
//	float yBinRanges[49] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300,310,320,330,340,350,360,370,380,390,400,410,420,430,440,450,460,470,480};
	float xBinRanges[33] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300,310,320};
	float yBinRanges[25] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240};


	float scaleBinRanges[] = {0.0f, .0078125f, .015625f, .03125f,.0625f,.125f,.25f,.50f, 1.0f,2.0f,4.0f};
	float orientationBinRanges[] = {0.0, 30, 60, 90,120,150,180,210, 240,270,300,330,360};

	int channels[] = {0,1,2,3};

	Mat matArray[2];

	vector<Point2f> centerPoints = computeReferencePointsInCurrentImageFrame(rAndThetas,keypoints);

	Mat pointsMat(centerPoints);
	Mat scaleAndAnglesMat(scaleAndAngles);
	matArray[0] = pointsMat;
	matArray[1] = scaleAndAnglesMat;

	SparseMat histogramMat;
	int histSize[] = {numberOfXbins,numberOfYbins,numberOfScaleBins,numberOfOrientationBins};
	const float * ranges[] = {xBinRanges,yBinRanges,scaleBinRanges,orientationBinRanges};


	calcHist( matArray, 2, channels, Mat(),  histogramMat, 4, histSize, ranges, false, false );

	cv::SparseMatConstIterator it = histogramMat.begin(), it_end = histogramMat.end();

	float totalItems= 0;
	 for(; it != it_end; ++it)
	 {
		 // print element indices and the element value
		 const SparseMat::Node* n = it.node();
		 cout <<"X Index: "<<n->idx[0] <<", Y Index: "<< n->idx[1] <<", Scale Index: "<< n->idx[2] <<", Orientation Index: "<<n->idx[3] <<
			 ", Count: " << it.value<float>()<< endl;

		totalItems +=it.value<float>();
	 }

	cout << "Total Items: " << totalItems << endl;
	 return histogramMat;

}


void SparseFeaturePointObjectClassifier::houghBinTheRemappedVectors(vector<HoughBin> & histogramBins, const vector<Vec2f> & rAndThetas, const vector<Vec2f>& scaleAndAngles, const  vector<KeyPoint> &keypoints)
{

	vector<int> xIndex;
	vector<int> yIndex;
	vector<int> scaleIndex;
	vector<int> orientationIndex;
	vector<int> currentBinAmount;


	vector<Point2f> centerPoints = computeReferencePointsInCurrentImageFrame(rAndThetas,keypoints);
	for(unsigned int i =0; i < rAndThetas.size(); i++)
	{

		computeBinIndices(centerPoints[i], scaleAndAngles[i], xIndex,yIndex,scaleIndex,orientationIndex);
		currentBinAmount = addToHistogramBin(xIndex, yIndex, scaleIndex, orientationIndex,i, histogramBins);


	}

/*	int totalItems = 0;
	for(vector<HoughBin>::iterator it = histogramBins.begin(); it!=histogramBins.end(); ++it)
	{
		cout << *it;
		totalItems += (*it).count;

	}
	cout << "Total Items: " << totalItems << endl;
*/

	//calcHist( matArray, 2, channels, Mat(),  histogramMat, 4, histSize, ranges, false, false );



	//histogramMat.

}

float SparseFeaturePointObjectClassifier::computeSqErrorForHoughBin(Mat & computedTransform,vector<int> & pointIndicesForTransform, int & numberOfValidPointsInBin,const HoughBin & bin, const vector<KeyPoint> & newKeypoints,const vector<KeyPoint> & origKeypoints)
{
	vector<int> currentBinPointIndices(bin.count);
	bool done = false;
	bool found = false;
	//float maxAllowedError = ((5.0f/2.0f) *(5.0f/2.0f)) * 2;
	float totalSquareError = -1.0;
	float currentSquareError = 0.0;
	vector<Point2f> newPoints(bin.count);
	vector<Point2f> origPoints(bin.count);
	vector<Point2f> nextNewPoints(bin.count);
	vector<Point2f> nextOrigPoints(bin.count);
	vector<int> nextCurrentBinPointIndices(bin.count);
	numberOfValidPointsInBin = -1;


	for(int i = 0; i < bin.count;i++)
	{
		currentBinPointIndices[i]=bin.keypointIndices[i];
		newPoints[i].x =newKeypoints[i].pt.x;
		newPoints[i].y = newKeypoints[i].pt.y;
		origPoints[i].x =origKeypoints[i].pt.x;
		origPoints[i].y = origKeypoints[i].pt.y;

	}

//	cout << "Bin Count: " << bin.count << endl;
	while(!done)
	{

		Mat origPointsMat(origPoints);
		Mat newPointsMat(newPoints);

		Mat affineTransform = estimateRigidTransform(origPointsMat,newPointsMat,true);
		affineTransform.convertTo(affineTransform,CV_32FC1);
		//cout << "Current Affine Transform: " << affineTransform << endl;
		Mat newPointsFromXformMat;

		transform(origPointsMat,newPointsFromXformMat,affineTransform);

//		Mat diffMat;
//		subtract(newPointsFromXformMat,newPointsMat,diffMat);

		nextNewPoints.clear();
		nextOrigPoints.clear();
		nextCurrentBinPointIndices.clear();
		totalSquareError = 0.0;
		for(int i = 0; i<newPointsFromXformMat.rows;i++)
		{

			currentSquareError = ((newPointsFromXformMat.at<float>(i,0) - newPointsMat.at<float>(i,0)) * (newPointsFromXformMat.at<float>(i,0) - newPointsMat.at<float>(i,0))) + ((newPointsFromXformMat.at<float>(i,1) - newPointsMat.at<float>(i,1)) * (newPointsFromXformMat.at<float>(i,1) - newPointsMat.at<float>(i,1)));
			totalSquareError += currentSquareError;
//			cout << "Sqr Error: " << currentSquareError << endl;
			/*if(currentSquareError > 100)
			{
				cout << "The Xfrom Points" << newPointsFromXformMat << endl;
				cout << "The Old Points" << origPointsMat << endl;
				cout << "The New Points" << newPointsMat << endl;
				cout << "How:?" << endl;
			}*/
			if(currentSquareError < params.maxIndSqError)
			{
				nextNewPoints.push_back(newPoints[i]);
				nextOrigPoints.push_back(origPoints[i]);
				nextCurrentBinPointIndices.push_back(currentBinPointIndices[i]);
			}

		}
		if(currentBinPointIndices.size() == nextCurrentBinPointIndices.size())
		{
			computedTransform = affineTransform;
			pointIndicesForTransform =currentBinPointIndices;
			done = true;
			found = true;
			numberOfValidPointsInBin = nextCurrentBinPointIndices.size();
		}

		newPoints = nextNewPoints;
		origPoints = nextOrigPoints;
		currentBinPointIndices =nextCurrentBinPointIndices;

		if(currentBinPointIndices.size() <3)
		{
			totalSquareError = -1.0;
			done = true;
		}
	}


	return totalSquareError;
}

vector<Mat> SparseFeaturePointObjectClassifier::computerValidTransfroms (vector<vector<int> > & pointIndicesForTransform,const vector<HoughBin> & bins, const vector<KeyPoint> & newKeypoints,const vector<KeyPoint> & origKeypoints)
{
	vector<Mat> validTransforms;
	//vector<vector<int>> validKeypointIndices;
	for(int i = 0; i < (int)bins.size()	; i++)
	{
		if(bins[i].count >= 3)
		{
			Mat currentTransform;
			vector<int> currentIndices;
			int currentNumberOfValidPoints;
			float currentSqError = computeSqErrorForHoughBin(currentTransform, currentIndices, currentNumberOfValidPoints,bins[i], newKeypoints,origKeypoints);
			float currentAveSqError = currentSqError/currentNumberOfValidPoints;
			if(currentSqError >=0 && currentAveSqError< params.maxAveSqError)
			{
				validTransforms.push_back(currentTransform);
				//validKeypointIndices.push_back(currentIndices);
				pointIndicesForTransform.push_back(currentIndices);
			}
		}
	}

	return validTransforms;

}


void SparseFeaturePointObjectClassifier::getTransforms(vector<Mat> & transformsList, vector<Point2f>& refPoints,const vector<DMatch> & matches,  const vector<KeyPoint>& queryKeypoints,const vector<KeyPoint>& trainedKeypoints, const vector<Vec2f>&  trainedRefVectors, const Point2f & trainedRefPoint)
{

	vector<Vec2f> matchedCenterPointVectors;
	vector<KeyPoint> matchedOriginalKeypoints;
	vector<KeyPoint> matchedNewKeypoints;


	//Take the original list of all keypoints and pair them down to only the matched keypoints
	convertMatchesToSubListVectors( matches, trainedRefVectors,trainedKeypoints, queryKeypoints, matchedCenterPointVectors,  matchedOriginalKeypoints,  matchedNewKeypoints);


	//I take the center point vectors from the training set and the keypoints from the training and query set and create the center point
	//vectors for the query set based on which training set keypoint they match
	//So if the new feature is 2xScale with 30 degrees compared to the training set of 1xScale with 50 degrees
	//The vector to center is scaled by 2x/1x = 2 and the angle is adjust by 20 degrees to
	//get the new center point vector
	vector<Vec2f> newScaleAndObjectAngles;
	vector<Vec2f> newCenterVectors = computeScaleAdjustedCenterPointVectors(matchedCenterPointVectors, matchedOriginalKeypoints, matchedNewKeypoints, newScaleAndObjectAngles);




	//Computer the Hough bins using custom code
	vector<HoughBin>  histogramBins;
	houghBinTheRemappedVectors(histogramBins, newCenterVectors, newScaleAndObjectAngles, matchedNewKeypoints);






	//Use the bins to compute possible transforms using affine error to filter
	vector<vector<int> > pointIndicesForTransform;
	transformsList = computerValidTransfroms (pointIndicesForTransform,histogramBins, matchedNewKeypoints,matchedOriginalKeypoints);

	//Setup the original Reference Point for calculating the new ref points
	Point3f oldRefPointForXForm(trainedRefPoint.x,trainedRefPoint.y,1);
	Mat refPointMat = Mat(oldRefPointForXForm).reshape(1);

	//Compute the new ref points
	for(int i=0; i < (int)transformsList.size(); i++)
	{
		Point2f relativelyCalculatedPoint;
		Mat newRefMat;//(relativelyCalculatedPoint);
		newRefMat = transformsList[i] * refPointMat;
		relativelyCalculatedPoint.x = newRefMat.at<float>(0,0);
		relativelyCalculatedPoint.y = newRefMat.at<float>(1,0);
		refPoints.push_back(relativelyCalculatedPoint);

	}



return;



}


void SparseFeaturePointObjectClassifier::paintCenterOnImage(Mat & image, vector<KeyPoint> & keypoints,Mat & descriptors)
{
	Mat dispImage = image.clone();
	int upside=0, leftside = 0;
	Point2f centerFloat = computeCenterPointOfKeyPoints(keypoints);
	Point_<int> pixel(centerFloat.x,centerFloat.y);//= Point_<int>((Point_<int>)centerFloat);



	circle(dispImage,pixel,5,Scalar(0,255,0));
	namedWindow("DispImage",1);
	imshow("DispImage",dispImage);

	Mat dispImage2 = image.clone();
	Mat tmpImage;
	if(image.channels() ==3)
	{
		cvtColor(image,tmpImage,CV_BGR2GRAY);
	}
	else
	{
		image.copyTo(tmpImage);
	}

	cout << "Key Points Total: " << keypoints.size() << endl;
	cout << "Key Points on left: "<< leftside << "KeyPoints up: " << upside <<endl;
	cout << "Center X: " << pixel.x << "Center Y: " << pixel.y << endl;

/*
	vector<KeyPoint> testingVec;

	testingVec.push_back(keypoints[0]);
	testingVec.push_back(keypoints[1]);
	testingVec.push_back(keypoints[2]);
	testingVec.push_back(keypoints[3]);

	testingVec[0].pt.x = 100;
	testingVec[0].pt.y = 400;
	testingVec[0].angle = 0.0;

	testingVec[1].pt.x = 150;
	testingVec[1].pt.y = 400;
	testingVec[1].angle = 0.0;

	testingVec[2].pt.x = 200;
	testingVec[2].pt.y = 200;
	testingVec[2].angle = 90.0;

	testingVec[3].pt.x = 250;
	testingVec[3].pt.y = 250;
	testingVec[3].angle = 135.0;


	cout << "Angle 0: "<<testingVec[0].angle << endl;
	cout << "Angle 1: "<<testingVec[1].angle << endl;


	Point2f rotatedPoint;

	Vec2f rAndTheta = computeVectorToCenterPoint(testingVec[0], centerFloat);
	Vec2f rAndThetaRelative = computeRelativeVectorToCenterPoint(testingVec[0], centerFloat);
	Point2f relativelyCalculatedPoint = computeReferenceInCurrentImageFrame(rAndThetaRelative, testingVec[1]);
	cout << "Original Angle: " << rAndTheta[1] << endl;
	//testingVec[0].angle = rAndTheta[1];

	Point3f origPoint(rAndTheta[0],0,1);




	Mat rotMatrix = getRotationMatrix2D(Point2f(0,0),-rAndTheta[1],1);

	rotMatrix.convertTo(rotMatrix,CV_32FC1);
	Mat pointMat = Mat(origPoint).reshape(1); // convert vector to Mat, O(1) operation
                   // make Nx3 1-channel matrix out of Nx1 3-channel.
                              // Also, an O(1) operation

	Mat rotMat = rotMatrix*pointMat;
	rotatedPoint.x= rotMat.at<float>(0,0)+testingVec[0].pt.x;
	rotatedPoint.y= rotMat.at<float>(1,0)+testingVec[0].pt.y;


	cout << "Theta 1: "<<testingVec[0].angle << endl;
	*/
	circle(tmpImage,centerFloat,3,Scalar(255,0,0));
	//circle(tmpImage,Point2f(origPoint.x+testingVec[0].pt.x,origPoint.y+testingVec[0].pt.y),7,Scalar(255,0,0));
	//circle(tmpImage,rotatedPoint,11,Scalar(255,0,0));
	//circle(tmpImage,relativelyCalculatedPoint,17,Scalar(255,0,0));

	//drawKeypoints(tmpImage,testingVec,dispImage2,Scalar(0,0,255.0),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	//Called for test image to compute initial vectors
	vector<Vec2f> centerVectors = computeRelativeCenterPointVectorsForKeypoints( keypoints);

	Ptr<DescriptorMatcher> matcher;
	Ptr<flann::IndexParams> indexParams = new flann::KDTreeIndexParams(4);
	Ptr<flann::SearchParams> searchParams = new flann::SearchParams(200);
	matcher = new FlannBasedMatcher(indexParams, searchParams);
	vector<vector<DMatch> > topNMatches;
	int numberOfNeighborsToFind = 2;
	Mat trainedDescriptors = descriptors.clone();

	matcher->knnMatch(descriptors,trainedDescriptors,topNMatches,numberOfNeighborsToFind);
	vector<DMatch> matches;
	for (unsigned int i = 0; i< topNMatches.size(); i++)
	{
		matches.push_back(topNMatches[i][0]);
	}

	vector<Vec2f> matchedCenterPointVectors;
	vector<KeyPoint> matchedOriginalKeypoints;
	vector<KeyPoint> matchedNewKeypoints;


	//Take the original list of all keypoints and pair them down to only the matched keypoints
	convertMatchesToSubListVectors( matches, centerVectors,keypoints, keypoints, matchedCenterPointVectors,  matchedOriginalKeypoints,  matchedNewKeypoints);


	//Now I need to modify the vectors for testing the binning code
	for(int i = 0; i< (int)matchedNewKeypoints.size(); i++)
	{
		if(i>30)
		{

			matchedNewKeypoints[i].angle += i;
			matchedNewKeypoints[i].size += i/10;
		}
		if(matchedNewKeypoints[i].angle >= 360)
		{

			matchedNewKeypoints[i].angle -= 360;
		}
		if(matchedNewKeypoints[i].angle >= 360)
		{

			matchedNewKeypoints[i].angle -= 360;
		}



	}

	//I take the center point vectors from the training set and the keypoints from the training and query set and create the center point
	//vectors for the query set based on which training set keypoint they match
	//So if the new feature is 2xScale with 30 degrees compared to the training set of 1xScale with 50 degrees
	//The vector to center is scaled by 2x/1x = 2 and the angle is adjust by 20 degrees to
	//get the new center point vector
	vector<Vec2f> newScaleAndObjectAngles;
	vector<Vec2f> newCenterVectors = computeScaleAdjustedCenterPointVectors(matchedCenterPointVectors, matchedOriginalKeypoints, matchedNewKeypoints, newScaleAndObjectAngles);


	//Computer the Hough bins using custom code
	vector<HoughBin>  histogramBins;
	houghBinTheRemappedVectors(histogramBins, newCenterVectors, newScaleAndObjectAngles, matchedNewKeypoints);

	//Computer the bins using openCV code as a test
	vector<HoughBin>  histogramBins2;
	SparseMat sparseMat = calcHistUsingOpenCvMethods(histogramBins2,  newCenterVectors, newScaleAndObjectAngles, matchedNewKeypoints);
/*
	for(vector<HoughBin>::iterator it = histogramBins.begin(); it!=histogramBins.end(); ++it)
	{
		int x,y,s,o;
		x = (*it).xIndex;
		y = (*it).yIndex;
		s = (*it).scaleIndex;
		o = (*it).orientationIndex;

		cv::SparseMatConstIterator itInner = sparseMat.begin(), itInner_end = sparseMat.end();
		cout << *it;

		int foundInSparse = 0;
		 for(; itInner != itInner_end; ++itInner)
		 {
			 // print element indices and the element value
			 const SparseMat::Node* n = itInner.node();

			 if(n->idx[0] == x && n->idx[1] ==y&& n->idx[2]==s &&n->idx[3] ==o  && (*it).count == itInner.value<float>())
			{
				foundInSparse++;

				break;
			}



		 }
		if(foundInSparse ==0)
		{
			cout <<"Unmatched Bin Here! " << endl;
			waitKey(0);
			string input;
			cin >> input;
		}

	}

	*/
	vector<vector<int> > pointIndicesForTransform;
	vector<Mat> transformsList = computerValidTransfroms (pointIndicesForTransform,histogramBins, matchedNewKeypoints,matchedOriginalKeypoints);
	Point2f oldCenterPoint = computeCenterPointOfKeyPoints(keypoints);
	Point3f oldCenterPointForXForm(oldCenterPoint.x,oldCenterPoint.y,1);
	Mat centerPointMat = Mat(oldCenterPointForXForm).reshape(1);


	drawKeypoints(tmpImage,keypoints,dispImage2,Scalar(0,0,255.0),DrawMatchesFlags::DEFAULT);//DRAW_RICH_KEYPOINTS);
	for(int i=0; i < (int)transformsList.size(); i++)
	{
		Point2f relativelyCalculatedPoint;
		Mat newCenterMat;//(relativelyCalculatedPoint);
		newCenterMat = transformsList[i] * centerPointMat;
		relativelyCalculatedPoint.x = newCenterMat.at<float>(0,0);
		relativelyCalculatedPoint.y = newCenterMat.at<float>(1,0);

		circle(dispImage2,relativelyCalculatedPoint,15,Scalar(255,0,255));


	}


	for(int i = 0; i < (int)centerVectors.size(); i++)
	{


		Point2f relativelyCalculatedPoint = computeReferenceInCurrentImageFrame(centerVectors[i], keypoints[i]);
		circle(dispImage2,relativelyCalculatedPoint,5,Scalar(255,0,0));
		cout << "Point painted: "<<relativelyCalculatedPoint << endl;

	}



	namedWindow("DispImage2",1);
	imshow("DispImage2",dispImage2);

	waitKey(0);
	destroyWindow("DispImage");
	destroyWindow("DispImage2");
}


void SparseFeaturePointObjectClassifier::filterFeatureMatches(vector<vector<DMatch> > & matches, vector<DMatch> & filteredMatches )
{
	filteredMatches.clear();
	filteredMatches.reserve(matches.size());



	for(unsigned int i = 0; i< matches.size(); i++)
	{
		if(matches[i].size() >=2)
		{
			float d0 = matches[i][0].distance;
			float d1 = matches[i][1].distance;
			if(d0< d1 * this->params.nearestNeighborDistanceRatio)
			{
				filteredMatches.push_back(matches[i][0]);

			}

		}
	}



	return;
}

void SparseFeaturePointObjectClassifier::filterFeatureMatches(vector<DMatch> & matches, vector<DMatch> & filteredMatches )
{
	filteredMatches = matches;

}

bool SparseFeaturePointObjectClassifier::HoughBin::operator==(const SparseFeaturePointObjectClassifier::HoughBin &other) const
{



	return ((this->scaleIndex == other.scaleIndex) && (this->orientationIndex == other.orientationIndex) && (this->xIndex == other.xIndex) && (this->yIndex == other.yIndex));




}

bool SparseFeaturePointObjectClassifier::HoughBin::operator!=(const SparseFeaturePointObjectClassifier::HoughBin &other) const
{



	return ((this->scaleIndex == other.scaleIndex) && (this->orientationIndex == other.orientationIndex) && (this->xIndex == other.xIndex) && (this->yIndex == other.yIndex));




}


SparseFeaturePointObjectClassifier::HoughBin& SparseFeaturePointObjectClassifier::HoughBin::operator=(const SparseFeaturePointObjectClassifier::HoughBin &rhs) {
    // Check for self-assignment!
    if (this == &rhs)
      return *this;

    this->count = rhs.count;
    this->keypointIndices = rhs.keypointIndices;
    this->orientationCenter = rhs.orientationCenter;
    this->orientationIndex = rhs.orientationIndex;
    this->scaleCenter = rhs.scaleCenter;
    this->scaleIndex = rhs.scaleIndex;
    this->xCenter = rhs.xCenter;
    this->xIndex = rhs.xIndex;
    this->yCenter = rhs.yCenter;
    this->yIndex = rhs.yIndex;
    return *this;
  }


SparseFeaturePointObjectClassifier::HoughBin& SparseFeaturePointObjectClassifier::HoughBin::operator+=(const HoughBin &rhs) {
    // Check for self-assignment!

    this->count += rhs.count;

    for(int i =0; i < rhs.keypointIndices.size(); i++)
    {
    	this->keypointIndices.push_back(rhs.keypointIndices[i]);
    }

    return *this;
  }



//************************************************************Multi thread below this line****************************************************************//


//*************************Multi Thread Support Structs

SparseFeaturePointObjectClassifier::HoughBinVecMultiThread::HoughBinVecMultiThread()
{

	int err = pthread_rwlock_init(&vectorRwLock,NULL);
	if(err != 0)
	{
		cout << "ERROR:  Hough Bin Vec Multi Thread rw lock init failed" << endl;


	}


}
SparseFeaturePointObjectClassifier::HoughBinVecMultiThread::~HoughBinVecMultiThread()
{


	pthread_rwlock_destroy(&vectorRwLock);


}
void SparseFeaturePointObjectClassifier::HoughBinVecMultiThread::addToBins(HoughBin & newBin)
{

	int index = this->findMatchingBin(newBin);
	if(index>=0)
	{
		this->addValuesToBin(newBin,index);

	}
	else
	{
		this->addBinToVec(newBin);
	}

	return;


}
vector<SparseFeaturePointObjectClassifier::HoughBin> SparseFeaturePointObjectClassifier::HoughBinVecMultiThread::getHoughBinVector()
{



	return this->bins;


}

vector<SparseFeaturePointObjectClassifier::HoughBin> * SparseFeaturePointObjectClassifier::HoughBinVecMultiThread::getHoughBinVectorPtr()
{



	return &(this->bins);


}

void SparseFeaturePointObjectClassifier::HoughBinVecMultiThread::clear()
{



	this->bins.clear();

	return;
}






int SparseFeaturePointObjectClassifier::HoughBinVecMultiThread::findMatchingBin(HoughBin &newBin)
{
	//Set initial index = -1
	int binIndexInVector = -1;


	//First get the reader Lock for the vector
	int err = pthread_rwlock_rdlock(&vectorRwLock);
	if(err !=0 )
	{
		cout << "ERROR:  In find Matching Bin unable to get read lock" << endl;


	}


	//Check each bin for the indices
	for(int i = 0; i< this->bins.size(); i++)
	{

		//if indices match set index and break
		if(newBin == this->bins[i])
		{
			binIndexInVector = i;
			break;

		}


	}

	err = pthread_rwlock_unlock(&vectorRwLock);
	if(err !=0 )
	{
		cout << "ERROR:  In find Matching Bin unable to release read lock" << endl;


	}

	//return the index
	return binIndexInVector;

}
void SparseFeaturePointObjectClassifier::HoughBinVecMultiThread::addBinToVec(HoughBin & newBin)
{
	//First get the writer Lock for the vector
	int err = pthread_rwlock_wrlock(&vectorRwLock);
	if(err !=0 )
	{
		cout << "ERROR:  In Add Bin to Vec unable to get write lock" << endl;


	}

	int binIndexInVector = -1;
	//Check each bin for the indices
	for(int i = 0; i< this->bins.size(); i++)
	{

		//if indices match set index and break
		if(newBin == this->bins[i])
		{
			binIndexInVector = i;
			bins[i] +=newBin;

			break;

		}


	}

	if(binIndexInVector<0)
	{
		for(int i = 0; i < binLocks.size(); i++)
		{
			err = pthread_rwlock_destroy(&binLocks[i]);
			if(err !=0 )
			{
				cout << "ERROR:  In Add Bin Vec unable to get Kill bin lock" << endl;
			}
		}

		int oldSize = binLocks.size();
		binLocks.resize(oldSize+1);
		for(int i = 0; i < binLocks.size(); i++)
		{
			err = pthread_rwlock_init(&binLocks[i],NULL);
			if(err !=0 )
			{
				cout << "ERROR:  In Add Bin Vec unable to get create new bin lock" << endl;
			}
		}

		this->bins.push_back(newBin);

	}



	err = pthread_rwlock_unlock(&vectorRwLock);
	if(err !=0 )
	{
		cout << "ERROR:  In Add Bin to Vec unable to release write lock" << endl;


	}


	return;

}
void SparseFeaturePointObjectClassifier::HoughBinVecMultiThread::addValuesToBin(HoughBin newBin,int binIndex)
{
	//First get the reader Lock for the vector
	int err = pthread_rwlock_rdlock(&vectorRwLock);
	if(err !=0 )
	{
		cout << "ERROR:  In add values to  Bin unable to get read lock" << endl;


	}



	err = pthread_rwlock_wrlock(&this->binLocks[binIndex]);
	if(err !=0 )
	{
		cout << "ERROR:  In Add values to bin unable to get bin write lock" << endl;


	}




	bins[binIndex] += newBin;

	err = pthread_rwlock_unlock(&this->binLocks[binIndex]);
	if(err !=0 )
	{
		cout << "ERROR:  In Add values to bin unable to release bin write lock" << endl;


	}




	err = pthread_rwlock_unlock(&vectorRwLock);
	if(err !=0 )
	{
		cout << "ERROR:  In Add Values to bin unable to release write lock" << endl;


	}




}



SparseFeaturePointObjectClassifier::MultiThreadedVectorsResult::MultiThreadedVectorsResult()
{

	this->sectionSize = 128;


	sectionLocksCreated = false;
	int err = pthread_mutex_init(&mainLock, NULL);




}
SparseFeaturePointObjectClassifier::MultiThreadedVectorsResult::MultiThreadedVectorsResult(int _sectionSize)
{

	this->sectionSize = _sectionSize;


	sectionLocksCreated = false;
	int err = pthread_mutex_init(&mainLock, NULL);



}
SparseFeaturePointObjectClassifier::MultiThreadedVectorsResult::~MultiThreadedVectorsResult()
{

	int err = pthread_mutex_destroy(&mainLock);
	if(this->sectionLocksCreated)
	{
		for(int i = 0; i < (int)this->sectionLocks.size(); i++)
		{
			err = pthread_mutex_destroy(&sectionLocks[i]);
		}
	}






}
void SparseFeaturePointObjectClassifier::MultiThreadedVectorsResult::createResultsVec(int numberOfResults)
{



	int err = 0;

	err = pthread_mutex_lock(&this->mainLock);
	this->resultsVec.resize(numberOfResults);

	int numberOfSectionLocks = numberOfResults/this->sectionSize;
	if(numberOfSectionLocks==0)
	{
		numberOfSectionLocks++;
	}

	if(this->sectionLocksCreated)
	{
		for(int i = 0; i < (int)this->sectionLocks.size(); i++)
		{
			err = pthread_mutex_destroy(&sectionLocks[i]);
		}
	}

	this->sectionLocks.resize(numberOfSectionLocks);


	for(int i =0; i< numberOfSectionLocks; i++)
	{
		err = pthread_mutex_init(&sectionLocks[i], NULL);

	}
	sectionLocksCreated = true;
	err = pthread_mutex_unlock(&this->mainLock);





}
void SparseFeaturePointObjectClassifier::MultiThreadedVectorsResult::copyToReultsVec(vector<Vec2f> newData, int startIndex)
{



	int lockIndex = startIndex/this->resultsVec.size();



	int err = pthread_mutex_lock(&this->sectionLocks[lockIndex]);
	for(int i = 0; i < newData.size(); i++)
	{

		this->resultsVec[i+startIndex] = newData[i];

	}

	err = pthread_mutex_unlock(&this->sectionLocks[lockIndex]);







}





void SparseFeaturePointObjectClassifier::MultiThreadedVectorsResult::copyToReultsVecNoLock(vector<Vec2f> newData, int startIndex)
{

	for(int i = 0; i < newData.size(); i++)
	{

		this->resultsVec[i+startIndex] = newData[i];

	}



}



void SparseFeaturePointObjectClassifier::MultiThreadedVectorsResult::copyToReultsVecMainLock(vector<Vec2f> newData, int startIndex)
{




	int err = pthread_mutex_lock(&this->mainLock);
	for(int i = 0; i < newData.size(); i++)
	{

		this->resultsVec[i+startIndex] = newData[i];

	}
	err = pthread_mutex_unlock(&this->mainLock);


}
vector<Vec2f> SparseFeaturePointObjectClassifier::MultiThreadedVectorsResult::getResultsVec()
{


	return this->resultsVec;
}
void SparseFeaturePointObjectClassifier::MultiThreadedVectorsResult::clear()
{


	this->resultsVec.clear();
	return;
}






SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierMultiThreadData::SparseFeaturePointObjectClassifierMultiThreadData()
{

	this->algorithmName = "SparesFeaturePointObjectClassifierMultiThreadData";

}

void SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierMultiThreadData::clear()
{
	this->estimatedRefPoints->clear();
	this->filteredMatchedKeypoints->clear();
	this->filteredMatchesStorage->clear();
	this->houghBinVec->clear();
	this->houghBinsToWorkOn.clear();
	this->transformsList->clear();
	this->vectorsToRef->clear();



}

void SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierMultiThreadData::clearNonShared()
{


	this->houghBinsToWorkOn.clear();

}

//**********************************Multithread classification



float SparseFeaturePointObjectClassifier::predictMultiThread(const Mat& queryDescriptors, const vector<KeyPoint> queryKeypoints,vector<vector <DMatch> > &matches,vector<Mat> & transformsList, vector<Point2f>& estimatedRefPoints, WorkerThreadInfo  * workerInfo)
{



	float returnVal = 0;

	if(((struct WorkerThreadInfo *)workerInfo)->coordinator)
	{

		returnVal = this->predictMultiThreadCoordinator(queryDescriptors,  queryKeypoints,matches,transformsList, estimatedRefPoints, (struct WorkerThreadInfo *)workerInfo);

	}
	else
	{
		this->predictMultiThreadWorker(queryDescriptors,  queryKeypoints,matches,transformsList, estimatedRefPoints, workerInfo);

	}




	return returnVal;


}
float SparseFeaturePointObjectClassifier::predictMultiThreadCoordinator(const Mat& queryDescriptors, const vector<KeyPoint> queryKeypoints,vector<vector <DMatch> > &matches,vector<Mat> & transformsList, vector<Point2f>& estimatedRefPoints, WorkerThreadInfo * workerInfo)
{

	float returnVal = 0.0;
	vector<DMatch> myFilteredMatches;


	struct SparseFeaturePointObjectClassifierMultiThreadData * multiThreadData = reinterpret_cast<struct SparseFeaturePointObjectClassifierMultiThreadData *> (workerInfo->multiThreadAlgorithmData);
	Thread * myThread = workerInfo->myThread;
	int myLogicalThreadId = myThread->getThreadLogicalId();
	int threadCount = workerInfo->myFeatureClassificationInfo->numberOfHorizontalProcs* workerInfo->myFeatureClassificationInfo->numberOfVerticalProcs;


	vector<WorkerThreadInfo *>currentWorkerInfo(threadCount);
	vector<SparseFeaturePointObjectClassifierMultiThreadData *> currentMultiThreadData(threadCount);

	for(int i = 0; i< threadCount; i++)
	{


		if(workerInfo->standAlone)
		{
			//This line uses the thread manager to get access to the other threads worker info

			struct GeneralWorkerThreadData * tmp  = reinterpret_cast<struct GeneralWorkerThreadData *> (workerInfo->threadManager->getThread(i)->getThreadParam());

			currentWorkerInfo[i] = reinterpret_cast<WorkerThreadInfo *> (tmp->featureClassificationData);

		}
		else
		{
			currentWorkerInfo[i] = reinterpret_cast<struct WorkerThreadInfo *> (workerInfo->threadManager->getThread(i)->getThreadParam());
		}

		currentMultiThreadData[i] = reinterpret_cast<struct SparseFeaturePointObjectClassifierMultiThreadData *> (currentWorkerInfo[i]->multiThreadAlgorithmData);



	}




	//Have everyone filter the matches they had

	//Apply neighbor ratio filtering to matches
	filterFeatureMatches( matches, myFilteredMatches );


	//Place the good matches into shared space
	multiThreadData->filteredMatchesStorage->copyToResultsVecMainLockResize(myFilteredMatches);



//	cout << "At first barrier" << endl;
	//Wait For everyone to get here
	myThread->waitAtBarrier();


	int totalNumberOfMatches = multiThreadData->filteredMatchesStorage->getResultsVecSizeNoLock();

	multiThreadData->filteredMatchedKeypoints->createResultsVec(totalNumberOfMatches);

	int matchesPerThread = totalNumberOfMatches/threadCount;
	int lastMatchesPerThread = totalNumberOfMatches- ((threadCount-1)* matchesPerThread);


	//Split up the work for computing bins and vectors
	for(int i = 0; i< threadCount; i++)
	{
		//Setup the index
		currentMultiThreadData[i]->myFilterMatchesIndex = i* matchesPerThread;
		currentMultiThreadData[i]->myFilterKeyPointsIndex = i* matchesPerThread;



		if(i != threadCount-1)
		{

			currentMultiThreadData[i]->myFilterMatchesCount = matchesPerThread;
			currentMultiThreadData[i]->myFilterKeyPointsCount = matchesPerThread;;

		}
		else
		{

			currentMultiThreadData[i]->myFilterMatchesCount = lastMatchesPerThread;
			currentMultiThreadData[i]->myFilterKeyPointsCount = lastMatchesPerThread;
		}

		if(i != myLogicalThreadId)
		{
			currentWorkerInfo[i]->myThread->wakeup();
		}


		//currentWorkerInfo[i];

	}

	vector<DMatch> myFilteredMatchesToWorkOn;
	vector<KeyPoint> myMatchedOriginalKeypoints;
	vector<KeyPoint> myMatchedNewKeypoints;
	vector<HoughBin> myHistogramBins;

	//Get my matches
	multiThreadData->filteredMatchesStorage->getResultsVecPortion(multiThreadData->myFilterMatchesIndex,multiThreadData->myFilterMatchesCount,myFilteredMatchesToWorkOn);

	//Compute the bins
	computeHoughBinsMultiThread(myMatchedOriginalKeypoints, myMatchedNewKeypoints,myHistogramBins,myFilteredMatchesToWorkOn, *(workerInfo->allQueryKeypoints),this->currentTrainKeypoints[0], this->currentTrainedRefVectors[0], this->currentTrainedRefPoints[0],multiThreadData->myFilterMatchesIndex,workerInfo);




	//Save the short lists to shared memory
	multiThreadData->filteredMatchedKeypoints->copyToReultsVecNoLock(myMatchedNewKeypoints,myMatchedOriginalKeypoints,multiThreadData->myFilterKeyPointsIndex);


	//Save the bins calculated to shared space
	for(int i =0; i < myHistogramBins.size(); i++)
	{

		multiThreadData->houghBinVec->addToBins(myHistogramBins[i]);
	}

//	cout << "At Second barrier" << endl;
	//Wait For everyone to get here
	myThread->waitAtBarrier();


	//Get the number bins
	int totalHoughBins = multiThreadData->houghBinVec->getHoughBinVectorPtr()->size();

//	cout << "Total Hough Bins: " << totalHoughBins << endl;

	//Allocate the memory for the bin indices
	for(int i = 0; i< threadCount; i++)
	{
		currentMultiThreadData[i]->houghBinsToWorkOn.reserve((totalHoughBins/threadCount)+1);

		//Tell them to get looking at their queue
//		if(i != myLogicalThreadId)
//		{
//			currentWorkerInfo[i]->myThread->wakeup();
//		}

	}


	int currentThreadQueue = 0;

	vector<SparseFeaturePointObjectClassifier::HoughBin> * sharedHoughBins = multiThreadData->houghBinVec->getHoughBinVectorPtr();

	//Assign bins to be computed on based on number of items within
	for(int i = 0; i< totalHoughBins; i++)
	{



		if((*sharedHoughBins)[i].count >=3)
		{

//			cout << "Filtered Bins: " << (*sharedHoughBins)[i] << endl;
//			if((*sharedHoughBins)[i].xIndex== 17 && (*sharedHoughBins)[i].yIndex ==18 && (*sharedHoughBins)[i].scaleIndex==7 && (*sharedHoughBins)[i].orientationIndex==0)
//			{
//				for(int j = 0; j < (*sharedHoughBins)[i].count;j++ )
//				{
//
//					cout << (*(multiThreadData->filteredMatchedKeypoints->getQueryResultsVecPtr()))[(*sharedHoughBins)[i].keypointIndices[j]].pt << endl;
//
//				}


//			}
			currentMultiThreadData[currentThreadQueue]->houghBinsToWorkOn.push_back(i);
			currentThreadQueue++;
			if(currentThreadQueue>= threadCount)
			{
				currentThreadQueue = 0;

			}

		}


	}

	//Lets them know it is the end of the queue
	for(int i = 0; i<threadCount; i++)
	{

		currentMultiThreadData[i]->houghBinsToWorkOn.push_back(-1);


	}

//	for(int i = 0; i<threadCount; i++)
//	{
//		cout << "Thread id: " << i << "  Bin Count: " << currentMultiThreadData[i]->houghBinsToWorkOn.size() << endl;
//	}

	for(int i = 0; i<threadCount; i++)
	{

		if(i != myLogicalThreadId)
		{
			currentWorkerInfo[i]->myThread->wakeup();
		}

	}

	//Once everyone else busy I will compute
	int lastHoughBinIndex = 0;

	int currentQueueIndex = 0;

	int currentHoughBinIndex = 0;
	vector<HoughBin> currentHoughBinsToProcess(multiThreadData->houghBinsToWorkOn.capacity());

	vector<Mat>  currentTransformsList;
	vector<Point2f> currentRefPoints;

	vector<Mat> myTransformsList;
	vector<Point2f> myRefPoints;

	//Now to compute transforms
	while(lastHoughBinIndex != -1)
	{
		if(currentQueueIndex < multiThreadData->houghBinsToWorkOn.size())
		{


			currentHoughBinsToProcess.clear();

			int currentHoughBinQueueSize = multiThreadData->houghBinsToWorkOn.size();
//			for(int i=0; i < currentHoughBinQueueSize; i++)
			for(currentQueueIndex; currentQueueIndex < currentHoughBinQueueSize; currentQueueIndex++)
			{
				currentHoughBinIndex = 	multiThreadData->houghBinsToWorkOn[currentQueueIndex];


				if(currentHoughBinIndex >=0)
				{
					currentHoughBinsToProcess.push_back((*sharedHoughBins)[currentHoughBinIndex]);
//					cout << "Current Hough Bin: " << (*sharedHoughBins)[currentHoughBinIndex] << endl;

				}



				lastHoughBinIndex = currentHoughBinIndex;

				//currentQueueIndex++;

			}

			//Operate on houghBinsToWorkOn[currentQueueIndex]
			currentTransformsList.clear();
			currentRefPoints.clear();
			getTransformsMultiThread(currentTransformsList, currentRefPoints,*(multiThreadData->filteredMatchedKeypoints->getTrainResultsVecPtr()),*(multiThreadData->filteredMatchedKeypoints->getQueryResultsVecPtr()),currentHoughBinsToProcess, this->currentTrainedRefPoints[0], workerInfo);
			for(int i = 0; i < currentTransformsList.size(); i++)
			{

				myTransformsList.push_back(currentTransformsList[i]);


			}

			for(int i = 0; i < currentRefPoints.size(); i++)
			{

				myRefPoints.push_back(currentRefPoints[i]);


			}

			//I could save back to main memory here but maybe not
		}
	}


	//Save my transforms count
	multiThreadData->myTransformsCount = myTransformsList.size();

//	cout << "KAt fourth barrier" << myThread->getThreadLogicalId() << "  " << multiThreadData->myTransformsCount << endl;

	//Wait for everyone
	myThread->waitAtBarrier();

	//Now we need to setup for putting the results back together
	int totalBins = 0;
	for(int i = 0; i< threadCount; i++)
	{

		currentMultiThreadData[i]->myTransformsIndex = totalBins;
		totalBins += currentMultiThreadData[i]->myTransformsCount;
	}


	multiThreadData->transformsList->createResultsVec(totalBins);
	multiThreadData->estimatedRefPoints->createResultsVec(totalBins);


//	cout << "At fifth barrier" << endl;
	//Wait for everyone
	myThread->waitAtBarrier();




	//Once all setup, go ahead and accumulate results
	multiThreadData->transformsList->copyToReultsVecNoLock(myTransformsList,multiThreadData->myTransformsIndex);
	multiThreadData->estimatedRefPoints->copyToReultsVecNoLock(myRefPoints,multiThreadData->myTransformsIndex);


//	cout << "At sixth barrier" << endl;
	//Wait for everyone
	myThread->waitAtBarrier();

//	for(int i = 0; i < multiThreadData->transformsList->getResultsVecPtr()->size(); i++)
//	{

//		cout << "Results Mat:" <<(*(multiThreadData->transformsList->getResultsVecPtr()))[i] << endl;

//	}

	//From accumulagte results compute the float output
	if(multiThreadData->transformsList->getResultsVecPtr()->size() >0)
	{
		returnVal = this->objClasses[0];


	}




	return returnVal;
}




void SparseFeaturePointObjectClassifier::computeHoughBinsMultiThread(vector<KeyPoint> &matchedOriginalKeypoints, vector<KeyPoint> & matchedNewKeypoints,vector<HoughBin> &histogramBins,const vector<DMatch> & matches,  const vector<KeyPoint>& queryKeypoints,const vector<KeyPoint>& trainedKeypoints, const vector<Vec2f>&  trainedRefVectors, const Point2f & trainedRefPoint, int matchStartIndex, WorkerThreadInfo  * workerInfo)
{

	vector<Vec2f> matchedCenterPointVectors;


	//Take the original list of all keypoints and pair them down to only the matched keypoints
	convertMatchesToSubListVectors( matches, trainedRefVectors,trainedKeypoints, queryKeypoints, matchedCenterPointVectors,  matchedOriginalKeypoints,  matchedNewKeypoints);


	//I take the center point vectors from the training set and the keypoints from the training and query set and create the center point
	//vectors for the query set based on which training set keypoint they match
	//So if the new feature is 2xScale with 30 degrees compared to the training set of 1xScale with 50 degrees
	//The vector to center is scaled by 2x/1x = 2 and the angle is adjust by 20 degrees to
	//get the new center point vector
	vector<Vec2f> newScaleAndObjectAngles;
	vector<Vec2f> newCenterVectors = computeScaleAdjustedCenterPointVectors(matchedCenterPointVectors, matchedOriginalKeypoints, matchedNewKeypoints, newScaleAndObjectAngles);



	//Computer the Hough bins using custom code
	houghBinTheRemappedVectors(histogramBins, newCenterVectors, newScaleAndObjectAngles, matchedNewKeypoints);


	for(int i = 0; i<histogramBins.size(); i++)
	{
		for(int j = 0; j < histogramBins[i].keypointIndices.size(); j++)
		{
			histogramBins[i].keypointIndices[j]+= matchStartIndex;
		}

	}


}







void SparseFeaturePointObjectClassifier::getTransformsMultiThread(vector<Mat> & transformsList, vector<Point2f>& refPoints,const vector<KeyPoint> &matchedOriginalKeypoints,const vector<KeyPoint> & matchedNewKeypoints,const vector<HoughBin> &histogramBins, const Point2f & trainedRefPoint, WorkerThreadInfo  * workerInfo)
{

	//Use the bins to compute possible transforms using affine error to filter
	vector<vector<int> > pointIndicesForTransform;
	transformsList = computerValidTransfroms (pointIndicesForTransform,histogramBins, matchedNewKeypoints,matchedOriginalKeypoints);

	//Setup the original Reference Point for calculating the new ref points
	Point3f oldRefPointForXForm(trainedRefPoint.x,trainedRefPoint.y,1);
	Mat refPointMat = Mat(oldRefPointForXForm).reshape(1);

	//Compute the new ref points
	for(int i=0; i < (int)transformsList.size(); i++)
	{
		Point2f relativelyCalculatedPoint;
		Mat newRefMat;//(relativelyCalculatedPoint);
		newRefMat = transformsList[i] * refPointMat;
		relativelyCalculatedPoint.x = newRefMat.at<float>(0,0);
		relativelyCalculatedPoint.y = newRefMat.at<float>(1,0);
		refPoints.push_back(relativelyCalculatedPoint);

	}



	return;



}



//You have no return because you are a worker thread that should merely give all data to coordinator;
void SparseFeaturePointObjectClassifier::predictMultiThreadWorker(const Mat& queryDescriptors, const vector<KeyPoint> queryKeypoints,vector<vector <DMatch> > &matches,vector<Mat> & transformsList, vector<Point2f>& estimatedRefPoints, WorkerThreadInfo  * workerInfo)
{
	vector<DMatch> myFilteredMatches;

	struct SparseFeaturePointObjectClassifierMultiThreadData * multiThreadData = reinterpret_cast<struct SparseFeaturePointObjectClassifierMultiThreadData *> (workerInfo->multiThreadAlgorithmData);
	Thread * myThread = workerInfo->myThread;


	//Have everyone filter the matches they had

	//Apply neighbor ratio filtering to matches
	filterFeatureMatches( matches, myFilteredMatches );


	//Place the good matches into shared space
	multiThreadData->filteredMatchesStorage->copyToResultsVecMainLockResize(myFilteredMatches);

	//Wait For everyone to get here
	myThread->waitAtBarrier();


	//Wait for more work to be given out
	myThread->waitForWakeup();


	//Do Bins


	vector<DMatch> myFilteredMatchesToWorkOn;
	vector<KeyPoint> myMatchedOriginalKeypoints;
	vector<KeyPoint> myMatchedNewKeypoints;
	vector<HoughBin> myHistogramBins;

	//Get my matches
	multiThreadData->filteredMatchesStorage->getResultsVecPortion(multiThreadData->myFilterMatchesIndex,multiThreadData->myFilterMatchesCount,myFilteredMatchesToWorkOn);

	//Compute some hough bins and a short list of matched only keypoints
	computeHoughBinsMultiThread(myMatchedOriginalKeypoints, myMatchedNewKeypoints,myHistogramBins,myFilteredMatchesToWorkOn, *(workerInfo->allQueryKeypoints),this->currentTrainKeypoints[0], this->currentTrainedRefVectors[0], this->currentTrainedRefPoints[0],multiThreadData->myFilterMatchesIndex ,workerInfo);

	//Save the short list to shared memory
	multiThreadData->filteredMatchedKeypoints->copyToReultsVecNoLock(myMatchedNewKeypoints,myMatchedOriginalKeypoints,multiThreadData->myFilterKeyPointsIndex);

	//Save the bins calculated to shared space
	for(int i =0; i < myHistogramBins.size(); i++)
	{
		multiThreadData->houghBinVec->addToBins(myHistogramBins[i]);
	}

	//Wait For everyone to get here
	myThread->waitAtBarrier();



	//Wait for more work to be given out
	myThread->waitForWakeup();

	//cout << "my id: " << myThread->getThreadLogicalId() << "  BC: " << multiThreadData->houghBinsToWorkOn.size() << endl;



	int lastHoughBinIndex = 0;

	int currentQueueIndex = 0;

	int currentHoughBinIndex = 0;
	vector<HoughBin> currentHoughBinsToProcess(multiThreadData->houghBinsToWorkOn.capacity());

	vector<SparseFeaturePointObjectClassifier::HoughBin> * sharedHoughBins = multiThreadData->houghBinVec->getHoughBinVectorPtr();

	vector<Mat> currentTransformsList;
	vector<Point2f> currentRefPoints;

	vector<Mat> myTransformsList;
	vector<Point2f>myRefPoints;

	//Now to compute transforms
	while(lastHoughBinIndex != -1)
	{
		if(currentQueueIndex < multiThreadData->houghBinsToWorkOn.size())
		{


			currentHoughBinsToProcess.clear();

			int currentHoughBinQueueSize = multiThreadData->houghBinsToWorkOn.size();


			for(currentQueueIndex; currentQueueIndex < currentHoughBinQueueSize; currentQueueIndex++)
			{
				currentHoughBinIndex = 	multiThreadData->houghBinsToWorkOn[currentQueueIndex];


				if(currentHoughBinIndex >=0)
				{


					currentHoughBinsToProcess.push_back((*sharedHoughBins)[currentHoughBinIndex]);
				}



				lastHoughBinIndex = currentHoughBinIndex;

				//currentQueueIndex++;

			}

			//Operate on houghBinsToWorkOn[currentQueueIndex]
			currentTransformsList.clear();
			currentRefPoints.clear();
			getTransformsMultiThread(currentTransformsList, currentRefPoints,*(multiThreadData->filteredMatchedKeypoints->getTrainResultsVecPtr()),*(multiThreadData->filteredMatchedKeypoints->getQueryResultsVecPtr()),currentHoughBinsToProcess, this->currentTrainedRefPoints[0], workerInfo);
			for(int i = 0; i < currentTransformsList.size(); i++)
			{

				myTransformsList.push_back(currentTransformsList[i]);


			}

			for(int i = 0; i < currentRefPoints.size(); i++)
			{

				myRefPoints.push_back(currentRefPoints[i]);


			}

			//I could save back to main memory here but maybe not
		}
	}


	//Save my transforms count
	multiThreadData->myTransformsCount = myTransformsList.size();
	//cout << "KAt fourth barrier" << myThread->getThreadLogicalId() << "  " << multiThreadData->myTransformsCount << endl;
	//Wait for everyone
	myThread->waitAtBarrier();

	//wait for coordinator to finish setting up data holder
	myThread->waitAtBarrier();




	//Once all setup, go ahead and accumulate results
	multiThreadData->transformsList->copyToReultsVecNoLock(myTransformsList,multiThreadData->myTransformsIndex);
	multiThreadData->estimatedRefPoints->copyToReultsVecNoLock(myRefPoints,multiThreadData->myTransformsIndex);

	//Wait for everyone (just a way for the coordinator to know we all finished)
	myThread->waitAtBarrier();


	//Coordinator will handle rest




	return;
}


/*
void SparseFeaturePointObjectClassifier::computeHoughBinsWorker(vector<KeyPoint> &matchedOriginalKeypoints, vector<KeyPoint> & matchedNewKeypoints,vector<HoughBin> &histogramBins,const vector<DMatch> & matches,  const vector<KeyPoint>& queryKeypoints,const vector<KeyPoint>& trainedKeypoints, const vector<Vec2f>&  trainedRefVectors, const Point2f & trainedRefPoint, WorkerThreadInfo  * workerInfo)
{

	vector<Vec2f> matchedCenterPointVectors;


	//Take the original list of all keypoints and pair them down to only the matched keypoints
	convertMatchesToSubListVectors( matches, trainedRefVectors,trainedKeypoints, queryKeypoints, matchedCenterPointVectors,  matchedOriginalKeypoints,  matchedNewKeypoints);


	//I take the center point vectors from the training set and the keypoints from the training and query set and create the center point
	//vectors for the query set based on which training set keypoint they match
	//So if the new feature is 2xScale with 30 degrees compared to the training set of 1xScale with 50 degrees
	//The vector to center is scaled by 2x/1x = 2 and the angle is adjust by 20 degrees to
	//get the new center point vector
	vector<Vec2f> newScaleAndObjectAngles;
	vector<Vec2f> newCenterVectors = computeScaleAdjustedCenterPointVectors(matchedCenterPointVectors, matchedOriginalKeypoints, matchedNewKeypoints, newScaleAndObjectAngles);


	//Computer the Hough bins using custom code
	houghBinTheRemappedVectors(histogramBins, newCenterVectors, newScaleAndObjectAngles, matchedNewKeypoints);
}


void SparseFeaturePointObjectClassifier::getTransformsWorker(vector<Mat> & transformsList, vector<Point2f>& refPoints,vector<KeyPoint> &matchedOriginalKeypoints, vector<KeyPoint> & matchedNewKeypoints,vector<HoughBin> &histogramBins, const Point2f & trainedRefPoint, WorkerThreadInfo  * workerInfo)
{
	//Use the bins to compute possible transforms using affine error to filter
	vector<vector<int> > pointIndicesForTransform;
	transformsList = computerValidTransfroms (pointIndicesForTransform,histogramBins, matchedNewKeypoints,matchedOriginalKeypoints);

	//Setup the original Reference Point for calculating the new ref points
	Point3f oldRefPointForXForm(trainedRefPoint.x,trainedRefPoint.y,1);
	Mat refPointMat = Mat(oldRefPointForXForm).reshape(1);

	//Compute the new ref points
	for(int i=0; i < (int)transformsList.size(); i++)
	{
		Point2f relativelyCalculatedPoint;
		Mat newRefMat;//(relativelyCalculatedPoint);
		newRefMat = transformsList[i] * refPointMat;
		relativelyCalculatedPoint.x = newRefMat.at<float>(0,0);
		relativelyCalculatedPoint.y = newRefMat.at<float>(1,0);
		refPoints.push_back(relativelyCalculatedPoint);

	}



return;



}

*/
