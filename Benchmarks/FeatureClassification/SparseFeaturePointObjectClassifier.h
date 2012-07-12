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
 * SparseFeaturePointObjectClassifier.h
 *
 *  Created on: Apr 21, 2011
 *      Author: jlclemon
 */






#ifndef SPARSEFEATUREPOINTOBJECTCLASSIFIER_H_
#define SPARSEFEATUREPOINTOBJECTCLASSIFIER_H_

#include <pthread.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#ifndef OPENCV_VER_2_2
#include <opencv2/video/tracking.hpp>
#endif

#include "MultiThreadedMatResult.h"
#include "ThreadManager.h"
#include "MultiThreadAlgorithmData.h"


using namespace cv;
using namespace std;

struct WorkerThreadInfo;
class SparseFeaturePointObjectClassifier : public CvStatModel
{

public:

	struct SparseFeaturePointObjectClassifierParams
	{

		static float GET_DEFAULT_NN_RATIO() {return .40f;}
		static float GET_DEFAULT_MAX_AVE_SQ_ERROR(){return 3.0f;};
		static float GET_DEFAULT_MAX_IND_SQ_ERROR(){return 9.0f;};
		float nearestNeighborDistanceRatio;//.49f; //.8
		float maxAveSqError;
		float maxIndSqError;
		SparseFeaturePointObjectClassifierParams();
		SparseFeaturePointObjectClassifierParams(float _nearestNeighborDistanceRatio,float _maxAveSqError,float maxIndSqError);
		~SparseFeaturePointObjectClassifierParams();
	};


	SparseFeaturePointObjectClassifier();
	SparseFeaturePointObjectClassifier(SparseFeaturePointObjectClassifierParams _params);
	virtual ~SparseFeaturePointObjectClassifier();



    bool train( const Mat& _trainDescriptors, const vector<KeyPoint>& _trainKeypoints, const float _objClass,SparseFeaturePointObjectClassifier::SparseFeaturePointObjectClassifierParams _params,
                        Mat &_trainImage, int trainImageId,string& _traingImageName, bool _update_base=false );

    float predict(const Mat& queryDescriptors, const vector<KeyPoint> queryKeypoints,vector<vector <DMatch> > &matches,vector<Mat> & transformsList, vector<Point2f>& estimatedRefPoints);
    float predictMultiThread(const Mat& queryDescriptors, const vector<KeyPoint> queryKeypoints,vector<vector <DMatch> > &matches,vector<Mat> & transformsList, vector<Point2f>& estimatedRefPoints, WorkerThreadInfo  * workerInfo);



    virtual void clear();
    virtual void save( const char* filename, const char* name=0 ) const;
	virtual void load( const char* filename, const char* name=0 );

	virtual void write( CvFileStorage* storage, const char* name ) const;
	virtual void read( CvFileStorage* storage, CvFileNode* node );

	friend ostream &operator<<(ostream &stream, SparseFeaturePointObjectClassifier & ob);
	struct HoughBin
	{
	public:
		vector<int> keypointIndices;
		int scaleIndex;
		int orientationIndex;
		int xIndex;
		int yIndex;


		int scaleCenter;
		int orientationCenter;
		int xCenter;
		int yCenter;


		int count;
		bool operator==(const HoughBin &other) const;
		bool operator!=(const HoughBin &other) const ;

		HoughBin& operator=(const SparseFeaturePointObjectClassifier::HoughBin &rhs);
		HoughBin& operator+=(const HoughBin &rhs);
		friend ostream& operator<<(ostream& stream, const SparseFeaturePointObjectClassifier::HoughBin bin);
	};
	class HoughBinVecMultiThread
	{


	public:

		HoughBinVecMultiThread();
		~HoughBinVecMultiThread();
		void addToBins(HoughBin & newBin);
		vector<HoughBin> getHoughBinVector();
		vector<HoughBin> * getHoughBinVectorPtr();
		void clear();


	protected:
		int findMatchingBin(HoughBin &newBin);
		void addBinToVec(HoughBin & newBin);
		void addValuesToBin(HoughBin newBin,int binIndex);
		vector<HoughBin> bins;
		pthread_rwlock_t vectorRwLock;
		vector<pthread_rwlock_t> binLocks;



	};

	class MultiThreadedVectorsResult {
	public:
		MultiThreadedVectorsResult();
		MultiThreadedVectorsResult(int _sectionSize);
		virtual ~MultiThreadedVectorsResult();
		void createResultsVec(int numberOfResults);
		void copyToReultsVec(vector<Vec2f> newData, int startIndex);
		void copyToReultsVecNoLock(vector<Vec2f> newData, int startIndex);
		void copyToReultsVecMainLock(vector<Vec2f> newData, int startIndex);
		vector<Vec2f> getResultsVec();
		void clear();
	protected:
		vector<Vec2f> resultsVec;
		vector<pthread_mutex_t> sectionLocks;
		pthread_mutex_t mainLock;
		int sectionSize;
		bool sectionLocksCreated;

	};


	struct SparseFeaturePointObjectClassifierMultiThreadData : MultiThreadAlgorithmData
	{
		//vector<Mat> * transformsList;
		MultiThreadedMatVectorsResult * transformsList;
		//vector<Point2f>* estimatedRefPoints;
		MultiThreadedPointVectorsResult* estimatedRefPoints;
		int myTransformsCount;
		int myTransformsIndex;

		HoughBinVecMultiThread * houghBinVec;
		vector<int>  houghBinsToWorkOn;

		MultiThreadedVectorsResult * vectorsToRef;
		MultiThreadedQueryTrainKeypointsResult * filteredMatchedKeypoints;
		MultiThreadedFilteredMatchResult * filteredMatchesStorage;
		int myFilterMatchesIndex;
		int myFilterMatchesCount;
		int myFilterKeyPointsIndex;
		int myFilterKeyPointsCount;
		virtual void clearNonShared();
		virtual void clear();
		SparseFeaturePointObjectClassifierMultiThreadData();

	};




protected:
	static const double pi = 3.14159265358979323846;
	SparseFeaturePointObjectClassifierParams params;

	vector<Mat> currentTrainImages;   //To hold the trained images
	vector<string> currentTrainImagesNames;  //To hold their names
	vector<int> currentTrainImageIds;  //To hold their names
	vector<vector<KeyPoint> >currentTrainKeypoints;  //The current Reference Keypoints
	//Mat currentDescriptors;                  //All the descriptors in one mat
	vector <Mat> currentDescriptorCollection; // The descriptors by image
	vector<Point2f> currentTrainedRefPoints;

	vector<vector<Vec2f> > currentTrainedRefVectors;
	vector<float> objClasses;





	float computeVectorAngleRelativeToImageAxis(float vectorAngle, float featureAngle);
	float makeVectorAngleRelativeToFeatureAngle(float vectorAngle, float featureAngle);
	float calcDistanceBetweenPoints(Point2f start, Point2f end);
	Point2f computeCenterPointOfKeyPoints(vector<KeyPoint> & keypoints);
	float calcVectorAngleBetweenPoints(Point2f start, Point2f end);
	Point2f computeReferenceInCurrentImageFrame(Vec2f rAndRelativeTheta, KeyPoint keypoint);
	Vec2f computeVectorToCenterPoint(KeyPoint & keypoint, Point2f centerPoint);
	Vec2f computeRelativeVectorToCenterPoint(KeyPoint & keypoint, Point2f centerPoint);
	void convertMatchesToSubListVectors(vector<DMatch> matches, const vector<Vec2f>& centerPointVectors, const vector<KeyPoint> & originalKeypoints, const vector<KeyPoint> & newKeypoints, vector<Vec2f>& matchedCenterPointVectors, vector<KeyPoint> & matchedOriginalKeypoints, vector<KeyPoint> & matchedNewKeypoints);
	vector<Vec2f> computeScaleAdjustedCenterPointVectors(const vector<Vec2f>& centerPointVectors, const vector<KeyPoint> & originalKeypoints, const vector<KeyPoint> & newKeypoints, vector<Vec2f> & scaleAndAngleDiff);
	vector<Vec2f> computeRelativeCenterPointVectorsForKeypoints(vector<KeyPoint> & keypoints);
	vector<Point2f> computeReferencePointsInCurrentImageFrame(const vector<Vec2f> &relativeCenterVectors, const vector<KeyPoint> &keypoints);
	vector<int> addToHistogramBin(vector<int> xIndex, vector<int> yIndex, vector<int> scaleIndex,vector<int> orientationIndex,int keypointIndex, vector<HoughBin> & histogramBins);
	void computeBinIndices(Point2f location, Vec2f scaleAndOrientation, vector<int> & xIndex, vector<int> & yIndex, vector<int> &scaleIndex, vector<int> &orientationIndex);
	SparseMat calcHistUsingOpenCvMethods(vector<HoughBin> & histogramBins, const vector<Vec2f> & rAndThetas, const vector<Vec2f>& scaleAndAngles, const  vector<KeyPoint> &keypoints);
	void houghBinTheRemappedVectors(vector<HoughBin> & histogramBins, const vector<Vec2f> & rAndThetas, const vector<Vec2f>& scaleAndAngles, const  vector<KeyPoint> &keypoints);
	float computeSqErrorForHoughBin(Mat & computedTransform,vector<int> & pointIndicesForTransform, int & numberOfValidPointsInBin,const HoughBin & bin, const vector<KeyPoint> & newKeypoints,const vector<KeyPoint> & origKeypoints);
	vector<Mat> computerValidTransfroms (vector<vector<int> > & pointIndicesForTransform,const vector<HoughBin> & bins, const vector<KeyPoint> & newKeypoints,const vector<KeyPoint> & origKeypoints);
	void getTransforms(vector<Mat> & transformsList, vector<Point2f>& refPoints,const vector<DMatch> & matches,  const vector<KeyPoint> &queryKeypoints,const vector<KeyPoint> &trainedKeypoints, const vector<Vec2f>& trainedRefVectors, const Point2f & trainedRefPoint);
	void paintCenterOnImage(Mat & image, vector<KeyPoint> & keypoints,Mat & descriptors);
	void filterFeatureMatches(vector<vector<DMatch> > & matches, vector<DMatch> & filteredMatches );
	void filterFeatureMatches(vector<DMatch> & matches, vector<DMatch> & filteredMatches );


	//***************************************************MultiThread Below***************************************

    float predictMultiThreadCoordinator(const Mat& queryDescriptors, const vector<KeyPoint> queryKeypoints,vector<vector <DMatch> > &matches,vector<Mat> & transformsList, vector<Point2f>& estimatedRefPoints, WorkerThreadInfo  * workerInfo);
    void predictMultiThreadWorker(const Mat& queryDescriptors, const vector<KeyPoint> queryKeypoints,vector<vector <DMatch> > &matches,vector<Mat> & transformsList, vector<Point2f>& estimatedRefPoints, WorkerThreadInfo  * workerInfo);
    void computeHoughBinsMultiThread(vector<KeyPoint> &matchedOriginalKeypoints, vector<KeyPoint> & matchedNewKeypoints,vector<HoughBin> &histogramBins,const vector<DMatch> & matches,  const vector<KeyPoint>& queryKeypoints,const vector<KeyPoint>& trainedKeypoints, const vector<Vec2f>&  trainedRefVectors, const Point2f & trainedRefPoint, int matchStartIndex,WorkerThreadInfo  * workerInfo);
    //void getTransformsMultiThread(vector<Mat> & transformsList, vector<Point2f>& refPoints,vector<KeyPoint> &matchedOriginalKeypoints, vector<KeyPoint> & matchedNewKeypoints,vector<HoughBin> &histogramBins, const Point2f & trainedRefPoint, WorkerThreadInfo  * workerInfo);
    void getTransformsMultiThread(vector<Mat> & transformsList, vector<Point2f>& refPoints,const vector<KeyPoint> &matchedOriginalKeypoints,const vector<KeyPoint> & matchedNewKeypoints,const vector<HoughBin> &histogramBins, const Point2f & trainedRefPoint, WorkerThreadInfo  * workerInfo);





};
ostream &operator<<(ostream &stream, SparseFeaturePointObjectClassifier & ob);
ostream& operator<<(ostream& stream, const SparseFeaturePointObjectClassifier::HoughBin bin);





#endif /* SPARSEFEATUREPOINTOBJECTCLASSIFIER_H_ */
