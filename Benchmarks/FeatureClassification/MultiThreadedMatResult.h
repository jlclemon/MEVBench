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
 * MultiThreadedMatResult.h
 *
 *  Created on: Apr 28, 2011
 *      Author: jlclemon
 */

#ifndef MULTITHREADEDMATRESULT_H_
#define MULTITHREADEDMATRESULT_H_

#include <pthread.h>
#include <vector>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/objdetect/objdetect.hpp>

using namespace cv;
using namespace std;


class MultiThreadedMatResult {
public:
	MultiThreadedMatResult();
	MultiThreadedMatResult(int _sectionSize);
	virtual ~MultiThreadedMatResult();
	void createResultsMat(int numberOfResults, int numberOfCols);
	void copyToReultsMat(Mat newData, int startIndex);
	void copyToReultsMatNoLock(Mat newData, int startIndex);
	void copyToReultsMatMainLock(Mat newData, int startIndex);
	Mat getResultsMat();

protected:
	Mat resultsMat;
	vector<pthread_mutex_t> sectionLocks;
	pthread_mutex_t mainLock;
	int sectionSize;
	bool sectionLocksCreated;

};





class MultiThreadedMatchResult {
public:
	MultiThreadedMatchResult();
	MultiThreadedMatchResult(int _sectionSize);
	virtual ~MultiThreadedMatchResult();
	void createResultsVec(int numberOfResults);
	void copyToReultsVec(vector<vector<DMatch> > newData, int startIndex);
	void copyToReultsVecNoLock(vector<vector<DMatch> > newData, int startIndex);
	void copyToReultsVecMainLock(vector<vector<DMatch> > newData, int startIndex);
	vector<vector<DMatch> > getResultsVec();

protected:
	vector<vector<DMatch> >resultsVec;
	vector<pthread_mutex_t> sectionLocks;
	pthread_mutex_t mainLock;
	int sectionSize;
	bool sectionLocksCreated;

};



class MultiThreadedFilteredMatchResult {
public:
	MultiThreadedFilteredMatchResult();
	MultiThreadedFilteredMatchResult(int _sectionSize);
	virtual ~MultiThreadedFilteredMatchResult();
	void createResultsVec(int numberOfResults);
	int increaseResultsVecSize(int amountToAdd);
	void copyToResultsVecMainLockAfterResize(vector<DMatch>  newData, int startIndex);
	void copyToResultsVecMainLockResize(vector<DMatch>  newData);
	int getResultsVecSizeNoLock();
	vector<DMatch>  getResultsVec();
	void  getResultsVecPortion(int startIndex, int portionSize, vector<DMatch> & portion);
	void clear();
protected:
	vector<DMatch> resultsVec;
	pthread_rwlock_t mainLock;

};




class MultiThreadedQueryTrainKeypointsResult {
public:
	MultiThreadedQueryTrainKeypointsResult();
	MultiThreadedQueryTrainKeypointsResult(int _sectionSize);
	virtual ~MultiThreadedQueryTrainKeypointsResult();
	void createResultsVec(int numberOfResults);
	void copyToReultsVec(vector<KeyPoint> newQueryData,vector<KeyPoint> newTrainData, int startIndex);
	void copyToReultsVecNoLock(vector<KeyPoint> newQueryData,vector<KeyPoint> newTrainData, int startIndex);
	void copyToReultsVecMainLock(vector<KeyPoint> newQueryData,vector<KeyPoint> newTrainData, int startIndex);
	int getResultsVecSizeNoLock();
	vector<KeyPoint>  getQueryResultsVec();
	vector<KeyPoint>  getTrainResultsVec();
	vector<KeyPoint>  * getQueryResultsVecPtr();
	vector<KeyPoint>  * getTrainResultsVecPtr();
	void clear();


protected:
	vector<KeyPoint> queryResultsVec;
	vector<KeyPoint> trainResultsVec;
	vector<pthread_mutex_t> sectionLocks;
	pthread_mutex_t mainLock;
	int sectionSize;
	bool sectionLocksCreated;

};

class MultiThreadedPointVectorsResult {
public:
	MultiThreadedPointVectorsResult();
	virtual ~MultiThreadedPointVectorsResult();
	void createResultsVec(int numberOfResults);
	void copyToReultsVec(vector<Point2f> newData, int startIndex);
	void copyToReultsVecNoLock(vector<Point2f> newData, int startIndex);
	void copyToReultsVecMainLock(vector<Point2f> newData, int startIndex);
	vector<Point2f> getResultsVec();
	vector<Point2f> * getResultsVecPtr();
	void clear();
protected:
	vector<Point2f> resultsVec;
	pthread_mutex_t mainLock;

};



class MultiThreadedMatVectorsResult {
public:
	MultiThreadedMatVectorsResult();
	virtual ~MultiThreadedMatVectorsResult();
	void createResultsVec(int numberOfResults);
	void copyToReultsVec(vector<Mat> newData, int startIndex);
	void copyToReultsVecNoLock(vector<Mat> newData, int startIndex);
	void copyToReultsVecMainLock(vector<Mat> newData, int startIndex);
	vector<Mat> getResultsVec();
	vector<Mat> * getResultsVecPtr();
	void clear();
protected:
	vector<Mat> resultsVec;
	pthread_mutex_t mainLock;

};





#endif /* MULTITHREADEDMATRESULT_H_ */
