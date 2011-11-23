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
 * MultiThreadedMatResult.cpp
 *
 *  Created on: Apr 28, 2011
 *      Author: jlclemon
 */

#include "MultiThreadedMatResult.h"

MultiThreadedMatResult::MultiThreadedMatResult()
{
	// TODO Auto-generated constructor stub
	this->sectionSize = 128;
	sectionLocksCreated = false;
	int err = pthread_mutex_init(&mainLock, NULL);
}

MultiThreadedMatResult::~MultiThreadedMatResult()
{
	// TODO Auto-generated destructor stub
	int err = pthread_mutex_destroy(&mainLock);
	if(this->sectionLocksCreated)
	{
		for(int i = 0; i < (int)this->sectionLocks.size(); i++)
		{
			err = pthread_mutex_destroy(&sectionLocks[i]);
		}
	}

}


MultiThreadedMatResult::MultiThreadedMatResult(int _sectionSize)
{
	this->sectionSize = _sectionSize;


	sectionLocksCreated = false;
	int err = pthread_mutex_init(&mainLock, NULL);

}

void MultiThreadedMatResult::createResultsMat(int numberOfResults, int numberOfCols)
{
	int err = 0;

	err = pthread_mutex_lock(&this->mainLock);
	this->resultsMat.create(numberOfResults,numberOfCols,CV_32FC1);

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


void MultiThreadedMatResult::copyToReultsMat(Mat newData, int startIndex)
{

	int lockIndex = startIndex/this->resultsMat.rows;

	int endIndex = newData.rows + startIndex;
	Mat chosenRows = this->resultsMat.rowRange(startIndex,endIndex);
	int err = pthread_mutex_lock(&this->sectionLocks[lockIndex]);
	newData.copyTo(chosenRows);
	err = pthread_mutex_unlock(&this->sectionLocks[lockIndex]);
}

void MultiThreadedMatResult::copyToReultsMatNoLock(Mat newData, int startIndex)
{
	int endIndex = newData.rows + startIndex;
	Mat chosenRows = this->resultsMat.rowRange(startIndex,endIndex);

	newData.copyTo(chosenRows);


}
void MultiThreadedMatResult::copyToReultsMatMainLock(Mat newData, int startIndex)
{
	int endIndex = newData.rows + startIndex;


	Mat chosenRows = this->resultsMat.rowRange(startIndex,endIndex);
	int err = pthread_mutex_lock(&this->mainLock);
	newData.copyTo(chosenRows);
	err = pthread_mutex_unlock(&this->mainLock);
}



Mat MultiThreadedMatResult::getResultsMat()
{
	int err = pthread_mutex_lock(&this->mainLock);
	Mat returnMat = resultsMat.clone();
	err = pthread_mutex_unlock(&this->mainLock);
	return returnMat;
}




MultiThreadedMatchResult::MultiThreadedMatchResult()
{

	this->sectionSize = 128;


	sectionLocksCreated = false;
	int err = pthread_mutex_init(&mainLock, NULL);




}
MultiThreadedMatchResult::MultiThreadedMatchResult(int _sectionSize)
{

	this->sectionSize = _sectionSize;


	sectionLocksCreated = false;
	int err = pthread_mutex_init(&mainLock, NULL);



}
MultiThreadedMatchResult::~MultiThreadedMatchResult()
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
void MultiThreadedMatchResult::createResultsVec(int numberOfResults)
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
void MultiThreadedMatchResult::copyToReultsVec(vector<vector<DMatch> > newData, int startIndex)
{



	int lockIndex = startIndex/this->resultsVec.size();



	int err = pthread_mutex_lock(&this->sectionLocks[lockIndex]);
	for(int i = 0; i < newData.size(); i++)
	{

		this->resultsVec[i+startIndex] = newData[i];

	}

	err = pthread_mutex_unlock(&this->sectionLocks[lockIndex]);







}





void MultiThreadedMatchResult::copyToReultsVecNoLock(vector<vector<DMatch> > newData, int startIndex)
{

	for(int i = 0; i < newData.size(); i++)
	{

		this->resultsVec[i+startIndex] = newData[i];

	}



}



void MultiThreadedMatchResult::copyToReultsVecMainLock(vector<vector<DMatch> > newData, int startIndex)
{




	int err = pthread_mutex_lock(&this->mainLock);
	for(int i = 0; i < newData.size(); i++)
	{

		this->resultsVec[i+startIndex] = newData[i];

	}
	err = pthread_mutex_unlock(&this->mainLock);


}


vector<vector<DMatch> > MultiThreadedMatchResult::getResultsVec()
{


	return this->resultsVec;
}




MultiThreadedFilteredMatchResult::MultiThreadedFilteredMatchResult()
{

	int err = pthread_rwlock_init(&mainLock,NULL);




}
MultiThreadedFilteredMatchResult::MultiThreadedFilteredMatchResult(int _sectionSize)
{

	int err = pthread_rwlock_init(&mainLock,NULL);



}
MultiThreadedFilteredMatchResult::~MultiThreadedFilteredMatchResult()
{

	int err = pthread_rwlock_destroy(&mainLock);






}
void MultiThreadedFilteredMatchResult::createResultsVec(int numberOfResults)
{



	int err = 0;

	err = pthread_rwlock_wrlock(&this->mainLock);
	this->resultsVec.reserve(numberOfResults);

	err = pthread_rwlock_unlock(&this->mainLock);





}


int MultiThreadedFilteredMatchResult::increaseResultsVecSize(int amountToAdd)
{
	int err = pthread_rwlock_wrlock(&this->mainLock);
	int startingSize = this->resultsVec.size();
	this->resultsVec.resize(startingSize+amountToAdd);

	err = pthread_rwlock_unlock(&this->mainLock);

	return startingSize;


}
void MultiThreadedFilteredMatchResult::copyToResultsVecMainLockAfterResize(vector<DMatch>  newData, int startIndex)
{

	for(int i = 0; i < newData.size(); i++)
	{
		int err = pthread_rwlock_rdlock(&this->mainLock);
		this->resultsVec[i+startIndex] = newData[i];
		err = pthread_rwlock_unlock(&this->mainLock);
	}

}
void MultiThreadedFilteredMatchResult::copyToResultsVecMainLockResize(vector<DMatch>  newData)
{

	this->copyToResultsVecMainLockAfterResize(newData,increaseResultsVecSize(newData.size()));





}

int MultiThreadedFilteredMatchResult::getResultsVecSizeNoLock()
{

	return this->resultsVec.size();

}
vector<DMatch>  MultiThreadedFilteredMatchResult::getResultsVec()
{


	return this->resultsVec;
}

void  MultiThreadedFilteredMatchResult::getResultsVecPortion(int startIndex, int portionSize, vector<DMatch> & portion)
{
	portion.resize(portionSize);
	for(int i = 0; i < portionSize; i++)
	{

		portion[i] = this->resultsVec[i+startIndex];

	}

	return;
}

void  MultiThreadedFilteredMatchResult::clear()
{

	this->resultsVec.clear();
	return;

}





MultiThreadedQueryTrainKeypointsResult::MultiThreadedQueryTrainKeypointsResult()
{

	this->sectionSize = 128;


	sectionLocksCreated = false;
	int err = pthread_mutex_init(&mainLock, NULL);




}
MultiThreadedQueryTrainKeypointsResult::MultiThreadedQueryTrainKeypointsResult(int _sectionSize)
{

	this->sectionSize = _sectionSize;


	sectionLocksCreated = false;
	int err = pthread_mutex_init(&mainLock, NULL);




}
MultiThreadedQueryTrainKeypointsResult::~MultiThreadedQueryTrainKeypointsResult()
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
void MultiThreadedQueryTrainKeypointsResult::createResultsVec(int numberOfResults)
{



	int err = 0;

	err = pthread_mutex_lock(&this->mainLock);
	this->queryResultsVec.resize(numberOfResults);
	this->trainResultsVec.resize(numberOfResults);

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
void MultiThreadedQueryTrainKeypointsResult::copyToReultsVec(vector<KeyPoint> newQueryData,vector<KeyPoint> newTrainData, int startIndex)
{



	int lockIndex = startIndex/this->queryResultsVec.size();



	int err = pthread_mutex_lock(&this->sectionLocks[lockIndex]);
	for(int i = 0; i < newQueryData.size(); i++)
	{

		this->queryResultsVec[i+startIndex] = newQueryData[i];
		this->trainResultsVec[i+startIndex] = newTrainData[i];


	}

	err = pthread_mutex_unlock(&this->sectionLocks[lockIndex]);







}





void MultiThreadedQueryTrainKeypointsResult::copyToReultsVecNoLock(vector<KeyPoint> newQueryData,vector<KeyPoint> newTrainData, int startIndex)
{

	for(int i = 0; i < newQueryData.size(); i++)
	{

		this->queryResultsVec[i+startIndex] = newQueryData[i];
		this->trainResultsVec[i+startIndex] = newTrainData[i];

	}



}



void MultiThreadedQueryTrainKeypointsResult::copyToReultsVecMainLock(vector<KeyPoint> newQueryData,vector<KeyPoint> newTrainData, int startIndex)
{




	int err = pthread_mutex_lock(&this->mainLock);
	for(int i = 0; i < newQueryData.size(); i++)
	{

		this->queryResultsVec[i+startIndex] = newQueryData[i];
		this->trainResultsVec[i+startIndex] = newTrainData[i];
	}
	err = pthread_mutex_unlock(&this->mainLock);


}

int MultiThreadedQueryTrainKeypointsResult::getResultsVecSizeNoLock()
{
	return this->queryResultsVec.size();

}
vector<KeyPoint>  MultiThreadedQueryTrainKeypointsResult::getQueryResultsVec()
{


	return this->queryResultsVec;
}

vector<KeyPoint> MultiThreadedQueryTrainKeypointsResult::getTrainResultsVec()
{


	return this->trainResultsVec;
}

vector<KeyPoint> * MultiThreadedQueryTrainKeypointsResult::getQueryResultsVecPtr()
{


	return &(this->queryResultsVec);
}

vector<KeyPoint> * MultiThreadedQueryTrainKeypointsResult::getTrainResultsVecPtr()
{


	return &(this->trainResultsVec);
}
void MultiThreadedQueryTrainKeypointsResult::clear()
{


	this->queryResultsVec.clear();
	this->trainResultsVec.clear();
}






MultiThreadedPointVectorsResult::MultiThreadedPointVectorsResult()
{



	int err = pthread_mutex_init(&mainLock, NULL);




}
MultiThreadedPointVectorsResult::~MultiThreadedPointVectorsResult()
{

	int err = pthread_mutex_destroy(&mainLock);






}
void MultiThreadedPointVectorsResult::createResultsVec(int numberOfResults)
{



	int err = 0;

	err = pthread_mutex_lock(&this->mainLock);
	this->resultsVec.resize(numberOfResults);

	err = pthread_mutex_unlock(&this->mainLock);





}
void MultiThreadedPointVectorsResult::copyToReultsVec(vector<Point2f> newData, int startIndex)
{







	for(int i = 0; i < newData.size(); i++)
	{

		this->resultsVec[i+startIndex] = newData[i];

	}









}





void MultiThreadedPointVectorsResult::copyToReultsVecNoLock(vector<Point2f> newData, int startIndex)
{

	for(int i = 0; i < newData.size(); i++)
	{

		this->resultsVec[i+startIndex] = newData[i];

	}



}



void MultiThreadedPointVectorsResult::copyToReultsVecMainLock(vector<Point2f> newData, int startIndex)
{




	int err = pthread_mutex_lock(&this->mainLock);
	for(int i = 0; i < newData.size(); i++)
	{

		this->resultsVec[i+startIndex] = newData[i];

	}
	err = pthread_mutex_unlock(&this->mainLock);


}
vector<Point2f> MultiThreadedPointVectorsResult::getResultsVec()
{


	return this->resultsVec;
}

vector<Point2f> * MultiThreadedPointVectorsResult::getResultsVecPtr()
{


	return &(this->resultsVec);
}

void MultiThreadedPointVectorsResult::clear()
{


	this->resultsVec.clear();

	return;
}











MultiThreadedMatVectorsResult::MultiThreadedMatVectorsResult()
{



	int err = pthread_mutex_init(&mainLock, NULL);




}
MultiThreadedMatVectorsResult::~MultiThreadedMatVectorsResult()
{

	int err = pthread_mutex_destroy(&mainLock);






}
void MultiThreadedMatVectorsResult::createResultsVec(int numberOfResults)
{



	int err = 0;

	err = pthread_mutex_lock(&this->mainLock);
	this->resultsVec.resize(numberOfResults);

	err = pthread_mutex_unlock(&this->mainLock);





}
void MultiThreadedMatVectorsResult::copyToReultsVec(vector<Mat> newData, int startIndex)
{







	for(int i = 0; i < newData.size(); i++)
	{

		this->resultsVec[i+startIndex] = newData[i];

	}









}





void MultiThreadedMatVectorsResult::copyToReultsVecNoLock(vector<Mat> newData, int startIndex)
{

	for(int i = 0; i < newData.size(); i++)
	{

		this->resultsVec[i+startIndex] = newData[i];

	}



}



void MultiThreadedMatVectorsResult::copyToReultsVecMainLock(vector<Mat> newData, int startIndex)
{




	int err = pthread_mutex_lock(&this->mainLock);
	for(int i = 0; i < newData.size(); i++)
	{

		this->resultsVec[i+startIndex] = newData[i];

	}
	err = pthread_mutex_unlock(&this->mainLock);


}
vector<Mat> MultiThreadedMatVectorsResult::getResultsVec()
{


	return this->resultsVec;
}

vector<Mat> * MultiThreadedMatVectorsResult::getResultsVecPtr()
{


	return &(this->resultsVec);
}
void MultiThreadedMatVectorsResult::clear()
{


	this->resultsVec.clear();
	return;
}


