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
 * KnnFlannClassification.cpp
 *
 *  Created on: Apr 13, 2011
 *      Author: jlclemon
 */

#include "KnnFlannClassification.h"

using namespace cv;

ostream &operator<<(ostream &stream, KnnFlannClassification & ob)
{

	stream << "Knn Flann Classification: Params Created:" << ob.createdParams << "  Number neighbors:" << ob.max_k << "  Training set size:" << ob.trainData.rows;


	return stream;
}


KnnFlannClassification::KnnFlannClassification() {
	// TODO Auto-generated constructor stub

	createdParams = true;
	this->max_k = 32;
	this->flannIndexParams = new flann::KDTreeIndexParams(4);
	this->flannSearchParams = new flann::SearchParams(200);
	knnFlannMatcher = new FlannBasedMatcher(this->flannIndexParams, this->flannSearchParams);

}

KnnFlannClassification::~KnnFlannClassification() {
	// TODO Auto-generated destructor stub

	if(createdParams)
	{
//		delete this->flannIndexParams;
//		delete this->flannSearchParams;
		createdParams = false;


	}


}


KnnFlannClassification::KnnFlannClassification(KnnFlannClassifionrParams & params)
{
	createdParams = false;
	this->max_k = params.numberOfNeighbers;


	this->flannIndexParams = params.flannIndexParams;
	this->flannSearchParams = params.flannSearchParams;
	knnFlannMatcher = new FlannBasedMatcher(this->flannIndexParams, this->flannSearchParams);






}

KnnFlannClassification::KnnFlannClassification(int numberOfNeighbors, Ptr<flann::IndexParams> _flannIndexParams,	Ptr<flann::SearchParams> _flannSearchParams )
{
	createdParams = false;
	this->max_k = numberOfNeighbors;

	this->flannIndexParams = _flannIndexParams;
	this->flannSearchParams = _flannSearchParams;
	knnFlannMatcher = new FlannBasedMatcher(this->flannIndexParams, this->flannSearchParams);

}




KnnFlannClassification::KnnFlannClassification( const Mat& _train_data, const Mat& _responses,
            const Mat& _sample_idx, int numberOfNeighbors, Ptr<flann::IndexParams> _flannIndexParams, Ptr<flann::SearchParams> _flannSearchParams,bool _is_regression )
{
	createdParams = false;

	this->max_k = numberOfNeighbors;

	if(createdParams)
	{
//		delete this->flannIndexParams;
//		delete this->flannSearchParams;
		createdParams = false;


	}



	this->flannIndexParams = _flannIndexParams;
	this->flannSearchParams = _flannSearchParams;
	knnFlannMatcher = new FlannBasedMatcher(this->flannIndexParams, this->flannSearchParams);
	//Run Train
	this->train(_train_data, responses,_sample_idx, _is_regression, false);


}



bool KnnFlannClassification::train( const Mat& _train_data, const Mat & _responses,
                    const Mat& _sample_idx, bool _is_regression, bool _update_base )
{

	if(_update_base)
	{
		this->trainData.push_back(_train_data);
		vector<Mat> newData;
		newData.push_back(_train_data);
		knnFlannMatcher->add(newData);
		this->responses.push_back(_responses);
		this->sampleIndices = _sample_idx.clone();
	}
	else
	{
		this->trainData = _train_data.clone();
		vector<Mat> newData;
		newData.push_back(this->trainData);
		knnFlannMatcher->clear();
		knnFlannMatcher->add(newData);
		this->responses = _responses.clone();
		double minVal;
		minMaxLoc(responses,&minVal);

		if(minVal < 0)
		{

			this->responseOffset = (int)(0-minVal);


		}
		else
		{
			this->responseOffset = 0;
		}


		this->sampleIndices = _sample_idx.clone();




	}

	this->knnFlannMatcher->train();
	return true;

}

bool KnnFlannClassification::train( const Mat& _train_data, const Mat & _responses, const Mat& _sample_idx,
		int numberOfNeighbors, Ptr<flann::IndexParams> _flannIndexParams, Ptr<flann::SearchParams> _flannSearchParams,
		bool is_regression, bool _update_base )
{

	createdParams = false;

	this->max_k = numberOfNeighbors;

	if(createdParams)
	{
//		delete this->flannIndexParams;
//		delete this->flannSearchParams;
		createdParams = false;


	}


	this->flannIndexParams = _flannIndexParams;
	this->flannSearchParams = _flannSearchParams;
	knnFlannMatcher = new FlannBasedMatcher(this->flannIndexParams, this->flannSearchParams);
	//Run Train
	return this->train(_train_data, _responses,_sample_idx, is_regression, false);



}


float KnnFlannClassification::predict(Mat & samples, Mat & responses)
{
	vector<vector<DMatch> > matches;
	vector<vector<float> > neighbor_responses;

	find_nearest(samples,responses, matches, neighbor_responses);

	return responses.at<float>(0,0);


}

float KnnFlannClassification::find_nearest( const cv::Mat & _samples, Mat & results,
    vector<vector<DMatch> > &matches, vector<vector<float> > &neighbor_responses)
{
	int responsesHistoArray[KNN_FLANN_CLASSIFICATION_MAX_RESPONSE_VALUES];
	vector<Mat> masks;

	int max_k_to_use = max_k; //(this->trainData.rows < max_k?this->trainData.rows:max_k);

	//cout << "Sample Rows"<<_samples.rows  << "  resultsRows " << results.rows<< endl;
	//vector <vector<DMatch> > matches;
	//knnMatch(descriptors,topNMatches,numberOfNeighborsToFind);
	knnFlannMatcher->knnMatch(_samples,matches,max_k_to_use,masks,false);
	//this->knnFlannMatcher->knnMatch(_samples,matches,32);
//	vector<vector<DMatch> >::iterator it_samples_start = matches.begin();
//	vector<vector<DMatch> >::iterator it_samples_end = matches.end();
//	vector<vector<DMatch> >::iterator current_it_samples = it_samples_start;
//	for(;current_it_samples != it_samples_end; ++current_it_samples)
//	{

	neighbor_responses.clear();
	neighbor_responses.resize(matches.size());

	int maxResponse =0;
	int maxResponseIndex =0;

	results.create(matches.size(),1,CV_32FC1);

	for(int i = 0; i < (int)matches.size(); i++)
	{
		for(int k = 0; k < KNN_FLANN_CLASSIFICATION_MAX_RESPONSE_VALUES; k++)
		{
			responsesHistoArray[k] = 0;

		}



		for(int j = 0; j< (int)matches[i].size(); j++)
		{
			float currentResponse = this->responses.at<float>(matches[i][j].trainIdx,0);

			neighbor_responses[i].push_back(currentResponse);
			int histoLocation = (int)(currentResponse + this->responseOffset);
			responsesHistoArray[histoLocation]++;
			if(maxResponse < responsesHistoArray[histoLocation])
			{
				maxResponse = responsesHistoArray[histoLocation];
				maxResponseIndex = histoLocation;


			}

		}

		results.at<float>(i,0) = (float)(maxResponseIndex - this->responseOffset);
		maxResponse =0;
		maxResponseIndex =0;

	}


	//cout << "Results low level " << results << endl;
	return results.at<float>(0,0);

}



float KnnFlannClassification::find_nearest( const Mat & _samples, Mat & results )
{

	int responsesHistoArray[KNN_FLANN_CLASSIFICATION_MAX_RESPONSE_VALUES];
	vector<Mat> masks;
	vector <vector<DMatch> > matches;
	int max_k_to_use = (this->trainData.rows < max_k?this->trainData.rows:max_k);


	knnFlannMatcher->knnMatch(_samples,matches,max_k_to_use,masks,false);

//	vector<vector<DMatch> >::iterator it_samples_start = matches.begin();
//	vector<vector<DMatch> >::iterator it_samples_end = matches.end();
//	vector<vector<DMatch> >::iterator current_it_samples = it_samples_start;
//	for(;current_it_samples != it_samples_end; ++current_it_samples)
//	{


	int maxResponse =0;
	int maxResponseIndex =0;

	results.create(matches.size(),1,CV_32FC1);

	for(int i = 0; i < (int)matches.size(); i++)
	{
		for(int k = 0; k < KNN_FLANN_CLASSIFICATION_MAX_RESPONSE_VALUES; k++)
		{
			responsesHistoArray[k] = 0;

		}

		for(int j = 0; j< (int)matches[i].size(); j++)
		{
			float currentResponse = this->responses.at<float>(matches[i][j].trainIdx,0);

			int histoLocation = (int)(currentResponse + this->responseOffset);
			responsesHistoArray[histoLocation]++;
			if(maxResponse < responsesHistoArray[histoLocation])
			{
				maxResponse = responsesHistoArray[histoLocation];
				maxResponseIndex = histoLocation;


			}

		}

		results.at<float>(i,0) = (float)(maxResponseIndex - this->responseOffset);
		maxResponse =0;
		maxResponseIndex =0;

	}


	return results.at<float>(0,0);


}


void KnnFlannClassification::save( const char* filename, const char* name ) const
{

	FileStorage fs(filename,FileStorage::WRITE);


	this->write(*fs,name);



}
void KnnFlannClassification::load( const char* filename, const char* name )
{
	FileStorage fs(filename,FileStorage::READ);


	this->read(*fs,*fs[name]);


}

void KnnFlannClassification::write( CvFileStorage* storage, const char* name ) const
{

	//this->knnFlannMatcher->write(storage);
	cout << "Warning: Saving nearest neighbor result not allowed.  You will need to recompute every time....";
	//this->knnFlannMatcher->write(storage);
	return;




}



void KnnFlannClassification::read( CvFileStorage* storage, CvFileNode* node )
{


	cout << "Warning: Loading nearest neighbor result not allowed.  You will need to recompute every time....";
	//this->knnFlannMatcher->read(*node);


	return;
}

void KnnFlannClassification::clear()
{

	if(createdParams)
	{
//		delete this->flannIndexParams;
//		delete this->flannSearchParams;
		createdParams = false;


	}


	this->flannIndexParams = NULL;
	this->flannSearchParams = NULL;


}
int KnnFlannClassification::get_max_k() const
{
	return this->max_k;

}
int KnnFlannClassification::get_var_count() const
{

	return this->trainData.cols;

}



int KnnFlannClassification::get_sample_count() const
{

	return this->trainData.rows;


}
bool KnnFlannClassification::is_regression() const
{

	return this->regression;

}


