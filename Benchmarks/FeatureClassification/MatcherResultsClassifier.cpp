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
 * MatcherResultsClassifier.cpp
 *
 *  Created on: Apr 19, 2011
 *      Author: jlclemon
 */

#include "MatcherResultsClassifier.h"


MatcherResultsClassifier::MatcherResultsClassifierParams::MatcherResultsClassifierParams():minClassValue(DEFAULT_MIN_CLASS_VALUE)
{




}
MatcherResultsClassifier::MatcherResultsClassifierParams::MatcherResultsClassifierParams(int _minClassValue): minClassValue(_minClassValue)
{



}


MatcherResultsClassifier::MatcherResultsClassifierParams::~MatcherResultsClassifierParams()
{


}



MatcherResultsClassifier::MatcherResultsClassifier() {
	// TODO Auto-generated constructor stub
	if(params.minClassValue < 0)
	{

		this->responseOffset = (int)(0-this->params.minClassValue);


	}
	else
	{
		this->responseOffset = 0;
	}



}

MatcherResultsClassifier::~MatcherResultsClassifier() {
	// TODO Auto-generated destructor stub
}

ostream &operator<<(ostream &stream, MatcherResultsClassifier&  ob)
{

	stream << "Matcher Results Classifier ";


	return stream;
}




MatcherResultsClassifier::MatcherResultsClassifier(MatcherResultsClassifierParams & _params)
{

	params = _params;
	if(params.minClassValue < 0)
	{

		this->responseOffset = (int)(0-this->params.minClassValue);


	}
	else
	{
		this->responseOffset = 0;
	}



}

MatcherResultsClassifier::MatcherResultsClassifier(const Mat & train_data, const  Mat &responses, MatcherResultsClassifierParams _params)
{


	this->train(train_data,responses,params);



}


void MatcherResultsClassifier::setParams(MatcherResultsClassifierParams & _params)
{

	params = _params;
	if(params.minClassValue < 0)
	{

		this->responseOffset = (int)(0-this->params.minClassValue);


	}
	else
	{
		this->responseOffset = 0;
	}




}
const MatcherResultsClassifier::MatcherResultsClassifierParams  MatcherResultsClassifier::getParams()
{

	return params;



}


void MatcherResultsClassifier::clear()
{

	params.minClassValue = 0;
	this->responseOffset = 0;
	this->trainDescriptors.release();
	this->trainResponses.release();
}



bool MatcherResultsClassifier::train( const Mat & train_data, const  Mat &responses, MatcherResultsClassifierParams _params)
{
	params = _params;

	this ->trainResponses = responses.clone();
	this ->trainDescriptors = train_data.clone();

	return true;

}


float MatcherResultsClassifier::predict(Mat & samples, Mat & sampleResponses, const vector<vector <DMatch> > &matches )
{


	float responsesHistoArray[MATCHER_RESULTS_CLASSIFICATION_MAX_RESPONSE_VALUES];


	float maxResponse =0;
	int maxResponseIndex =0;

	sampleResponses.create(matches.size(),1,CV_32FC1);

	for(int i = 0; i < (int)matches.size(); i++)
	{
		for(int k = 0; k < MATCHER_RESULTS_CLASSIFICATION_MAX_RESPONSE_VALUES; k++)
		{
			responsesHistoArray[k] = 0;

		}

		for(int j = 0; j< (int)matches[i].size(); j++)
		{
			float currentResponse = trainResponses.at<float>(matches[i][j].trainIdx,0);

			int histoLocation = (int)(currentResponse + this->responseOffset);
			responsesHistoArray[histoLocation]+= 1/(matches[i][j].distance+.0000001);
			if(maxResponse < responsesHistoArray[histoLocation])
			{
				maxResponse = responsesHistoArray[histoLocation];
				maxResponseIndex = histoLocation;


			}

		}

		sampleResponses.at<float>(i,0) = (float)(maxResponseIndex - this->responseOffset);
		maxResponse =0;
		maxResponseIndex =0;

	}


	return sampleResponses.at<float>(0,0);



}
float MatcherResultsClassifier::predict(Mat & samples, Mat & sampleResponses, const vector<vector <DMatch> >& matches ,vector<vector<float> > &neighbor_responses)
{

	float responsesHistoArray[MATCHER_RESULTS_CLASSIFICATION_MAX_RESPONSE_VALUES];

	neighbor_responses.clear();
	neighbor_responses.resize(matches.size());

	float maxResponse =0;
	int maxResponseIndex =0;

	sampleResponses.create(matches.size(),1,CV_32FC1);

	for(int i = 0; i < (int)matches.size(); i++)
	{
		for(int k = 0; k < MATCHER_RESULTS_CLASSIFICATION_MAX_RESPONSE_VALUES; k++)
		{
			responsesHistoArray[k] = 0;

		}



		for(int j = 0; j< (int)matches[i].size(); j++)
		{
			float currentResponse = trainResponses.at<float>(matches[i][j].trainIdx,0);

			neighbor_responses[i].push_back(currentResponse);
			int histoLocation = (int)(currentResponse + this->responseOffset);
			responsesHistoArray[histoLocation]+=1/(matches[i][j].distance +.0000001);
			if(maxResponse < responsesHistoArray[histoLocation])
			{
				maxResponse = responsesHistoArray[histoLocation];
				maxResponseIndex = histoLocation;


			}

		}

		sampleResponses.at<float>(i,0) = (float)(maxResponseIndex - this->responseOffset);
		maxResponse =0;
		maxResponseIndex =0;

	}


	cout << "Results low level " << sampleResponses << endl;
	return sampleResponses.at<float>(0,0);



}


float MatcherResultsClassifier::predict(Mat & sample, const vector <DMatch>matches)
{

	int responsesHistoArray[MATCHER_RESULTS_CLASSIFICATION_MAX_RESPONSE_VALUES];
	float currentResponse = 0;

	int maxResponse =0;
	int maxResponseIndex =0;



	for(int k = 0; k < MATCHER_RESULTS_CLASSIFICATION_MAX_RESPONSE_VALUES; k++)
	{
		responsesHistoArray[k] = 0;

	}

	for(int j = 0; j< (int)matches.size(); j++)
	{
		float currentResponse = trainResponses.at<float>(matches[j].trainIdx,0);

		int histoLocation = (int)(currentResponse + this->responseOffset);
		responsesHistoArray[histoLocation]++;
		if(maxResponse < responsesHistoArray[histoLocation])
		{
			maxResponse = responsesHistoArray[histoLocation];
			maxResponseIndex = histoLocation;


		}

	}



	currentResponse = (float)(maxResponseIndex - this->responseOffset);




	return currentResponse;



}










void MatcherResultsClassifier::save( const char* filename, const char* name ) const
{

	FileStorage fs(filename,FileStorage::WRITE);


	this->write(*fs,name);



}
void MatcherResultsClassifier::load( const char* filename, const char* name )
{
	FileStorage fs(filename,FileStorage::READ);


	this->read(*fs,*fs[name]);


}

void MatcherResultsClassifier::write( CvFileStorage* storage, const char* name ) const
{


	cout << "Warning: Saving Matcher Results Classifier not allowed.  You will need to recompute every time....";

	return;




}



void MatcherResultsClassifier::read( CvFileStorage* storage, CvFileNode* node )
{


	cout << "Warning: Loading Matcher Results Classifier not allowed.  You will need to recompute every time....";
	//this->knnFlannMatcher->read(*node);


	return;
}





