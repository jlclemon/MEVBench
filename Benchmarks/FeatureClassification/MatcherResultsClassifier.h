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
 * MatcherResultsClassifier.h
 *
 *  Created on: Apr 19, 2011
 *      Author: jlclemon
 */



#ifndef MATCHERRESULTSCLASSIFIER_H_
#define MATCHERRESULTSCLASSIFIER_H_

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/objdetect/objdetect.hpp>


using namespace cv;

class MatcherResultsClassifier :public CvStatModel
{


public:

	struct MatcherResultsClassifierParams
	{
        static const int DEFAULT_MIN_CLASS_VALUE = 0;


        int minClassValue;
        MatcherResultsClassifierParams();
        MatcherResultsClassifierParams(int _minClassValue);
        ~MatcherResultsClassifierParams();
	};



	MatcherResultsClassifier(MatcherResultsClassifierParams & _params);
	MatcherResultsClassifier(const Mat & train_data, const  Mat &responses, MatcherResultsClassifierParams _params);
	MatcherResultsClassifier();
	virtual ~MatcherResultsClassifier();


	virtual void clear();

	bool train( const Mat & train_data, const  Mat &responses, MatcherResultsClassifierParams _params);
	float predict(Mat & samples,  Mat & sampleResponses, const vector<vector <DMatch> >& matches );
	float predict(Mat & samples,  Mat & sampleResponses, const vector<vector <DMatch> >& matches ,vector<vector<float> > &neighbor_responses);

	float predict(Mat & sample, const vector <DMatch>matches);



	virtual void save( const char* filename, const char* name=0 ) const;
	virtual void load( const char* filename, const char* name=0 );

	virtual void write( CvFileStorage* storage, const char* name ) const;
	virtual void read( CvFileStorage* storage, CvFileNode* node );
	void setParams(MatcherResultsClassifierParams & _params);
	const MatcherResultsClassifierParams getParams();

	friend ostream &operator<<(ostream &stream, MatcherResultsClassifier & ob);

protected:
	static const int MATCHER_RESULTS_CLASSIFICATION_MAX_RESPONSE_VALUES=200;

	MatcherResultsClassifierParams params;
	int responseOffset;
	Mat trainResponses;
	Mat trainDescriptors;


};

ostream &operator<<(ostream &stream, MatcherResultsClassifier ob);

#endif /* MATCHERRESULTSCLASSIFIER_H_ */
