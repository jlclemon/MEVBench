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
 * KnnFlannClassification.h
 *
 *  Created on: Apr 13, 2011
 *      Author: jlclemon
 */
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#ifndef KNNFLANNCLASSIFICATION_H_
#define KNNFLANNCLASSIFICATION_H_

using namespace cv;
using namespace std;

class KnnFlannClassification : public CvStatModel
{
public:


	struct KnnFlannClassifionrParams
	{
        static const int DEFAULT_NUMBER_OF_NEIGHBORS = 32;

		int numberOfNeighbers;

		Ptr<flann::IndexParams> flannIndexParams;
		Ptr<flann::SearchParams> flannSearchParams;


		KnnFlannClassifionrParams();
		KnnFlannClassifionrParams(int numberOfNeighbors, Ptr<flann::IndexParams> _flannIndexParams,	Ptr<flann::SearchParams> _flannSearchParams);




	};


	KnnFlannClassification();

	KnnFlannClassification(KnnFlannClassifionrParams & params);
	KnnFlannClassification(int numberOfNeighbors, Ptr<flann::IndexParams> _flannIndexParams, Ptr<flann::SearchParams> _flannSearchParams );




	KnnFlannClassification( const Mat& _train_data, const Mat& _responses,
                const Mat& _sample_idx, int numberOfNeighbors, Ptr<flann::IndexParams> _flannIndexParams, Ptr<flann::SearchParams> _flannSearchParams,bool _is_regression=false);

	virtual ~KnnFlannClassification();

    bool train( const Mat& _train_data, const Mat & _responses,
                        const Mat& _sample_idx, bool _is_regression=false, bool _update_base=false );

    bool train( const Mat& _train_data, const Mat & _responses, const Mat& _sample_idx,
    		int numberOfNeighbors, Ptr<flann::IndexParams> _flannIndexParams, Ptr<flann::SearchParams> _flannSearchParams,
    		bool _is_regression=false, bool _update_base=false );


    float predict(Mat & samples, Mat & responses);

    float find_nearest( const Mat & _samples, Mat & results,
        vector<vector<DMatch> > & matches, vector<vector<float> > & neighbor_responses );

    float find_nearest( const Mat & _samples, Mat & results );

    virtual void save( const char* filename, const char* name=0 ) const;
    virtual void load( const char* filename, const char* name=0 );

    void write( CvFileStorage* storage, const char* name ) const;
    void read( CvFileStorage* storage, CvFileNode* node );

    void clear();
    int get_max_k() const;
    int get_var_count() const;
    int get_sample_count() const;
    bool is_regression() const;

	friend ostream &operator<<(ostream &stream, KnnFlannClassification &ob);
protected:

	static const int KNN_FLANN_CLASSIFICATION_MAX_RESPONSE_VALUES = 200;
    int max_k;
    Mat trainData;
    Mat responses;
    int minResponse;
    int maxResponse;
    int responseOffset;

    Mat sampleIndices;
    bool regression;

    Ptr<DescriptorMatcher> knnFlannMatcher;

	Ptr<flann::IndexParams> flannIndexParams;
	Ptr<flann::SearchParams> flannSearchParams;
	bool createdParams;



};
ostream &operator<<(ostream &stream, KnnFlannClassification & ob);
#endif /* KNNFLANNCLASSIFICATION_H_ */
