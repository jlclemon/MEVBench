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
 * HessKnnMatcher.h
 *
 *  Created on: Apr 15, 2011
 *      Author: jlclemon
 */

#ifndef HESSKNNMATCHER_H_
#define HESSKNNMATCHER_H_

#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/ml/ml.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <iostream>
using namespace cv;

class HessKnnMatcher
{
public:
	struct HessMatcherParams
	{
		static  int GET_DEFAULT_NUMBER_NEIGHBORS() { return 2;}
		static int GET_DEFAULT_NUMBER_CHECKS() {return 200;}
		static double GET_DEFAULT_NEIGHBOR_RATIO() {return 200;}

		int numberOfNeighbors;
		int numberOfChecks;
		double nearestNeighborRatio;
		HessMatcherParams();
		HessMatcherParams(int _numberOfNeighbors, int _numberOfChecks, double _nearestNeighborRatio);


	};



	HessKnnMatcher();
	HessKnnMatcher(HessMatcherParams params);
	HessKnnMatcher(const Mat& _trainDescriptors, HessMatcherParams params);
	void add(const Mat& _trainDescriptors);
	bool empty();
	void clear();
	void train();
	const vector<Mat> getTrainDescriptors() const;


	virtual ~HessKnnMatcher();


	void knnMatch(const Mat& queryDescriptors, const Mat& trainDescriptors, vector<vector<DMatch> >& matches, int k);
	void knnMatch(const Mat& queryDescriptors, vector<vector<DMatch> >& matches, int k);



	/** FEATURE_OXFD <BR> FEATURE_LOWE */
	enum MatchFeatureType
	{
		MATCH_FEATURE_OXFD,
		MATCH_FEATURE_LOWE,
	};

	/** FEATURE_FWD_MATCH <BR> FEATURE_BCK_MATCH <BR> FEATURE_MDL_MATCH */
/*	enum MatchFeatureMatchType
	{
		FEATURE_FWD_MATCH,
		FEATURE_BCK_MATCH,
		FEATURE_MDL_MATCH,
	};
*/

	/* colors in which to display different feature types */
//	#define FEATURE_OXFD_COLOR CV_RGB(255,255,0)
//	#define FEATURE_LOWE_COLOR CV_RGB(255,0,255)

	#ifndef ABS
	#define ABS(x) ( ( (x) < 0 )? -(x) : (x) )
	#endif

	/**
	Structure to represent an affine invariant image feature.  The fields
	x, y, a, b, c represent the affine region around the feature:

	a(x-u)(x-u) + 2b(x-u)(y-v) + c(y-v)(y-v) = 1
	*/
	struct MatchFeature
	{
		double x;                      /**< x coord */
		double y;                      /**< y coord */
		double a;                      /**< Oxford-type affine region parameter */
		double b;                      /**< Oxford-type affine region parameter */
		double c;                      /**< Oxford-type affine region parameter */
		double scl;                    /**< scale of a Lowe-style feature */
		double ori;                    /**< orientation of a Lowe-style feature */
		int d;                         /**< descriptor length */
		double * descr;		   		/**< descriptor */
		int type;                      /**< feature type, OXFD or LOWE */
		int category;                  /**< all-purpose feature category */
		int feature_id;					/**< Id for identifying the feature>**/
		int matched_feature_id;			/**< Id for identifying the feature you are matched to>**/
		struct MatchFeature* fwd_match;     /**< matching feature from forward image */
		struct MatchFeature* bck_match;     /**< matching feature from backmward image */
		struct MatchFeature* mdl_match;     /**< matching feature from model */
		CvPoint2D64f img_pt;           /**< location in image */
		CvPoint2D64f mdl_pt;           /**< location in model */
		void* feature_data;            /**< user-definable data */
		MatchFeature();
		~MatchFeature();

		MatchFeature & operator= (const MatchFeature & other);
		MatchFeature(const MatchFeature &other);


	};


	void allocateFeatureDescrData(MatchFeature * features,int numberOfFeatures, int descriptorLength);
	void freeFeatureDescrData(MatchFeature * features,int numberOfFeatures);
	void convertMatDescToMatchFeatureVector(const  Mat &descriptor, vector<struct MatchFeature> & features);
protected:




	/** a node in a k-d tree */
	struct kd_node
	{
		int ki;                      /**< partition key index */
		double kv;                   /**< partition key value */
		int leaf;                    /**< 1 if node is a leaf, 0 otherwise */
		struct MatchFeature* features;    /**< features at this node */
		int n;                       /**< number of features */
		struct kd_node* kd_left;     /**< left child */
		struct kd_node* kd_right;    /**< right child */
	};


	struct kd_node * currentKdRoot;
	vector<MatchFeature> currentMatchFeatures;
	vector<Mat> descriptorCollection;
	Mat trainDescriptors;
	HessMatcherParams currentParams;

	//	int currentNumberOfNeighbors;
//	int currentNumberOfChecks;
//	double currentNearestNeighborRatio;

	//int knnMatchInternal(vector<MatchFeature> queryFeatures,struct kd_node * kd_root, vector<vector<DMatch> > & matches, int numberOfNeighbors, int numberOfChecks);

	int knnMatchInternal(vector<MatchFeature> & queryFeatures,vector<MatchFeature> &trainFeatures, struct kd_node * kd_root, vector<vector<DMatch> > &matches, int numberOfNeighbors, int numberOfChecks);
	/*************************** Function Prototypes *****************************/

	/**
	A function to build a k-d tree database from keypoints in an array.

	@param features an array of features; <EM>this function rearranges the order
		of the features in this array, so you should take appropriate measures
		if you are relying on the order of the features (e.g. call this function
		before order is important)</EM>
	@param n the number of features in \a features

	@return Returns the root of a kd tree built from \a features.
	*/
	struct kd_node* kdtree_build( struct MatchFeature* features, int n );



	/**
	Finds an image feature's approximate k nearest neighbors in a kd tree using
	Best Bin First search.

	@param kd_root root of an image feature kd tree
	@param feat image feature for whose neighbors to search
	@param k number of neighbors to find
	@param nbrs pointer to an array in which to store pointers to neighbors
		in order of increasing descriptor distance; memory for this array is
		allocated by this function and must be freed by the caller using
		free(*nbrs)
	@param max_nn_chks search is cut off after examining this many tree entries

	@return Returns the number of neighbors found and stored in \a nbrs, or
		-1 on error.
	*/
	int kdtree_bbf_knn( struct kd_node* kd_root, struct MatchFeature* feat,
							  int k, struct MatchFeature*** nbrs, int max_nn_chks );


	/**
	Finds an image feature's approximate k nearest neighbors within a specified
	spatial region in a kd tree using Best Bin First search.

	@param kd_root root of an image feature kd tree
	@param feat image feature for whose neighbors to search
	@param k number of neighbors to find
	@param nbrs pointer to an array in which to store pointers to neighbors
		in order of increasing descriptor distance; memory for this array is
		allocated by this function and must be freed by the caller using
		free(*nbrs)
	@param max_nn_chks search is cut off after examining this many tree entries
	@param rect rectangular region in which to search for neighbors
	@param model if true, spatial search is based on kdtree features' model
		locations; otherwise it is based on their image locations

	@return Returns the number of neighbors found and stored in \a nbrs
		(in case \a k neighbors could not be found before examining
		\a max_nn_checks keypoint entries).
	*/
	int kdtree_bbf_spatial_knn( struct kd_node* kd_root,
									struct MatchFeature* feat, int k,
									struct MatchFeature*** nbrs, int max_nn_chks,
									CvRect rect, int model );


	/**
	De-allocates memory held by a kd tree

	@param kd_root pointer to the root of a kd tree
	*/
	void kdtree_release( struct kd_node* kd_root );














	struct bbf_data
	{
		double d;
		void* old_data;
	};

	class HessMinPQ
	{

	public:
		/* initial # of priority queue elements for which to allocate space */
		#define MINPQ_INIT_NALLOCD 512

		/********************************** Structures *******************************/

		/** an element in a minimizing priority queue */
		struct pq_node
		{
			void* data;
			int key;
		};


		/** a minimizing priority queue */
		struct min_pq
		{
			struct pq_node* pq_array;    /* array containing priority queue */
			int nallocd;                 /* number of elements allocated */
			int n;                       /**< number of elements in pq */
		};


		/*************************** Function Prototypes *****************************/

		/**
		Creates a new minimizing priority queue.
		*/
		struct min_pq* minpq_init();


		/**
		Inserts an element into a minimizing priority queue.

		@param min_pq a minimizing priority queue
		@param data the data to be inserted
		@param key the key to be associated with \a data

		@return Returns 0 on success or 1 on failure.
		*/
		int minpq_insert( struct min_pq* min_pq, void* data, int key );


		/**
		Returns the element of a minimizing priority queue with the smallest key
		without removing it from the queue.

		@param min_pq a minimizing priority queue

		@return Returns the element of \a min_pq with the smallest key or NULL
			if \a min_pq is empty
		*/
		void* minpq_get_min( struct min_pq* min_pq );


		/**
		Removes and returns the element of a minimizing priority queue with the
		smallest key.

		@param min_pq a minimizing priority queue

		@return Returns the element of \a min_pq with the smallest key of NULL
			if \a min_pq is empty
		*/
		void* minpq_extract_min( struct min_pq* min_pq );


		/**
		De-allocates the memory held by a minimizing priorioty queue

		@param min_pq pointer to a minimizing priority queue
		*/
		void minpq_release( struct min_pq** min_pq );
	private:

		void restore_minpq_order( struct pq_node*, int, int );
		void decrease_pq_node_key( struct pq_node*, int, int );

		/* returns the array index of element i's parent */
		__inline int parent( int i )
		{
			return ( i - 1 ) / 2;
		}


		/* returns the array index of element i's right child */
		__inline int right( int i )
		{
			return 2 * i + 2;
		}


		/* returns the array index of element i's left child */
		__inline int left( int i )
		{
			return 2 * i + 1;
		}

		int array_double( void** array, int n, int size );
	};

	/************************* Local Function Prototypes *************************/

	struct kd_node* kd_node_init( struct MatchFeature*, int );
	void expand_kd_node_subtree( struct kd_node* );
	void assign_part_key( struct kd_node* );
	double median_select( double*, int );
	double rank_select( double*, int, int );
	void insertion_sort( double*, int );
	int partition_array( double*, int, double );
	void partition_features( struct kd_node* );
	struct kd_node* explore_to_leaf( struct kd_node*, struct MatchFeature*,
											struct HessMinPQ::min_pq* );
	int insert_into_nbr_array( struct MatchFeature*, struct MatchFeature**, int, int );
	int within_rect( CvPoint2D64f, CvRect );

	double descr_dist_sq( struct MatchFeature* f1, struct MatchFeature* f2 );



















	HessMinPQ hessMinPQObj;


};





#endif /* HESSKNNMATCHER_H_ */
