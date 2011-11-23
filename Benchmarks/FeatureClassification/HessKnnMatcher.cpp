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
 * HessKnnMatcher.cpp
 *
 *  Created on: Apr 15, 2011
 *      Author: jlclemon
 */

#include "HessKnnMatcher.h"







HessKnnMatcher::HessKnnMatcher() {
	// TODO Auto-generated constructor stub

	currentKdRoot = NULL;

}

HessKnnMatcher::~HessKnnMatcher() {
	// TODO Auto-generated destructor stub


	if(currentKdRoot != NULL)
	{
		kdtree_release(this->currentKdRoot);
	}

}



HessKnnMatcher::HessMatcherParams::HessMatcherParams(): numberOfNeighbors(GET_DEFAULT_NUMBER_NEIGHBORS()), numberOfChecks(GET_DEFAULT_NUMBER_CHECKS()), nearestNeighborRatio(GET_DEFAULT_NEIGHBOR_RATIO())
{
}


HessKnnMatcher::HessKnnMatcher(HessMatcherParams params)
{
	this->currentParams = params;


}
HessKnnMatcher::HessKnnMatcher(const Mat& _trainDescriptors, HessMatcherParams params)
{
	this->currentParams = params;
	this->add(trainDescriptors);


}


HessKnnMatcher::MatchFeature::MatchFeature()
{
	descr = NULL;

}

HessKnnMatcher::MatchFeature & HessKnnMatcher::MatchFeature::operator= (const MatchFeature & other)
{
	if(this == &other)
	{

		return *this;
	}

	if(descr != NULL)
	{
		free(descr);

	}




	x =other.x;                      /**< x coord */

	y= other.y;                      /**< y coord */

	a =other.a;                      /**< Oxford-type affine region parameter */

	b =other.b;                      /**< Oxford-type affine region parameter */

	c =other.c;                      /**< Oxford-type affine region parameter */

	scl = other.scl;                    /**< scale of a Lowe-style feature */

	ori = other.ori;                    /**< orientation of a Lowe-style feature */

	d = other.d;                         /**< descriptor length */


	if(other.descr != NULL)
	{

		descr = (double *)calloc(d, sizeof(double));
		for(int i =0; i < d; i++)
		{
			descr[i] = other.descr[i];

		}



	}
	else
	{
		descr = NULL;

	}



	type = other.type;                      /**< feature type, OXFD or LOWE */

	category = other.category;                  /**< all-purpose feature category */

	feature_id =other.feature_id;					/**< Id for identifying the feature>**/

	matched_feature_id =other.matched_feature_id;			/**< Id for identifying the feature you are matched to>**/

	fwd_match = other.fwd_match;     /**< matching feature from forward image */

	bck_match = other.bck_match;     /**< matching feature from backmward image */

	mdl_match = other.mdl_match;     /**< matching feature from model */

	img_pt = other.img_pt;           /**< location in image */

	mdl_pt =other.mdl_pt;           /**< location in model */

	feature_data = other.feature_data;            /**< user-definable data */




	return *this;

}


HessKnnMatcher::MatchFeature::MatchFeature(const MatchFeature &other)
{


	x =other.x;                      /**< x coord */

	y= other.y;                      /**< y coord */

	a =other.a;                      /**< Oxford-type affine region parameter */

	b =other.b;                      /**< Oxford-type affine region parameter */

	c =other.c;                      /**< Oxford-type affine region parameter */

	scl = other.scl;                    /**< scale of a Lowe-style feature */

	ori = other.ori;                    /**< orientation of a Lowe-style feature */

	d = other.d;                         /**< descriptor length */


	if(other.descr != NULL)
	{

		descr = (double *)calloc(d, sizeof(double));
		for(int i =0; i < d; i++)
		{
			descr[i] = other.descr[i];

		}



	}
	else
	{
		descr = NULL;

	}



	type = other.type;                      /**< feature type, OXFD or LOWE */

	category = other.category;                  /**< all-purpose feature category */

	feature_id =other.feature_id;					/**< Id for identifying the feature>**/

	matched_feature_id =other.matched_feature_id;			/**< Id for identifying the feature you are matched to>**/

	fwd_match = other.fwd_match;     /**< matching feature from forward image */

	bck_match = other.bck_match;     /**< matching feature from backmward image */

	mdl_match = other.mdl_match;     /**< matching feature from model */

	img_pt = other.img_pt;           /**< location in image */

	mdl_pt =other.mdl_pt;           /**< location in model */

	feature_data = other.feature_data;            /**< user-definable data */




}



HessKnnMatcher::MatchFeature::~MatchFeature()
{
	if(descr != NULL)
	{
		free(descr);

	}
}



void HessKnnMatcher::add(const Mat& _trainDescriptors)
{
	Mat newDescriptors = _trainDescriptors.clone();

	descriptorCollection.push_back(newDescriptors);

	if(trainDescriptors.empty())
	{
		trainDescriptors = newDescriptors.clone();

	}
	else
	{
		trainDescriptors.push_back(newDescriptors);
	}

	convertMatDescToMatchFeatureVector(trainDescriptors, currentMatchFeatures);



	currentKdRoot = HessKnnMatcher::kdtree_build( &currentMatchFeatures[0], currentMatchFeatures.size() );

}
bool HessKnnMatcher::empty()
{
	bool results = false;

	if(currentKdRoot == NULL || currentMatchFeatures.empty() || descriptorCollection.empty() || trainDescriptors.empty())
	{
		results = true;
	}

	return results;

}


void HessKnnMatcher::clear()
{

	if(currentKdRoot != NULL)
	{
		kdtree_release(this->currentKdRoot);
		currentKdRoot = NULL;
	}
	this->currentMatchFeatures.clear();
	this->descriptorCollection.clear();
	this->trainDescriptors.release();


}

void HessKnnMatcher::train()
{



}

const vector<Mat> HessKnnMatcher::getTrainDescriptors() const
{
	return this->descriptorCollection;

}



int HessKnnMatcher::knnMatchInternal(vector<MatchFeature> & queryFeatures,vector<MatchFeature> & trainFeatures, struct kd_node * kd_root, vector<vector<DMatch> > &matches, int numberOfNeighbors, int numberOfChecks)
{
	int numberOfMatches;
	int numberOfQueryFeatures = queryFeatures.size();
	int numberOfTrainedFeatures = trainFeatures.size();


	if(numberOfTrainedFeatures >0 && numberOfQueryFeatures >0 && kd_root != NULL  && numberOfChecks > 0 && numberOfNeighbors)
	{
		struct MatchFeature * feat;
		struct MatchFeature** nbrs;
		struct MatchFeature* queryFeat = &queryFeatures[0];

		double d0;
		int k, i, m = 0;
		DMatch currentMatch;
		/*
		cout << "Descriptors as of now: " << this->trainDescriptors << endl;
		for (int p=0; p< this->currentMatchFeatures.size(); p++)
		{
			std::cout << "Feature Id: "<<this->currentMatchFeatures[p].feature_id << endl;
			std::cout << "Desc:  [" << this->currentMatchFeatures[p].descr[0];
			for(int v = 1; v< this->currentMatchFeatures[p].d; v++)
			{
				std::cout << " , " << this->currentMatchFeatures[p].descr[v];

			}
			std::cout << "]" << endl;
		}
		*/
		matches.clear();
		matches.reserve(numberOfQueryFeatures);
		for( i = 0; i < numberOfQueryFeatures; i++ )
		{
			feat = queryFeat + i;
			feat->feature_id = i;

			k = kdtree_bbf_knn( kd_root, feat, numberOfNeighbors, &nbrs, numberOfChecks );
			vector<DMatch> currentMatchSet(k);
			if(k >0)
			{

				for(int j =0; j < k; j++)
				{

					d0 = descr_dist_sq( feat, nbrs[j] );
					queryFeat[i].fwd_match = nbrs[j];
					queryFeat[i].matched_feature_id = nbrs[j]->feature_id;
					currentMatchSet[j].distance = d0;
					currentMatchSet[j].queryIdx = queryFeat[i].feature_id;
					currentMatchSet[j].trainIdx = queryFeat[i].matched_feature_id;
					m++;
				}

				matches.push_back(currentMatchSet);
			}


			if(nbrs != NULL)
			{
				free( nbrs );
			}
		}
		numberOfMatches = m;
	}

	return numberOfMatches;
}

void HessKnnMatcher::convertMatDescToMatchFeatureVector(const  Mat &descriptor, vector<struct MatchFeature> & features)
{

	if(!descriptor.empty() )
	{
		features.resize(descriptor.rows);
		this->allocateFeatureDescrData(&features[0],descriptor.rows,descriptor.cols);

		for(int i=0; i < (int)features.size(); i++)
		{
			features[i].feature_id = i;
			features[i].type = MATCH_FEATURE_LOWE;
			features[i].d = descriptor.cols;

			for(int j = 0; j<features[i].d; j++)
			{
				features[i].descr[j] = descriptor.at<float>(i,j);

			}

		}
	}

}








void HessKnnMatcher::knnMatch(const Mat& queryDescriptors, const Mat& trainDescriptors, vector<vector<DMatch> >& matches, int k)
{
	vector<MatchFeature> trainFeatures;
	convertMatDescToMatchFeatureVector(trainDescriptors, trainFeatures);

	vector<MatchFeature> queryFeatures;
	convertMatDescToMatchFeatureVector(queryDescriptors, queryFeatures);




	struct kd_node* tmpKdRoot = HessKnnMatcher::kdtree_build( &trainFeatures[0], trainFeatures.size() );

	int numberOfNeighbors;
	if( k <=0)
	{
		numberOfNeighbors = this->currentParams.numberOfNeighbors;

	}
	else
	{
		numberOfNeighbors = k;

	}


	int numberOfMatches = HessKnnMatcher::knnMatchInternal(queryFeatures,trainFeatures, tmpKdRoot, matches, numberOfNeighbors, this->currentParams.numberOfChecks);


	kdtree_release(tmpKdRoot);
	return;
}


void HessKnnMatcher::knnMatch(const Mat& queryDescriptors, vector<vector<DMatch> >& matches, int k)
{

	vector<MatchFeature> queryFeatures;
	convertMatDescToMatchFeatureVector(queryDescriptors, queryFeatures);
	int numberOfNeighbors;

	if( k <=0)
	{
		numberOfNeighbors = this->currentParams.numberOfNeighbors;

	}
	else
	{
		numberOfNeighbors = k;

	}

	int numberOfMatches =  knnMatchInternal(queryFeatures,this->currentMatchFeatures, this->currentKdRoot, matches, numberOfNeighbors,this->currentParams.numberOfChecks);



	return;
}



/******************** Functions prototyped in keyptdb.h **********************/


void HessKnnMatcher::allocateFeatureDescrData(MatchFeature * features ,int numberOfFeatures, int descriptorLength)
{
	if(features != NULL)
	{
		for(int i = 0; i < numberOfFeatures; i++)
		{
			features[i].descr = (double *)calloc(descriptorLength, sizeof(double));

		}
	}


}

void HessKnnMatcher::freeFeatureDescrData(MatchFeature * features ,int numberOfFeatures)
{
	if(features != NULL)
	{
		for(int i = 0; i < numberOfFeatures; i++)
		{
			free(features[i].descr);
		}
	}


}



/*
A function to build a k-d tree database from keypoints in an array.

@param features an array of features
@param n the number of features in features

@return Returns the root of a kd tree built from features or NULL on error.
*/
struct HessKnnMatcher::kd_node* HessKnnMatcher::kdtree_build( struct MatchFeature* features, int n )
{
	struct kd_node* kd_root;

	if( ! features  ||  n <= 0 )
	{
		fprintf( stderr, "Warning: kdtree_build(): no features, %s, line %d\n",
				__FILE__, __LINE__ );
		return NULL;
	}

	kd_root = kd_node_init( features, n );
	expand_kd_node_subtree( kd_root );

	return kd_root;
}



/*
Finds an image feature's approximate k nearest neighbors in a kd tree using
Best Bin First search.

@param kd_root root of an image feature kd tree
@param feat image feature for whose neighbors to search
@param k number of neighbors to find
@param nbrs pointer to an array in which to store pointers to neighbors
	in order of increasing descriptor distance
@param max_nn_chks search is cut off after examining this many tree entries

@return Returns the number of neighbors found and stored in nbrs, or
	-1 on error.
*/
int HessKnnMatcher::kdtree_bbf_knn( struct kd_node* kd_root, struct MatchFeature* feat, int k,
					struct MatchFeature*** nbrs, int max_nn_chks )
{
	struct kd_node* expl;
	struct HessMinPQ::min_pq* min_pq;
	struct MatchFeature* tree_feat, ** _nbrs;
	struct bbf_data* bbf_data;
	int i, t = 0, n = 0;

	if( ! nbrs  ||  ! feat  ||  ! kd_root )
	{
		fprintf( stderr, "Warning: NULL pointer error, %s, line %d\n",
				__FILE__, __LINE__ );
		return -1;
	}

	_nbrs = (struct MatchFeature**)calloc( k, sizeof( struct matchFeature* ) );
	min_pq = hessMinPQObj.minpq_init();
	hessMinPQObj.minpq_insert( min_pq, kd_root, 0 );
	while( min_pq->n > 0  &&  t < max_nn_chks )
	{
		expl = (struct kd_node*)hessMinPQObj.minpq_extract_min( min_pq );
		if( ! expl )
		{
			fprintf( stderr, "Warning: PQ unexpectedly empty, %s line %d\n",
					__FILE__, __LINE__ );
			goto fail;
		}

		expl = explore_to_leaf( expl, feat, min_pq );
		if( ! expl )
		{
			fprintf( stderr, "Warning: PQ unexpectedly empty, %s line %d\n",
					__FILE__, __LINE__ );
			goto fail;
		}

		for( i = 0; i < expl->n; i++ )
		{
			tree_feat = &expl->features[i];
			bbf_data = (struct bbf_data * )malloc( sizeof( struct bbf_data ) );
			if( ! bbf_data )
			{
				fprintf( stderr, "Warning: unable to allocate memory,"
					" %s line %d\n", __FILE__, __LINE__ );
				goto fail;
			}
			bbf_data->old_data = tree_feat->feature_data;
			bbf_data->d = descr_dist_sq(feat, tree_feat);
			tree_feat->feature_data = bbf_data;
			n += insert_into_nbr_array( tree_feat, _nbrs, n, k );
		}
		t++;
	}

	hessMinPQObj.minpq_release( &min_pq );
	for( i = 0; i < n; i++ )
	{
		bbf_data = (struct bbf_data*)_nbrs[i]->feature_data;
		_nbrs[i]->feature_data = bbf_data->old_data;
		free( bbf_data );
	}
	*nbrs = _nbrs;
	return n;

fail:
hessMinPQObj.minpq_release( &min_pq );
	for( i = 0; i < n; i++ )
	{
		bbf_data = (struct bbf_data*)_nbrs[i]->feature_data;
		_nbrs[i]->feature_data = bbf_data->old_data;
		free( bbf_data );
	}
	free( _nbrs );
	*nbrs = NULL;
	return -1;
}



/*
Finds an image feature's approximate k nearest neighbors within a specified
spatial region in a kd tree using Best Bin First search.

@param kd_root root of an image feature kd tree
@param feat image feature for whose neighbors to search
@param k number of neighbors to find
@param nbrs pointer to an array in which to store pointers to neighbors
	in order of increasing descriptor distance
@param max_nn_chks search is cut off after examining this many tree entries
@param rect rectangular region in which to search for neighbors
@param model if true, spatial search is based on kdtree features' model
	locations; otherwise it is based on their image locations

@return Returns the number of neighbors found and stored in \a nbrs
	(in case \a k neighbors could not be found before examining
	\a max_nn_checks keypoint entries).
*/
int HessKnnMatcher::kdtree_bbf_spatial_knn( struct kd_node* kd_root, struct MatchFeature* feat,
						   int k, struct MatchFeature*** nbrs, int max_nn_chks,
						   CvRect rect, int model )
{
	struct MatchFeature** all_nbrs, ** sp_nbrs;
	CvPoint2D64f pt;
	int i, n, t = 0;

	n = kdtree_bbf_knn( kd_root, feat, max_nn_chks, &all_nbrs, max_nn_chks );
	sp_nbrs = (struct MatchFeature**)calloc( k, sizeof( struct MatchFeature* ) );
	for( i = 0; i < n; i++ )
	{
		if( model )
			pt = all_nbrs[i]->mdl_pt;
		else
			pt = all_nbrs[i]->img_pt;

		if( within_rect( pt, rect ) )
		{
			sp_nbrs[t++] = all_nbrs[i];
			if( t == k )
				goto end;
		}
	}
end:
	free( all_nbrs );
	*nbrs = sp_nbrs;
	return t;
}



/*
De-allocates memory held by a kd tree

@param kd_root pointer to the root of a kd tree
*/
void HessKnnMatcher::kdtree_release( struct HessKnnMatcher::kd_node* kd_root )
{
	if( ! kd_root )
		return;
	kdtree_release( kd_root->kd_left );
	kdtree_release( kd_root->kd_right );
	free( kd_root );
}



/************************ Functions prototyped here **************************/


/*
Initializes a kd tree node with a set of features.  The node is not
expanded, and no ordering is imposed on the features.

@param features an array of image features
@param n number of features

@return Returns an unexpanded kd-tree node.
*/
struct HessKnnMatcher::kd_node* HessKnnMatcher::kd_node_init( struct MatchFeature* features, int n )
{
	struct kd_node* kd_node;

	kd_node = (struct kd_node *)malloc( sizeof( struct kd_node ) );
	memset( kd_node, 0, sizeof( struct kd_node ) );
	kd_node->ki = -1;
	kd_node->features = features;
	kd_node->n = n;

	return kd_node;
}



/*
Recursively expands a specified kd tree node into a tree whose leaves
contain one entry each.

@param kd_node an unexpanded node in a kd tree
*/
void HessKnnMatcher::expand_kd_node_subtree( struct kd_node* kd_node )
{
	/* base case: leaf node */
	if( kd_node->n == 1  ||  kd_node->n == 0 )
	{
		kd_node->leaf = 1;
		return;
	}

	assign_part_key( kd_node );
	partition_features( kd_node );

	if( kd_node->kd_left )
		expand_kd_node_subtree( kd_node->kd_left );
	if( kd_node->kd_right )
		expand_kd_node_subtree( kd_node->kd_right );
}



/*
Determines the descriptor index at which and the value with which to
partition a kd tree node's features.

@param kd_node a kd tree node
*/
void HessKnnMatcher::assign_part_key( struct kd_node* kd_node )
{
	struct MatchFeature* features;
	double kv, x, mean, var, var_max = 0;
	double* tmp;
	int d, n, i, j, ki = 0;

	features = kd_node->features;
	n = kd_node->n;
	d = features[0].d;

	/* partition key index is that along which descriptors have most variance */
	for( j = 0; j < d; j++ )
	{
		mean = var = 0;
		for( i = 0; i < n; i++ )
			mean += features[i].descr[j];
		mean /= n;
		for( i = 0; i < n; i++ )
		{
			x = features[i].descr[j] - mean;
			var += x * x;
		}
		var /= n;

		if( var > var_max )
		{
			ki = j;
			var_max = var;
		}
	}

	/* partition key value is median of descriptor values at ki */
	tmp = (double *)calloc( n, sizeof( double ) );
	for( i = 0; i < n; i++ )
		tmp[i] = features[i].descr[ki];
	kv = median_select( tmp, n );
	free( tmp );

	kd_node->ki = ki;
	kd_node->kv = kv;
}



/*
Finds the median value of an array.  The array's elements are re-ordered
by this function.

@param array an array; the order of its elelemts is reordered
@param n number of elements in array

@return Returns the median value of array.
*/
double HessKnnMatcher::median_select( double* array, int n )
{
	return rank_select( array, n, (n - 1) / 2 );
}



/*
Finds the element of a specified rank in an array using the linear time
median-of-medians algorithm by Blum, Floyd, Pratt, Rivest, and Tarjan.
The elements of the array are re-ordered by this function.

@param array an array; the order of its elelemts is reordered
@param n number of elements in array
@param r the zero-based rank of the element to be selected

@return Returns the element from array with zero-based rank r.
*/
double HessKnnMatcher::rank_select( double* array, int n, int r )
{
	double* tmp, med;
	int gr_5, gr_tot, rem_elts, i, j;

	/* base case */
	if( n == 1 )
		return array[0];

	/* divide array into groups of 5 and sort them */
	gr_5 = n / 5;
	gr_tot = cvCeil( n / 5.0 );
	rem_elts = n % 5;
	tmp = array;
	for( i = 0; i < gr_5; i++ )
	{
		insertion_sort( tmp, 5 );
		tmp += 5;
	}
	insertion_sort( tmp, rem_elts );

	/* recursively find the median of the medians of the groups of 5 */
	tmp = (double *)calloc( gr_tot, sizeof( double ) );
	for( i = 0, j = 2; i < gr_5; i++, j += 5 )
		tmp[i] = array[j];
	if( rem_elts )
		tmp[i++] = array[n - 1 - rem_elts/2];
	med = rank_select( tmp, i, ( i - 1 ) / 2 );
	free( tmp );

	/* partition around median of medians and recursively select if necessary */
	j = partition_array( array, n, med );
	if( r == j )
		return med;
	else if( r < j )
		return rank_select( array, j, r );
	else
	{
		array += j+1;
		return rank_select( array, ( n - j - 1 ), ( r - j - 1 ) );
	}
}



/*
Sorts an array in place into increasing order using insertion sort.

@param array an array
@param n number of elements
*/
void HessKnnMatcher::insertion_sort( double* array, int n )
{
	double k;
	int i, j;

	for( i = 1; i < n; i++ )
	{
		k = array[i];
		j = i-1;
		while( j >= 0  &&  array[j] > k )
		{
			array[j+1] = array[j];
			j -= 1;
		}
		array[j+1] = k;
	}
}



/*
Partitions an array around a specified value.

@param array an array
@param n number of elements
@param pivot value around which to partition

@return Returns index of the pivot after partitioning
*/
int HessKnnMatcher::partition_array( double* array, int n, double pivot )
{
	double tmp;
	int p, i, j;

	i = -1;
	for( j = 0; j < n; j++ )
		if( array[j] <= pivot )
		{
			tmp = array[++i];
			array[i] = array[j];
			array[j] = tmp;
			if( array[i] == pivot )
				p = i;
		}
	array[p] = array[i];
	array[i] = pivot;

	return i;
}



/*
Partitions the features at a specified kd tree node to create its two
children.

@param kd_node a kd tree node whose partition key is set
*/
void HessKnnMatcher::partition_features( struct kd_node* kd_node )
{
	struct MatchFeature* features, tmp;
	double kv;
	int n, ki, p, i, j = -1;

	features = kd_node->features;
	n = kd_node->n;
	ki = kd_node->ki;
	kv = kd_node->kv;
	for( i = 0; i < n; i++ )
		if( features[i].descr[ki] <= kv )
		{
			tmp = features[++j];
			features[j] = features[i];
			features[i] = tmp;
			if( features[j].descr[ki] == kv )
				p = j;
		}
	tmp = features[p];
	features[p] = features[j];
	features[j] = tmp;

	/* if all records fall on same side of partition, make node a leaf */
	if( j == n - 1 )
	{
		kd_node->leaf = 1;
		return;
	}

	kd_node->kd_left = kd_node_init( features, j + 1 );
	kd_node->kd_right = kd_node_init( features + ( j + 1 ), ( n - j - 1 ) );
}



/*
Explores a kd tree from a given node to a leaf.  Branching decisions are
made at each node based on the descriptor of a given feature.  Each node
examined but not explored is put into a priority queue to be explored
later, keyed based on the distance from its partition key value to the
given feature's desctiptor.

@param kd_node root of the subtree to be explored
@param feat feature upon which branching decisions are based
@param min_pq a minimizing priority queue into which tree nodes are placed
	as described above

@return Returns a pointer to the leaf node at which exploration ends or
	NULL on error.
*/
struct HessKnnMatcher::kd_node* HessKnnMatcher::explore_to_leaf( struct kd_node* kd_node, struct MatchFeature* feat,
										struct HessMinPQ::min_pq* min_pq )
{
	struct kd_node* unexpl, * expl = kd_node;
	double kv;
	int ki;

	while( expl  &&  ! expl->leaf )
	{
		ki = expl->ki;
		kv = expl->kv;

		if( ki >= feat->d )
		{
			fprintf( stderr, "Warning: comparing imcompatible descriptors, %s" \
					" line %d\n", __FILE__, __LINE__ );
			return NULL;
		}
		if( feat->descr[ki] <= kv )
		{
			unexpl = expl->kd_right;
			expl = expl->kd_left;
		}
		else
		{
			unexpl = expl->kd_left;
			expl = expl->kd_right;
		}

		if( hessMinPQObj.minpq_insert( min_pq, unexpl, (int)(ABS( kv - feat->descr[ki] )) ) )
		{
			fprintf( stderr, "Warning: unable to insert into PQ, %s, line %d\n",
					__FILE__, __LINE__ );
			return NULL;
		}
	}

	return expl;
}



/*
Inserts a feature into the nearest-neighbor array so that the array remains
in order of increasing descriptor distance from the search feature.

@param feat feature to be inderted into the array; it's feature_data field
	should be a pointer to a bbf_data with d equal to the squared descriptor
	distance between feat and the search feature
@param nbrs array of nearest neighbors neighbors
@param n number of elements already in nbrs and
@param k maximum number of elements in nbrs

@return If feat was successfully inserted into nbrs, returns 1; otherwise
	returns 0.
*/
int HessKnnMatcher::insert_into_nbr_array( struct MatchFeature* feat, struct MatchFeature** nbrs,
								  int n, int k )
{
	struct bbf_data* fdata, * ndata;
	double dn, df;
	int i, ret = 0;

	if( n == 0 )
	{
		nbrs[0] = feat;
		return 1;
	}

	/* check at end of array */
	fdata = (struct bbf_data*)feat->feature_data;
	df = fdata->d;
	ndata = (struct bbf_data*)nbrs[n-1]->feature_data;
	dn = ndata->d;
	if( df >= dn )
	{
		if( n == k )
		{
			feat->feature_data = fdata->old_data;
			free( fdata );
			return 0;
		}
		nbrs[n] = feat;
		return 1;
	}

	/* find the right place in the array */
	if( n < k )
	{
		nbrs[n] = nbrs[n-1];
		ret = 1;
	}
	else
	{
		nbrs[n-1]->feature_data = ndata->old_data;
		free( ndata );
	}
	i = n-2;
	while( i >= 0 )
	{
		ndata = (struct bbf_data*)nbrs[i]->feature_data;
		dn = ndata->d;
		if( dn <= df )
			break;
		nbrs[i+1] = nbrs[i];
		i--;
	}
	i++;
	nbrs[i] = feat;

	return ret;
}



/*
Determines whether a given point lies within a specified rectangular region

@param pt point
@param rect rectangular region

@return Returns 1 if pt is inside rect or 0 otherwise
*/
int HessKnnMatcher::within_rect( CvPoint2D64f pt, CvRect rect )
{
	if( pt.x < rect.x  ||  pt.y < rect.y )
		return 0;
	if( pt.x > rect.x + rect.width  ||  pt.y > rect.y + rect.height )
		return 0;
	return 1;
}



/*
Calculates the squared Euclidian distance between two feature descriptors.

@param f1 first feature
@param f2 second feature

@return Returns the squared Euclidian distance between the descriptors of
f1 and f2.
*/
double HessKnnMatcher::descr_dist_sq( struct MatchFeature* f1, struct MatchFeature* f2 )
{
	double diff, dsq = 0;
	double* descr1, * descr2;
	int i, d;

	d = f1->d;
	if( f2->d != d )
		return DBL_MAX;
	descr1 = f1->descr;
	descr2 = f2->descr;

	for( i = 0; i < d; i++ )
	{
		diff = descr1[i] - descr2[i];
		dsq += diff*diff;
	}
	return dsq;
}



















/*
Creates a new minimizing priority queue.
*/
struct HessKnnMatcher::HessMinPQ::min_pq * HessKnnMatcher::HessMinPQ::minpq_init()
{
	struct min_pq* min_pq;

	min_pq = (struct min_pq*)malloc( sizeof( struct min_pq ) );
	min_pq->pq_array = (struct pq_node*)calloc( MINPQ_INIT_NALLOCD, sizeof( struct pq_node ) );
	min_pq->nallocd = MINPQ_INIT_NALLOCD;
	min_pq->n = 0;

	return min_pq;
}



/**
Inserts an element into a minimizing priority queue.

@param min_pq a minimizing priority queue
@param data the data to be inserted
@param key the key to be associated with \a data

@return Returns 0 on success or 1 on failure.
*/
int HessKnnMatcher::HessMinPQ::minpq_insert( struct min_pq* min_pq, void* data, int key )
{
	int n = min_pq->n;

	/* double array allocation if necessary */
	if( min_pq->nallocd == n )
	{
		min_pq->nallocd = array_double( (void **)&min_pq->pq_array, min_pq->nallocd,
										sizeof( struct pq_node ) );
		if( ! min_pq->nallocd )
		{
			fprintf( stderr, "Warning: unable to allocate memory, %s, line %d\n",
					__FILE__, __LINE__ );
			return 1;
		}
	}

	min_pq->pq_array[n].data = data;
	min_pq->pq_array[n].key = INT_MAX;
	decrease_pq_node_key( min_pq->pq_array, min_pq->n, key );
	min_pq->n++;

	return 0;
}



/*
Returns the element of a minimizing priority queue with the smallest key
without removing it from the queue.

@param min_pq a minimizing priority queue

@return Returns the element of \a min_pq with the smallest key or NULL
if \a min_pq is empty
*/
void* HessKnnMatcher::HessMinPQ::minpq_get_min( struct min_pq* min_pq )
{
	if( min_pq->n < 1 )
	{
		fprintf( stderr, "Warning: PQ empty, %s line %d\n", __FILE__, __LINE__ );
		return NULL;
	}
	return min_pq->pq_array[0].data;
}



/*
Removes and returns the element of a minimizing priority queue with the
smallest key.

@param min_pq a minimizing priority queue

@return Returns the element of \a min_pq with the smallest key of NULL
if \a min_pq is empty
*/
void* HessKnnMatcher::HessMinPQ::minpq_extract_min( struct min_pq* min_pq )
{
	void* data;

	if( min_pq->n < 1 )
	{
		fprintf( stderr, "Warning: PQ empty, %s line %d\n", __FILE__, __LINE__ );
		return NULL;
	}
	data = min_pq->pq_array[0].data;
	min_pq->n--;
	min_pq->pq_array[0] = min_pq->pq_array[min_pq->n];
	restore_minpq_order( min_pq->pq_array, 0, min_pq->n );

	return data;
}


/*
De-allocates the memory held by a minimizing priorioty queue

@param min_pq pointer to a minimizing priority queue
*/
void HessKnnMatcher::HessMinPQ::minpq_release( struct min_pq** min_pq )
{
	if( ! min_pq )
	{
		fprintf( stderr, "Warning: NULL pointer error, %s line %d\n", __FILE__,
				__LINE__ );
		return;
	}
	if( *min_pq  &&  (*min_pq)->pq_array )
	{
		free( (*min_pq)->pq_array );
		free( *min_pq );
		*min_pq = NULL;
	}
}


/************************ Functions prototyped here **************************/

/*
Decrease a minimizing pq element's key, rearranging the pq if necessary

@param pq_array minimizing priority queue array
@param i index of the element whose key is to be decreased
@param key new value of element <EM>i</EM>'s key; if greater than current
	key, no action is taken
*/
void HessKnnMatcher::HessMinPQ::decrease_pq_node_key( struct pq_node* pq_array, int i, int key )
{
	struct pq_node tmp;

	if( key > pq_array[i].key )
		return;

	pq_array[i].key = key;
	while( i > 0  &&  pq_array[i].key < pq_array[parent(i)].key )
	{
		tmp = pq_array[parent(i)];
		pq_array[parent(i)] = pq_array[i];
		pq_array[i] = tmp;
		i = parent(i);
	}
}



/*
Recursively restores correct priority queue order to a minimizing pq array

@param pq_array a minimizing priority queue array
@param i index at which to start reordering
@param n number of elements in \a pq_array
*/
void HessKnnMatcher::HessMinPQ::restore_minpq_order( struct pq_node* pq_array, int i, int n )
{
	struct pq_node tmp;
	int l, r, min = i;

	l = left( i );
	r = right( i );
	if( l < n )
		if( pq_array[l].key < pq_array[i].key )
			min = l;
	if( r < n )
		if( pq_array[r].key < pq_array[min].key )
			min = r;

	if( min != i )
	{
		tmp = pq_array[min];
		pq_array[min] = pq_array[i];
		pq_array[i] = tmp;
		restore_minpq_order( pq_array, min, n );
	}
}



/*
Doubles the size of an array with error checking

@param array pointer to an array whose size is to be doubled
@param n number of elements allocated for \a array
@param size size in bytes of elements in \a array

@return Returns the new number of elements allocated for \a array.  If no
	memory is available, returns 0 and frees array.
*/
int HessKnnMatcher::HessMinPQ::array_double( void** array, int n, int size )
{
	void* tmp;

	tmp = realloc( *array, 2 * n * size );
	if( ! tmp )
	{
		fprintf( stderr, "Warning: unable to allocate memory in array_double(),"
				" %s line %d\n", __FILE__, __LINE__ );
		if( *array )
			free( *array );
		*array = NULL;
		return 0;
	}
	*array = tmp;
	return n*2;
}



