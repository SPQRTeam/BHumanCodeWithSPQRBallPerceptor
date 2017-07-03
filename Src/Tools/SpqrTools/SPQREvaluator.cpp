#include "SPQREvaluator.h"
#include "SPQRClassifierInvoker.h"



#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"



inline SPQREvaluator::Feature :: Feature()
{
	rect = cv::Rect();
	for( int i = 0; i < 16; i++ )
		p[i] = 0;
}

inline int SPQREvaluator::Feature :: calc( int _offset ) const
{
	int cval = CALC_SUM_( p[5], p[6], p[9], p[10], _offset );

	return (CALC_SUM_( p[0], p[1], p[4], p[5], _offset ) >= cval ? 128 : 0) |   // 0
			(CALC_SUM_( p[1], p[2], p[5], p[6], _offset ) >= cval ? 64 : 0) |    // 1
			(CALC_SUM_( p[2], p[3], p[6], p[7], _offset ) >= cval ? 32 : 0) |    // 2
			(CALC_SUM_( p[6], p[7], p[10], p[11], _offset ) >= cval ? 16 : 0) |  // 5
			(CALC_SUM_( p[10], p[11], p[14], p[15], _offset ) >= cval ? 8 : 0)|  // 8
			(CALC_SUM_( p[9], p[10], p[13], p[14], _offset ) >= cval ? 4 : 0)|   // 7
			(CALC_SUM_( p[8], p[9], p[12], p[13], _offset ) >= cval ? 2 : 0)|    // 6
			(CALC_SUM_( p[4], p[5], p[8], p[9], _offset ) >= cval ? 1 : 0);
}

inline void SPQREvaluator::Feature :: updatePtrs( const cv::Mat& _sum )
{
	const int* ptr = (const int*)_sum.data;
	size_t step = _sum.step/sizeof(ptr[0]);
	cv::Rect tr = rect;
	CV_SUM_PTRS( p[0], p[1], p[4], p[5], ptr, tr, step );
	tr.x += 2*rect.width;
	CV_SUM_PTRS( p[2], p[3], p[6], p[7], ptr, tr, step );
	tr.y += 2*rect.height;
	CV_SUM_PTRS( p[10], p[11], p[14], p[15], ptr, tr, step );
	tr.x -= 2*rect.width;
	CV_SUM_PTRS( p[8], p[9], p[12], p[13], ptr, tr, step );
}


//----------------------------------------------  predictor functions -------------------------------------

template<class FEval>
inline int predictOrdered( SPQRClassifier& cascade, cv::Ptr<cv::FeatureEvaluator> &_featureEvaluator, double& sum )
{

	int nstages = (int)cascade.data.stages.size();
	int nodeOfs = 0, leafOfs = 0;
	FEval& featureEvaluator = (FEval&)*_featureEvaluator;
	float* cascadeLeaves = &cascade.data.leaves[0];
	SPQRClassifier::Data::DTreeNode* cascadeNodes = &cascade.data.nodes[0];
	SPQRClassifier::Data::DTree* cascadeWeaks = &cascade.data.classifiers[0];
	SPQRClassifier::Data::Stage* cascadeStages = &cascade.data.stages[0];

	for( int si = 0; si < nstages; si++ )
	{
		SPQRClassifier::Data::Stage& stage = cascadeStages[si];
		int wi, ntrees = stage.ntrees;
		sum = 0;

		for( wi = 0; wi < ntrees; wi++ )
		{
			SPQRClassifier::Data::DTree& weak = cascadeWeaks[stage.first + wi];
			int idx = 0, root = nodeOfs;

			do
			{
				SPQRClassifier::Data::DTreeNode& node = cascadeNodes[root + idx];
				double val = featureEvaluator(node.featureIdx);
				idx = val < node.threshold ? node.left : node.right;
			}
			while( idx > 0 );
			sum += cascadeLeaves[leafOfs - idx];
			nodeOfs += weak.nodeCount;
			leafOfs += weak.nodeCount + 1;
		}
		if( sum < stage.threshold )
			return -si;
	}
	return 1;
}

template<class FEval>
inline int predictCategorical( SPQRClassifier& cascade, cv::Ptr<cv::FeatureEvaluator> &_featureEvaluator, double& sum )
{


	int nstages = (int)cascade.data.stages.size();
	int nodeOfs = 0, leafOfs = 0;
	FEval& featureEvaluator = (FEval&)*_featureEvaluator;
	size_t subsetSize = (cascade.data.ncategories + 31)/32;
	int* cascadeSubsets = &cascade.data.subsets[0];
	float* cascadeLeaves = &cascade.data.leaves[0];
	SPQRClassifier::Data::DTreeNode* cascadeNodes = &cascade.data.nodes[0];
	SPQRClassifier::Data::DTree* cascadeWeaks = &cascade.data.classifiers[0];
	SPQRClassifier::Data::Stage* cascadeStages = &cascade.data.stages[0];

	for(int si = 0; si < nstages; si++ )
	{
		SPQRClassifier::Data::Stage& stage = cascadeStages[si];
		int wi, ntrees = stage.ntrees;
		sum = 0;

		for( wi = 0; wi < ntrees; wi++ )
		{
			SPQRClassifier::Data::DTree& weak = cascadeWeaks[stage.first + wi];
			int idx = 0, root = nodeOfs;
			do
			{
				SPQRClassifier::Data::DTreeNode& node = cascadeNodes[root + idx];
				int c = featureEvaluator(node.featureIdx);
				const int* subset = &cascadeSubsets[(root + idx)*subsetSize];
				idx = (subset[c>>5] & (1 << (c & 31))) ? node.left : node.right;
			}
			while( idx > 0 );
			sum += cascadeLeaves[leafOfs - idx];
			nodeOfs += weak.nodeCount;
			leafOfs += weak.nodeCount + 1;
		}
		if( sum < stage.threshold )
			return -si;
	}
	return 1;
}

template<class FEval>
inline int predictOrderedStump( SPQRClassifier& cascade, cv::Ptr<cv::FeatureEvaluator> &_featureEvaluator, double& sum )
{
	int nodeOfs = 0, leafOfs = 0;
	FEval& featureEvaluator = (FEval&)*_featureEvaluator;
	float* cascadeLeaves = &cascade.data.leaves[0];
	SPQRClassifier::Data::DTreeNode* cascadeNodes = &cascade.data.nodes[0];
	SPQRClassifier::Data::Stage* cascadeStages = &cascade.data.stages[0];

	int nstages = (int)cascade.data.stages.size();
	for( int stageIdx = 0; stageIdx < nstages; stageIdx++ )
	{
		SPQRClassifier::Data::Stage& stage = cascadeStages[stageIdx];
		sum = 0.0;

		int ntrees = stage.ntrees;
		for( int i = 0; i < ntrees; i++, nodeOfs++, leafOfs+= 2 )
		{
			SPQRClassifier::Data::DTreeNode& node = cascadeNodes[nodeOfs];
			double value = featureEvaluator(node.featureIdx);
			sum += cascadeLeaves[ value < node.threshold ? leafOfs : leafOfs + 1 ];
		}

		if( sum < stage.threshold )
			return -stageIdx;
	}

	return 1;
}

template<class FEval>
inline int predictCategoricalStump( SPQRClassifier& cascade, cv::Ptr<cv::FeatureEvaluator> &_featureEvaluator, double& sum )
{

	int nstages = (int)cascade.data.stages.size();
	int nodeOfs = 0, leafOfs = 0;
	FEval& featureEvaluator = (FEval&)*_featureEvaluator;
	size_t subsetSize = (cascade.data.ncategories + 31)/32;
	int* cascadeSubsets = &cascade.data.subsets[0];
	float* cascadeLeaves = &cascade.data.leaves[0];
	SPQRClassifier::Data::DTreeNode* cascadeNodes = &cascade.data.nodes[0];
	SPQRClassifier::Data::Stage* cascadeStages = &cascade.data.stages[0];

#ifdef HAVE_TEGRA_OPTIMIZATION
	float tmp = 0; // float accumulator -- float operations are quicker
#endif
	for( int si = 0; si < nstages; si++ )
	{
		SPQRClassifier::Data::Stage& stage = cascadeStages[si];
		int wi, ntrees = stage.ntrees;
#ifdef HAVE_TEGRA_OPTIMIZATION
		tmp = 0;
#else
		sum = 0;
#endif

		for( wi = 0; wi < ntrees; wi++ )
		{
			SPQRClassifier::Data::DTreeNode& node = cascadeNodes[nodeOfs];
			int c = featureEvaluator(node.featureIdx);
			const int* subset = &cascadeSubsets[nodeOfs*subsetSize];
#ifdef HAVE_TEGRA_OPTIMIZATION
			tmp += cascadeLeaves[ subset[c>>5] & (1 << (c & 31)) ? leafOfs : leafOfs+1];
#else
			sum += cascadeLeaves[ subset[c>>5] & (1 << (c & 31)) ? leafOfs : leafOfs+1];
#endif
			nodeOfs++;
			leafOfs += 2;
		}
#ifdef HAVE_TEGRA_OPTIMIZATION
		if( tmp < stage.threshold ) {
			sum = (double)tmp;
			return -si;
		}
#else
	if( sum < stage.threshold )
		return -si;
#endif
	}

#ifdef HAVE_TEGRA_OPTIMIZATION
	sum = (double)tmp;
#endif

	return 1;

}


