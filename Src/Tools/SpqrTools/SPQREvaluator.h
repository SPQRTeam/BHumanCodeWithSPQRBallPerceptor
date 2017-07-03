#pragma once

#define CC_CASCADE_PARAMS "cascadeParams"
#define CC_STAGE_TYPE     "stageType"
#define CC_FEATURE_TYPE   "featureType"
#define CC_HEIGHT         "height"
#define CC_WIDTH          "width"

#define CC_STAGE_NUM    "stageNum"
#define CC_STAGES       "stages"
#define CC_STAGE_PARAMS "stageParams"

#define CC_BOOST            "BOOST"
#define CC_MAX_DEPTH        "maxDepth"
#define CC_WEAK_COUNT       "maxWeakCount"
#define CC_STAGE_THRESHOLD  "stageThreshold"
#define CC_WEAK_CLASSIFIERS "weakClassifiers"
#define CC_INTERNAL_NODES   "internalNodes"
#define CC_LEAF_VALUES      "leafValues"

#define CC_FEATURES       "features"
#define CC_FEATURE_PARAMS "featureParams"
#define CC_MAX_CAT_COUNT  "maxCatCount"

//#define CC_HAAR   "HAAR"
#define CC_RECTS  "rects"
#define CC_TILTED "tilted"

#define CC_LBP  "LBP"
#define CC_RECT "rect"

//#define CC_HOG  "HOG"

#define CV_SUM_PTRS( p0, p1, p2, p3, sum, rect, step )                    \
		/* (x, y) */                                                          \
		(p0) = sum + (rect).x + (step) * (rect).y,                            \
		/* (x + w, y) */                                                      \
		(p1) = sum + (rect).x + (rect).width + (step) * (rect).y,             \
		/* (x + w, y) */                                                      \
		(p2) = sum + (rect).x + (step) * ((rect).y + (rect).height),          \
		/* (x + w, y + h) */                                                  \
		(p3) = sum + (rect).x + (rect).width + (step) * ((rect).y + (rect).height)

#define CV_TILTED_PTRS( p0, p1, p2, p3, tilted, rect, step )                        \
		/* (x, y) */                                                                    \
		(p0) = tilted + (rect).x + (step) * (rect).y,                                   \
		/* (x - h, y + h) */                                                            \
		(p1) = tilted + (rect).x - (rect).height + (step) * ((rect).y + (rect).height), \
		/* (x + w, y + w) */                                                            \
		(p2) = tilted + (rect).x + (rect).width + (step) * ((rect).y + (rect).width),   \
		/* (x + w - h, y + w + h) */                                                    \
		(p3) = tilted + (rect).x + (rect).width - (rect).height                         \
		+ (step) * ((rect).y + (rect).width + (rect).height)

#define CALC_SUM_(p0, p1, p2, p3, offset) \
		((p0)[offset] - (p1)[offset] - (p2)[offset] + (p3)[offset])

#define CALC_SUM(rect,offset) CALC_SUM_((rect)[0], (rect)[1], (rect)[2], (rect)[3], offset)



#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


class SPQREvaluator : public cv::FeatureEvaluator
{

public:
	struct Feature
	{
		Feature();
		Feature( int x, int y, int _block_w, int _block_h  ) :
			rect(x, y, _block_w, _block_h) {}

		int calc( int offset ) const;
		void updatePtrs( const cv::Mat& sum );
		bool read(const cv::FileNode& node );

		cv::Rect rect; // weight and height for block
		const int* p[16]; // fast
	};


	SPQREvaluator();
	virtual ~SPQREvaluator();

	virtual bool read( const cv::FileNode& node );
	virtual cv::Ptr<cv::FeatureEvaluator> clone() const;
	virtual int getFeatureType() const { return cv::FeatureEvaluator::LBP; }

	virtual bool setImage(const cv::Mat& image, cv::Size _origWinSize);
	virtual bool setWindow(cv::Point pt);

	int operator()(int featureIdx) const
	{ return featuresPtr[featureIdx].calc(offset); }
	virtual int calcCat(int featureIdx) const
	{ return (*this)(featureIdx); }

protected:
	cv::Size origWinSize;
	cv::Ptr<cv::vector<Feature> > features;
	Feature* featuresPtr; // optimization
	cv::Mat sum0, sum;
	cv::Rect normrect;
	int offset;
};






