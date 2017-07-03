
#include "SPQREvaluator.h"
#include "SPQRClassifier.h"


#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"



class SPQRClassifierInvoker : public cv::ParallelLoopBody
{

public:
	SPQRClassifierInvoker( SPQRClassifier& _cc, cv::Size _sz1,
			int _stripSize, int _yStep, double _factor,
			cv::vector<cv::Rect>& _vec, cv::vector<int>& _levels,
			cv::vector<double>& _weights, bool outputLevels,
			const cv::Mat& _mask, cv::Mutex* _mtx)
{
		classifier = &_cc;
		processingRectSize = _sz1;
		stripSize = _stripSize;
		yStep = _yStep;
		scalingFactor = _factor;
		rectangles = &_vec;
		rejectLevels = outputLevels ? &_levels : 0;
		levelWeights = outputLevels ? &_weights : 0;
		mask = _mask;
		mtx = _mtx;


}

	void operator()(const cv::Range& range) const
	{
		cv::Ptr<cv::FeatureEvaluator> evaluator = classifier->featureEvaluator->clone();

		cv::Size winSize(cvRound(classifier->data.origWinSize.width * scalingFactor), cvRound(classifier->data.origWinSize.height * scalingFactor));

		int y1 = range.start * stripSize;
		int y2 = cv::min(range.end * stripSize, processingRectSize.height);
		for( int y = y1; y < y2; y += yStep )
		{
			for( int x = 0; x < processingRectSize.width; x += yStep )
			{
				if ( (!mask.empty()) && (mask.at<uchar>(cv::Point(x,y))==0)) {
					continue;
				}

				double gypWeight;
				int result = classifier->runAt(evaluator, cv::Point(x, y), gypWeight);


				if( rejectLevels )
				{
					if( result == 1 )
						result =  -(int)classifier->data.stages.size();
					if( classifier->data.stages.size() + result < 4 )
					{
						mtx->lock();

						rectangles->push_back(cv::Rect(cvRound(x*scalingFactor), cvRound(y*scalingFactor), winSize.width, winSize.height));
						rejectLevels->push_back(-result);
						levelWeights->push_back(gypWeight);
						mtx->unlock();
					}
				}
				else if( result > 0 )
				{
					mtx->lock();


					rectangles->push_back(cv::Rect(cvRound(x*scalingFactor), cvRound(y*scalingFactor),
							winSize.width, winSize.height));

					mtx->unlock();
				}
				if( result == 0 )
					x += yStep;
			}
		}
	};
	SPQRClassifier* classifier;
	cv::vector<cv::Rect>* rectangles;
	cv::Size processingRectSize;
	int stripSize, yStep;
	double scalingFactor;
	cv::vector<int> *rejectLevels;
	cv::vector<double> *levelWeights;
	cv::Mat mask;
	cv::Mutex* mtx;
};
