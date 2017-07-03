#pragma once

#include <cstdio>
#include "SPQREvaluator.h"

#include <string>

#include <iostream>

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"





//used for cascade detection algorithm for ROC-curve calculating


//---
//---------------------------------------- Classifier Cascade --------------------------------------------


class SPQRClassifier
{

public:


	SPQRClassifier()
	{
	}

	SPQRClassifier(const cv::string& filename)
	{
		load(filename);
	};


	cv::Ptr<cv::CascadeClassifier::MaskGenerator> maskGenerator;

	bool empty();


	bool load(const cv::string& filename);

	int runAt( cv::Ptr<cv::FeatureEvaluator>& evaluator, cv::Point pt, double& weight );

	bool setImage( cv::Ptr<cv::FeatureEvaluator>& evaluator, const cv::Mat& image );

	void setMaskGenerator(cv::Ptr<cv::CascadeClassifier::MaskGenerator> _maskGenerator);

	cv::Ptr<cv::CascadeClassifier::MaskGenerator> getMaskGenerator();

	void setFaceDetectionMaskGenerator();


	struct getRect { cv::Rect operator ()(const CvAvgComp& e) const { return e.rect; } };


	bool detectSingleScale( const cv::Mat& image, int stripCount, cv::Size processingRectSize,
			int stripSize, int yStep, double factor, cv::vector<cv::Rect>& candidates,
			cv::vector<int>& levels, cv::vector<double>& weights, bool outputRejectLevels,
		 	const cv::Mat& currentMask);

	bool isOldFormatCascade() const;

	int getFeatureType() const;

	cv::Size getOriginalWindowSize() const;

	bool setImage(const cv::Mat& image);

	void detectMultiScale( const cv::Mat& image, cv::vector<cv::Rect>& objects,
			cv::vector<int>& rejectLevels,
			cv::vector<double>& levelWeights,
			double scaleFactor, int minNeighbors,
			int flags, cv::Size minObjectSize, cv::Size maxObjectSize,
			bool outputRejectLevels,
		 	CameraInfo theCameraInfo,
			CameraMatrix theCameraMatrix,
			FieldDimensions theFieldDimensions,
		 	int offset_y);

	void detectMultiScale( const cv::Mat& image, cv::vector<cv::Rect>& objects,
			double scaleFactor, int minNeighbors,
			int flags, cv::Size minObjectSize, cv::Size maxObjectSize,
		 	CameraInfo theCameraInfo,
			CameraMatrix theCameraMatrix,
			FieldDimensions theFieldDimensions,
			int offset_y);

	bool read(const cv::FileNode &root);

	class CV_EXPORTS SimilarRects
	{
	public:
	    SimilarRects(double _eps) : eps(_eps) {}
	    inline bool operator()(const cv::Rect& r1, const cv::Rect& r2) const
	    {
	        double delta = eps*(std::min(r1.width, r2.width) + std::min(r1.height, r2.height))*0.5;
	        return std::abs(r1.x - r2.x) <= delta &&
	            std::abs(r1.y - r2.y) <= delta &&
	            std::abs(r1.x + r1.width - r2.x - r2.width) <= delta &&
	            std::abs(r1.y + r1.height - r2.y - r2.height) <= delta;
	    }
	    double eps;
	};

	class Data
	{
	public:
		struct CV_EXPORTS DTreeNode
		{
			int featureIdx;
			float threshold; // for ordered features only
			int left;
			int right;
		};

		struct CV_EXPORTS DTree
		{
			int nodeCount;
		};

		struct CV_EXPORTS Stage
		{
			int first;
			int ntrees;
			float threshold;
		};

		bool read(const cv::FileNode &node);

		bool isStumpBased;

		int stageType;
		int featureType;
		int ncategories;
		cv::Size origWinSize;

		cv::vector<Stage> stages;
		cv::vector<DTree> classifiers;
		cv::vector<DTreeNode> nodes;
		cv::vector<float> leaves;
		cv::vector<int> subsets;



	};
	Data data;
    //cv::Mat imagecopy;
    //int bot=180;
    //int top=130;
	cv::Ptr<cv::FeatureEvaluator> featureEvaluator;
	cv::Ptr<CvHaarClassifierCascade> oldCascade;



};



