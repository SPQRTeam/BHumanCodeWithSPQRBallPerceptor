
#include <cstdio>

#include "SPQREvaluator.cpp"
//#include "SPQRClassifier.h"


#include <string>

#include <iostream>

enum stage { BOOST = 0 };
using namespace cv;



void SPQRgroupRectangles(cv::vector<cv::Rect>& rectList, int groupThreshold,
		double eps, cv::vector<int>* weights, cv::vector<double>* levelWeights)
{
	if( groupThreshold <= 0 || rectList.empty() )
	{
		if( weights )
		{
			size_t i, sz = rectList.size();
			weights->resize(sz);
			for( i = 0; i < sz; i++ )
				(*weights)[i] = 1;
		}
		return;
	}

	cv::vector<int> labels;
	int nclasses = partition(rectList, labels, SPQRClassifier::SimilarRects(eps));

	cv::vector<cv::Rect> rrects(nclasses);
	cv::vector<int> rweights(nclasses, 0);
	cv::vector<int> rejectLevels(nclasses, 0);
	cv::vector<double> rejectWeights(nclasses, DBL_MIN);
	int i, j, nlabels = (int)labels.size();
	for( i = 0; i < nlabels; i++ )
	{
		int cls = labels[i];
		rrects[cls].x += rectList[i].x;
		rrects[cls].y += rectList[i].y;
		rrects[cls].width += rectList[i].width;
		rrects[cls].height += rectList[i].height;
		rweights[cls]++;
	}
	if ( levelWeights && weights && !weights->empty() && !levelWeights->empty() )
	{
		for( i = 0; i < nlabels; i++ )
		{
			int cls = labels[i];
			if( (*weights)[i] > rejectLevels[cls] )
			{
				rejectLevels[cls] = (*weights)[i];
				rejectWeights[cls] = (*levelWeights)[i];
			}
			else if( ( (*weights)[i] == rejectLevels[cls] ) && ( (*levelWeights)[i] > rejectWeights[cls] ) )
				rejectWeights[cls] = (*levelWeights)[i];
		}
	}

	for( i = 0; i < nclasses; i++ )
	{
		cv::Rect r = rrects[i];
		float s = 1.f/rweights[i];
		rrects[i] = cv::Rect(cv::saturate_cast<int>(r.x*s),
				cv::saturate_cast<int>(r.y*s),
				cv::saturate_cast<int>(r.width*s),
				cv::saturate_cast<int>(r.height*s));
	}

	rectList.clear();
	if( weights )
		weights->clear();
	if( levelWeights )
		levelWeights->clear();

	for( i = 0; i < nclasses; i++ )
	{
		cv::Rect r1 = rrects[i];

		int n1 = levelWeights ? rejectLevels[i] : rweights[i];
		double w1 = rejectWeights[i];
		if( n1 <= groupThreshold )
			continue;
		// filter out small face rectangles inside large rectangles
		for( j = 0; j < nclasses; j++ )
		{
			int n2 = rweights[j];

			if( j == i || n2 <= groupThreshold )
				continue;
			cv::Rect r2 = rrects[j];

			int dx = cv::saturate_cast<int>( r2.width * eps );
			int dy = cv::saturate_cast<int>( r2.height * eps );

			if( i != j &&
					r1.x >= r2.x - dx &&
					r1.y >= r2.y - dy &&
					r1.x + r1.width <= r2.x + r2.width + dx &&
					r1.y + r1.height <= r2.y + r2.height + dy &&
					(n2 > std::max(3, n1) || n1 < 3) )
				break;
		}

		if( j == nclasses )
		{
			rectList.push_back(r1);
			if( weights )
				weights->push_back(n1);
			if( levelWeights )
				levelWeights->push_back(w1);
		}
	}

}


void SPQRgroupRectangles(cv::vector<cv::Rect>& rectList, int groupThreshold, double eps)
{
	SPQRgroupRectangles(rectList, groupThreshold, eps, 0, 0);
}

void SPQRgroupRectangles(cv::vector<cv::Rect>& rectList, cv::vector<int>& weights, int groupThreshold, double eps)
{
	SPQRgroupRectangles(rectList, groupThreshold, eps, &weights, 0);
}
//used for cascade detection algorithm for ROC-curve calculating
void SPQRgroupRectangles(cv::vector<cv::Rect>& rectList, cv::vector<int>& rejectLevels, cv::vector<double>& levelWeights, int groupThreshold, double eps)
{
	SPQRgroupRectangles(rectList, groupThreshold, eps, &rejectLevels, &levelWeights);
}


bool SPQRClassifier::empty()
{
	return oldCascade.empty() && data.stages.empty();
}


bool SPQRClassifier::load(const cv::string& filename)
{
	oldCascade.release();
	data = Data();
	featureEvaluator.release();

	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if( !fs.isOpened() )
		return false;

	if( read(fs.getFirstTopLevelNode()) )
		return true;

	fs.release();

	oldCascade = cv::Ptr<CvHaarClassifierCascade>((CvHaarClassifierCascade*)cvLoad(filename.c_str(), 0, 0, 0));
	return !oldCascade.empty();
}

bool SPQRClassifier::read(const cv::FileNode& root)
{
	if( !data.read(root) )
		return false;

	// load features
	featureEvaluator = cv::FeatureEvaluator::create(data.featureType);
	cv::FileNode fn = root[CC_FEATURES];
	if( fn.empty() )
		return false;

	return featureEvaluator->read(fn);
}


bool SPQRClassifier::Data::read(const cv::FileNode &root)
{
	static const float THRESHOLD_EPS = 1e-5f;

	// load stage params
	cv::string stageTypeStr = (cv::string)root[CC_STAGE_TYPE];
	if( stageTypeStr == CC_BOOST )
		stageType = BOOST;
	else
		return false;

	cv::string featureTypeStr = (cv::string)root[CC_FEATURE_TYPE];
	if( featureTypeStr == CC_LBP )
		featureType = cv::FeatureEvaluator::LBP;
	else
		return false;

	origWinSize.width = (int)root[CC_WIDTH];
	origWinSize.height = (int)root[CC_HEIGHT];
	CV_Assert( origWinSize.height > 0 && origWinSize.width > 0 );

	isStumpBased = (int)(root[CC_STAGE_PARAMS][CC_MAX_DEPTH]) == 1 ? true : false;

	// load feature params
	cv::FileNode fn = root[CC_FEATURE_PARAMS];
	if( fn.empty() )
		return false;

	ncategories = fn[CC_MAX_CAT_COUNT];
	int subsetSize = (ncategories + 31)/32,
			nodeStep = 3 + ( ncategories>0 ? subsetSize : 1 );

	// load stages
	fn = root[CC_STAGES];
	if( fn.empty() )
		return false;

	stages.reserve(fn.size());
	classifiers.clear();
	nodes.clear();

	cv::FileNodeIterator it = fn.begin(), it_end = fn.end();

	for( int si = 0; it != it_end; si++, ++it )
	{
		cv::FileNode fns = *it;
		Stage stage;
		stage.threshold = (float)fns[CC_STAGE_THRESHOLD] - THRESHOLD_EPS;
		fns = fns[CC_WEAK_CLASSIFIERS];
		if(fns.empty())
			return false;
		stage.ntrees = (int)fns.size();
		stage.first = (int)classifiers.size();
		stages.push_back(stage);
		classifiers.reserve(stages[si].first + stages[si].ntrees);

		cv::FileNodeIterator it1 = fns.begin(), it1_end = fns.end();
		for( ; it1 != it1_end; ++it1 ) // weak trees
		{
			cv::FileNode fnw = *it1;
			cv::FileNode internalNodes = fnw[CC_INTERNAL_NODES];
			cv::FileNode leafValues = fnw[CC_LEAF_VALUES];
			if( internalNodes.empty() || leafValues.empty() )
				return false;

			DTree tree;
			tree.nodeCount = (int)internalNodes.size()/nodeStep;
			classifiers.push_back(tree);

			nodes.reserve(nodes.size() + tree.nodeCount);
			leaves.reserve(leaves.size() + leafValues.size());
			if( subsetSize > 0 )
				subsets.reserve(subsets.size() + tree.nodeCount*subsetSize);

			cv::FileNodeIterator internalNodesIter = internalNodes.begin(), internalNodesEnd = internalNodes.end();

			for( ; internalNodesIter != internalNodesEnd; ) // nodes
			{
				DTreeNode node;
				node.left = (int)*internalNodesIter; ++internalNodesIter;
				node.right = (int)*internalNodesIter; ++internalNodesIter;
				node.featureIdx = (int)*internalNodesIter; ++internalNodesIter;
				if( subsetSize > 0 )
				{
					for( int j = 0; j < subsetSize; j++, ++internalNodesIter )
						subsets.push_back((int)*internalNodesIter);
					node.threshold = 0.f;
				}
				else
				{
					node.threshold = (float)*internalNodesIter; ++internalNodesIter;
				}
				nodes.push_back(node);
			}

			internalNodesIter = leafValues.begin(), internalNodesEnd = leafValues.end();

			for( ; internalNodesIter != internalNodesEnd; ++internalNodesIter ) // leaves
				leaves.push_back((float)*internalNodesIter);
		}
	}

	return true;
}


int SPQRClassifier::runAt( cv::Ptr<cv::FeatureEvaluator>& evaluator, cv::Point pt, double& weight )
{
	CV_Assert( oldCascade.empty() );

	assert(
			data.featureType == cv::FeatureEvaluator::LBP );

	if( !evaluator->setWindow(pt) )
		return -1;
	if( data.isStumpBased )
	{
		if( data.featureType == cv::FeatureEvaluator::LBP )
			return predictCategoricalStump<SPQREvaluator>( *this, evaluator, weight );
		else
			return -2;
	}
	else
	{
		if( data.featureType == cv::FeatureEvaluator::LBP )
			return predictCategorical<SPQREvaluator>( *this, evaluator, weight );
		else
			return -2;
	}
}


bool SPQRClassifier::setImage( cv::Ptr<cv::FeatureEvaluator>& evaluator, const cv::Mat& image )
{
	return empty() ? false : evaluator->setImage(image, data.origWinSize);
}

void SPQRClassifier::setMaskGenerator(cv::Ptr<cv::CascadeClassifier::MaskGenerator> _maskGenerator)
{
	maskGenerator=_maskGenerator;
}

cv::Ptr<cv::CascadeClassifier::MaskGenerator> SPQRClassifier::getMaskGenerator()
{
	return maskGenerator;
}

void SPQRClassifier::setFaceDetectionMaskGenerator()
{
#ifdef HAVE_TEGRA_OPTIMIZATION
	setMaskGenerator(tegra::getCascadeClassifierMaskGenerator(*this));
#else
	setMaskGenerator(cv::Ptr<cv::CascadeClassifier::MaskGenerator>());
#endif
}


bool SPQRClassifier::detectSingleScale( const cv::Mat& image, int stripCount, cv::Size processingRectSize,
		int stripSize, int yStep, double factor, cv::vector<cv::Rect>& candidates,
		cv::vector<int>& levels, cv::vector<double>& weights, bool outputRejectLevels, const cv::Mat& currentMask)
{
	if( !featureEvaluator->setImage( image, data.origWinSize ) )
		return false;


#if defined (LOG_CASCADE_STATISTIC)
	logger.setImage(image);
#endif

	cv::vector<cv::Rect> candidatesVector;
	cv::vector<int> rejectLevels;
	cv::vector<double> levelWeights;
	cv::Mutex mtx;
	if( outputRejectLevels )
	{
		parallel_for_(cv::Range(0, stripCount), SPQRClassifierInvoker( *this, processingRectSize, stripSize, yStep, factor,
				candidatesVector, rejectLevels, levelWeights, true, currentMask, &mtx));
		levels.insert( levels.end(), rejectLevels.begin(), rejectLevels.end() );
		weights.insert( weights.end(), levelWeights.begin(), levelWeights.end() );
	}
	else
	{
		parallel_for_(cv::Range(0, stripCount), SPQRClassifierInvoker( *this, processingRectSize, stripSize, yStep, factor,
				candidatesVector, rejectLevels, levelWeights, false, currentMask, &mtx));
	}
	candidates.insert( candidates.end(), candidatesVector.begin(), candidatesVector.end() );



#if defined (LOG_CASCADE_STATISTIC)
	logger.write();
#endif

	return true;
}

bool SPQRClassifier::isOldFormatCascade() const
{
	return !oldCascade.empty();
}
int SPQRClassifier::getFeatureType() const
{
	return featureEvaluator->getFeatureType();
}

cv::Size SPQRClassifier::getOriginalWindowSize() const
{
	return data.origWinSize;
}

bool SPQRClassifier::setImage(const cv::Mat& image)
{
	return featureEvaluator->setImage(image, data.origWinSize);
}


void SPQRClassifier::detectMultiScale( const cv::Mat& image, cv::vector<cv::Rect>& objects,
		cv::vector<int>& rejectLevels,
		cv::vector<double>& levelWeights,
		double scaleFactor, int minNeighbors,
		int flags, cv::Size minObjectSize, cv::Size maxObjectSize,
		bool outputRejectLevels,
	 	CameraInfo theCameraInfo,
		CameraMatrix theCameraMatrix,
		FieldDimensions theFieldDimensions,
		int offset_y)
{
	const double GROUP_EPS = 0.2;
	CV_Assert( scaleFactor > 1 && image.depth() == CV_8U );

	if( empty() )
		return;

	if( isOldFormatCascade() )
	{
		cv::MemStorage storage(cvCreateMemStorage(0));
		CvMat _image = image;
		CvSeq* _objects = cvHaarDetectObjectsForROC( &_image, oldCascade, storage, rejectLevels, levelWeights, scaleFactor,
				minNeighbors, flags, minObjectSize, maxObjectSize, outputRejectLevels );
		cv::vector<CvAvgComp> vecAvgComp;
		cv::Seq<CvAvgComp>(_objects).copyTo(vecAvgComp);
		objects.resize(vecAvgComp.size());
		std::transform(vecAvgComp.begin(), vecAvgComp.end(), objects.begin(), getRect());
		return;
	}

	objects.clear();

	if (!maskGenerator.empty()) {
		maskGenerator->initializeMask(image);
	}


	if( maxObjectSize.height == 0 || maxObjectSize.width == 0 )
		maxObjectSize = image.size();

	cv::Mat grayImage = image;
	if( grayImage.channels() > 1 )
	{
		cv::Mat temp;
		cvtColor(grayImage, temp, CV_BGR2GRAY);
		grayImage = temp;
	}

	cv::Mat imageBuffer(image.rows + 1, image.cols + 1, CV_8U);
	cv::vector<cv::Rect> candidates;


	for( double factor = 1; ; factor *= scaleFactor )
	{
		cv::Size originalWindowSize = getOriginalWindowSize();

		cv::Size windowSize( cvRound(originalWindowSize.width*factor), cvRound(originalWindowSize.height*factor) );
		cv::Size scaledImageSize( cvRound( grayImage.cols/factor ), cvRound( grayImage.rows/factor ) );
		cv::Size processingRectSize( scaledImageSize.width - originalWindowSize.width, scaledImageSize.height - originalWindowSize.height );

		//std::cerr << "processingRectSize " << processingRectSize << std::endl;
		//std::cerr << "windowSize " << windowSize.width << std::endl;
		if( processingRectSize.width <= 0 || processingRectSize.height <= 0 )
			break;
		if( windowSize.width > maxObjectSize.width || windowSize.height > maxObjectSize.height )
			break;
		if( windowSize.width < minObjectSize.width || windowSize.height < minObjectSize.height )
			continue;


		cv::Mat scaledImage( scaledImageSize, CV_8U, imageBuffer.data );
		resize( grayImage, scaledImage, scaledImageSize, 0, 0, CV_INTER_LINEAR );

		int yStep;
		if( getFeatureType() == cv::FeatureEvaluator::HOG )
		{
			yStep = 4;
		}
		else
		{
			yStep = factor > 2. ? 1 : 2;
		}

		int stripCount, stripSize;

		const int PTS_PER_THREAD = 1000;
		stripCount = ((processingRectSize.width/yStep)*(processingRectSize.height + yStep-1)/yStep + PTS_PER_THREAD/2)/PTS_PER_THREAD;
		stripCount = std::min(std::max(stripCount, 1), 100);
		stripSize = (((processingRectSize.height + stripCount - 1)/stripCount + yStep-1)/yStep)*yStep;

		cv::Mat currentMask;
		if (!maskGenerator.empty()) {
			//std::cerr << "----------------not empty------------" << std::endl;
			currentMask=maskGenerator->generateMask(image);
			resize( currentMask, currentMask, scaledImageSize, 0, 0, CV_INTER_LINEAR );
		}

		//std::cerr << "originalWindowSize: "<< originalWindowSize<<  ", windowSize.width: " << windowSize.width << ", factor: " << factor << ", processingRectSize.width: " << processingRectSize.width << ", processingRectSize.height: " << processingRectSize.height << ", stripCount: " << stripCount << ", stripSize: " << stripSize << std::endl;

		if( !detectSingleScale( scaledImage, stripCount, processingRectSize, stripSize, yStep, factor, candidates,
				rejectLevels, levelWeights, outputRejectLevels, currentMask ) )
			break;
	}


	objects.resize(candidates.size());
	std::copy(candidates.begin(), candidates.end(), objects.begin());

	if( outputRejectLevels )
	{
		SPQRgroupRectangles( objects, rejectLevels, levelWeights, minNeighbors, GROUP_EPS );
	}
	else
	{
		SPQRgroupRectangles( objects, minNeighbors, GROUP_EPS );
	}

}


void SPQRClassifier::detectMultiScale( const cv::Mat& image, cv::vector<cv::Rect>& objects,
		double scaleFactor, int minNeighbors,
		int flags, cv::Size minObjectSize, cv::Size maxObjectSize,
	 	CameraInfo theCameraInfo,
		CameraMatrix theCameraMatrix,
		FieldDimensions theFieldDimensions,
		int offset_y)
{
	cv::vector<int> fakeLevels;
	cv::vector<double> fakeWeights;
	SPQRClassifier::detectMultiScale( image, objects, fakeLevels, fakeWeights, scaleFactor,
			minNeighbors, flags, minObjectSize, maxObjectSize, false, theCameraInfo,
			theCameraMatrix, theFieldDimensions, offset_y);
}



