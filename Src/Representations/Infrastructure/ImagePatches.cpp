/**
 * @author Felix Thielke
 */

#include "ImagePatches.h"
//#include "Tools/Debugging/DebugImages.h"
#include "Tools/Debugging/DebugDrawings.h"

void ImagePatch::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(offset);
  STREAM(width);
  STREAM(height);
  STREAM_REGISTER_FINISH;
}

//void ImagePatch::toImage(cv::Mat orig, cv::Mat& dest) const
//{}

void ImagePatches::draw() const
{
}
