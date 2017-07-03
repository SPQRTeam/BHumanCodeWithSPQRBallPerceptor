/**
 * @author Felix Thielke
 */

#pragma once

#include "Image.h"
#include "Tools/Streams/Streamable.h"
#include "opencv2/core/core.hpp"

struct ImagePatch : public Streamable
{
public:
  enum PatchType
  {
    framePatroling,
    kalmanPrediction,
    lastFramePrediction,
    bottomObstacles,
  };

  // Start is the upper left corner
  Vector2s offset;
  unsigned short width;
  unsigned short height;
  PatchType type;
  short minNeighbors;

  int minBallSize;
  int maxBallSize;

  bool isHighResolution;

  ImagePatch() : offset((short)0, (short)0), width(0), height(0), type(PatchType::framePatroling), minNeighbors(3), isHighResolution(false) {}
  ImagePatch(const Image& image, const Vector2s& offset, const unsigned short width, const unsigned short height, const PatchType type, const short minNeighbors) : offset(offset), width(width), height(height), type(type), minNeighbors(minNeighbors)
  {
    ASSERT(offset.x() >= 0 && offset.x() + width <= image.cv_image.cols);
    ASSERT(offset.y() >= 0 && offset.y() + height <= image.cv_image.rows);
    isHighResolution = false;
  }
  ~ImagePatch(){};

private:
  virtual void serialize(In* in, Out* out);
};

STREAMABLE(ImagePatches,
{
public:
  //void toImage(cv::Mat orig, cv::Mat & dest) const;
  void draw() const,

  (std::vector<ImagePatch>) patches,
});
