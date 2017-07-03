/**
 * @file BallSpotPatchesProvider.cpp
 * This file declares a module that provides patches of the ball 
 * in the image.
 * @author Francesco Del Duchetto
 */

#include "BallSpotPatchesProvider.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <iostream>

//#ifdef TARGET_ROBOT
//    #define INCLUDE_PATCHES_ON_OBSTACLES
//#endif

#define UPPER_DIVIDE_6_PATROLING
//#define UPPER_DIVIDE_3_PATROLING

#define USE_HIGH_RES_IMAGE


#define IS_GOALIE (theRobotInfo.number == 1)

#ifdef PENALTY_STRIKER_GOALIE
    #undef USE_HIGH_RES_IMAGE
    #define UPPER_DIVIDE_3_PATROLING
#endif

  
MAKE_MODULE(BallSpotPatchesProvider, perception)

 
void BallSpotPatchesProvider::update(ImagePatches& imagePatches)
{
  CameraInfo::Camera camera = theCameraInfo.camera;
  
  if(DEBUG)
	std::cerr << "start frame" << std::endl;

  //> First remove all the old patches
  imagePatches.patches.clear();

#ifdef PENALTY_STRIKER_GOALIE
  if (camera == CameraInfo::lower)
    return;
#endif

  //> Last time seen in this frame
  int n_frame_not_seen = (camera == CameraInfo::upper) ? theBallPercept.n_frames_not_seen_up : theBallPercept.n_frames_not_seen_btm;
  if (DEBUG)
	std::cerr << "frame not seen " << n_frame_not_seen << std::endl;

  //>>> lastFramePrediction PATCH
  if (theBallPercept.camera == camera && n_frame_not_seen < 2) // seen in the same camera in the last step
  {
	  if (DEBUG)
		std::cerr << "lastFramePrediction" << std::endl;

      //> take a ROI around the previous ball
      int roi_range = FROMBHUMANF(theBallPercept.radiusInImage * 3);

      ImagePatch patch;
      patch.offset = Vector2s(std::max((int)(FROMBHUMANF(theBallPercept.positionInImage.x()) - roi_range), 0),
      std::max((int)(FROMBHUMANF(theBallPercept.positionInImage.y()) - roi_range), 0));
      patch.width = std::min(roi_range << 1, theImage.cv_image.cols - (int)(FROMBHUMANF(theBallPercept.positionInImage.x())) + roi_range);
      patch.height = std::min(roi_range << 1, theImage.cv_image.rows - (int)(FROMBHUMANF(theBallPercept.positionInImage.y())) + roi_range);
      patch.type = ImagePatch::PatchType::lastFramePrediction;

      //> if the ROI is on the predicted ball and on an obstacle/own body remove it
      Vector2i center = Vector2i(patch.offset.x() + (patch.width >> 1), patch.offset.y() + (patch.height >> 1));
      if (checkBallPosition(TOBHUMANVI(center)))
      {
          if (isOnObstacle(center))
            patch.minNeighbors = 25;
          else
            patch.minNeighbors = 3;

          computeMinMaxBallDiameter(patch);
          imagePatches.patches.push_back(patch);
          if (DEBUG)
          {
			std::cerr << "------------------- roi " << std::endl;
			std::cerr << "lastFramePrediction 2" << std::endl;
		  }

      }
  }

    if (!IS_GOALIE)
    {
      //>>> kalmanPrediction PATCH
      Vector2f pointInImage;
      if (Transformation::robotToImage(Transformation::fieldToRobot(theRobotPose, theTeamBallModel.position),
          theCameraMatrix, theCameraInfo, pointInImage))
      {
        if (DEBUG)
          std::cerr << "kalmanPrediction" << std::endl;
      
        int ballDiameter = FROMBHUMANF(computeBallDiameter(Vector2i(pointInImage.x(), pointInImage.y())));
        pointInImage = FROMBHUMANVF(pointInImage);
        pointInImage.y() -= ballDiameter >> 1;
        if (pointInImage.x() > 0 && pointInImage.x() < ((camera==CameraInfo::upper)?UPPER_CAMERA_WIDTH:LOWER_CAMERA_WIDTH) &&
            pointInImage.y() > 0 && pointInImage.y() < ((camera==CameraInfo::upper)?UPPER_CAMERA_HEIGHT:LOWER_CAMERA_HEIGHT))
        {
          ImagePatch patch = ImagePatch(theImage,
                  Vector2s(std::max((int)(pointInImage.x() - ballDiameter * KALMAN_ROI_SIZE), 0), std::max((int)(pointInImage.y() - (ballDiameter * KALMAN_ROI_SIZE)), 0)),
                  std::min(ballDiameter * (KALMAN_ROI_SIZE << 1), (int)(theImage.cv_image.cols - (pointInImage.x() - ballDiameter * KALMAN_ROI_SIZE))),
                  std::min(ballDiameter * (KALMAN_ROI_SIZE << 1), (int)(theImage.cv_image.rows - (pointInImage.y() - (ballDiameter * KALMAN_ROI_SIZE)))),
                  ImagePatch::PatchType::kalmanPrediction,
                  (isOnObstacle(Vector2i(pointInImage.x(), pointInImage.y()))?25:3));
          computeMinMaxBallDiameter(patch);
          imagePatches.patches.push_back(patch);
          if (DEBUG)
			std::cerr << "kalmanPrediction 2" << std::endl;
      
        }

        CROSS("module:CNSBallPerceptor:candidates", TOBHUMANF(pointInImage.x()), TOBHUMANF(pointInImage.y()), 20, 4, Drawings::solidPen, ColorRGBA::orange);
      }
    }



  //>>> framePatroling PATCH
  if (IS_GOALIE && theBallPercept.n_frames_not_seen_up < 2)
  {
      ImagePatch patch;
      patch.type = ImagePatch::PatchType::framePatroling;
      patch.minNeighbors = 3;
      if (camera == CameraInfo::upper)
      {
          patch.offset.x() = 0;
          patch.offset.y() = FROMBHUMANF(theBallPercept.positionInImage.y());
          patch.height = FROMBHUMANF(480 - theBallPercept.positionInImage.y());
          patch.width = FROMBHUMANF(640);
      }
      else
      {
          patch.offset = Vector2s(0, 0);
          patch.width = FROMBHUMANF(320);
          patch.height =  FROMBHUMANF(240);
      }
      computeMinMaxBallDiameter(patch);
      imagePatches.patches.push_back(patch);

  }
  else
  {
      ImagePatch patch;
      if (getPatrolingPatch(patch, n_frame_not_seen, camera))
      {
          // If the ROI we find is outside skip and find another ROI
          
		  int right = theCameraInfo.width - 3;
		  int height = theCameraInfo.height - 3;
		  int horizon = std::max(2, (int) theImageCoordinateSystem.origin.y());
		  horizon = std::min(horizon, height);
          horizon = FROMBHUMANF(horizon);

          computeMinMaxBallDiameter(patch);

          // remove ROIs on the horizon and with too little possible balls
          if (patch.offset.y() + patch.height <= horizon || patch.maxBallSize <= 16)
          {
			  if (getPatrolingPatch(patch, n_frame_not_seen + 1, camera))
			  {
				  computeMinMaxBallDiameter(patch);
                  imagePatches.patches.push_back(patch);
              }
          }
          else
          {
              if (patch.offset.y() < horizon)
              {
                  patch.height = (patch.offset.y() + patch.height) - horizon;
                  patch.offset.y() = horizon;
              }
              imagePatches.patches.push_back(patch);
          }
      }

#ifdef USE_HIGH_RES_IMAGE
      if (!IS_GOALIE)
      {
          for (uint i=0; i < imagePatches.patches.size(); ++i)
          {
              if (((imagePatches.patches.at(i).maxBallSize + imagePatches.patches.at(i).minBallSize) >> 1) <= 16)
              {
                  imagePatches.patches.at(i).isHighResolution = true;
                  imagePatches.patches.at(i).minBallSize *= 2;
                  imagePatches.patches.at(i).maxBallSize *= 2;
                  imagePatches.patches.at(i).offset = TOBHUMANVS(imagePatches.patches.at(i).offset);
                  imagePatches.patches.at(i).width = TOBHUMANF(imagePatches.patches.at(i).width);
                  imagePatches.patches.at(i).height = TOBHUMANF(imagePatches.patches.at(i).height);
                  //std::cerr << "min size: " << imagePatches.patches.at(i).minBallSize << " " << imagePatches.patches.at(i).maxBallSize<< std::endl;
              }
          }
      }
#endif
  }


  //>> bottomObstacles PATCHES
/*#ifdef INCLUDE_PATCHES_ON_OBSTACLES
  for(const auto player : thePlayersPercept.players)
  {
	ImagePatch patch;
    int ballDiameter = FROMBHUMANF(computeBallDiameter(Vector2i(player.x1, player.y2)));
    //LINE("module:CNSBallPerceptor:candidates", 0, player.y2, 100, player.y2, 5, Drawings::dottedPen, ColorRGBA::yellow);
  
    patch.offset = FROMBHUMANF(Vector2s(player.x1, player.y2 - ballDiameter * 2));
    patch.width = FROMBHUMANF(player.x2 - player.x1);
    patch.height = ballDiameter * 3;
    patch.type = ImagePatch::PatchType::bottomObstacles;
    patch.minNeighbors = 25;
    imagePatches.patches.push_back(patch);
  }
#endif

  Compute ball min and max sizes
  for (int i = 0; i < imagePatches.patches.size(); ++i)
  {
    computeMinMaxBallDiameter(imagePatches.patches.at(i));
  }
 */

}

bool BallSpotPatchesProvider::getPatrolingPatch(ImagePatch& patch, int n_frame_not_seen, CameraInfo::Camera camera)
{
  patch.type = ImagePatch::PatchType::framePatroling;
  patch.minNeighbors = 3;
  if (camera == CameraInfo::upper)
  {
#if defined(UPPER_DIVIDE_3_PATROLING)
    if (n_frame_not_seen % 3 ==  0)
    {
      if (DEBUG)
		std::cerr << "------------- 0" << std::endl;
  
      patch.offset = Vector2s(0, FROMBHUMANF(280));
      patch.width = FROMBHUMANF(640);
      patch.height = FROMBHUMANF(200);
    }
    else if (n_frame_not_seen % 3 == 1)
    {
      if (DEBUG)
		std::cerr << "------------- 1" << std::endl;
  
      patch.offset = Vector2s(0, FROMBHUMANF(140));
      patch.width = FROMBHUMANF(640);
      patch.height = FROMBHUMANF(200);
    }
    else
    {
      if (DEBUG)
		std::cerr << "------------- 2" << std::endl;
  
      patch.offset = Vector2s(0, 0);
      patch.width = FROMBHUMANF(640);
      patch.height = FROMBHUMANF(200);
    }
#elif defined(UPPER_DIVIDE_6_PATROLING)
     if (n_frame_not_seen % 6 ==  0)

     {
       if (DEBUG)
		std::cerr << "------------- 0" << std::endl;
   
       patch.offset = Vector2s(0, FROMBHUMANF(280));
       patch.width = FROMBHUMANF(360);
       patch.height = FROMBHUMANF(200);
     }
     else if (n_frame_not_seen % 6 == 1)
     {
       if (DEBUG)
		std::cerr << "------------- 1" << std::endl;
   
       patch.offset = Vector2s(FROMBHUMANF(280), FROMBHUMANF(280));
       patch.width = FROMBHUMANF(360);
       patch.height = FROMBHUMANF(200);
     }
     else if (n_frame_not_seen % 6 == 2)
     {
       if (DEBUG)
		std::cerr << "------------- 2" << std::endl;
   
       patch.offset = Vector2s(0, FROMBHUMANF(140));
       patch.width = FROMBHUMANF(360);
       patch.height = FROMBHUMANF(200);
     }
     else if (n_frame_not_seen % 6 == 3)
     {
       if (DEBUG)
		std::cerr << "------------- 3" << std::endl;
   
       patch.offset = Vector2s(FROMBHUMANF(280), FROMBHUMANF(140));
       patch.width = FROMBHUMANF(360);
       patch.height = FROMBHUMANF(200);
     }
     else if (n_frame_not_seen % 6 == 4)
     {
       if (DEBUG)
		std::cerr << "------------- 4" << std::endl;
   
       patch.offset = Vector2s(0, 0);
       patch.width = FROMBHUMANF(360);
       patch.height = FROMBHUMANF(200);
     }
     else
     {
       if (DEBUG)
		std::cerr << "------------- 5" << std::endl;
   
       patch.offset = Vector2s(FROMBHUMANF(280), FROMBHUMANF(0));
       patch.width = FROMBHUMANF(360);
       patch.height = FROMBHUMANF(200);
     }
#else
    patch.offset = Vector2s(0, FROMBHUMANF(0));
    patch.width = FROMBHUMANF(640);
    patch.height = FROMBHUMANF(480);
#endif
  }
  else // BOTTOM CAMERA
  {
    patch.offset = Vector2s(0, 0);
    patch.width = FROMBHUMANF(320);
    patch.height =  FROMBHUMANF(240);
  }

  return true;
}

bool BallSpotPatchesProvider::isOnObstacle(Vector2i ball)
{
  for(const auto player : thePlayersPercept.players)
  {
    if (ball.x() > player.x1 && ball.x() < player.x2 && ball.y() > player.y1 && ball.y() < player.y2)
      return true;
  }
  return false;
}

bool BallSpotPatchesProvider::checkBallPosition(Vector2i ball)
{
#ifdef TARGET_ROBOT
  // check inside body contour
  if (!theBodyContour.isValidPoint(ball))
  {
    return false;
  }
#endif

  // calculate the image limits
  CameraInfo::Camera camera = theCameraInfo.camera;
  int right = theCameraInfo.width - 3;
  int height = theCameraInfo.height - 3;
  int horizon = std::max(2, (int) theImageCoordinateSystem.origin.y());
  horizon = std::min(horizon, height);
  
  if (ball.y() <= horizon - 20)
    return false;

#ifndef INCLUDE_PATCHES_ON_OBSTACLES
  if (isOnObstacle(ball))
    return false;
#endif

  return true;
  //LINE("module:CNSBallPerceptor:candidates", 0, horizon, right, horizon, 5, Drawings::solidPen, ColorRGBA::green);
}

void BallSpotPatchesProvider::computeMinMaxBallDiameter(ImagePatch& patch)
{

  std::vector<int> addCoords = {patch.offset.x(), patch.offset.y(), patch.offset.x() + patch.width, patch.offset.y() + patch.height};
  for (int i = 0; i < 2; ++i)
  {
    const Vector2i& spot = Vector2i(addCoords.at((i << 1) + 0), addCoords.at((i << 1) + 1));

    short diameter = FROMBHUMANF(computeBallDiameter(TOBHUMANVI(spot)));
    if (i == 0)
      patch.minBallSize = diameter;
    else
      patch.maxBallSize = diameter;

    CIRCLE("module:CNSBallPerceptor:candidates", TOBHUMANF(addCoords.at((i << 1) + 0)), TOBHUMANF(addCoords.at((i << 1) + 1)), TOBHUMANF((diameter >> 1)),
            3, Drawings::dottedPen, ColorRGBA::blue, Drawings::BrushStyle::noBrush, ColorRGBA::green);
  }

/*  approxDiameter.at(0).width = max(approxDiameter.at(0).width, 24);
    approxDiameter.at(0).height = max(approxDiameter.at(0).height, 24);
  
  
    std::cerr << "min rad: " << approxDiameter.at(0)<< ", " << approxDiameter.at(0) << std::endl;
    std::cerr << "max rad: " << approxDiameter.at(1)<< ", " << approxDiameter.at(1) << std::endl;
  
    return approxDiameter;
*/
}

short BallSpotPatchesProvider::computeBallDiameter(Vector2i pointInImage)
{
  Vector2f correctedStart = theImageCoordinateSystem.toCorrected(pointInImage);
  Vector3f cameraToStart(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x()
        - correctedStart.x(), theCameraInfo.opticalCenter.y() - correctedStart.y());
  Vector3f unscaledField = theCameraMatrix.rotation * cameraToStart;
  if(unscaledField.z() >= 0.f)
  {
    return 0;// above horizon
  }
  const float scaleFactor = (theCameraMatrix.translation.z() - theFieldDimensions.ballRadius) / unscaledField.z();
  cameraToStart *= scaleFactor;
  unscaledField *= scaleFactor;
  /*if(unscaledField.topRows(2).squaredNorm() > sqrMaxBallDistance)
    {
        return false; // too far away, computed with respect to the field.
    }
  */
  cameraToStart.y() += cameraToStart.y() > 0 ? -theFieldDimensions.ballRadius : theFieldDimensions.ballRadius;
  cameraToStart /= scaleFactor;
  float appRad = std::abs(theCameraInfo.opticalCenter.x() - cameraToStart.y() - correctedStart.x());

  return (short) appRad << 1;
}
