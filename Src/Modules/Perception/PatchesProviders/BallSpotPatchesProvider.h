/**
 * @file BallSpotPatchesProvider.h
 * This file declares a module that provides patches of the ball 
 * in the image.
 * @author Francesco Del Duchetto
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Infrastructure/ImagePatches.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/SPQRDefinitions.h"
#include "Representations/Perception/PlayersPercepts/PlayersPercept.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "opencv2/core/core.hpp"

#ifdef TARGET_ROBOT
	#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#endif


MODULE(BallSpotPatchesProvider,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(FieldDimensions),
  REQUIRES(Image),
  REQUIRES(PlayersPercept),
  USES(TeamBallModel),
#ifdef TARGET_ROBOT
  REQUIRES(BodyContour),
#endif
  USES(RobotInfo),
  USES(RobotPose),
  USES(BallPercept),
  PROVIDES(ImagePatches),
  LOADS_PARAMETERS(
  {,
    (int)(2) KALMAN_ROI_SIZE,
    (bool) DEBUG, 
  }),
  
});

class BallSpotPatchesProvider : public BallSpotPatchesProviderBase
{
private:
	
	
   /**
  * The main method of this module.
  * Determine the image patch and the position of the ball
  * @param imagePatches the images taken by camera
  */
  void update(ImagePatches& imagePatches);
   
   
   /**
  * Compare the ball position and the players position
  * @param ball the coordinates of the ball
  * @return true if there's a player near the ball
  * else
  * @return false 
  */
  bool isOnObstacle(Vector2i ball);
   
   
   /**
  * Determine if the ball is inside the camera's image
  * @param ball the coordinates of the ball
  * @return false if the ball is above horizon
  * else
  * @return true if the ball is in image limits
  */
  bool checkBallPosition(Vector2i ball);
   
   
   /**
  * Determine the offset, the width and the height of the patch
  * subject to the number of the frame not seen
  * @param patch the image patch 
  * @param n_frame_not_seen number of the frame not seen
  * @param camera the camera which takes the frames
  * @return true if the operation was completed successfully
  */
  bool getPatrolingPatch(ImagePatch& patch, int n_frame_not_seen, CameraInfo::Camera camera);


   /**
  * Determine an estimate of the min and the max of ball diameter
  * using the function computeBallDiameter 
  * @param patch the image patch
  * @return assignments to patch.minBallSize and patch.maxBallSize
  */
  void computeMinMaxBallDiameter(ImagePatch& patch);
   
   
   /**
  * Calculate the ball diameter using the CameraInfo 
  * corrected by a scaleFactor
  * @param pointInImage the coordinates of the ball
  * @return 0 if the ball is above the horizon
  * else
  * @return a short which represent the ball diameter
  */
  short computeBallDiameter(Vector2i pointInImage);
};
