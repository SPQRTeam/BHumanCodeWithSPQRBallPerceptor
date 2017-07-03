/**
 * @file CascadeBallPerceptor.cpp
 * This file declares a module that provides the white and black ball percept
 * using cascade.
 * If possible, it uses the full YCbCr422 image.
 * @author Francesco Del Duchetto
 * @author Tiziano Manoni
 */

#include "CascadeBallPerceptor.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Eigen.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"
#include "Platform/SystemCall.h"
#include <algorithm>
#include <iostream>

#define IS_GOALIE (theRobotInfo.number == 1)
#define TOBHUMANF_RES(x,highRes) ((highRes)?x:TOBHUMANF(x))

using namespace cv;

MAKE_MODULE(CascadeBallPerceptor, perception)


CascadeBallPerceptor::CascadeBallPerceptor()
{
	string configDirectory;
	char currentWorkingDirectory[1024];
	int counter;

	configDirectory = "";

	if (SystemCall::getMode() == SystemCall::simulatedRobot)
	{
		if (getcwd(currentWorkingDirectory,1024)) {;}

		configDirectory = currentWorkingDirectory;

		configDirectory = configDirectory.substr(0,configDirectory.rfind("/")) + "/";
	}
	else configDirectory = "Config/";

#ifdef TARGET_ROBOT
	ball_cascade_name_bottom = configDirectory + "bottom_small_lbp.xml";
    ball_cascade_name_top = configDirectory + "top_small_lbp.xml";
	ball_cascade_name_top_highRes = configDirectory + "top_small_lbp.xml";
#else
	ball_cascade_name_bottom = configDirectory + "bottom_small_lbp.xml";
    ball_cascade_name_top = configDirectory + "top_small_lbp.xml";
	ball_cascade_name_top_highRes = configDirectory + "top_small_lbp.xml";
#endif

	
	if (!top_cascade.load(ball_cascade_name_top)) {
		std::cerr << "---------------------(!)Error loading CASCADE TOP" << std::endl;
	}
	if (!top_cascade_highRes.load(ball_cascade_name_top_highRes)) {
		std::cerr << "---------------------(!)Error loading CASCADE TOP HIGH RESOLUTION" << std::endl;
	}
	if (!bottom_cascade.load(ball_cascade_name_bottom)) {
		std::cerr << "---------------------(!)Error loading CASCADE BOTTOM" << std::endl;
	}

}

void CascadeBallPerceptor::update(BallPercept& ballPercept)
{
  DECLARE_DEBUG_DRAWING("module:CNSBallPerceptor:candidates", "drawingOnImage");

  double t_o = (double)getTickCount();


  initialize();

  CameraInfo::Camera cam = theCameraInfo.camera;

  //> First of all we think, that no ball was found...
  ballPercept.status = BallPercept::notSeen;
  if(!theCameraMatrix.isValid)
  {
      return;
  }

  //> Take patches and check on each of them sequentially. Stop as soon as a ball is found.
  for (ImagePatch patch : theImagePatches.patches)
  {
    if (DEBUG)
		std::cerr << "patch: " << patch.offset.x() << ", " << patch.offset.y() << ", "
		<< patch.width << ", " << patch.height << ", " << patch.type << std::endl;

	
	/*if (patch.type != ImagePatch::PatchType::framePatroling)
		continue;
	*/

    //> Use cascade to find the ball
    std::vector<Vector2i> ballFound;
    std::vector<short> radius_vector;
    if (findBallOnPatch(patch, ballPercept, cam, ballFound, radius_vector))
    {
		int ballFound_size = ballFound.size();
		for (int b=0; b<ballFound_size; ++b)
		{
			ballPercept.positionInImage.x() = TOBHUMANF_RES(ballFound[b].x(), patch.isHighResolution);
			ballPercept.positionInImage.y() = TOBHUMANF_RES(ballFound[b].y(), patch.isHighResolution);
			ballPercept.radiusInImage = TOBHUMANF_RES(radius_vector[b], patch.isHighResolution);
			ballPercept.status = !calculateBallOnField(ballPercept) ? BallPercept::calculateBallOnField :
						!checkBallOnField(ballPercept) ? BallPercept::notSeen :
						BallPercept::seen;
			ballPercept.camera = cam;
            if (DEBUG)
				std::cerr << "ball seen " << (ballPercept.status==BallPercept::seen) << std::endl;

			if (ballPercept.status == BallPercept::seen)
				break;
		}
    }
	//if (IS_GOALIE)
	//	break;
	if (ballPercept.status == BallPercept::seen)
		break;
  }

  if (ballPercept.status == BallPercept::notSeen)
  {
    if (cam == CameraInfo::upper)
	  ++ballPercept.n_frames_not_seen_up;
    else
      ++ballPercept.n_frames_not_seen_btm;
  }
  else
  {
	if (cam == CameraInfo::upper)
		ballPercept.n_frames_not_seen_up = 0;
	else
		ballPercept.n_frames_not_seen_btm = 0;

	ballPercept.timeWhenLastSeen = theFrameInfo.time;
  }

  if (DEBUG_SPEED)
  {
	if (cam == CameraInfo::upper)
	{
		t_o = ((double)getTickCount() - t_o) / getTickFrequency();
		std::cout << "UPPER - Times passed in seconds: " << t_o << std::endl;
	}
	else
	{
		t_o = ((double)getTickCount() - t_o) / getTickFrequency();
		std::cout << "LOWER - Times passed in seconds: " << t_o << std::endl;
	}
  }

}

bool CascadeBallPerceptor::findBallOnPatch(ImagePatch patch, BallPercept ballPercept, CameraInfo::Camera cam,
			std::vector<Vector2i>& ball_vector, std::vector<short>& radius_vector)
{
  if (DEBUG)
	std::cerr << "searching on: " << patch.type << std::endl;

  //> Check if the size is too small for the cascade
  if (patch.maxBallSize > 16)
  {
  	/*>Check only if it's a kalman prediction ROI
	   or if the ROI is not a different camera of the last perception
	   (we always check kalman ROI to preserve continuity in the perception btw
	   the two cameras, expecially in the transition from upper to lower)
	*/
    if (patch.type == ImagePatch::PatchType::kalmanPrediction
		|| (!(ballPercept.n_frames_not_seen_up == 0 && cam == CameraInfo::lower)
          && !(ballPercept.n_frames_not_seen_btm == 0 && cam == CameraInfo::upper)))
    {

      if (DEBUG)
		std::cerr << "minBall: " << patch.minBallSize << ", maxBall: " << patch.maxBallSize << std::endl;

	  Mat frame;
	  if (patch.isHighResolution)
	  {
		  frame = theImage.cv_image_high_resolution;

		 // std::cerr << "high " << std::endl;
		 // std::cerr << "frame: " << frame.size() << std::endl;
		 // std::cerr << "patch: " << patch.offset.x() << ", " << patch.offset.y() << ", "
		 // 	<< patch.width << ", " << patch.height << ", " << patch.type << std::endl;
	  }
	  else
	  	frame = theImage.cv_image;

	  cvtColor(frame, frame, CV_BGR2GRAY);

	  //> Take ROI from original image
	  Mat roi = frame(Rect(
	          std::max(0, (int) patch.offset.x()),
	          std::max(0, (int) patch.offset.y()),
	          std::min(frame.cols - patch.offset.x(), (int) patch.width),
	          std::min(frame.rows - patch.offset.y(), (int) patch.height)
	        ));

    if (DEBUG)
    {
		std::cerr << "patch: " << patch.offset.x() << ", " << patch.offset.y() << ", "
			<< patch.width << ", " << patch.height << ", " << patch.type << std::endl;

		std::cerr << "roi: " << roi.size() << std::endl;
	}

	  std::vector<Rect> balls;

      if (cam == CameraInfo::upper)
	  {
		if (patch.isHighResolution)
		  top_cascade_highRes.detectMultiScale(roi,
		  	  balls,
		  	  SCALE_FACTOR,
#ifdef TARGET_ROBOT
              patch.minNeighbors,
#else
              30,
#endif
		  	  FLAGS,
		  	  Size(patch.minBallSize, patch.minBallSize), //min
		  	  Size(patch.maxBallSize, patch.maxBallSize),  //max
              //Size(50, 50), //min
		  	  //Size(50, 50),  //max
		  	  theCameraInfo,
		  	  theCameraMatrix,
		  	  theFieldDimensions,
		  	  patch.offset.y()
		);
		else
		  top_cascade.detectMultiScale(roi,
			  balls,
			  SCALE_FACTOR,
#ifdef TARGET_ROBOT
              patch.minNeighbors,
#else
              30,
#endif
			  FLAGS,
			  Size(patch.minBallSize, patch.minBallSize), //min
			  Size(patch.maxBallSize, patch.maxBallSize),  //max
			  //Size(50, 50), //min
			  //Size(50, 50),  //max
			  theCameraInfo,
			  theCameraMatrix,
			  theFieldDimensions,
			  patch.offset.y()
		  );
	  }
      else
      {
        bottom_cascade.detectMultiScale(roi,
          balls,
          SCALE_FACTOR,
#ifdef TARGET_ROBOT
          patch.minNeighbors,
#else
          30,
#endif
          FLAGS,
          Size(patch.minBallSize, patch.minBallSize), //min
          Size(patch.maxBallSize, patch.maxBallSize),  //max
          //Size(10, 10), //min
          //Size(10, 10),  //max
		  theCameraInfo,
		  theCameraMatrix,
		  theFieldDimensions,
		  patch.offset.y()
        );
      }
      if (DEBUG)
      {
		std::cerr << "patch: " << patch.offset.x() << ", " << patch.offset.y() << ", "
			<< patch.width << ", " << patch.height << ", " << patch.type << std::endl;
		std::cerr << "minBall: " << patch.minBallSize << ", maxBall: " << patch.maxBallSize << std::endl;
	  }

      RECTANGLE("module:CNSBallPerceptor:candidates", TOBHUMANF_RES(patch.offset.x(), patch.isHighResolution),
          TOBHUMANF_RES(patch.offset.y(), patch.isHighResolution), TOBHUMANF_RES(patch.offset.x(), patch.isHighResolution) + TOBHUMANF_RES(patch.width, patch.isHighResolution),
          TOBHUMANF_RES(patch.offset.y(), patch.isHighResolution) + TOBHUMANF_RES(patch.height, patch.isHighResolution), 5, Drawings::solidPen,
		  (patch.isHighResolution)? ColorRGBA::white :
          (patch.type == ImagePatch::PatchType::framePatroling)? ColorRGBA::blue :
          (patch.type == ImagePatch::PatchType::kalmanPrediction)? ColorRGBA::orange :
          ColorRGBA::green);

	 int ball_size = balls.size();
	  for (int b = ball_size-1; b >= 0; --b)
      {
        Vector2i ball = Vector2i(balls[b].x + (cvRound(balls[b].width) >> 1) + patch.offset.x(),
            	balls[b].y + (cvRound(balls[b].height) >> 1) + patch.offset.y());
        int radius = cvRound(balls[b].height)  >> 1;
        if (DEBUG)
			std::cerr << "ball size: " << radius << std::endl;

        if (checkBallOnImage(TOBHUMANVI(ball), patch))
        {
            if (SAVE_POSITIVES)
			{
				std::string name;
				if (cam == CameraInfo::upper)
					name = std::to_string(theImage.timeStamp) + "_pos_up_log_" + std::to_string(patch.type) + ".png";
				else
					name = std::to_string(theImage.timeStamp) + "_pos_lw_log_" + std::to_string(patch.type) + ".png";
				if (cv::imwrite( name, roi )) {
					std::cerr << std::to_string(theImage.timeStamp).c_str() << " salvata" << std::endl;
				} else {
					std::cerr << "error saving " << std::to_string(theImage.timeStamp).c_str() << std::endl;
				}
			}


			ball_vector.push_back(ball);
			radius_vector.push_back(radius);
        }
      }

	  if (ball_vector.size() > 0)
	  	return true;

      if (SAVE_NEGATIVES)
      {
		std::string name;
		if (cam == CameraInfo::upper)
		name = std::to_string(theImage.timeStamp) + "_neg_up_log_" + std::to_string(patch.type) + ".png";
		else
		name = std::to_string(theImage.timeStamp) + "_neg_lw_log_" + std::to_string(patch.type) + ".png";
		if (cv::imwrite( name, roi )) {
			std::cerr << std::to_string(theImage.timeStamp).c_str() << " salvata" << std::endl;
		} else {
			std::cerr << "error saving " << std::to_string(theImage.timeStamp).c_str() << std::endl;
		}
	  }

    }
  }
  return false;
}

bool CascadeBallPerceptor::checkBallOnImage(Vector2i ball, ImagePatch patch)
{

  if (BLOCK_BALLS_INSIDE_BODYCONTOUR)
  {
	//> check inside body contour
	if (!theBodyContour.isValidPoint(ball))
	{
		if (DEBUG_BALL_BLOCKER)
			std::cerr << "### BALL BLOCKER ->  inside BodyContour  " << std::endl;
  
		return false;
	}
  }


  //> calculate the image limits
  int height = theCameraInfo.height - 3;
  int right = theCameraInfo.width - 3;

  int horizon = std::max(2, (int) theImageCoordinateSystem.origin.y());
  horizon = std::min(horizon, height);


  if (ball.y() <= horizon)
    return false;

  if (BLOCK_BALLS_ON_OBSTACLES_IMAGE)
  {
	//> check on obstacles on image
	for (const auto player : thePlayersPercept.players)
	{
		short maxBallDiameterOnFeet = IISC::getImageBallRadiusByCenter(Vector2f(player.x2, player.y2), theCameraInfo, theCameraMatrix, theFieldDimensions);
		maxBallDiameterOnFeet = maxBallDiameterOnFeet << 1;
		LINE("module:CNSBallPerceptor:candidates", player.x1FeetOnly, player.y2 - (maxBallDiameterOnFeet << 1),
					player.x2FeetOnly, player.y2 - (maxBallDiameterOnFeet << 1), 5, Drawings::solidPen, ColorRGBA::white);
			
	#ifdef TARGET_ROBOT
		if (ball.x() > player.x1FeetOnly && ball.x() < player.x2FeetOnly && ball.y() > player.y1 && ball.y() < (player.y2 - (maxBallDiameterOnFeet << 1)))
	#else
		if (ball.x() > player.x1 && ball.x() < player.x2 && ball.y() > player.y1 && ball.y() < player.y2)
	#endif
		{
			if (DEBUG_BALL_BLOCKER)
				std::cerr << "### BALL BLOCKER ->  inside PlayersPercept" << std::endl;
  
			return false;
		}
	}
  }


  	return true;
}

void CascadeBallPerceptorScaler::initialize()
{
    typedef CascadeBallPerceptorBase B;
    theCameraInfo = B::theCameraInfo;
    //theBallSpots = B::theBallSpots;

    //> do not copy "table"
    theImageCoordinateSystem.rotation = B::theImageCoordinateSystem.rotation;
    theImageCoordinateSystem.invRotation = B::theImageCoordinateSystem.invRotation;
    theImageCoordinateSystem.origin = B::theImageCoordinateSystem.origin;
    theImageCoordinateSystem.offset = B::theImageCoordinateSystem.offset;
    theImageCoordinateSystem.a = B::theImageCoordinateSystem.a;
    theImageCoordinateSystem.b = B::theImageCoordinateSystem.b;

/*    if(theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
      {
          theCameraInfo.width *= 2;
          theCameraInfo.height *= 2;
          theCameraInfo.opticalCenter *= 2.f;
          theCameraInfo.focalLength *= 2.f;
          theCameraInfo.focalLengthInv /= 2.f;
          theCameraInfo.focalLenPow2 *= 4.f;
  
          theImageCoordinateSystem.origin *= 2.f;
          theImageCoordinateSystem.b *= 0.5f;
  
          for(BallSpot& b : theBallSpots.ballSpots)
          {
              b.position *= 2;
              ++b.position.x(); // original y channel was the second one
          }
      }
*/
    theImageCoordinateSystem.setCameraInfo(theCameraInfo);
}

bool CascadeBallPerceptor::calculateBallOnField(BallPercept& ballPercept) const
{
    const Vector2f correctedCenter = theImageCoordinateSystem.toCorrected(ballPercept.positionInImage);
    Vector3f cameraToBall(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x() - correctedCenter.x(), theCameraInfo.opticalCenter.y() - correctedCenter.y());
    cameraToBall.normalize(theFieldDimensions.ballRadius * theCameraInfo.focalLength / ballPercept.radiusInImage);
    Vector3f rotatedCameraToBall = theCameraMatrix.rotation * cameraToBall;
    const Vector3f sizeBasedCenterOnField = theCameraMatrix.translation + rotatedCameraToBall;
    const Vector3f bearingBasedCenterOnField =  theCameraMatrix.translation - rotatedCameraToBall * ((theCameraMatrix.translation.z() - theFieldDimensions.ballRadius) / rotatedCameraToBall.z());

	Vector2f positionOnField;
    if(rotatedCameraToBall.z() < 0)
    {
        positionOnField.x() = bearingBasedCenterOnField.x();
        positionOnField.y() = bearingBasedCenterOnField.y();
    }
    else
    {
        positionOnField.x() = sizeBasedCenterOnField.x();
        positionOnField.y() = sizeBasedCenterOnField.y();
    }

	//> Compute velocity
	Vector2f velocity = Vector2f::Zero();
	float norm = 0.f;
	if (ballPercept.n_frames_not_seen_up < 4 || ballPercept.n_frames_not_seen_btm < 4)
	{
		//> if is coming toward us is less than zero
		velocity = positionOnField - ballPercept.positionOnField;
		norm = velocity.norm();
	}
	ballPercept.positionOnField = positionOnField;
	ballPercept.velocity = velocity;
	ballPercept.velocity_norm = norm;

    return true;
}

bool CascadeBallPerceptor::checkBallOnField(BallPercept& ballPercept) const
{
    //> Not sure about self-localization => Do not use it to check ball position
    if(theRobotPose.validity < 1.f)
    {
        return true;
    }

    if (BLOCK_BALLS_OUTSIDE_FIELD)
    {
		//> Check, if the computed ball position is still on the carpet
		Pose2f currentRobotPose = theRobotPose + theOdometer.odometryOffset;
		Vector2f absoluteBallPosition = (currentRobotPose * ballPercept.positionOnField);
		if (!(fabs(absoluteBallPosition.x()) < theFieldDimensions.xPosOpponentFieldBorder + 300.f) ||
			!(fabs(absoluteBallPosition.y()) < theFieldDimensions.yPosLeftFieldBorder + 300.f))
		{
			if (DEBUG_BALL_BLOCKER)
				std::cerr << "### BALL BLOCKER ->  outside FieldBorders" << std::endl;
  
			return false;
		}
	}


    if (BLOCK_BALLS_ON_OBSTACLES_FIELD)
    {
		//> check on obstacles on field
		for (const auto obstacle : theObstacleModel.obstacles)
		{
			float obstacleRadius = (obstacle.left - obstacle.right).norm() * .5f;
			if (ballPercept.positionOnField.x() > obstacle.center.x() - Obstacle::getRobotDepth() &&
				ballPercept.positionOnField.x() < obstacle.center.x() + Obstacle::getRobotDepth() &&
				ballPercept.positionOnField.y() > obstacle.center.y() - obstacleRadius &&
				ballPercept.positionOnField.y() < obstacle.center.y() + obstacleRadius)
			{
				if (DEBUG_BALL_BLOCKER)
					std::cerr << "### BALL BLOCKER ->  inside ObstacleModel" << std::endl;
  
				return false;
			}
		}
	}


    if (BLOCK_BALLS_WITH_BAD_RADIUS)
    {
		//> check ball radius given by the cascade with the computed one w.r.t. the position
		short computedDiameter = IISC::getImageBallRadiusByCenter(ballPercept.positionInImage, theCameraInfo, theCameraMatrix, theFieldDimensions);
		computedDiameter = computedDiameter << 1;
		short absDiff = abs(computedDiameter - (int(ballPercept.radiusInImage) << 1));
		//CIRCLE("module:CNSBallPerceptor:candidates", ballPercept.positionInImage.x(), ballPercept.positionInImage.y(), computedDiameter >> 1,
		//        3, Drawings::dottedPen, ColorRGBA::green, Drawings::BrushStyle::noBrush, ColorRGBA::green);
		ballPercept.radiusInImage = computedDiameter >> 1;
		if (absDiff > DIFFERENCE_RADIUS_THRESHOLD)
		{
			if (DEBUG_BALL_BLOCKER)
				std::cerr << "### BALL BLOCKER ->  radius difference > " << absDiff << std::endl;
  
			return false;
		}
	}


	return true;
}
