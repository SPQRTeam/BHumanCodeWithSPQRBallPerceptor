/**
 * @file BallPercept.h
 *
 * Very simple representation of a seen ball
 *
 * @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 * @autor Jesse
 */

#pragma once

#include "Tools/Streams/Enum.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include <vector>
#include "Representations/Infrastructure/CameraInfo.h"

STREAMABLE(BallPercept,
{
  ENUM(Status,
  {,
    notSeen, /**< The ball was not seen. */
    seen,    /**< The ball was seen. */
    guessed, /**< Would be ok for a moving ball. */
    calculateBallOnField,
    seenHighResolution,
  });

  BallPercept() = default;
  inline BallPercept(const Vector2f& positionInImage, const float radiusInImage, const Vector2f& relativePositionOnField, const float radiusOnField, const BallPercept::Status status);

  /** Draws the ball*/
  void draw() const,

  (int)(CameraInfo::lower) camera,
  (int)(1000) n_frames_not_seen_up,
  (int)(1000) n_frames_not_seen_btm,
  (Vector2f)(0.f,0.f) previousPositionInImage,
  (Vector2f)(0.f,0.f) previousPositionOnField,
  (unsigned)(0) timeWhenLastSeen,

  (Vector2f) positionInImage,         /**< The position of the ball in the current image */
  (float) radiusInImage,              /**< The radius of the ball in the current image */
  (Status)(notSeen) status,           /**< Indicates, if the ball was seen in the current image. */
  (Vector2f) positionOnField,         /**< Ball position relative to the robot. */
  (float)(50.f) radiusOnField,        /**< The radius of the ball on the field in mm */

  (Vector2f) (Vector2f::Zero()) velocity,
  (float)(0.f) velocity_norm,

  (Vector2f) positionOnFieldHighResolution,
});

inline BallPercept::BallPercept(const Vector2f& positionInImage, const float radiusInImage, const Vector2f& positionOnField, const float radiusOnField, const BallPercept::Status status = BallPercept::Status::seen) :
  positionInImage(positionInImage), radiusInImage(radiusInImage), status(status), positionOnField(positionOnField), radiusOnField(radiusOnField) {}
