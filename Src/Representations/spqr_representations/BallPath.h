/**
 * @file zmpProvider.h
 *
 * This file computes zero moment point of the robot 
 *
 * @author Akshay Dhonthi
 */

#pragma once

#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/RobotParts/Limbs.h"
#include "Tools/Streams/EnumIndexedArray.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Function.h"

STREAMABLE(BallPath,
{
  BallPath() = default;
  ,

  (float)(0.0) errorTheta,
  (float)(0.0) errorDistX,
  (float)(0.0) errorDistY,
  (float)(0.0) oppBallDist,

});