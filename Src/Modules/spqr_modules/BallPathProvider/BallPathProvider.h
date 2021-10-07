/**
 * @file zmpProvider.h
 *
 * This file computes Ball Path 
 *
 * @author Akshay Dhonthi
 */

#pragma once

#include "Representations/spqr_representations/BallPath.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Module/Module.h"

#include "Representations/Communication/RobotInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/ActivationGraph.h"

#include "Representations/Modeling/TeamPlayersModel.h"

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

using namespace Eigen;

MODULE(BallPathProvider,
{,
  REQUIRES(JointAngles),

  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(FootSupport),
  REQUIRES(RobotModel),
  REQUIRES(OwnTeamInfo),
  REQUIRES(BallModel),
  REQUIRES(TeamBallModel),
  USES(ActivationGraph),
  REQUIRES(TeamPlayersModel),

  USES(LibCheck),
  PROVIDES(BallPath),
});

class BallPathProvider: public BallPathProviderBase
{
  public:
    void update(BallPath & ballPath) override;
    Pose2f targetPoint1, targetPoint2, feedbackPoint1, feedbackPoint2;
};
