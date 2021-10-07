/**
 * @file WalkToPassForced.cpp
 *
 * This file implements the WalkToPassForced skill, that overrides the passage target from the PassShare representation
 * with a custom (possibily unreachable) one.
 *
 * @author Emanuele Musumeci
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/spqr_representations/PassShare.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"

SKILL_IMPLEMENTATION(WalkToPassForcedImpl,
{,
  IMPLEMENTS(WalkToPassForced),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  REQUIRES(FieldBall),
  REQUIRES(RobotPose),
  REQUIRES(PassShare),
  REQUIRES(TeamData),
  REQUIRES(FieldDimensions),
  CALLS(Activity),
  MODIFIES(MotionRequest),
  MODIFIES(BehaviorStatus),
});

class WalkToPassForcedImpl : public WalkToPassForcedImplBase
{
  void execute(const WalkToPassForced& p) override
  {

    Vector2f target = getTeammatePositionFromNumber(p.passingTo);
    if(target.x() == theFieldDimensions.xPosOpponentGroundline && target.y() == 0) return;
    theBehaviorStatus.shootingTo = target;
    theBehaviorStatus.passTarget = p.passingTo;
    theLibCheck.inc(LibCheck::passTarget);
    theActivitySkill(BehaviorStatus::passing);

    Angle target_angle = ( theRobotPose.inversePose * Vector2f(target.x(), target.y() ) ).angle();
    theMotionRequest.motion = MotionRequest::walk;
    theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
    theMotionRequest.walkRequest.target = Pose2f( target_angle, theFieldBall.positionRelative.x() - p.offsetX,
                                             theFieldBall.positionRelative.y() - p.offsetY );
    theMotionRequest.walkRequest.speed = Pose2f(1.f,1.f,1.f);
    theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
    theLibCheck.inc(LibCheck::motionRequest);
  }

  Vector2f getTeammatePositionFromNumber(int number)
  {
    for(const auto teammate : theTeamData.teammates)
    {
      if(teammate.number == number) return teammate.theRobotPose.translation;
    }
    return Vector2f(theFieldDimensions.xPosOpponentGroundline, 0);
  }
  
  bool isDone(const WalkToPassForced&) const override
  {
    return theMotionInfo.motion == MotionRequest::walk;
  }

};

MAKE_SKILL_IMPLEMENTATION(WalkToPassForcedImpl);
