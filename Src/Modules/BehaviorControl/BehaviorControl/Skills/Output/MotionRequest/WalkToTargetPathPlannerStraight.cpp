/**
 * @file WalkToTargetPathPlanner.cpp
 *
 * This file implements the implementation of the WalkToTargetPathPlannerStraight skill.
 * This skill allows a robot to move avoiding obstacles and keeping the orientation fixed, facing the opponent goal
 * @author Emanuele Antonioni
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/BehaviorControl/PathPlanner.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Modeling/RobotPose.h"
#include <iostream>

SKILL_IMPLEMENTATION(WalkToTargetPathPlannerStraightImpl,
{,
  IMPLEMENTS(WalkToTargetPathPlannerStraight),
  REQUIRES(LibCheck),
  REQUIRES(PathPlanner),
  REQUIRES(MotionInfo),
  REQUIRES(RobotPose),
  MODIFIES(MotionRequest),
});

class WalkToTargetPathPlannerStraightImpl : public WalkToTargetPathPlannerStraightImplBase
{
  void execute(const WalkToTargetPathPlannerStraight& p) override
  {
    MotionRequest mr = thePathPlanner.plan(p.target,p.speed,false);
      theMotionRequest.motion = mr.motion;
      theMotionRequest.walkRequest.mode = mr.walkRequest.mode;
      theMotionRequest.walkRequest.target = Pose2f(0.f,mr.walkRequest.target.translation.x(), mr.walkRequest.target.translation.y());
      theMotionRequest.walkRequest.speed = Pose2f(-theRobotPose.rotation,mr.walkRequest.speed.translation.x(), mr.walkRequest.speed.rotation.normalize());
      theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
      theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const WalkToTargetPathPlannerStraight&) const override
  {
    return theMotionInfo.motion == MotionRequest::walk;
  }
};

MAKE_SKILL_IMPLEMENTATION(WalkToTargetPathPlannerStraightImpl);
