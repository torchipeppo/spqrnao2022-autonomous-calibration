/**
 * @file WalkToTargetPathPlanner.cpp
 *
 * This file implements the implementation of the WalkToTargetPathPlanner skill.
 * his skill allows a robot to move avoiding obstacles.
 * 
 * @author Emanuele Antonioni
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/BehaviorControl/PathPlanner.h"
#include "Representations/MotionControl/MotionRequest.h"

SKILL_IMPLEMENTATION(WalkToTargetPathPlannerImpl,
{,
  IMPLEMENTS(WalkToTargetPathPlanner),
  REQUIRES(LibCheck),
  REQUIRES(PathPlanner),
  REQUIRES(MotionInfo),
  MODIFIES(MotionRequest),
});

class WalkToTargetPathPlannerImpl : public WalkToTargetPathPlannerImplBase
{
  void execute(const WalkToTargetPathPlanner& p) override
  {
    MotionRequest mr = thePathPlanner.plan(p.target,p.speed,false);
      theMotionRequest.motion = mr.motion;
      theMotionRequest.walkRequest.mode = mr.walkRequest.mode;
      theMotionRequest.walkRequest.target = mr.walkRequest.target;
      theMotionRequest.walkRequest.speed = mr.walkRequest.speed;
      theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
      theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const WalkToTargetPathPlanner&) const override
  {
    return theMotionInfo.motion == MotionRequest::walk;
  }
};

MAKE_SKILL_IMPLEMENTATION(WalkToTargetPathPlannerImpl);
