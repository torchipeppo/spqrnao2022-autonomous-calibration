/**
 * @file WalkQuarterCircleDownLeft.cpp
 *
 * This file implements the implementation of the WalkQuarterCircleDownLeft skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"
#include "Representations/Modeling/RobotPose.h"

// #include ""
SKILL_IMPLEMENTATION(WalkQuarterCircleDownLeftImpl,
{,
  IMPLEMENTS(WalkQuarterCircleDownLeft),
  CALLS(Stand),
  CALLS(LookForward),
  CALLS(WalkAtRelativeSpeed),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
    REQUIRES(RobotPose),

  // REQUIRES(WalkAtAbsoluteSpeed),
  MODIFIES(MotionRequest),
});

class WalkQuarterCircleDownLeftImpl : public WalkQuarterCircleDownLeftImplBase
{
   option(WalkQuarterCircleDownLeft)//, (const float&) TurnRate)
  {
 initial_state(start){
        transition{
            goto turning;
        }
    }
    state(turning)
    {
        transition
        {
            if(theRobotPose.rotation.toDegrees() < -84.5 && theRobotPose.rotation.toDegrees() > -95.5)
                goto moving;
        }

        action
        {
            if(theRobotPose.rotation.toDegrees() >= -89.5 && theRobotPose.rotation.toDegrees() <= 90)
                theWalkAtRelativeSpeedSkill(Pose2f(-1,0,0));
            else
                theWalkAtRelativeSpeedSkill(Pose2f(1,0,0));
        }
    }
    state(moving)
    {
        transition
        {
            if(theRobotPose.rotation.toDegrees() > 175 || theRobotPose.rotation.toDegrees() < -175)
                goto targetState;
        }
        action
        {
            theWalkAtRelativeSpeedSkill(Pose2f(-p.TurnRate,1,0));
        }
    }

    target_state(targetState)
    {
        transition
        {
            ;
        }
        action
        {
            theStandSkill();
        }
    }
}
};

MAKE_SKILL_IMPLEMENTATION(WalkQuarterCircleDownLeftImpl);
