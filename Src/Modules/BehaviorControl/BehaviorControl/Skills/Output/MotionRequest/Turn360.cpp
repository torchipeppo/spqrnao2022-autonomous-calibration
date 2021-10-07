/**
 * @file Turn360.cpp
 *
 * This file implements the implementation of the Turn360 skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

// #include ""
SKILL_IMPLEMENTATION(Turn360Impl,
{,
  IMPLEMENTS(Turn360),
  CALLS(Stand),
  CALLS(LookForward),
  CALLS(WalkAtRelativeSpeed),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  // REQUIRES(WalkAtAbsoluteSpeed),
  MODIFIES(MotionRequest),
});

class Turn360Impl : public Turn360ImplBase
{
   option(Turn360)
  {
  initial_state(zero)
    {
        transition
        {
            if(state_time > 5450)//9400)
                goto end;
        }
        action 
        {
            
            theWalkAtRelativeSpeedSkill(Pose2f(1.f, 0.0001f,0.0001f));
            theLookForwardSkill();
        }
    }

    target_state(end){
        transition
        {
            //;
        }
        action
        {
            theStandSkill();
            theLookForwardSkill();
        }
    }
}
};

MAKE_SKILL_IMPLEMENTATION(Turn360Impl);
