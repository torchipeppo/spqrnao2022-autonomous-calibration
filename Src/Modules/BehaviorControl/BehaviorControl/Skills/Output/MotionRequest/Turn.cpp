/**
 * @file Turn.cpp
 *
 * This file implements the implementation of the Turn skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"
#include "Representations/Modeling/BallModel.h"
// #include ""
SKILL_IMPLEMENTATION(TurnImpl,
{,
  IMPLEMENTS(Turn),
  CALLS(Stand),
  CALLS(LookForward),
  CALLS(WalkAtRelativeSpeed),
  CALLS(LookAtPoint),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  REQUIRES(BallModel),

  // REQUIRES(WalkAtAbsoluteSpeed),
  MODIFIES(MotionRequest),
//   DEFINES_PARAMETERS(
//   {,
//     (float)(0.8f) angle,
//     (Pose2f) relTarget,
//     }),
});

class TurnImpl : public TurnImplBase
{
 
   option(Turn)
  {
     float angle = theLibCheck.angleToTarget(p.target.translation.x(), p.target.translation.y());
     Pose2f relTarget = theLibCheck.glob2Rel(p.target.translation.x(), p.target.translation.y());
    DECLARE_DEBUG_DRAWING("representation:ObstacleModel:rectangle", "drawingOnField");

    LINE("representation:ObstacleModel:rectangle", 0, 0 , relTarget.translation.x(), relTarget.translation.y(), 20, Drawings::solidPen, ColorRGBA::black);

    initial_state(start)
    {
        transition
        {   
            
            
            if(std::abs(angle) > Angle::fromDegrees(3.f)){
                goto turnToTarget;
            }else{
                goto targetState;
            }
            
        }
        action
        {   
            // lookAtBall();
            //std::cout<<"1"<<std::endl;
            // Stand();
           theStandSkill();
           theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0));        //   lookAtBall();

        }
    }

    state(turnToTarget)
    {
        transition
        {
            if(std::abs(angle) < Angle::fromDegrees(3.f)){
                
                goto targetState;
            }

        }
        action
        {   
            
            if(angle < 0){
                
                theWalkAtRelativeSpeedSkill(Pose2f(-1.f,0.001f,0.001f));
            }else{
                theWalkAtRelativeSpeedSkill(Pose2f(1.f,0.001f,0.001f));
            }
        }
    }


    
    target_state(targetState){
        
    }

}
//     void execute(const TurnImpl& p)// override
//   {
//       p.targ
//     // theMotionRequest.motion = MotionRequest::walk;
//     // theMotionRequest.walkRequest.mode = WalkRequest::relativeSpeedMode;
//     // theMotionRequest.walkRequest.speed = p.speed;
//     // theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
//     // theLibCheck.inc(LibCheck::motionRequest);
//   }
};

MAKE_SKILL_IMPLEMENTATION(TurnImpl);
