/**
 * @file Searcher1Card.cpp
 *
 * This file implements a behavior for searching the ball on the field
 *
 * @author Emanuele Antonioni
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/spqr_representations/PassShare.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Role.h"
#include "Tools/Math/BHMath.h"
#include "Platform/SystemCall.h"
#include <string>
CARD(Searcher1Card,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(Stand),
  CALLS(LookLeftAndRight),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(WalkToTargetPathPlanner),
  CALLS(WalkToTargetPathPlannerStraight),
  CALLS(WalkToApproach),
  
  REQUIRES(PassShare),
  REQUIRES(FieldBall),
  REQUIRES(LibCheck),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(Role),
  REQUIRES(GameInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (float)(400.f) positionTh,
    (float)(300.f) smallPositionTh,
    
  }),
});

class Searcher1Card : public Searcher1CardBase
{
    
  
  bool preconditions() const override
  {
    return theRole.role == Role::searcher_1;
  }

  bool postconditions() const override
  {
    return true;
  }

  option
  {
    theActivitySkill(BehaviorStatus::searcher1);

    initial_state(start)
    {
      transition
      {
        goto walkToPoint_far;
         
      }

      action
      {
        theLookLeftAndRightSkill();
        theStandSkill();
      }
    }

    state(walkToPoint_far)
    {
      transition
      {
        
       
        if(theLibCheck.distance(theRobotPose, Pose2f(theFieldDimensions.xPosOwnGroundline + 1000.f, -500.f)) <= smallPositionTh){
            goto walkToPoint_near;
        }
      }

      action
      {
        theLookLeftAndRightSkill();
        theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f), Pose2f(theFieldDimensions.xPosOwnGroundline + 1000.f, -1000.f));
        
      }
    }

    state(walkToPoint_near){
      transition{
         if(theLibCheck.distance(theRobotPose, Pose2f(theFieldDimensions.xPosOwnGroundline + 1000.f, -500.f)) <= 300){
            if(std::abs(theRobotPose.rotation.toDegrees()) < 10 ){
                goto lookAround;
            }
            
        }
      }
      action{
        theLookLeftAndRightSkill();
        theWalkToTargetPathPlannerStraightSkill(Pose2f(1.f,1.f,1.f), Pose2f(theFieldDimensions.xPosOwnGroundline + 1000.f, -1000.f));
      }
    }

    state(lookAround){
        transition{

        }action{
            theLookLeftAndRightSkill();
            theStandSkill();
        }
    }

  }
   Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }

  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }
};

MAKE_CARD(Searcher1Card);
