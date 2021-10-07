/**
 * @file Searcher3Card.cpp
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
#include "Representations/Modeling/BallModel.h"
#include "Platform/SystemCall.h"
#include <string>
CARD(Searcher3Card,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(WalkToTargetPathPlanner),
  CALLS(WalkToTargetPathPlannerStraight),
  CALLS(WalkToApproach),
  CALLS(LookLeftAndRight),
  CALLS(LookRightAndLeft),
  CALLS(ParametricLookLeftAndRight),
  
  REQUIRES(PassShare),
  REQUIRES(FieldBall),
  REQUIRES(LibCheck),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(BallModel),
  REQUIRES(Role),
  REQUIRES(GameInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (Angle)(5_deg) ballAlignThreshold,
    
  }),
});

class Searcher3Card : public Searcher3CardBase
{
    
  
  bool preconditions() const override
  {
    return theRole.role == Role::searcher_3;
  }

  bool postconditions() const override
  {
    return true;
  }

  option
  {
    theActivitySkill(BehaviorStatus::searcher3);

    initial_state(start)
    {
      transition
      {
        goto turnToOldBall;
         
      }

      action
      {
        
      }
    }

    state(turnToOldBall)
    {
      transition
      {
        
        if(std::abs(theBallModel.lastPerception.angle()) < ballAlignThreshold)
          goto walkToOldBall;
          
      }

      action
      {
        theLookLeftAndRightSkill();
        //theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theBallModel.lastPerception.angle(), 0.f, 0.f));
        float walkSpeed;
        if(theBallModel.lastPerception.angle() > 0)
        {
          walkSpeed = 1.f;
        }
        else
        {
          walkSpeed = -1.f;
        }
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }

    state(walkToOldBall)
    {
      transition
      {
        Pose2f globBall = theLibCheck.rel2Glob(theBallModel.lastPerception.x(), theBallModel.lastPerception.y());
        if(state_time > 8000 || theLibCheck.distance(theRobotPose.translation, globBall) < 600.f)
          goto turnAround;
          
      }

      action
      {
        theLookLeftAndRightSkill();
        Pose2f globBall = theLibCheck.rel2Glob(theBallModel.lastPerception.x(), theBallModel.lastPerception.y());
        theWalkToTargetPathPlannerSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 
            std::max(std::min(globBall.translation.x(), theFieldDimensions.xPosOpponentGroundline), theFieldDimensions.xPosOwnGroundline), 
            std::max( std::min(globBall.translation.y(),  theFieldDimensions.yPosRightFieldBorder ), theFieldDimensions.yPosLeftFieldBorder)));
        
      }
    }

    state(turnAround)
    {
      transition
      {
          
      }

      action
      {
        if(theRobotPose.translation.y() > 0)
        {
          theLookRightAndLeftSkill();
          theWalkAtRelativeSpeedSkill(Pose2f(-0.5f,0.f,0.f));
        }
        else
        {
          theLookLeftAndRightSkill();
          theWalkAtRelativeSpeedSkill(Pose2f(0.5f,0.f,0.f));
        }
      }
    }
  }

  
};

MAKE_CARD(Searcher3Card);
