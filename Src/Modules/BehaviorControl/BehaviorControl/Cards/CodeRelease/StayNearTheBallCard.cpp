/**
 * @file StayNearTheBallCard.cpp
 *
 * This file implements a behavior for stay between the ball and your goal
 * when the opponents have to do a free kick in 
 * 
 * TODO add a state to slightly move horizontally after reach the position(?)
 *
 * @author Elisa Foderaro
 */


#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Role.h"
#include "Tools/Math/BHMath.h"

CARD(StayNearTheBallCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkAtAbsoluteSpeed),
  CALLS(WalkToBallController),
  CALLS(WalkToTargetPathPlanner),
  CALLS(Kick),
  REQUIRES(FieldBall),
  REQUIRES(BallModel),
  REQUIRES(LibCheck),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(Role),

  REQUIRES(GameInfo),

  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(50) initialWaitTime,
    (int)(3500) ballNotSeenTimeout,
    (float)(500) fieldLineInsideOffset,
    (float)(200) fieldLineOutsideOffset,
    (Rangef)({-0.5f, 0.5f}) smallRange,
    }),
});

class StayNearTheBallCard : public StayNearTheBallCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return false;
  }

  option
  {
    theActivitySkill(BehaviorStatus::debug_standing);

    initial_state(start)
    {
      transition
      {
        if(theRole.role == Role::searcher_1 || theRole.role == Role::searcher_2 || theRole.role == Role::searcher_4)
          goto searchForBall;

        goto approach_position;
      }

      action
      {
        theLookAtPointSkill(Vector3f(theFieldBall.teamPositionRelative.x(), theFieldBall.teamPositionRelative.y(), 0.f));
        theStandSkill();
      }
    }

    state(approach_position)
    {
      transition
      {
        if(theRole.role == Role::searcher_1 || theRole.role == Role::searcher_2 || theRole.role == Role::searcher_4)
          goto searchForBall;
      }

      action
      {
        Pose2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
        Angle targetAngle = Angle(std::atan2(globalBall.translation.y(), globalBall.translation.x()-theFieldDimensions.xPosOwnGoal));
        float distanceFromBall = 1000.f;
        Pose2f targetPos = Pose2f(globalBall.translation.x() - (distanceFromBall*std::cos(targetAngle)),
                                globalBall.translation.y() - (distanceFromBall*std::sin(targetAngle)));

        theLookAtPointSkill(Vector3f(theFieldBall.teamPositionRelative.x(), theFieldBall.teamPositionRelative.y(), 0.f));
        theWalkToTargetPathPlannerSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), targetPos);
      }
    }   
    
    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto start;
      }

      action
      {
        theLookForwardSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }
  }
};

MAKE_CARD(StayNearTheBallCard);
