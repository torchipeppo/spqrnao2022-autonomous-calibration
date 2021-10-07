/**
 * @file ApproachBallOnTheLineCard.cpp
 *
 * This file implements a behavior for approaching a ball which is on the line of the field
 * and has to be kicked inside.
 * 
 * TODO evaluate if keep the "turn" state
 * + modify the card to avoid the Akshay's approacher to walk backwards
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

CARD(ApproachBallOnTheLineCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkAtAbsoluteSpeed),
  CALLS(WalkToBallController),
  CALLS(WalkToTarget),
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

class ApproachBallOnTheLineCard : public ApproachBallOnTheLineCardBase
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
    theActivitySkill(BehaviorStatus::reaching_ball);

    initial_state(start)
    {
      transition
      {
        if(theRole.role == Role::searcher_1 || theRole.role == Role::searcher_2 || theRole.role == Role::searcher_4)
          goto searchForBall;

        float diffAngleToLine = theRobotPose.rotation - pi/2;
        if(theRobotPose.translation.y() < 0)
          diffAngleToLine = theRobotPose.rotation + pi/2;

        if(std::abs(theRobotPose.translation.y()) < theFieldDimensions.yPosLeftSideline - fieldLineInsideOffset)
          goto approach_outside;
        else if (std::abs(theRobotPose.translation.y()) < theFieldDimensions.yPosLeftSideline)
          if(smallRange.isInside(diffAngleToLine))
            goto walk_outside;
          else
            goto turn;
        else
          goto approach;
      }

      action
      {
        theLookAtPointSkill(Vector3f(theFieldBall.teamPositionRelative.x(), theFieldBall.teamPositionRelative.y(), 0.f));
        theStandSkill();
      }
    }

    state(approach_outside)
    {
      transition
      {
        if(theRole.role == Role::searcher_1 || theRole.role == Role::searcher_2 || theRole.role == Role::searcher_4)
          goto searchForBall;


        float diffAngleToLine = theRobotPose.rotation - pi/2;
        if(theRobotPose.translation.y() < 0)
          diffAngleToLine = theRobotPose.rotation + pi/2;

        if(std::abs(theRobotPose.translation.y()) >= theFieldDimensions.yPosLeftSideline - fieldLineInsideOffset) {
          if(smallRange.isInside(diffAngleToLine))
            goto walk_outside;
          else
            goto turn;
        }
          
      }

      action
      {
        Vector2f position = theBallModel.estimate.position;

        //It's the offset on Y w.r.t. the local robot coordinates
        //It corresponds to an offset along the X on the field
        float ballAlignOffsetY = -700;
        
        if((theLibCheck.rel2Glob(position.x(),position.y()).translation.x() > theRobotPose.translation.x() && theRobotPose.translation.y()>0) || 
          (theLibCheck.rel2Glob(position.x(),position.y()).translation.x() <= theRobotPose.translation.x() && theRobotPose.translation.y()<=0)) {
          ballAlignOffsetY = 700;
        }

        theLookAtPointSkill(Vector3f(theFieldBall.teamPositionRelative.x(), theFieldBall.teamPositionRelative.y(), 0.f));
        theWalkAtRelativeSpeedSkill(Pose2f(std::atan2(position.y() + ballAlignOffsetY, position.x()), walkSpeed, 0));
      }
    }

    state(walk_outside) {
      transition
      {
        if(theRole.role == Role::searcher_1 || theRole.role == Role::searcher_2 || theRole.role == Role::searcher_4)
          goto searchForBall;

        if(std::abs(theRobotPose.translation.y()) >= theFieldDimensions.yPosLeftSideline + fieldLineOutsideOffset)
          goto approach;
      }

      action
      {
        theLookAtPointSkill(Vector3f(theFieldBall.teamPositionRelative.x(), theFieldBall.teamPositionRelative.y(), 0.f));
        theWalkAtRelativeSpeedSkill(Pose2f(0.f, walkSpeed, 0.f));
      }
    }

    state(approach)
    {
      transition
      {
        if(theRole.role == Role::searcher_1 || theRole.role == Role::searcher_2 || theRole.role == Role::searcher_4)
          goto searchForBall;
      }

      action
      {
        Pose2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
        Pose2f fromTarget = Vector2f(globalBall.translation.x(), globalBall.translation.y());
        Pose2f toTarget = Vector2f(theFieldDimensions.xPosOpponentGoal, 0.f);
        
        //TODO set a proper flag(?)
        if(theGameInfo.setPlay == SET_PLAY_CORNER_KICK) {
          toTarget = Vector2f(0.f, 0.f);
        }

        theLookAtPointSkill(Vector3f(theFieldBall.teamPositionRelative.x(), theFieldBall.teamPositionRelative.y(), 0.f));
        //Taken the same values as in CarryBallWithController
        //(fromTarget, toTarget, xOffset, yOffset, USE_FOOT_OFFSET && !kickWithRightFoot, gainWhenBallOnSide, kp1, kp2, kp3)
        theWalkToBallControllerSkill(fromTarget, toTarget, 50.f, 0.f, true, 6.f, 0.05f, -0.007f, 0.004f);
      }
    }
    
    //Todo: evaluate if it is worth to take this status
    //evaluate also if it is worth to insert it also in the approach phase
    state(turn)
    {
      transition
      {
        if(theRole.role == Role::searcher_1 || theRole.role == Role::searcher_2 || theRole.role == Role::searcher_4)
          goto searchForBall;

        float diffAngleToLine = theRobotPose.rotation - pi/2;
        if(theRobotPose.translation.y() < 0)
          diffAngleToLine = theRobotPose.rotation + pi/2;

        // Pose2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
        // float angleToBall = theLibCheck.angleToTarget(globalBall.translation.x(), globalBall.translation.y());

        if(std::abs(theRobotPose.translation.y()) >= theFieldDimensions.yPosLeftSideline + fieldLineOutsideOffset)
        //   if(std::abs(angleToBall) < pi/3)
        //     goto approach;
          float ok;
        else if(std::abs(smallRange.isInside(diffAngleToLine)))
          goto walk_outside;
      }

      action
      {
        theLookAtPointSkill(Vector3f(theFieldBall.teamPositionRelative.x(), theFieldBall.teamPositionRelative.y(), 0.f));
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
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

  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }

  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }
};

MAKE_CARD(ApproachBallOnTheLineCard);
