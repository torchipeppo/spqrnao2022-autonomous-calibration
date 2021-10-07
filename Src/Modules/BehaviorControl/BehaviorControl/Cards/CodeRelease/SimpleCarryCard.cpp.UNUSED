#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"

#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"



CARD(SimpleCarryCard,
{,
  CALLS(Stand),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTargetPathPlanner),
  CALLS(WalkToApproach),
  CALLS(Kick),
  CALLS(InWalkKick),

  REQUIRES(RobotPose),
  REQUIRES(FieldBall),
  REQUIRES(LibCheck),

  DEFINES_PARAMETERS(
  {,
    (int)(50) initialWaitTime,
    (int)(1000) ballNotSeenTimeout,
    (float)(300.f) posStartApprX,         // Offset on x axis respect to ball to start approach
    (Rangef)({-18.f, 18.f}) reachedRange, // Range to test if a position is reached
    (float)(360.f) tooFarValueX,                // Value on x axis to detect if the ball is too far to approach
    (float)(100.f) tooFarValueY,                // Value on y axis to detect if the ball is too far to approach
    (float)(10) minKickWaitTime,
    (float)(3000) maxKickWaitTime,

    (float)(145.f) approachOffsetX,
    (float)(-55.f) approachOffsetY,
  }),
});



class SimpleCarryCard : public SimpleCarryCardBase
{
  bool preconditions() const override
  {
    return true;
  }


  bool postconditions() const override
  {
    return true; // std::cout << "I'm HERE" << std::endl; // String saved for debug
  }


  option
  {

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
          goto walkToBall;
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }


    state(walkToBall)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;

        // If the robot has reached the position
        Vector2f target = theFieldBall.positionOnField - Vector2f(posStartApprX, 0.f);
        if(reachedRange.isInside(target.y() - theRobotPose.translation.y()) &&
          reachedRange.isInside(target.x() - theRobotPose.translation.x()))
          goto approach;
      }

      action
      {
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(),
                                     theFieldBall.positionRelative.y(),
                                       0.f));
        theWalkToTargetPathPlannerSkill(Pose2f(0.7f,0.7f,0.7f),
                                        Pose2f(theFieldBall.positionOnField - Vector2f(posStartApprX, 0.f)));
      }
    }


    state(approach)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;

        // If the ball is no more in a good position
        if((theFieldBall.positionOnField.y() - theRobotPose.translation.y()) > tooFarValueY ||
          (theFieldBall.positionOnField.x() - theRobotPose.translation.x()) > tooFarValueX)
          goto walkToBall;

        // If we are in position we can kick the ball
        float targetX = theFieldBall.positionOnField.x()-approachOffsetX;
        float targetY = theFieldBall.positionOnField.y()-approachOffsetY;
        if(reachedRange.isInside(targetX - theRobotPose.translation.x()) &&
           reachedRange.isInside(targetY - theRobotPose.translation.y()))
           goto kick;
      }

      action
      {
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(),
                                     theFieldBall.positionRelative.y(),
                                       0.f));
        theWalkToApproachSkill(theLibCheck.goalTarget(false),
                               approachOffsetX, approachOffsetY, true);
      }
    }


    state(kick)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;

        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theKickSkill.isDone()))
          goto start;
      }

      action
      {
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(),
                                     theFieldBall.positionRelative.y(),
                                       0.f));
        theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::right), Pose2f(Angle::fromDegrees(0.f), theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y()));
            
        
      }
    }


    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto walkToBall;
      }

      action
      {
        theLookForwardSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(0.8f, 0.f, 0.f));
      }
    }

  }


};


MAKE_CARD(SimpleCarryCard);
