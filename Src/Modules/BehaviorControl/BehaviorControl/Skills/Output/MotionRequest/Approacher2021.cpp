/**
 * @file Approacher2021.cpp
 *
 * Implementation of the approaching behavior employed in Challenge 3
 * of RoboCup 2021. (formerly called ChallengeThreeApproach.cpp)
 * 
 * @author Francesco Petri, Fabian Fonseca, Emanuele Musumeci (base structure)
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/Configuration/BallSpecification.h"

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Math/BHMath.h"
#include <iostream>

#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"



#define DEBUG_PRINT(message) if (USE_DEBUG_PRINTS) {std::cout<<message<<std::endl;}

//if these's any need to change range amplitude,
//you may turn any or all of the constants here into cfg parameters.
#define STATIC_APPROACH_RADIUS (p.offsetX + 50.0)
#define smallApproachXRange (Rangef{p.offsetX-15.f, p.offsetX+5.f})
#define smallApproachYRange (Rangef{-p.offsetY-50.f, -p.offsetY+10.f})


SKILL_IMPLEMENTATION(Approacher2021Impl,
{,
  IMPLEMENTS(Approacher2021),
  REQUIRES(LibCheck),
  REQUIRES(FieldBall),
  REQUIRES(RobotPose),
  REQUIRES(BallModel),
  REQUIRES(BallSpecification),
  
  CALLS(Activity),

  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(LookLeftAndRight),

  CALLS(Stand),

  CALLS(WalkToTarget),
  CALLS(WalkToTargetPathPlanner),
  CALLS(WalkAtRelativeSpeed),

  CALLS(SetTarget),

  CALLS(Kick),

  //TODO: check back here and remove any unused parameters
  //      same for the functions below
  //      also remove params from the cfg
  LOADS_PARAMETERS(
  {,
    (int) initialWaitTime,
    (int) realignmentTimeout,

    //Used to align to the static approach point
    (Rangef) approachXRange,
    (Rangef) approachYRange,

    (Rangef) smallBallAlignmentRange,

    //The smaller angle range is used when aligning precisely just before the kick 
    (Rangef) smallerBallAlignmentRange,

    (bool) USE_DEBUG_PRINTS,

    (float) walkToBallFar_dontLookAround_squareRadius,
    (unsigned char) seenPercentageThreshold,
  }),
});

class Approacher2021Impl : public Approacher2021ImplBase
{
  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }
  
  Pose2f computeApproachPoint(float radius, float angle)
  {
    Vector2f ballPositionGlobal = theFieldBall.positionOnField;
    float dynamicOffsetX = (float) (radius * cos(pi + angle));
    float dynamicOffsetY = (float) (radius * sin(pi + angle));
    return Pose2f(-angle, ballPositionGlobal.x() + dynamicOffsetX, ballPositionGlobal.y() - dynamicOffsetY);
  }

  // check if we are currently seeing the ball
  bool canSeeBall() {
    return theBallModel.seenPercentage >= seenPercentageThreshold;
  }







  option(Approacher2021)
  {
    initial_state(start)
    {
      DEBUG_PRINT("APPROACH_AND_KICK: start")
      transition
      {
        if(state_time > initialWaitTime)
        {
          DEBUG_PRINT("start -> walkToBall: TIMEOUT")
          goto walkToBall;
        }
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(turnToBall)
    {
      transition
      {        
        if(smallBallAlignmentRange.isInside(calcAngleToTarget(p.target.translation).toDegrees()))
        {
          DEBUG_PRINT("turnToBall -> walkToBall: aligned to ball")
          goto walkToBall;
        }
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(p.normalWalkSpeed, p.normalWalkSpeed, p.normalWalkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
      }
    }

    state(walkToBall)
    {
      transition
      {
        Vector2f ballRelative = theFieldBall.positionRelative;
        float offsetBallYPosition = ballRelative.y() + p.offsetY;

        //if the LOCAL ball is inside the approach on the x axis 
        //AND the LOCAL ball is inside the approach range on the y axis 
        //AND the robot is aligned with the target
        if(approachXRange.isInside(ballRelative.x())
        && approachYRange.isInside(ballRelative.y())
        && smallerBallAlignmentRange.isInside(calcAngleToTarget(p.target.translation).toDegrees()))
        {
          DEBUG_PRINT("walkToBall -> approachToKick: OK to approach")
          goto approachToKick;
        }

        //NEWLY ADDED
        if(ballRelative.norm() < approachXRange.max
        && approachXRange.isInside(ballRelative.x())
        && !approachYRange.isInside(offsetBallYPosition)
        && theRobotPose.translation.x() < theFieldBall.positionOnField.x())
        {
          //Case B: Robot is in the X range but not in the Y range
          DEBUG_PRINT("walkToBall -> secondStepAlignment: kick Case B")
          goto secondStepAlignment;
        }
      }

      action
      {
        Vector2f ballRelative = theFieldBall.positionRelative;

        if (!p.allowLookAround || ballRelative.squaredNorm() <= walkToBallFar_dontLookAround_squareRadius) {
          theLookAtPointSkill(Vector3f(ballRelative.x(), ballRelative.y(), 0.f));
        }
        else {
          theLookLeftAndRightSkill();
        }

        float ballToTargetAngle = theLibCheck.angleBetweenPoints(theFieldBall.positionOnField, p.target.translation);
        Pose2f staticApproachPoint = computeApproachPoint(STATIC_APPROACH_RADIUS, ballToTargetAngle);
        theWalkToTargetPathPlannerSkill(Pose2f(p.normalWalkSpeed, p.normalWalkSpeed, p.normalWalkSpeed), staticApproachPoint);
      }
    }

    state(firstStepAlignment)
    {
      transition
      {
        if(theFieldBall.positionRelative.x() > theBallSpecification.radius*2)
        {
          DEBUG_PRINT("firstStepAlignment -> secondStepAlignment")
          goto secondStepAlignment;
        }
      }
      
      action
      {
        Vector2f ballRelative = theFieldBall.positionRelative;
        Vector2f globalBall = theFieldBall.positionOnField;

        if (!p.allowLookAround || canSeeBall()) {
          theLookAtPointSkill(Vector3f(ballRelative.x(), ballRelative.y(), 0.f));
        }
        else {
          theLookLeftAndRightSkill();
        }

        theWalkAtRelativeSpeedSkill(Pose2f(theLibCheck.angleToTarget(globalBall.x(), globalBall.y()), -1.0, 0.0));
      }
    }

    state(secondStepAlignment)
    {
      transition
      {
        Vector2f ballRelative = theFieldBall.positionRelative;

        if(ballRelative.norm() > approachXRange.max)
        {
          DEBUG_PRINT("secondStepAlignment -> walkToBall: out of range")
          goto walkToBall;
        }

        float offsetBallYPosition = ballRelative.y() + p.offsetY;

        if(approachXRange.isInside(ballRelative.x())
        && approachYRange.isInside(offsetBallYPosition)
        && smallBallAlignmentRange.isInside(Angle(theLibCheck.angleToTarget(p.target.translation.x(), p.target.translation.y())).toDegrees()))
        {
          DEBUG_PRINT("secondStepAlignment -> approachToKick: aligned to ball")
          goto approachToKick;
        }

        //NEWLY ADDED
        if(state_time > realignmentTimeout)
        {
          DEBUG_PRINT("secondStepAlignment -> approachToKick: TIMEOUT")
          goto approachToKick;
        }

//BELOW copied from the KICK state
        //Case E: Robot not in the X range but is behind the ball and is not in the Y range 
        if(ballRelative.x() < approachXRange.min)
        {
          if(Rangef(-theBallSpecification.radius*2, theBallSpecification.radius*2).isInside(ballRelative.x()))
          {
            if(!approachYRange.isInside(offsetBallYPosition))
            {
              DEBUG_PRINT("secondStepAlignment -> firstStepAlignment: Case E")
              goto firstStepAlignment; //go back
            }
          }
        }
        
        //Case F: Robot is between the target and the ball
        if(ballRelative.x() < -theBallSpecification.radius*2)
        {
          if(approachYRange.isInside(offsetBallYPosition))
          {
            DEBUG_PRINT("secondStepAlignment -> walkToBall: Case F")
            goto walkToBall;
          }
        }
        
        //Case G: the robot is between the target and the ball and to the side wrt the ball

        if(ballRelative.x() < -theBallSpecification.radius*2)
        {
          if(!approachYRange.isInside(offsetBallYPosition))
          {
            DEBUG_PRINT("secondStepAlignment -> firstStepAlignment: Case G")
            goto firstStepAlignment;
          }
        }
      }

      action
      {
        if (!p.allowLookAround || canSeeBall()) {
          theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        }
        else {
          theLookLeftAndRightSkill();
        }

        float ballToTargetAngle = theLibCheck.angleBetweenPoints(theFieldBall.positionOnField, p.target.translation);

        Vector2f globalApproachPoint = computeApproachPoint(STATIC_APPROACH_RADIUS, ballToTargetAngle).translation;
        Vector2f relativeApproachPoint = theLibCheck.glob2Rel(globalApproachPoint.x(), globalApproachPoint.y()).translation;

        Pose2f approachPoint = Pose2f(theLibCheck.angleToTarget(p.target.translation.x(), p.target.translation.y()), relativeApproachPoint.x() - p.offsetX, relativeApproachPoint.y() + p.offsetY);

        theWalkToTargetSkill(Pose2f(p.normalWalkSpeed, p.normalWalkSpeed, p.normalWalkSpeed), approachPoint);
      }
    }

    //Approach the ball precisely (last robot movements before the kick)
    state(approachToKick){
      transition
      {
        Vector2f ballRelative = theFieldBall.positionRelative;

        float offsetBallYPosition = ballRelative.y() + p.offsetY;

        if(!approachXRange.isInside(ballRelative.x())
        || !approachYRange.isInside(offsetBallYPosition)
        || !smallBallAlignmentRange.isInside(Angle(theLibCheck.angleToTarget(p.target.translation.x(), p.target.translation.y())).toDegrees()))
        {
          DEBUG_PRINT("approachToKick -> secondStepAlignment: disaligned")
          goto secondStepAlignment;
        }

        if(smallApproachXRange.isInside(ballRelative.x())
           && smallApproachYRange.isInside(ballRelative.y()))
        {
            DEBUG_PRINT("approachToKick -> ready_to_kick: ready to kick")
            //goto stand_before_kick;
            goto ready_to_kick;
        }
      }

      action
      {
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        
        Pose2f ball = theFieldBall.positionRelative;
        float x_ball = ball.translation.x();         
        float y_ball = ball.translation.y();         

        x_ball = x_ball - p.offsetX;
        y_ball = y_ball + p.offsetY;

        Pose2f localTarget = Pose2f(theLibCheck.angleToTarget(p.target.translation.x(), p.target.translation.y()), x_ball, y_ball);
        theWalkToTargetSkill(Pose2f(p.slowerWalkSpeed, p.slowerWalkSpeed, p.slowerWalkSpeed), localTarget);
        
      }
    }

    target_state(ready_to_kick)
    {
      transition
      {
        ;
        //TODO for the future: might it make sense to add a transition like
        //"if the ball is moved away, go back to start"
        //so if anything happens we don't just stand like dorks
        //waiting for the caller to notice?
      }
      action
      {
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        theStandSkill();
      }
    }
  }
};

MAKE_SKILL_IMPLEMENTATION(Approacher2021Impl);
