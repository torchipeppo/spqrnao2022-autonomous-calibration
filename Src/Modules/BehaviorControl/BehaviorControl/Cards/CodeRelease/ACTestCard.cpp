/**
 * @file ACTestCard.cpp
 * 
 * Card that defines the behavior for testing an autonomous calibration that's already been performed.
 *
 * @author Francesco Petri, Amila Sikalo
 */

//disabling warnings while importing so I don't see irrelevant messages when compiling
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#pragma GCC diagnostic ignored "-Wimplicit-int-float-conversion"
#pragma GCC diagnostic ignored "-Wreorder-ctor"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#pragma GCC diagnostic ignored "-Wenum-enum-conversion"
#pragma GCC diagnostic ignored "-Wenum-float-conversion"

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"

//see above
#pragma GCC diagnostic pop

CARD(ACTestCard,
{,
  REQUIRES(FieldBall),
  REQUIRES(RobotPose),
  REQUIRES(FieldDimensions),

  CALLS(Activity),
  CALLS(Say),

  CALLS(LookLeftAndRight),
  CALLS(LookAtPoint),
  CALLS(LookForward),

  CALLS(Stand),
  CALLS(WalkToTargetPathPlanner),
});

#define DISTANCE_TO_BALL_THRESHOLD 1000.0f
#define REPORT_BALL_IN_GLOBAL true

#define WALK_TO_GOAL_THRESHOLD 50.0f     // millimeters
#define OPPONENT_GOAL Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.0f)

class ACTestCard : public ACTestCardBase
{


  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return false;
  }

  bool spoken = false;

  option
  {

    initial_state(start)
    {
      transition
      {
        if (theFieldBall.ballWasSeen()) {
          spoken = false;
          goto walk_to_ball;
        }
      }

      action
      {
        // framework wants an Activity at all times, as well as motion directives for both head and body
        theActivitySkill(BehaviorStatus::unknown);
        theLookLeftAndRightSkill();
        theStandSkill();
        if (!spoken) {
          theSaySkill("Here I go");
          spoken = true;
        }
      }
    }

    state(walk_to_ball) {
      transition
      {
        if (theFieldBall.positionRelative.norm() < DISTANCE_TO_BALL_THRESHOLD) {
          goto report_ball;
        }
      }
      action
      {
        theActivitySkill(BehaviorStatus::unknown);
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.0f));
        theWalkToTargetPathPlannerSkill(Pose2f(0.7f, 0.7f, 0.7f), Pose2f(theFieldBall.positionOnField.x(), theFieldBall.positionOnField.y()));
      }
    }

    state(report_ball)
    {
      transition
      {
        // TODO doesn't seem to transition? investigate next time
        if (state_time > 1000) {
          spoken = false;
          goto walk_to_goal;
        }
      }
      action
      {
        theActivitySkill(BehaviorStatus::unknown);
        Vector3f ball(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.0f);
        theLookAtPointSkill(ball);
        theStandSkill();
        std::string msg = "The ball is at ";
        if (REPORT_BALL_IN_GLOBAL) {
          msg += std::to_string(theFieldBall.positionOnField.x());
          msg += " x, ";
          msg += std::to_string(theFieldBall.positionOnField.y());
          msg += " y";
        }
        else {
          msg += std::to_string(theFieldBall.positionRelative.x());
          msg += " x, ";
          msg += std::to_string(theFieldBall.positionRelative.y());
          msg += " y";
        }
        if (!spoken) {
          theSaySkill(msg);
          spoken = true;
        }
      }
    }

    state(walk_to_goal) {
      transition
      {
        if ((theRobotPose.translation - OPPONENT_GOAL).norm() < WALK_TO_GOAL_THRESHOLD) {
          goto report_goal;
        }
      }
      action
      {
        theActivitySkill(BehaviorStatus::unknown);
        theLookForwardSkill();
        theWalkToTargetPathPlannerSkill(Pose2f(0.7f, 0.7f, 0.7f), Pose2f(OPPONENT_GOAL.x(), OPPONENT_GOAL.y()));
      }
    }

    state(report_goal)
    {
      transition
      {
        // TODO doesn't seem to transition? investigate next time
        if (state_time > 1000) {
          spoken = false;
          goto sleep;
        }
      }
      action
      {
        theActivitySkill(BehaviorStatus::unknown);
        theLookForwardSkill();
        theStandSkill();
        if (!spoken) {
          theSaySkill("I think I got to the goal");
          spoken = true;
        }
      }
    }

    state(sleep)
    {
      transition
      {
        ;
      }
      action
      {
        theActivitySkill(BehaviorStatus::unknown);
        theLookForwardSkill();
        theStandSkill();
      }
    }

  }

};

MAKE_CARD(ACTestCard);
