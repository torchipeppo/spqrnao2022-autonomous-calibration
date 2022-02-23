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

//see above
#pragma GCC diagnostic pop

CARD(ACTestCard,
{,
  REQUIRES(FieldBall),

  CALLS(Activity),
  CALLS(Say),

  CALLS(LookLeftAndRight),
  CALLS(LookAtPoint),

  CALLS(Stand),
  CALLS(WalkToTargetPathPlanner),
});

#define DISTANCE_TO_BALL_THRESHOLD 1000.0f
#define REPORT_BALL_IN_GLOBAL true

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

  option
  {

    initial_state(start)
    {
      transition
      {
        if (theFieldBall.ballWasSeen()) {
          goto walk_to_ball;
        }
      }

      action
      {
        // framework wants an Activity at all times, as well as motion directives for both head and body
        theActivitySkill(BehaviorStatus::unknown);
        theLookLeftAndRightSkill();
        theStandSkill();
        theSaySkill("Here I go");
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
        theSaySkill(msg);
      }
    }

    state(walk_to_goal)
    {
      transition
      {
        ;
      }
      action
      {
        theSaySkill("So' stanco");
      }
    }

  }

};

MAKE_CARD(ACTestCard);
