/**
 * @file StrikerShootCard.cpp
 *
 * Card to approach the ball and shoot.
 *
 * @author Emanuele Musumeci (main) and Francesco Petri (adaptations for C3) (based on Emanuele Antonioni's basic approacher behavior structure)
 */

//disabling warnings while importing so I don't see irrelevant messages when compiling
//these are only meant to be here while I work
//TODO remove asap
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#pragma GCC diagnostic ignored "-Wimplicit-int-float-conversion"
#pragma GCC diagnostic ignored "-Wreorder-ctor"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/BallCarrierModel/BallCarrierModel.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/spqr_representations/PassShare.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Tools/Math/BHMath.h"
#include "Platform/SystemCall.h"
#include <string>

//see above
#pragma GCC diagnostic pop

// set to true to make the striker always kick, disregarding other cards. Useful in desperate cases.
#define BERSERK false

CARD(StrikerShootCard,
{,
  CALLS(Activity),
  CALLS(Say),
  
  CALLS(LookForward),
  CALLS(LookAtPoint),

  CALLS(LookLeftAndRight),

  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(Kick),
  CALLS(WalkToTargetPathPlanner),

  CALLS(SetTarget),

  CALLS(KeyFrameArms),

  CALLS(Approacher2021),

  REQUIRES(FieldBall),
  REQUIRES(BallModel),

  REQUIRES(LibCheck),

  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(BallSpecification),
  REQUIRES(FrameInfo),

  USES(BehaviorStatus),

  REQUIRES(GameInfo),

  // TODO maybe make a cfg, but wait if we have any common params with the others
  DEFINES_PARAMETERS(
  {,
    (int)(50) initialWaitTime,
    (int)(1200) time_to_stand_before_kick,
    (int)(500) time_to_stand_after_kick,
    (float)(230) BALL_KICK_OFFSET_X,
    (float)(50) BALL_KICK_OFFSET_Y,
    //Used to align to the static approach point
    (Rangef)({100.f, 700.f}) approachXRange,
    (Rangef)({-100.f, 100.f}) approachYRange,
    //Used to align very precisely just before kicking
    (Rangef)({210.f, 240.f}) smallApproachXRange,
    (Rangef)({-100.f, -40.f}) smallApproachYRange,
    (Rangef)({-10.f, 10.f}) smallBallAlignmentRange,
    (Rangef)({-5.f, 5.f}) smallerBallAlignmentRange,
    (int)(3000) maxKickWaitTime,
    (int)(10) minKickWaitTime,
    (float)(50.0f) areaIntervalThresholdPre,
    (float)(50.0f) areaIntervalThresholdPost,
  }),
});

class StrikerShootCard : public StrikerShootCardBase
{
  Vector2f chosenTarget = Vector2f(theFieldDimensions.xPosOpponentGoal, 0.f);

  bool preconditions() const override
  {
    if (BERSERK) {
      return true;
    }


    // can only shoot in the opponent half of the field
    if (theRobotPose.translation.x() <= -theFieldDimensions.centerCircleRadius) {
      return false;
    }
    // that said, shoot as soon as you have a targetable area
    FreeGoalTargetableArea targetArea = theLibCheck.goalTargetWithArea(false, false).second;
    return targetArea.interval > areaIntervalThresholdPre;
  }

  // NOTE these pre/post cond.s may fluctuate a bit, but let's wait till we have more cards
  bool postconditions() const override
  {
    if (BERSERK) {
      return false;
    }

    // can only shoot in the opponent half of the field
    if (theRobotPose.translation.x() <= -theFieldDimensions.centerCircleRadius) {
      return true;
    }
    // that said, shoot as soon as you have a targetable area
    FreeGoalTargetableArea targetArea = theLibCheck.goalTargetWithArea(false, false).second;
    return targetArea.interval < areaIntervalThresholdPost;
  }

  option
  {
    theActivitySkill(BehaviorStatus::approachAndKick);

    initial_state(start)
    {
      std::cout<<"APPROACHER2021: start"<<std::endl;
      transition
      {
        if(state_time > initialWaitTime)
        {
          std::cout<<"start -> approach: TIMEOUT"<<std::endl;
          goto approach;
        }
      }

      action
      {
        //theSaySkill("striker shooting");
        theLookForwardSkill();
        theStandSkill();
      }
    }

    // the target can change freely as long as we are away from the ball, and every X seconds once we get close to avoid indecision
    state(approach)
    {
      transition
      {
        Vector2f ballRelative = theFieldBall.positionRelative;
        bool close = approachXRange.isInside(ballRelative.x()) && approachYRange.isInside(ballRelative.y());
        if (close && state_time) {
          goto approach_fixedtarget;
        }
        if (theApproacher2021Skill.isDone()) {
          std::cout<<"approach -> stand_before_kick: DONE"<<std::endl;
          goto stand_before_kick;
        }
      }
      action
      {
        chosenTarget = theLibCheck.goalTarget(false, false);
        theApproacher2021Skill(chosenTarget, 150.f, 50.f);
      }
    }

    // once we get close to the ball, the target stays mostly fixed to avoid indecision. it changes every X seconds.
    state(approach_fixedtarget)
    {
      transition
      {
        Vector2f ballRelative = theFieldBall.positionRelative;
        bool close = approachXRange.isInside(ballRelative.x()) && approachYRange.isInside(ballRelative.y());
        if (!close || state_time > 10000) {
          goto approach;
        }
        if (theApproacher2021Skill.isDone()) {
          std::cout<<"approach -> stand_before_kick: DONE"<<std::endl;
          goto stand_before_kick;
        }
      }
      action
      {
        theApproacher2021Skill(chosenTarget, 230.f, 50.f, 0.8f, 0.5f);
      }
    }

    state(stand_before_kick)
    {
      transition
      {
        if (state_time > time_to_stand_before_kick) {
          std::cout<<"stand_before_kick -> kick: TIMEOUT"<<std::endl;
          goto kick;
        }
      }
      action
      {
        Vector2f chosenRelative = theBallModel.estimate.position;

        theLookAtPointSkill(Vector3f(chosenRelative.x(), chosenRelative.y(), 0.f));
        theStandSkill();
      }
    }

    state(kick)
    {
      transition
      {
        Vector2f chosenRelative = theBallModel.estimate.position;

        float offsetBallYPosition = chosenRelative.y() + BALL_KICK_OFFSET_Y;

        Vector2f ballPositionGlobal = theLibCheck.rel2Glob(chosenRelative.x(), chosenRelative.y()).translation;

        //Refer to the drawing BallCarriern Alignment chart for case letters:
        //https://docs.google.com/drawings/d/1FcS2yrCbGkUmbWM1GRGHuXTYnEhcrbkovdwFayTGWkc/edit?usp=sharing

        //Case 0: if the robot is too far away from the ball, just walkToBall
        if(theLibCheck.distance(theRobotPose, ballPositionGlobal) >= approachXRange.max)
        {
          std::cout<<"kick -> stand_after_kick: Case 0"<<std::endl;
          goto stand_after_kick;
        }
        
        //Case B: Robot is in the X range but not in the Y range
        if(approachXRange.isInside(chosenRelative.x()))
        {
          if(!approachYRange.isInside(offsetBallYPosition))
          {
            std::cout<<"kick -> approach: Case B"<<std::endl;
            goto approach;
          }
        }
        
        //Case E: Robot not in the X range but is behind the ball and is not in the Y range 
        if(chosenRelative.x() < approachXRange.min)
        {
          if(Rangef(-theBallSpecification.radius*2, theBallSpecification.radius*2).isInside(chosenRelative.x()))
          {
            if(!approachYRange.isInside(offsetBallYPosition))
            {
              std::cout<<"kick -> approach: Case E"<<std::endl;
              goto approach; //go back
            }
          }
        }
        
        //Case D + wrong angle: Robot not in the X range but is behind the ball and is in the Y range but is not aligned (angle) to the ball
        if(chosenRelative.x() < approachXRange.min)
        {
          if(chosenRelative.x() > theBallSpecification.radius*2)
          {
            if(approachYRange.isInside(offsetBallYPosition))
            {
              if(!smallBallAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees()))
              {
                //If it is not aligned to the target first turn to the target, then the conditions will be checked again to perform further necessary alignments
                std::cout<<"kick -> approach: Case D + wrong angle"<<std::endl;
                goto approach;
              }
            }
          }
        }
        
        //Case F: Robot is between the target and the ball
        if(chosenRelative.x() < -theBallSpecification.radius*2)
        {
          if(approachYRange.isInside(offsetBallYPosition))
          {
            std::cout<<"kick -> approach: Case F"<<std::endl;
            goto approach;
          }
        }
        
        //Case G: the robot is between the target and the ball and to the side wrt the ball
        if(chosenRelative.x() < -theBallSpecification.radius*2)
        {
          if(!approachYRange.isInside(offsetBallYPosition))
          {  
            std::cout<<"kick -> approach: Case G"<<std::endl;
            goto approach;
          }
        }

        if(state_time > maxKickWaitTime)
        {  
          std::cout<<"kick -> stand_after_kick: TIMEOUT"<<std::endl;
          goto stand_after_kick;
        }

        if(state_time > minKickWaitTime && theKickSkill.isDone()){
          std::cout<<"kick -> stand_after_kick: kick done"<<std::endl;
          goto stand_after_kick;
        }
      }

      action
      {
        Vector2f chosenRelative = theBallModel.estimate.position;

        theLookAtPointSkill(Vector3f(chosenRelative.x(), chosenRelative.y(), 0.f));

        float distance = 9999.f;    // shoot as strong as possible
        theKickSkill(false, distance, false); // parameters: (kick_type, mirror, distance, armsFixed)
        
        theSetTargetSkill(chosenTarget);
      }
    }

    state (stand_after_kick)
    {
      transition
      {
        if (state_time > time_to_stand_after_kick) {
          std::cout<<"time_to_stand_after_kick -> approach: TIMEOUT"<<std::endl;
          goto approach;
        }
      }
      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }
  }

  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }

};

MAKE_CARD(StrikerShootCard);
