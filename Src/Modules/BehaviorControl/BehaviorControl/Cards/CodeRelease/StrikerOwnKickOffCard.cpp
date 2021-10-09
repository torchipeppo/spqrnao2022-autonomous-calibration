/**
 * @file StrikerOwnKickOffCard.cpp
 *
 * Handle own kickoff by passing to the jolly
 *
 * @author Francesco Petri
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
#include "Representations/Communication/TeamData.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Settings.h"
#include "Platform/SystemCall.h"
#include <string>

//see above
#pragma GCC diagnostic pop

CARD(StrikerOwnKickOffCard,
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

  REQUIRES(TeamData),
  REQUIRES(PassShare),
  REQUIRES(GameInfo),

  USES(BehaviorStatus),


  LOADS_PARAMETERS(
    {,
    (float) ballThresholdPre,
    (float) ballThresholdPost,
    
    // TODO maybe make a cfg, but wait if we have any common params with the others
    // Coming from the Striker pass card
    (int) initialWaitTime,
    //Used to align to the static approach point
    //Used to align very precisely just before kicking
    (Rangef) smallApproachXRange,
    (Rangef) smallApproachYRange,
    (Rangef) smallBallAlignmentRange,
    (Rangef) smallerBallAlignmentRange,
    (int) maxKickWaitTime,
    (int) minKickWaitTime,

    (Rangef) approachXRange,
    (Rangef) approachYRange,
    (int) time_to_stand_before_kick,
    (int) time_to_stand_after_kick,
    (float) BALL_KICK_OFFSET_X,
    (float) BALL_KICK_OFFSET_Y,

    (float) INITIAL_PASS_TARGET_X,
    (float) INITIAL_PASS_TARGET_Y,

    (float) OPPONENT_TOO_NEAR_FOR_KICKOFF_THRESHOLD,

    (bool) ENABLE_ADVANCED_KICKOFF,
  }),
});

class StrikerOwnKickOffCard : public StrikerOwnKickOffCardBase
{
  bool kickoff_completed = false;

  bool preconditions() const override
  {
    // check that it is us who are kicking off...
    return ENABLE_ADVANCED_KICKOFF &&
        getJollyIndex() > -1 &&
        theGameInfo.kickingTeam == Global::getSettings().teamNumber &&
        theGameInfo.secsRemaining > (600-9);

        // ...and check the kickoff conditions
        // theLibCheck.isKickoffStriker(ballThresholdPre);
  }

  bool postconditions() const override
  { 
    // std::cout << "secsRemaining end half" << theGameInfo.secsRemaining << std::endl;
    // exit if the kicking team changes...
    return theGameInfo.kickingTeam != Global::getSettings().teamNumber ||
        // ...or the kickoff conditions are no longer true...
        // !theLibCheck.isKickoffStriker(ballThresholdPost) ||
        // ...or the timeout has passed
        //theGameInfo.secsRemaining < (600-9);
        theLibCheck.distance(theLibCheck.nearestOpponent(), theRobotPose) < OPPONENT_TOO_NEAR_FOR_KICKOFF_THRESHOLD ||
        kickoff_completed;
  }

  int getJollyIndex() const {
      for(unsigned i = 0; i < theTeamData.teammates.size(); i++){
          if(theTeamData.teammates.at(i).role == Role::RoleType::jolly){
              return i;
          }
      }
      return -1;
  }

  //NOT USED -> We're using a custom target specified in the cfg
  Vector2f chooseTarget() const {
    int jolly_index = getJollyIndex();
    if (jolly_index >-1){
      Teammate jolly = theTeamData.teammates.at(jolly_index);
      Vector2f jollyFinalPositionGuess = Vector2f(INITIAL_PASS_TARGET_X, jolly.theRobotPose.translation.y());
      int jolly_y_sign = jolly.theRobotPose.translation.y() >=0 ? 1 : -1;
      return jollyFinalPositionGuess + Vector2f(0, -jolly_y_sign*0);
    }else{
      return Vector2f(INITIAL_PASS_TARGET_X, INITIAL_PASS_TARGET_Y);
    }
  }

  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }

  // float jolly_y = theFieldDimensions.yPosRightFieldBorder - 1000;  // this value is out of the field and represents "unknown"
  int jolly_index = -1;
  Vector2f chosenTarget;

  option
  {
    theActivitySkill(BehaviorStatus::codeReleasePositionForKickOff);

    initial_state(wait_for_jolly_to_appear_in_teammates)
    {
      // std::cout<<"own kickoff: wait_for_jolly_to_appear_in_teammates"<<std::endl;
      transition
      {
        if (jolly_index != -1) {
          goto early_approach;
        }
      }

      action
      {
        theSaySkill("striker ready for kick off");
        theLookForwardSkill();
        theStandSkill();
        jolly_index = getJollyIndex();
      }
    }

    // prepare for a preliminary approach while the jolly positions itself. the assumed target is based on a guess of where the jolly will end u positioning itself.
    state(early_approach)
    {
      transition
      {
        if (option_time > 7000) {
          jolly_index = getJollyIndex();
          if (jolly_index != -1) {
            Teammate jolly = theTeamData.teammates.at(jolly_index);
            Vector2f ballRelative = theFieldBall.positionRelative;
            bool close = approachXRange.isInside(ballRelative.x()) && approachYRange.isInside(ballRelative.y());
            if (close && state_time && jolly.theRobotPose.translation.x() > 500.f) {
              goto approach_fixedtarget;
            }
          }
          if (theApproacher2021Skill.isDone()) {
            std::cout<<"approach -> stand_before_kick: DONE"<<std::endl;
            goto stand_before_kick;
          }
        }
      }
      action
      {
        // chosenTarget =  thePassShare.passTarget.translation;
        // Consider other fixed positions to pass the ball
        chosenTarget =  chooseTarget();
        //chosenTarget = Vector2f(INITIAL_PASS_TARGET_X, INITIAL_PASS_TARGET_Y);
        theApproacher2021Skill(chosenTarget, 150.f, 50.f);
      }
    }

    state(approach_fixedtarget)
    {
      transition
      {
        Vector2f ballRelative = theFieldBall.positionRelative;
        bool close = approachXRange.isInside(ballRelative.x()) && approachYRange.isInside(ballRelative.y());
        if (!close || state_time > 10000) {
          goto early_approach;
        }
        if (theApproacher2021Skill.isDone()) {
          std::cout<<"approach -> stand_before_kick: DONE"<<std::endl;
          goto stand_before_kick;
        }
      }
      action
      {
        theApproacher2021Skill(chosenTarget, 150.f, 50.f);
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
        std::cout<<"KICKING TOOOOOOOOOOOOOOOOOO; " << chosenTarget << std::endl;

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
            goto early_approach;
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
              goto early_approach; //go back
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
                goto early_approach;
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
            goto early_approach;
          }
        }
        
        //Case G: the robot is between the target and the ball and to the side wrt the ball
        if(chosenRelative.x() < -theBallSpecification.radius*2)
        {
          if(!approachYRange.isInside(offsetBallYPosition))
          {  
            std::cout<<"kick -> approach: Case G"<<std::endl;
            goto early_approach;
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



        float distance = (theRobotPose.translation - chosenTarget).norm();
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
          goto early_approach;
        }
      }
      action
      {
        kickoff_completed = true;
        theLookForwardSkill();
        theStandSkill();
      }
    }
  }
  
};

MAKE_CARD(StrikerOwnKickOffCard);