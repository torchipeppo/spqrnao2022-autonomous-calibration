/**
 * @file GoalieApproachAndKickCard.cpp
 *
 * This file implements a behavior for approaching the ball and kick it to a given target.
 *
 * @author Emanuele Antonioni
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Tools/Math/BHMath.h"

CARD(GoalieApproachAndKickCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(Kick),
  CALLS(WalkToTargetPathPlanner),
  CALLS(WalkToTargetPathPlannerStraight),
  CALLS(WalkToApproach),
  REQUIRES(FieldBall),
  REQUIRES(LibCheck),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(BallModel),
  REQUIRES(TeamBallModel),
  USES(TeamData),

  REQUIRES(GameInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(50) initialWaitTime,
    (int)(3500) ballNotSeenTimeout,
    (Angle)(5_deg) ballAlignThreshold,
    (float)(100.f) ballYThreshold,
    (float)(180.f) ballOffsetX,
    (float)(450.f) ballXnearTh,
    (Rangef)({160.f, 200.f}) ballOffsetXRange,
    (Rangef)({-350.f, 350.f}) approachYRange,
    (Rangef)({-150.f, 150.f}) smallApproachYRange,
    (Rangef)({150.f, 300.f}) smallApproachXRange,
    (float)(45.f) ballOffsetY,
    (Rangef)({35.f, 55.f}) ballOffsetYRange,
    (int)(10) minKickWaitTime,
    (int)(3000) maxKickWaitTime,
    (float)(1000.f) approachTreshold,
    (Angle)(6_deg) angle_target_treshold,
  }),
});

class GoalieApproachAndKickCard : public GoalieApproachAndKickCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    Vector2f velocity=theBallModel.estimate.velocity;
    Vector2f position=theBallModel.estimate.position;

    float myDistanceToBall = theBallModel.estimate.position.norm();
    bool iAmMostNearPlayer = true;
    for(unsigned i = 0; i < theTeamData.teammates.size(); i++){
        if((theTeamData.teammates.at(i).theRobotPose.translation -
                theTeamBallModel.position).norm() < (myDistanceToBall+400.f)){
            iAmMostNearPlayer = false;
        }
    }

    return (velocity.x() < 0 && velocity.norm()/position.norm() >= 0.7)                                     // conditions to dive in Goalie2020 common transition, TODO: add other conditions not to dive outside area
    || ( ( ((int)theGameInfo.setPlay != SET_PLAY_GOAL_FREE_KICK && !theLibCheck.isGoalieInKickAwayRange     // conditions to exit from goalieKickAway state in Goalie2020
          && (theBallModel.estimate.position.norm() > 175 && !iAmMostNearPlayer)) ||
          (!theLibCheck.isBallInKickAwayRange && !iAmMostNearPlayer) ) || (theLibCheck.timeSinceBallWasSeen > 1000)); 
    // OBS: in 2019 version, there was also condition action_done
  }

  option
  {
    theActivitySkill(BehaviorStatus::approachAndKick);

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
          goto turnToBall;
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
        
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(theFieldBall.positionRelative.angle()) < ballAlignThreshold)
          goto walkToBall_far;
          
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
        
      }
    }

    state(walkToBall_far)
    {
      transition
      {
        
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(theFieldBall.positionOnField.x() - ballOffsetX > theRobotPose.translation.x()){
          if(theFieldBall.positionOnField.x() < theRobotPose.translation.x() + ballXnearTh){
            if(approachYRange.isInside(theFieldBall.positionOnField.y() - theRobotPose.translation.y())){
              goto walkToBall_near;
            }
          }
        }
      }

      action
      {
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f), Pose2f(theFieldBall.positionOnField - Vector2f( ballOffsetX, 0.f)));
        
      }
    }

    state(walkToBall_near){
      transition{
        
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        
        if(smallApproachXRange.isInside(theFieldBall.positionRelative.x()) 
            && smallApproachYRange.isInside(theFieldBall.positionRelative.y())){
                goto approach;
        }
      }
      action{
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        theWalkToTargetPathPlannerStraightSkill(Pose2f(1.f,1.f,1.f), Pose2f(theFieldBall.positionOnField) - Pose2f(ballOffsetX, 50.f));
      }
    }
    
    state(approach){
      transition{
        const Angle angleToTarget = calcAngleToTarget(Pose2f(2000.f,0.f));  //TODO: change target s.t. perform passage or free area
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        
        if(theFieldBall.positionRelative.norm() < 0 ){
          goto turnToBall;
        }
        
        if(!smallApproachXRange.isInside(theFieldBall.positionRelative.x())){
          goto walkToBall_far;
        }
        

        if(std::abs(angleToTarget) < angle_target_treshold && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) 
            && ballOffsetYRange.isInside(theFieldBall.positionRelative.y())){
                goto kick;
        }
      }
      action{
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        theWalkToApproachSkill(Pose2f(2000.f,0.f),                          //TODO: change target s.t. perform passage or free area
        ballOffsetX, ballOffsetY, true);
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
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(Angle::fromDegrees(0.f), theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
        
      }
    }

    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
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

MAKE_CARD(GoalieApproachAndKickCard);
