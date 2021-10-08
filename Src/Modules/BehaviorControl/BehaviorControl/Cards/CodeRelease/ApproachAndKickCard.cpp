/**
 * @file ApproachAndKickCard.cpp
 *
 * This file implements a behavior for approaching the ball and kick it to a given target.
 *
 * @author Emanuele Antonioni
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/BallCarrierModel/BallCarrierModel.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/spqr_representations/PassShare.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"

#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/BehaviorControl/Libraries/LibPathPlanner.h"

#include "Tools/Math/BHMath.h"
#include "Platform/SystemCall.h"
#include <string>

CARD(ApproachAndKickCard,
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
  CALLS(GoalTarget),
  CALLS(SetTarget),
  CALLS(LogFloatParameter),
  CALLS(LogStringParameter),

  REQUIRES(PassShare),
  REQUIRES(FieldBall),
  REQUIRES(BallCarrierModel),
  REQUIRES(LibCheck),
  REQUIRES(LibPathPlanner),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamPlayersModel),
  USES(BehaviorStatus),

  REQUIRES(GameInfo),


  //USING CFG PARAMETERS (FOUND IN FILE spqrnao2021/Config/Scenarios/Default/BehaviorControl/approachAndKickCard.cfg).
  //IF THERE IS ANY PROBLEM COMMENT THE FIRST BLOCK OF PARAMETERS AND UNCOMMENT THE SECOND

  LOADS_PARAMETERS(
  {,
    (float) walkSpeed,
    (int) initialWaitTime,
    (int) ballNotSeenTimeout,
    (float) ballAlignThreshold_degrees,
    (float) ballYThreshold,
    (float) ballOffsetX,
    (float) ballXnearTh,
    (Rangef) ballOffsetXRange,
    (Rangef) approachYRange,
    (Rangef) smallApproachYRange,
    (Rangef) smallApproachXRange,
    (float) ballOffsetY,
    (Rangef) ballOffsetYRange,
    (int) minKickWaitTime,
    (int) maxKickWaitTime,
    (float) approachTreshold,
    (float) angle_target_treshold_degrees,
    (bool) debugText,
    (float) goalKickThreshold,
    (float) nearGoalThreshold,
  }),

  /*DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(50) initialWaitTime,
    (int)(3500) ballNotSeenTimeout,
    (Angle)(5_deg) ballAlignThreshold,
    (float)(100.f) ballYThreshold,
    (float)(180.f) ballOffsetX,
    (float)(450.f) ballXnearTh,
    (Rangef)({170.f, 190.f}) ballOffsetXRange,
    (Rangef)({-350.f, 350.f}) approachYRange,
    (Rangef)({-150.f, 150.f}) smallApproachYRange,
    (Rangef)({150.f, 300.f}) smallApproachXRange,
    (float)(-75.f) ballOffsetY,
    (Rangef)({-85.f, -65.f}) ballOffsetYRange,
    (int)(10) minKickWaitTime,
    (int)(3000) maxKickWaitTime,
    (float)(1000.f) approachTreshold,
    (Angle)(20_deg) angle_target_treshold,
    (bool)(true) debugText,
    (float)(1000) goalKickThreshold,
    (float)(2500) nearGoalThreshold,
  }),*/
});

class ApproachAndKickCard : public ApproachAndKickCardBase
{

  // These two variables are used in order to let the robot say through PlaySound what is the distance from the target.
  double distanceConfirmed = 0.0;
  bool alreadyEnqueued = false;

  Angle angle_target_treshold = Angle::fromDegrees(angle_target_treshold_degrees);
  Angle ballAlignThreshold = Angle::fromDegrees(ballAlignThreshold_degrees);

  Vector2f chosenTarget;
  Vector2f goalTarget;


  // Check whether the striker should kick.
  // The choice between passing and carrying ball is done further down in the hierarchy.
  bool preconditions() const override
  {
    
    //std::cout << "pre " << theRobotPose.translation.norm() << '\n';

    //choose to kick if there is a clear scoring opportunity (striker close enough to the goal with no opponents in sight)
    if (
      theFieldBall.positionOnField.x() > theFieldDimensions.xPosOpponentPenaltyMark - 1200.0f //&&
      //theBallCarrierModel.isTargetOnGoal && !theBallCarrierModel.isFallbackPath
      //TO FRANCESCO FROM EMANUELE MUSUMECI: I had to relax a bit the precondition for the kick card 
      //(theBallCarrierModel.isTargetOnGoal || theBallCarrierModel.isFallbackPath)
    ) {
      //std::cout << "Clean shot, here I go!!" << '\n';
      return true;
    }
    //otherwise, it's best to pass or carry the ball
    else {
      //std::cout << "No kick for now" << '\n';
      return false;
    }
    
    
  }


  //exit when the preconditions don't hold true anymore.
  //this also includes some hysteresis to make sure the striker sticks with a decision.
  bool postconditions() const override
  {
    if (!(
      theFieldBall.positionOnField.x() > theFieldDimensions.xPosOpponentPenaltyMark - 1400.0f &&
      theBallCarrierModel.isTargetOnGoal &&
      !theBallCarrierModel.isFallbackPath
    )) {
      //std::cout << "Shouldn't kick anymore" << '\n';
      return true;
    }
    else {
      //std::cout << "Still kicking" << '\n';
      return false;
    }
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
        theWalkToTargetPathPlannerSkill(Pose2f(0.8f,0.8f,0.8f), Pose2f(theFieldBall.positionOnField - Vector2f( ballOffsetX, 0.f)));

        chosenTarget = theLibCheck.goalTarget(false, false);
        goalTarget = theLibCheck.goalTarget(false, false);
        //chosenTarget = theBallCarrierModel.dynamicTarget.translation;
      }
    }

    state(walkToBall_near)
    {
      transition
      {

        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;

        if(smallApproachXRange.isInside(theFieldBall.positionRelative.x())
            && smallApproachYRange.isInside(theFieldBall.positionRelative.y())){
                goto approach;
        }
      }
      action{
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        theWalkToTargetPathPlannerStraightSkill(Pose2f(0.8f,0.8f,0.8f), Pose2f(theFieldBall.positionOnField) - Pose2f(ballOffsetX, 50.f));
      }
    }

    state(approach)
    {
      transition{
        const Angle angleToTarget = calcAngleToTarget(chosenTarget);

        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;

        //std::cout<<"\ntheFieldBall.positionRelative.norm():"<<theFieldBall.positionRelative.norm()<<std::endl;
        //std::cout<<"theFieldBall.positionRelative.norm()<0:"<<(theFieldBall.positionRelative.norm()<0)<<std::endl;
        if(theFieldBall.positionRelative.norm() < 0 ){
          goto turnToBall;
        }

        //std::cout<<"\ntheFieldBall.positionRelative.x():"<<theFieldBall.positionRelative.x()<<std::endl;
        //std::cout<<"smallApproachXRange: ("<<smallApproachXRange.min<<", "<<smallApproachXRange.max<<")"<<std::endl;
        //std::cout<<"!smallApproachXRange.isInside(theFieldBall.positionRelative.x()): "<<!smallApproachXRange.isInside(theFieldBall.positionRelative.x())<<std::endl;
        if(!smallApproachXRange.isInside(theFieldBall.positionRelative.x())){
          goto walkToBall_far;
        }

        //std::cout<<"\nangleToTarget:"<<abs(angleToTarget)<<std::endl;
        //std::cout<<"angle_target_treshold:"<<angle_target_treshold<<std::endl;
        //std::cout<<"std::abs(angleToTarget) < angle_target_treshold:"<<(std::abs(angleToTarget) < angle_target_treshold)<<std::endl;
        //std::cout<<"\ntheFieldBall.positionRelative.x():"<<theFieldBall.positionRelative.x()<<std::endl;
        //std::cout<<"ballOffsetXRange: ("<<ballOffsetXRange.min<<", "<<ballOffsetXRange.max<<")"<<std::endl;
        //std::cout<<"ballOffsetXRange.isInside(theFieldBall.positionRelative.x()): "<<ballOffsetXRange.isInside(theFieldBall.positionRelative.x())<<std::endl;
        //std::cout<<"\ntheFieldBall.positionRelative.y():"<<theFieldBall.positionRelative.y()<<std::endl;
        //std::cout<<"ballOffsetYRange: ("<<ballOffsetYRange.min<<", "<<ballOffsetYRange.max<<")"<<std::endl;
        //std::cout<<"ballOffsetYRange.isInside(theFieldBall.positionRelative.y()): "<<ballOffsetYRange.isInside(theFieldBall.positionRelative.y())<<std::endl;
        if(std::abs(angleToTarget) < angle_target_treshold && ballOffsetXRange.isInside(theFieldBall.positionRelative.x())
            && ballOffsetYRange.isInside(theFieldBall.positionRelative.y())){
                // We could let robot saying " KICK AT GOAL "
                goto kick;
        }
      }
      action{
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));

        theGoalTargetSkill(goalTarget);
        theSetTargetSkill(chosenTarget);
        theWalkToApproachSkill(chosenTarget, ballOffsetX, ballOffsetY, true);

        double distanceTarget =  (chosenTarget - theFieldBall.positionOnField).norm();
        // Since we are kicking, we don't want the ball to arrive just on the opponent goal line. So let's add 2 meters.
        distanceConfirmed = distanceTarget+3000.f;
        //const Angle angleToTarget = calcAngleToTarget(target);
        //std::cout<< "TAR_X:"<<target.x()<<"\tTAR_Y:"<<target.y()<<"\tDISTANCE TO TARGET:"<< distanceTarget<<"\tBallX:"<<theFieldBall.positionRelative.x()<<"\tBallY:"<<theFieldBall.positionRelative.y()<<"\tCHECKx:"<<ballOffsetXRange.isInside(theFieldBall.positionRelative.x())<<"\tCHECKy"<<ballOffsetYRange.isInside(theFieldBall.positionRelative.y())<<"\tyRange:["<<ballOffsetYRange.min<<","<<ballOffsetYRange.max<<"]\tangleToTarget:"<<std::abs(angleToTarget)<<"\tangleTreshold:"<<angle_target_treshold<<"\tNORM:"<<theFieldBall.positionRelative.norm()<<"\n";

      }
    }


    state(kick)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout)){
          alreadyEnqueued = false;
          goto searchForBall;
        }

        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theKickSkill.isDone())){
          alreadyEnqueued = false;
          goto start;
        }
      }

      action
      {
        if ( not alreadyEnqueued){
            alreadyEnqueued = true;
            std::string distanceTargetString = std::to_string(int(distanceConfirmed/1000.f));
            // SystemCall::say("KICKING TO DISTANCE");
            // SystemCall::say(distanceTargetString.c_str());
            // SystemCall::say("METERS");

        }
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        //theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(Angle::fromDegrees(0.f), theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));

        theKickSkill(false, (float)distanceConfirmed, false);
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

MAKE_CARD(ApproachAndKickCard);
