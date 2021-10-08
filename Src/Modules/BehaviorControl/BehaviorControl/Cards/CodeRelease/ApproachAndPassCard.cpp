/**
 * @file ApproachAndPass.cpp
 *
 * This file implements a behavior for passing the ball to a given target.
 * This is a slightly modified version of the ApproachAndKickCard.
 *
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/spqr_representations/PassShare.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/BehaviorControl/BallCarrierModel/BallCarrierModel.h"
#include "Tools/Math/BHMath.h"


#include "Platform/SystemCall.h"
#include <string>

//TODO: Investigate and fix oscillation problem

CARD(ApproachAndPassCard,
{,
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(Kick),
  CALLS(WalkToTargetPathPlanner),
  CALLS(WalkToTargetPathPlannerStraight),

  CALLS(WalkToApproach),

  CALLS(Stand),
  CALLS(Activity),
  REQUIRES(PassShare),
  REQUIRES(MotionInfo),
  REQUIRES(FieldBall),
  REQUIRES(LibCheck),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(BallCarrierModel),
  USES(BehaviorStatus),

  REQUIRES(GameInfo),

  REQUIRES(TeamPlayersModel),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(50) initialWaitTime,
    (int)(3500) ballNotSeenTimeout,
    (Angle)(5_deg) alignThreshold,
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
  }),
});


class ApproachAndPassCard : public ApproachAndPassCardBase
{
  // These two variables are used in order to let the robot say through PlaySound what is the distance from the target.
  double distanceConfirmed = 0.0;
  bool alreadyEnqueued = false;

  // Because the Pass card was made lower-priority wrt the Kick card,
  // the dealer now only reaches this card if the Kick preconditions were not met.
  // Here we decide whether to pass (prioritized if possible)
  // or to carry the ball alone for the moment
  bool preconditions() const override
  {
    //needless to say, if we don't have a pass avaliable we can't pass
    if (thePassShare.readyPass == 0) {
      //std::cout << "No pass available, can't pass" << '\n';
      return false;
    }

    //if there is a pass pass we run the conditions in libcheck
    //to decide whether to actually do it or not
    bool shouldPass = theLibCheck.strikerPassCommonConditions(0);
    if (shouldPass) {
      //std::cout << "Pass conditions are go" << '\n';
      return true;
    }
    else {
      //std::cout << "Maybe it's better to play by myself for now" << '\n';
      return false;
    }
  }

  //These conditions check when there is no good pass available anymore (or, indirectly, when the pass has been performed already)
  //or if the shootingTo target (where the ball is being kicked at) has been changed (for any reason).
  //NOTICE: the only code that sets thePassShare.readyPass to 0 is contained in PassShareProvider
  bool postconditions() const override
  {
    /*
    return thePassShare.readyPass == 0 ||
            std::abs(theBehaviorStatus.shootingTo.x())<std::abs(theRobotPose.translation.x());
    */
    if (
      thePassShare.readyPass == 0 ||
      std::abs(theBehaviorStatus.shootingTo.x())<std::abs(theRobotPose.translation.x())
    ) {
      //std::cout << "pass post: early check is true" << '\n';
      return true;
    }

    //give up passing if a shooting opportunity comes up,
    //because that has priority
    if (
      theFieldBall.positionOnField.x() > theFieldDimensions.xPosOpponentPenaltyMark - 1200.0f //&&
      //TO FRANCESCO FROM EMANUELE MUSUMECI: I had to relax a bit the precondition for the kick card 
      //theBallCarrierModel.isTargetOnGoal && !theBallCarrierModel.isFallbackPath
      //(theBallCarrierModel.isTargetOnGoal || theBallCarrierModel.isFallbackPath)
      
    ) {
      //std::cout << "Kicking opportunity arose, leaving pass card" << '\n';
      return true;
    }

    //otherwise check the same conditions as precond. to decide whether to keep passing
    bool shouldPass = theLibCheck.strikerPassCommonConditions(1);
    if (shouldPass) {
      //std::cout << "Still passing" << '\n';
      return true;
    }
    else {
      //std::cout << "Shouldn't pass anymore" << '\n';
      return false;
    }
  }

  option
  {
    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
        {
          goto turnToBall;
        }
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }

    //Walk around, looking for the ball
    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen()) //If the ball is seen...
        {
          goto turnToBall;
        }
      }

      action
      {
        theLookForwardSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }

    //Turn to the ball until acceptably aligned
    state(turnToBall)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          goto searchForBall;
        }
        if(std::abs(theFieldBall.positionRelative.angle()) < alignThreshold)
        {
          goto walkToBall_far;
        }
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
      }
    }

    //Approach the ball if far
    state(walkToBall_far)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          goto searchForBall;
        }
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

      }
    }

    //Approach the ball if near (only the transition conditions change wrt to walkToBall_far)
    state(walkToBall_near){
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          goto searchForBall;
        }

        if(smallApproachXRange.isInside(theFieldBall.positionRelative.x())
            && smallApproachYRange.isInside(theFieldBall.positionRelative.y())){
                goto approachToPass;
              }
      }
      action
      {
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        theWalkToTargetPathPlannerStraightSkill(Pose2f(0.8f,0.8f,0.8f), Pose2f(theFieldBall.positionOnField) - Pose2f(ballOffsetX, 50.f));
      }
    }

    //Approach the ball precisely (last robot movements before the kick)
    state(approachToPass){
      transition
      {
        // std::cout<<"topass"<<thePassShare.passTarget<<std::endl;
        const Angle angleToTarget = calcAngleToTarget(thePassShare.passTarget.translation);
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          goto searchForBall;
        }

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

      action
      {
        //Set the BehaviorStatus to the current PassShare
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        Vector2f passTarget = thePassShare.passTarget.translation;
        double distanceTarget =  (passTarget - theFieldBall.positionOnField).norm();
        distanceConfirmed = distanceTarget;
        //const Angle angleToTarget = calcAngleToTarget(passTarget);
        //std::cout<< "TAR_X:"<<passTarget.x()<<"\tTAR_Y:"<<passTarget.y()<<"\tDISTANCE TO TARGET:"<< distanceTarget<<"\tBallX:"<<theFieldBall.positionRelative.x()<<"\tBallY:"<<theFieldBall.positionRelative.y()<<"\tCHECKx:"<<ballOffsetXRange.isInside(theFieldBall.positionRelative.x())<<"\tCHECKy"<<ballOffsetYRange.isInside(theFieldBall.positionRelative.y())<<"\tyRange:["<<ballOffsetYRange.min<<","<<ballOffsetYRange.max<<"]\tangleToTarget:"<<std::abs(angleToTarget)<<"\tangleTreshold:"<<angle_target_treshold<<"\tNORM:"<<theFieldBall.positionRelative.norm()<<"\n";

        theWalkToApproachSkill(passTarget, ballOffsetX, ballOffsetY, true);
      }
    }

    //Kick the ball to complete the passage
    state(kick)
    {
      transition
      {
        //if we're done kicking, go back to the initial state
        if(theMotionInfo.walkRequest.walkKickRequest.kickType != WalkKicks::none)
        {
          alreadyEnqueued = false;
          goto start;
        }
      }
      action
      {
        distanceConfirmed=distanceConfirmed*1; //jsut to check the jolly head position view
        if ( not alreadyEnqueued){
            alreadyEnqueued = true;
            // std::string distanceTargetString = std::to_string(int(distanceConfirmed/1000.f));
            std::string distanceTargetString = std::to_string(int(distanceConfirmed/1000.f));
            // SystemCall::say("PASSING TO DISTANCE");
            // SystemCall::say(distanceTargetString.c_str());
            // SystemCall::say("METERS");

        }
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        theKickSkill(false, (float)distanceConfirmed, false); // parameters: (kyck_type, mirror, distance, armsFixed)

      }
    }
  }

  bool isAligned(Pose2f target_pose)
  {
    return calcAngleToTarget(target_pose)< alignThreshold;
  }

  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }


  bool shouldIPass() const
  {
    return true;
  }
};

MAKE_CARD(ApproachAndPassCard);
