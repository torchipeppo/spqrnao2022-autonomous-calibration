/**
 * @file WalkToBallCard.cpp
 *
 * This file implements a basic striker behavior for the code release.
 * Normally, this would be decomposed into at least
 * - a ball search behavior card
 * - a skill for getting behind the ball
 *
 * @author Arne Hasselbring
 */


// TODO FIX 2020 EMANUELE

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Tools/Math/BHMath.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include <iostream>
using namespace std;

#define DO_TURN_THRESHOLD 1080.f //si girano quando si allontanano di un metro dalla posizione
#define TOLERANCE_DEGREES_FOR_TURN 10_deg
#define STATIC_THRESHOLD 400.f
#define NOT_MOVING_THRESHOLD -1000.f
#define TURN_TO_BALL_THRESHOLD 20.f

#define WAIT_TEAMMATES

CARD(SupporterCoreCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(LookLeftAndRight),
  CALLS(LookAtGlobalBall),
  CALLS(Stand),
  CALLS(Esorcista),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(SpecialAction),
  CALLS(WalkAtAbsoluteSpeed),
  CALLS(WalkToTargetPathPlannerStraight),
  CALLS(WalkToTargetPathPlanner),
  CALLS(InWalkKick),
  CALLS(KeyFrameArms),
  REQUIRES(FieldBall),

  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(BallModel),
  REQUIRES(TeamBallModel),
  REQUIRES(GameInfo),
  REQUIRES(LibCheck),
  USES(TeamData),
  // MODIFIES(ArmMotionRequest),


  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (float)(1000.f) radiusSupp,
    (int)(50) initialWaitTime,
    (int)(3500) ballNotSeenTimeout,
    (float)(400.f) positionTh,
    (float)(300.f) smallPositionTh,
    }),
});

class SupporterCoreCard : public SupporterCoreCardBase
{
  


  bool preconditions() const override
  {
    // variable to see what car is active in each each role.
    Vector2f velocity = theBallModel.estimate.velocity;
    Vector2f position = theBallModel.estimate.position;
    return true;//here should be added the condition for the actual state of the striker
    //return (theGameInfo.state == STATE_PLAYING  && velocity.x() >= 0 && velocity.norm()/position.norm() >= 0.7);
  }

  bool postconditions() const override
  {
    Vector2f velocity = theBallModel.estimate.velocity;
    Vector2f position = theBallModel.estimate.position;

    return velocity.norm()/position.norm() < 0.7 && (theFieldBall.teamPositionRelative.norm() < 1000.f ||
      theLibCheck.distance(theRobotPose, Pose2f(theFieldDimensions.xPosOwnGroundline, 0.f)) > 1500.f  );
  }

  option
  {
    theActivitySkill(BehaviorStatus::defenderCore);
    initial_state(start)
    {
      transition
      {
        #ifdef PENALTY_DEFENDER           
        //   goto supporterPose; ///not still implemented
        #endif
          goto turnToPose;               
      }
      action
      {

        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed),
                         Pose2f(theLibCheck.glob2Rel(theLibCheck.supporterPosition.x(),
                                                     theLibCheck.supporterPosition.y())));
        theLookForwardSkill();
      }
    }

  
    state(turnToPose)
    {  
      transition{
               if(std::abs( theLibCheck.radiansToDegree(theLibCheck.angleForSupporter) ) < 10.f )
                goto walkToPose;
      }
        
      action{
        if(state_time > 500.f){
            theLookForwardSkill();

             if((theLibCheck.angleForSupporter) > 0.f)
                    theWalkAtRelativeSpeedSkill(Pose2f(20.f, 0.0001f,0.0001f));
                else
                    theWalkAtRelativeSpeedSkill(Pose2f(-20.f, 0.0001f,0.0001f));

        }
        #ifndef PENALTY_DEFENDER   
        #endif              
        else{
          theStandSkill();
          theLookLeftAndRightSkill();
        }
      }
    }
 state(walkToPose)
    {
        transition
        {
//            if(/*(std::abs(theLibCodeRelease.angleForSupporter) > TOLERANCE_DEGREES_FOR_TURN) &&*/
//                    (theRobotPose.translation - theLibCodeRelease.supporterPosition).norm() > DO_TURN_THRESHOLD)
//                goto turnToPose;

//            for(int i = 0; i < theTeamData.teammates.size(); i++){
//                if((theTeamData.teammates.at(i).theRobotPose.translation -
//                        theLibCodeRelease.supporterPosition).norm() < (radiusSupp)){
//                    radiusSupp -= 10.f;
//                    goto stand;
//                }
//            }

#ifdef WAIT_TEAMMATES
            for(unsigned i = 0; i < theTeamData.teammates.size(); i++){
                if((theTeamData.teammates.at(i).theRobotPose.translation -
                        theRobotPose.translation).norm() < (radiusSupp)){
                    radiusSupp -= 10.f;
                    goto stand;
                }
            }
#endif

            if((theRobotPose.translation - theLibCheck.supporterPosition).norm() < STATIC_THRESHOLD)
                goto wait;
        }
        action
        {

            theLookLeftAndRightSkill();
            // std::cout<<"optimal supporter pos "<<theLibCheck.supporterPosition.transpose()<<std::endl;
            if((theLibCheck.glob2Rel(theLibCheck.supporterPosition.x(),
                                           theLibCheck.supporterPosition.y())).translation.norm() < 3*STATIC_THRESHOLD){


                theWalkToTargetSkill(Pose2f(50.f, 50.f, 50.f), Pose2f(theLibCheck.glob2Rel(theLibCheck.supporterPosition.x(),
                                                                                     theLibCheck.supporterPosition.y())));
            } else {
                theWalkToTargetPathPlannerSkill(Pose2f(50.f, 80.0f, 80.0f), Pose2f(theLibCheck.supporterPosition.x(),
                                                              theLibCheck.supporterPosition.y()));
                //Arms back
    
                theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
            }
        }
    }
        state(stand){
        transition{
//            std::cout<< "I'm the supporter and I am waiting!"<< std::endl;
            if (theTeamBallModel.position.x() < NOT_MOVING_THRESHOLD || 1){
                if((theRobotPose.translation - theLibCheck.supporterPosition).norm() > STATIC_THRESHOLD)
                    goto walkToPose;
                else
                    goto wait;
            }
        }
        action{
            theLookLeftAndRightSkill();

            theStandSkill();
        }
    }
     state(wait)
    {
        transition
        {
            radiusSupp = 1000.f;
            if (theTeamBallModel.position.x() < NOT_MOVING_THRESHOLD || 1){
                if((theRobotPose.translation - theLibCheck.supporterPosition).norm() > 4*STATIC_THRESHOLD)
                    goto walkToPose;
            }
        }

        action
        {

            float myDistanceToBall = theBallModel.estimate.position.norm();
            bool iAmMostNearPlayer = true;
            for(unsigned i = 0; i < theTeamData.teammates.size(); i++){

                if((theTeamData.teammates.at(i).theRobotPose.translation -
                        theTeamBallModel.position).norm() < (myDistanceToBall+400.f) && theTeamData.teammates.at(i).number != 1){
                    iAmMostNearPlayer = false;
                }
            }
            if (std::abs(theLibCheck.angleToGoal) > Angle::fromDegrees(TURN_TO_BALL_THRESHOLD)){
                theWalkToTargetSkill(Pose2f(50.f, 50.f, 50.f),
                             Pose2f(theLibCheck.angleToGoal/*theBallModel.estimate.position.angle()*/, 0.f, 0.f));
            // } else if (iAmMostNearPlayer) {

            // theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left),Pose2f(0.f, (float)theFieldDimensions.xPosOpponentGroundline, 0.f) );
            } else {
                theStandSkill();
                radiusSupp = 1000.f;
//                Stopper();
            }

            //head
            if(theLibCheck.timeSinceBallWasSeen < 2000){
                theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0));

            } else if(theLibCheck.timeSinceBallWasSeen < 5000){
              theLookAtGlobalBallSkill();
            } else {
                (theBallModel.lastPerception.y() > 0) ?
                             //Esorcista(1) : Esorcista(-1);
                             theEsorcistaSkill(1):theEsorcistaSkill(-1);
            }
        }
    }
    
  }
};

MAKE_CARD(SupporterCoreCard);