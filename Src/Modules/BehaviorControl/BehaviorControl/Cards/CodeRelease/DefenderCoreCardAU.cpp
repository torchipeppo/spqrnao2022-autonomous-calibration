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
#define AU_STATIC_THRESHOLD 100.f
#define CORNER_STATE 3

#define WAIT_TEAMMATES

CARD(DefenderCoreCardAU,
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
    (float)(1000.f) radiusDef,
    (int)(50) initialWaitTime,
    (int)(3500) ballNotSeenTimeout,
    (float)(400.f) positionTh,
    (float)(300.f) smallPositionTh,
    }),
});

class DefenderCoreCardAU : public DefenderCoreCardAUBase
{
  


  bool preconditions() const override
  {
    Vector2f velocity = theBallModel.estimate.velocity;
    Vector2f position = theBallModel.estimate.position;
    return true;
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
    theActivitySkill(BehaviorStatus::defenderCoreAU);
    initial_state(start)
    {
      transition
      {
        #ifdef PENALTY_DEFENDER           
        //   goto defenderPose; ///not still implemented
        #endif
          goto walkToPose;               
      }
      action
      {

        // theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed),
        //                  Pose2f(theLibCheck.glob2Rel(theLibCheck.defenderPosition.x(),
        //                                                    theLibCheck.defenderPosition.y())));
        theWalkToTargetPathPlannerSkill(Pose2f(1.f, 1.f, 1.f),
                         Pose2f(theLibCheck.defenderPosition.x(),
                                theLibCheck.defenderPosition.y()));
        theLookForwardSkill();
      }
    }
   state(turnToBall)
    {
        transition
        {
            // moveAway if some robot without pathPlanner is coming toward you
            if( (theLibCheck.nearestTemmate().translation- theRobotPose.translation).norm() < 800.f) {
                goto moveAway;
            }
            // if i'm in my angle's range to the TeamBall
            if( std::abs(theLibCheck.radiansToDegree(theLibCheck.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y()))) < 15.f)
                    goto wait;
        }
        action
        {
            float angle = theLibCheck.radiansToDegree(theLibCheck.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y()));
            if(angle   > 0.f )  theWalkAtRelativeSpeedSkill(Pose2f(20.f, 0.0001f,0.0001f));
            else theWalkAtRelativeSpeedSkill(Pose2f(-20.f, 0.0001f,0.0001f));

        }
    }
state(coverUp)
    {
        transition
        {
            float defenderBallY = theLibCheck.defenderDynamicY();
            Vector2f defenderCoverPos = Vector2f(theLibCheck.defenderPosition.x(), defenderBallY);
            // moveAway if some robot without pathPlanner is coming toward you
            if( (theLibCheck.nearestTemmate().translation- theRobotPose.translation).norm() < 800.f) {
                goto moveAway;
            }
            // if ball is far, come back to deafult position
            if ( theTeamBallModel.position.x() >= 400.f || (int)theGameInfo.setPlay == CORNER_STATE )
                    goto walkToPose;
            // approach the ball during the walkPlanner or if the ball is behind the defender
            // if (   (theBallModel.estimate.position.norm() <= 9*AU_STATIC_THRESHOLD || theTeamBallModel.position.x() < theLibCheck.defenderPosition.x())
            //         && theLibCheck.defenderNearestBall() )
            //         goto engageBall;

            if((theRobotPose.translation - defenderCoverPos).norm() < AU_STATIC_THRESHOLD &&
                std::abs(theLibCheck.radiansToDegree(theLibCheck.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y())))  < 20.f    )
                    goto wait;
            else if((theRobotPose.translation - defenderCoverPos).norm() < AU_STATIC_THRESHOLD)
                    goto turnToBall;
        }
        action
        {
            float defenderBallY = theLibCheck.defenderDynamicY();
            float DefenderBallAngle = theLibCheck.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y());
            Pose2f relTarget = theLibCheck.glob2Rel(theLibCheck.defenderPosition.x(), defenderBallY);

            // to walk always whit angleToGoal
            // if( std::abs(defenderBallY - theLibCheck.defenderPosition.y() ) < AU_STATIC_THRESHOLD ){
            //     WalkToTargetPathPlanner(Pose2f(1.f, 1.f, 1.f),
            //                 Pose2f(DefenderBallAngle ,
            //                     theLibCheck.defenderPosition.x(),
            //                     defenderBallY));
            // }
            // else {  WalkToTarget(Pose2f(0.5f, 1.f, 1.f),
            //                 Pose2f(DefenderBallAngle ,
            //                     relTarget.translation.x(),
            //                     relTarget.translation.y() ) );
            theWalkToTargetPathPlannerSkill(Pose2f(1.f, 1.f, 1.f),
                                    Pose2f(DefenderBallAngle ,
                                    theLibCheck.defenderPosition.x(),
                                    defenderBallY));
            //head
            if(theLibCheck.timeSinceBallWasSeen < 2000){
                theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0));
            } else if(theLibCheck.timeSinceBallWasSeen < 5000){
                theLookAtGlobalBallSkill();
            } else {
                theLookForwardSkill();
            }

        }
    }
  
    state(turnToPose)
    {  
         transition
        {
            // moveAway if some robot without pathPlanner is coming toward you
            if( (theLibCheck.nearestTemmate().translation- theRobotPose.translation).norm() < 800.f) {
                goto moveAway;
            }
            // if i'm in my angle's range
            if( std::abs(theLibCheck.radiansToDegree(theLibCheck.angleToGoal))  < 10.f   )
                    goto wait;
        }
        action
        {
            if((theLibCheck.radiansToDegree(theLibCheck.angleToGoal))   > 0.f )  theWalkAtRelativeSpeedSkill(Pose2f(20.f, 0.0001f,0.0001f));
            else theWalkAtRelativeSpeedSkill(Pose2f(-20.f, 0.0001f,0.0001f));

        }
    }
 state(walkToPose)
    {
        transition
        {
            // moveAway if some robot without pathPlanner is coming toward you
            if( (theLibCheck.nearestTemmate().translation- theRobotPose.translation).norm() < 800.f) {
                goto moveAway;
            }
            // if the ball is visible and teamBall is valid go to cover-up a dynamic position
            if( (theLibCheck.timeSinceBallWasSeen < 2000 || theTeamBallModel.isValid ) &&
                 theTeamBallModel.position.x()< 400.f && (int)theGameInfo.setPlay != CORNER_STATE ) goto coverUp;
            // if i'm in my position's range && if i'm in my angle's range to lookforward
            if((theRobotPose.translation - theLibCheck.defenderPosition).norm() < AU_STATIC_THRESHOLD &&
                std::abs(theLibCheck.radiansToDegree(theLibCheck.angleToGoal))  < 10.f    )
                    goto wait;
            // if i'm in my position's range && corner state
            if((theRobotPose.translation - theLibCheck.defenderPosition).norm() < AU_STATIC_THRESHOLD && (int)theGameInfo.setPlay == CORNER_STATE )
                    goto turnToBall;
            // if i'm in my position's range
            else if((theRobotPose.translation - theLibCheck.defenderPosition).norm() < AU_STATIC_THRESHOLD)
                    goto turnToPose;
        }
        action
        {
           theWalkToTargetPathPlannerSkill(Pose2f(1.f, 1.f, 1.f),
                        Pose2f(theLibCheck.angleToGoal ,
                            theLibCheck.defenderPosition.x(),
                            theLibCheck.defenderPosition.y()));
            //head
            theLookForwardSkill();
        }
    }
        state(moveAway)
    {
        transition
        {
            float defenderBallY = theLibCheck.defenderDynamicY();
            Vector2f defenderCoverPos = Vector2f(theLibCheck.defenderPosition.x(), defenderBallY);
            // for(const auto& mate : theTeamData.teammates){
            //     // if( ((theRobotPose.translation - defenderCoverPos).norm() < AU_STATIC_THRESHOLD ||
            //     //     (theRobotPose.translation - theLibCheck.defenderPosition).norm() < AU_STATIC_THRESHOLD )
            //     //     && (theLibCheck.nearestTemmate().translation- theRobotPose.translation).norm() >= AU_STATIC_THRESHOLD  )  goto walkToPose;
            //     if((theLibCheck.nearestTemmate().translation- theRobotPose.translation).norm() >= AU_STATIC_THRESHOLD  )  goto walkToPose;
            // }
            if((theLibCheck.nearestTemmate().translation- theRobotPose.translation).norm() >= 800.f  )  goto walkToPose;
        }
        action
        {
            Vector2f defenderCoverPos = Vector2f(theLibCheck.defenderPosition.x(), theLibCheck.defenderDynamicY());
            float DefenderBallAngle = theLibCheck.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y());
            Pose2f relTarget1 = theLibCheck.glob2Rel(theLibCheck.defenderPosition.x(), theRobotPose.translation.y()+300.f);
            Pose2f relTarget2 = theLibCheck.glob2Rel(theLibCheck.defenderPosition.x(), theRobotPose.translation.y()-300.f);
            
            if( theLibCheck.nearestTemmate().translation.y() <  theRobotPose.translation.y()) 
                // WalkToTargetPathPlanner(Pose2f(1.f, 1.f, 1.f),
                //                         Pose2f(DefenderBallAngle ,
                //                         theLibCheck.defenderPosition.x(),
                //                         theRobotPose.translation.y()+300.f));
                 theWalkToTargetSkill(Pose2f(0.5f, 1.f, 1.f),
                            Pose2f(DefenderBallAngle ,
                                relTarget1.translation.x(),
                                relTarget1.translation.y() ) );
            else //WalkToTargetPathPlanner(Pose2f(1.f, 1.f, 1.f),
            //                             Pose2f(DefenderBallAngle ,
            //                             theLibCheck.defenderPosition.x(),
            //                             theRobotPose.translation.y()-300.f));
                 theWalkToTargetSkill(Pose2f(0.5f, 1.f, 1.f),
                            Pose2f(DefenderBallAngle ,
                                relTarget2.translation.x(),
                                relTarget2.translation.y() ) );
        }
    }
//         state(stand){
//         transition{
// //            std::cout<< "I'm the Defender and I am waiting!"<< std::endl;
//             if (theTeamBallModel.position.x() < NOT_MOVING_THRESHOLD || 1){
//                 if((theRobotPose.translation - theLibCheck.defenderPosition).norm() > STATIC_THRESHOLD)
//                     goto walkToPose;
//                 else
//                     goto wait;
//             }
//         }
//         action{
//             theStandSkill();
//         }
//     }

    state(stopBall)
    {
        transition
        {
            if (state_time > 5000)  goto wait;
        }
        action
        {
          theLookAtPointSkill(Vector3f(theFieldBall.teamPositionRelative.x(), theFieldBall.teamPositionRelative.y(), 0.f));
          theSpecialActionSkill(SpecialActionRequest::stopBall);
        }
    }
     state(wait)
    {
        transition
        {
            // moveAway if some robot without pathPlanner is coming toward you
            if( (theLibCheck.nearestTemmate().translation- theRobotPose.translation).norm() < 800.f) {
                goto moveAway;
            }
            // check to engage the ball
            Vector2f ballVelocity = theBallModel.estimate.velocity;
            Vector2f ballPosition = theBallModel.estimate.position;
            // StopBall if the ball comes towards me
            if(theGameInfo.state == STATE_PLAYING  && ballVelocity.x()<0
                && ballVelocity.norm()/ballPosition.norm()>2.f && ballVelocity.norm() > 20000.f){
                double teta = atan(ballVelocity.y()/ballVelocity.x());
                float l = (float)(ballPosition.x()*tan(teta));
                float lato = ballPosition.y()-l;
                if(lato<=200 && lato >= -200)
                    goto stopBall;
            }
            // engage the ball just if i'm the nearest and if the ball is visible && doesn't move && is in range of 900.f
            // if (theLibCheck.defenderNearestBall() && theLibCheck.timeSinceBallWasSeen < 2000 &&
            //     ballVelocity.norm() < 200.f &&  (ballPosition.norm() <= 9*AU_STATIC_THRESHOLD || theTeamBallModel.position.x() < theLibCheck.defenderPosition.x())
            //     && (int)theGameInfo.setPlay != CORNER_STATE  && theLibCheck.defenderNearestBall() ) goto engageBall;
            // track the ball to cover-up
            float distance = theLibCheck.defenderDynamicDistance();
            if( distance >  800.f && theTeamBallModel.isValid && theTeamBallModel.position.x() < 400.f && (int)theGameInfo.setPlay != CORNER_STATE ) {
                goto coverUp;
            }
            // if the teamBall is not valid come back to default position
            if( (!theTeamBallModel.isValid || theTeamBallModel.position.x() >= 400.f)
                && (theRobotPose.translation - theLibCheck.defenderPosition).norm() > 2*AU_STATIC_THRESHOLD ) {
                goto walkToPose;
            }


        }
        action
        {
            //body
            theStandSkill();
            //head
            if(theLibCheck.timeSinceBallWasSeen < 2000){
                theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0));
            } else if(theLibCheck.timeSinceBallWasSeen < 5000){
                theLookAtGlobalBallSkill();
            } else {
                (theBallModel.lastPerception.y() > 0) ?
                            theEsorcistaSkill(1) : theEsorcistaSkill(-1);
            }
        }
    }
    
  }
};

MAKE_CARD(DefenderCoreCardAU);