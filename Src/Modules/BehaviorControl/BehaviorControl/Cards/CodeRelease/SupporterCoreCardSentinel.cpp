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
#include "Representations/spqr_representations/RoleAndContext.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include <iostream>
using namespace std;

// #define DO_TURN_THRESHOLD 1080.f //si girano quando si allontanano di un metro dalla posizione
// #define TOLERANCE_DEGREES_FOR_TURN 10_deg
// #define STATIC_THRESHOLD 400.f
// #define NOT_MOVING_THRESHOLD -1000.f
// #define TURN_TO_BALL_THRESHOLD 20.f
// #define AU_STATIC_THRESHOLD 100.f
// #define CORNER_STATE 3
#define SUPPORTER_MOVE_THRESHOLD 250
#define SUPPORTER_WALK_THRESHOLD 900

// #define WAIT_TEAMMATES

CARD(SupporterCoreCardSentinel,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(LookLeftAndRight),
  CALLS(LookAtGlobalBall),
  CALLS(Stand),
  CALLS(Esorcista),
  CALLS(Turn360),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(SpecialAction),
  CALLS(WalkAtAbsoluteSpeed),
  CALLS(WalkToTargetPathPlannerStraight),
  CALLS(WalkToTargetPathPlanner),
  CALLS(InWalkKick),
  CALLS(KeyFrameArms),
  CALLS(GoBehindStriker),

  REQUIRES(FieldBall),
  REQUIRES(ObstacleModel),

  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(BallModel),
  REQUIRES(TeamBallModel),
  REQUIRES(GameInfo),
  REQUIRES(LibCheck),
//   REQUIRES(HeadMotionRequest),

  USES(RoleAndContext),
//   USES(HeadMotionRequest),
//   MODIFIES(HeadMotionRequest),

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

class SupporterCoreCardSentinel : public SupporterCoreCardSentinelBase
{
  


  bool preconditions() const override
  {
    Vector2f velocity = theBallModel.estimate.velocity;
    Vector2f position = theBallModel.estimate.position;
    Vector2f strikerPosition;
    for (const auto &teammate : theTeamData.teammates)
    {
      if (teammate.role == Role::RoleType::striker)
      {
        strikerPosition = Vector2f(teammate.theRobotPose.translation.x(), teammate.theRobotPose.translation.y());
      }
    }
    
    bool isStriker_defender = theLibCheck.rel2Glob(position.x(),position.y()).translation.x() <0;//strikerPosition.x()<0&&position.x()<0;
    return !isStriker_defender;
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
    theActivitySkill(BehaviorStatus::supporterCoreSentinel);
    initial_state(start)
    {
      transition
      {
        #ifdef PENALTY_SUPPORTER          
        //   goto SUPPORTERPose; ///not still implemented
        #endif
          Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }
            // if(theRobotPose.translation.x() > strikerPosition.x() && theTeamBallModel.position.x() > 2000)
            //    {
            //         goto moveBehindStriker_defender;// here should be added cost function for the optimal position in support mode
            //    }

            if(theRobotPose.translation.x() > strikerPosition.x() && theTeamBallModel.position.x() <0)
               {
                    goto moveBehindStriker_defender;// here should be added cost function for the optimal position in support mode
               }

            if((theRobotPose.translation-theLibCheck.getSupporterPosition()).norm() > SUPPORTER_MOVE_THRESHOLD)
                {
                    goto move;            
                }
      }
      action
      {

  
        if(state_time < 7000){
        theStandSkill();
        theLookLeftAndRightSkill();
        }
        else
        {
                    // std::cout<< "joder macho"<< std::endl;

            theTurn360Skill();
        }
      }
    }
//should be a skill? may be used for the other roles.
 state(moveAroundObstacle)
    {
        transition
        {
            if((theRobotPose.translation-theLibCheck.getSupporterPosition()).norm() < 100.f){
                if(theRoleAndContext.ball_holding_Context == 2 || theRoleAndContext.ball_holding_Context == 3){
                    if(theTeamBallModel.position.x() < 0.f || theLibCheck.opponentOnOurField())
                     {
                            std::cout<< "supporter sent turnopponent_from_movearound"<< std::endl;

                            goto turnToOpponent;
                     }
                    else {
                        std::cout<< "supporter sent turnToball_from_movearound"<< std::endl;

                        goto turnToBall;
                    }
                }
                else{
                    if(theTeamBallModel.position.x() < 0.f || !theLibCheck.opponentOnOurField())
                    {
                        std::cout<< "supporter sent turnToball_from_movearound"<< std::endl;

                        goto turnToBall;
                    }
                    else 
                    {
                        std::cout<< "supporter sent turnopponent_from_movearound"<< std::endl;

                        goto turnToOpponent;
                    }
                }
            }
        }
        action 
        {
            // std::cout<< "movearound_action  "<<theLibCheck.getSupporterPosition().x()<<"    "<<theLibCheck.getSupporterPosition().y()<< std::endl;
            theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f),theLibCheck.getSupporterPosition());
                        theLookLeftAndRightSkill();

        }
    }
    state(moveBehindStriker_defender)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }
            if(theTeamBallModel.position.x() < 0.f)
            {
                std::cout<< "movefromBehindStricker"<< std::endl;

                goto move;
            }
            if(theRobotPose.translation.x() < -500.f+strikerPosition.x())
             {

              goto stand;
             }
        }
        action 
        {
            if(theTeamBallModel.position.x() >= 2000){
              theGoBehindStrikerSkill(Vector2f(-500,0.f));
        }
        }
    }
    //should be a skill?
   state(turnToBall)
    {
        transition
        {
         Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }
            if(theRobotPose.translation.x() > strikerPosition.x() && theTeamBallModel.position.x() > 2000)
                goto moveBehindStriker_defender;

            if((std::abs(theLibCheck.radiansToDegree(theLibCheck.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y()))) < 10.f))
                goto stand;
        }
        action
        {
            float angle = theLibCheck.radiansToDegree(theLibCheck.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y()));
            if(angle   > 0.f )  theWalkAtRelativeSpeedSkill(Pose2f(20.f, 0.0001f,0.0001f));
            else theWalkAtRelativeSpeedSkill(Pose2f(-20.f, 0.0001f,0.0001f));
            theLookForwardSkill();
        }
    }
    //this part sshould be deleted??
   state(move)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }
            if(theRobotPose.translation.x() > strikerPosition.x() && theTeamBallModel.position.x() > 2000)
                goto moveBehindStriker_defender;

            if((theRobotPose.translation-theLibCheck.getSupporterPosition()).norm() < 100){
               if(theRoleAndContext.ball_holding_Context == 2 || theRoleAndContext.ball_holding_Context == 3){
                    if(theTeamBallModel.position.x() < 0.f || theLibCheck.opponentOnOurField())
                      {
                       goto turnToOpponent;// face opponent??
                      }
                    else {
                        goto turnToBall;
                    }
                }
                else{
                    if(theTeamBallModel.position.x() < 0.f || !theLibCheck.opponentOnOurField())
                        {
                           goto turnToBall;
                        }
                    else 
                        {
                           goto turnToOpponent;
                        }
                }
            }

            Vector2f targetPos = theLibCheck.getJollyPosition();
            Vector2f relTargetPos = theLibCheck.glob2Rel(targetPos.x(),targetPos.y()).translation;

            // Check if there's an obstacle in front of the robot
            for(const auto& obs : theObstacleModel.obstacles){
                if(obs.center.x() > 0.f){
                    if(obs.center.x() < relTargetPos.x())
                    {
                        //  std::cout<< "moveAroundObstacle"<< std::endl;
                       goto moveAroundObstacle;
                    }
                }
            }

        }
        action 
        {
             //std::cout<< "move action"<< std::endl;
            if((theRobotPose.translation-theLibCheck.getSupporterPosition()).norm() > SUPPORTER_WALK_THRESHOLD){
                
                theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f),theLibCheck.getSupporterPosition());
                theLookLeftAndRightSkill();
            }
            else {
                
                Vector2f targetPosition = theLibCheck.getSupporterPosition();
                Vector2f relTargetPos = theLibCheck.glob2Rel(targetPosition.x(),targetPosition.y()).translation;
                theWalkToTargetSkill(Pose2f(1.f,1.f,1.f),Pose2f(relTargetPos.x(),relTargetPos.y()));
                theLookForwardSkill();
            }
        }
    }
    state(turnToOpponent)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }
            if(theRobotPose.translation.x() > strikerPosition.x() && theTeamBallModel.position.x() > 2000)
                goto moveBehindStriker_defender;

            Vector2f interCalcPosition = theLibCheck.getSupporterPosition() - strikerPosition;
            Vector2f interCalc2Position = Vector2f(interCalcPosition.x()*1.3,interCalcPosition.y()*1.3);
            Vector2f targetPosition = strikerPosition + interCalc2Position;

            if(theRobotPose.translation.y() > strikerPosition.y()){
                // face topright-right
                float angleToTarget = std::abs(theLibCheck.radiansToDegree(theLibCheck.angleToTarget(targetPosition.x(),targetPosition.y())));
                if(angleToTarget < 80.f && angleToTarget > 35.f && theRobotPose.rotation.toDegrees() > -90.f && theRobotPose.rotation.toDegrees() < 135.f)
                    goto stand;
            }
            else{
                //face bottomright-right
                float angleToTarget = std::abs(theLibCheck.radiansToDegree(theLibCheck.angleToTarget(targetPosition.x(),targetPosition.y())));
                if(angleToTarget < 80.f && angleToTarget > 35.f && theRobotPose.rotation.toDegrees() < 90.f && theRobotPose.rotation.toDegrees() > -135.f)
                    goto stand;
            }
        }
        action
        {
            //should be for the sentinel card????///////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }

            Vector2f interCalcPosition = theLibCheck.getSupporterPosition();
            Vector2f interCalc2Position = Vector2f((interCalcPosition.x() - strikerPosition.x())*1.3,(interCalcPosition.y() - strikerPosition.y())*1.3);
            Vector2f targetPosition = strikerPosition + interCalc2Position;

            // theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            // theHeadMotionRequest.mode = HeadMotionRequest::targetMode;
            Pose2f relLookPos = theLibCheck.glob2Rel(targetPosition.x(),targetPosition.y());
            // theHeadMotionRequest.target = Vector3f(relLookPos.translation.x(),relLookPos.translation.y(),20.f);
            // theHeadMotionRequest.speed = 1;
            if(theRobotPose.translation.y() > strikerPosition.y()){
                // face topright-right
                theWalkAtRelativeSpeedSkill(Pose2f(-20.f, 0.0001f,0.0001f));
            }
            else{
                //face bottomright-right
                theWalkAtRelativeSpeedSkill(Pose2f(20.f, 0.0001f,0.0001f));
            }    
            theLookAtPointSkill(Vector3f(relLookPos.translation.x(),relLookPos.translation.y(),20.f));
            //predict the next possible ball position??
            
        }
    }
    
        state(stand){
        transition{
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }
            if(theRobotPose.translation.x() > strikerPosition.x() && theTeamBallModel.position.x() > 2000)
                goto moveBehindStriker_defender;

            if((theRobotPose.translation-theLibCheck.getSupporterPosition()).norm() > SUPPORTER_MOVE_THRESHOLD)
                goto move;
        }
        action{
                    if(state_time < 7000){
        theStandSkill();
        theLookLeftAndRightSkill();
        }
        else
        {
                   

            theTurn360Skill();
        }
        }
    }

    
    
  }
};
MAKE_CARD(SupporterCoreCardSentinel);