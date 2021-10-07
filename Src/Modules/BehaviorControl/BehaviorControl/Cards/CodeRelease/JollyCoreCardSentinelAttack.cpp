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
#include "Representations/Communication/RobotInfo.h"
#include "Representations/spqr_representations/PassShare.h"
#include <iostream>
using namespace std;

// #define DO_TURN_THRESHOLD 1080.f //si girano quando si allontanano di un metro dalla posizione
// #define TOLERANCE_DEGREES_FOR_TURN 10_deg
// #define STATIC_THRESHOLD 400.f
// #define NOT_MOVING_THRESHOLD -1000.f
// #define TURN_TO_BALL_THRESHOLD 20.f
// #define AU_STATIC_THRESHOLD 100.f
// #define CORNER_STATE 3
#define JOLLY_MOVE_THRESHOLD 250
#define JOLLY_WALK_THRESHOLD 900

// #define WAIT_TEAMMATES

CARD(JollyCoreCardSentinelAttack,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(LookLeftAndRight),
  CALLS(LookAtGlobalBall),
  CALLS(Stand),
  CALLS(Esorcista),
  CALLS(Turn360),
  CALLS(Turn),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(SpecialAction),
  CALLS(WalkAtAbsoluteSpeed),
  CALLS(WalkToTargetPathPlannerStraight),
  CALLS(WalkToTargetPathPlanner),
  CALLS(InWalkKick),
  CALLS(KeyFrameArms),
  CALLS(GoBehindStriker),

  REQUIRES(PassShare),
  REQUIRES(FieldBall),
  REQUIRES(ObstacleModel),

  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(BallModel),
  REQUIRES(TeamBallModel),
  REQUIRES(GameInfo),
  REQUIRES(LibCheck),
  REQUIRES(RobotInfo),
  REQUIRES(FrameInfo),
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
    (float)(0.f) Intenttion_time,
    //(float)(-300.f) OfssetWRTStriker,
    (Vector2f)(-300,0.f) OfssetWRTStriker,
    }),
});

class JollyCoreCardSentinelAttack : public JollyCoreCardSentinelAttackBase
{
  


  bool preconditions() const override
  {
    Vector2f velocity = theBallModel.estimate.velocity;
    Vector2f loc_position = theBallModel.estimate.position;
    Vector2f position = theLibCheck.rel2Glob(loc_position.x(),loc_position.y()).translation;
    Vector2f strikerPosition;
    bool striker_ready_to_pass=false;

    for (const auto &teammate : theTeamData.teammates)
    {
      if (teammate.role == Role::RoleType::striker)
      {
        strikerPosition = Vector2f(teammate.theRobotPose.translation.x(), teammate.theRobotPose.translation.y());
        
      }
    }
    
    // goto receivePass;
    std::tuple<int,int,Pose2f> strikerPS = theLibCheck.strikerPassShare();
            // std::cout <<"pass share?: " << thePassShare.readyPass << std::endl;
    striker_ready_to_pass=std::get<0>(strikerPS) == 1 ;
    bool isStriker_attaker =(std::abs( position.x()))<250 || striker_ready_to_pass;//strikerPosition.x()<0&&position.x()<0;
    // std::cout <<"Jolly attaker pos : " << (std::abs( position.x()))<<"  "<<isStriker_attaker <<std::endl;
    // std::cout <<"is jolly attaker  : " << isStriker_attaker << std::endl;
    return isStriker_attaker;
    //return (theGameInfo.state == STATE_PLAYING  && velocity.x() >= 0 && velocity.norm()/position.norm() >= 0.7);
    // return true;
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
    theActivitySkill(BehaviorStatus::jollyCoreCardSentinelAttack);
    initial_state(stand)
    {
      transition
      {
        // (option_time) as thatMomentInTime;
        #ifdef PENALTY_JOLLY          
        //   goto JOLLYPose; ///not still implemented
        #endif
            Intenttion_time=option_time;
            //std::cout <<" Jolly sentinel attack " << Intenttion_time<<std::endl;
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }
            // goto receivePass;
            std::tuple<int,int,Pose2f> strikerPS = theLibCheck.strikerPassShare();
            // std::cout <<"pass share?: " << thePassShare.readyPass << std::endl;
            if(std::get<0>(strikerPS) == 1 && std::get<1>(strikerPS) == theRobotInfo.number
            && (std::get<2>(strikerPS).translation-strikerPosition).norm() > 1500 && std::get<2>(strikerPS).translation != Vector2f(theFieldDimensions.xPosOpponentGroundline,0.f))
            {
                goto receivePass;
            }

            if((theRobotPose.translation-theLibCheck.getJollyPosition()).norm() > JOLLY_MOVE_THRESHOLD)
            {
                //std::cout <<"Going to move plan, jolly attacker" << std::endl;
                goto movePlan;
            }
            // goto movePlan;
      }
      action
      {

  
        // std::cout <<"Stand, jolly attacker" << std::endl;
        theStandSkill();
        theLookLeftAndRightSkill();

      }
    }
    state(movePlan)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                    break;
                }
            }
            std::tuple<int,int,Pose2f> strikerPS = theLibCheck.strikerPassShare();
            if(std::get<0>(strikerPS) == 1 && std::get<1>(strikerPS) == theRobotInfo.number
            && (std::get<2>(strikerPS).translation-strikerPosition).norm() > 1500 && std::get<2>(strikerPS).translation != Vector2f(theFieldDimensions.xPosOpponentGroundline,0.f))
                {
                    goto receivePass;
                }

            Vector2f targetPos = theLibCheck.getJollyPosition();
            Vector2f relTargetPos = theLibCheck.glob2Rel(targetPos.x(),targetPos.y()).translation;

            // // Check if there's an obstacle in front of the robot
            // for(const auto& obs : theObstacleModel.obstacles){
            //     // if(obs.center.x() < relTargetPos.x() && std::abs(obs.center.x()) < 800.f)
            //     if(obs.center.x() < relTargetPos.x() && obs.center.x() > 0.f && obs.center.x() < 400.f)
            //     {
            //         std::cout <<"Obstacle found jolly attack:   "<< obs.center.x() <<std::endl;
            //         goto moveAroundObstacle;
            //     }
            // }

            if((theRobotPose.translation-targetPos).norm() < JOLLY_WALK_THRESHOLD)
                {
                    goto moveWalk;
                }
        }
        action
        {
            theLookLeftAndRightSkill();
            // lookLeftAndRight();
            theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f),theLibCheck.getJollyPosition());
            //
            //  WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f),Pose2f(0.f,900.f,-2000.f));
        }
    }
        state(receivePass)
    {
        transition
        {
          // turn toward the goal but looking at the ball
          //std::cout <<"pass share? ready to receive: " << thePassShare.readyReceive << std::endl;
          if (thePassShare.readyReceive==1)
            {
                
                goto turnToGoal;
            }
       }
        action
        {
            //Looking at the ball
            theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0));      //   lookAtBall();
        }
    }
    state(moveWalk)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                    break;
                }
            }
            std::tuple<int,int,Pose2f> strikerPS = theLibCheck.strikerPassShare();
            if(std::get<0>(strikerPS) == 1 && std::get<1>(strikerPS) == theRobotInfo.number
            && (std::get<2>(strikerPS).translation-strikerPosition).norm() > 1500 && std::get<2>(strikerPS).translation != Vector2f(theFieldDimensions.xPosOpponentGroundline,0.f))
                goto receivePass;

            if((theRobotPose.translation-theLibCheck.getJollyPosition()).norm() < 100.f)
                goto turnToBall;
        }
        action
        {
            theLookForwardSkill();
            Vector2f targetPosition = theLibCheck.getJollyPosition();
            Vector2f relTargetPos = theLibCheck.glob2Rel(targetPosition.x(),targetPosition.y()).translation;
            theWalkToTargetSkill(Pose2f(1.f,1.f,1.f),Pose2f(relTargetPos.x(),relTargetPos.y()));
        }
    }
      state(turnToGoal)
    {
        transition
        {
            if((std::abs(theLibCheck.radiansToDegree(theLibCheck.angleToTarget(theFieldDimensions.xPosOpponentGroundline,0))) < 10.f))
            {
                goto waitBall;
            }
        }
        action
        {
          // get the angle the ball and the goal
          // initialize ball position to the target in case we don't know where the ball is
          std::tuple<int,int,Pose2f> strikerPS = theLibCheck.strikerPassShare();
          Vector2f ballPos = std::get<2>(strikerPS).translation;

         //std::cout<<" jolly attack target pos ball stimate "<<ballPos.transpose()<<std::endl;
          if(std::get<0>(strikerPS)==0)
          {
          // get the local ball if seen, otherwise use global
          if(theFrameInfo.time-theBallModel.timeWhenLastSeen < 500.f) {
            ballPos = theBallModel.estimate.position;
          } else if (theTeamBallModel.isValid) {//for what is that?
            ballPos = theTeamBallModel.position;
          }
          }
          Vector2f goalPos = Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f);
          /* float dx21 = x2-x1; */
          /* float dx31 = x3-x1; */
          /* float dy21 = y2-y1; */
          /* float dy31 = y3-y1; */
          /* float m12 = std::sqrt(dx21*dx21 + dy21*dy21); */
          /* float m13 = std::sqrt(dx31*dx31 + dy31*dy31); */
          /* float theta = std::cos((dx21*dx31 + dy21*dy31) / (m12 * m13)); */
          Pose2f angleBallGoal = Pose2f(goalPos.dot(ballPos)*0.25);//intersection point? 0.25???
     
          theTurnSkill(angleBallGoal);
        //   theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0));        //   lookAtBall();
        theLookAtPointSkill(Vector3f(ballPos.x(), ballPos.y(), 0));        //   lookAtBall();

          /* LookForward(); */
          /* if(theLibCheck.radiansToDegree(theLibCheck.angleToTarget(theFieldDimensions.xPosOpponentGroundline,0)) > 0.f) */
          /*   WalkAtRelativeSpeed(Pose2f(20.f, 0.0001f,0.0001f)); */
          /* else */
          /*   WalkAtRelativeSpeed(Pose2f(-20.f, 0.0001f,0.0001f)); */
        }
    }
//should be a skill? may be used for the other roles.
state(moveAroundObstacle)
    {
       transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                    break;
                }
            }
            std::tuple<int,int,Pose2f> strikerPS = theLibCheck.strikerPassShare();
            // std::cout<<"  where to kick "<<(std::get<2>(strikerPS).translation-strikerPosition).norm()<<std::endl;
            if(std::get<0>(strikerPS) == 1 && std::get<1>(strikerPS) == theRobotInfo.number
            && (std::get<2>(strikerPS).translation-strikerPosition).norm() > 1500 && std::get<2>(strikerPS).translation != Vector2f(theFieldDimensions.xPosOpponentGroundline,0.f))
            {
                //std::cout<<" goto receive pass"<<std::endl;
                goto receivePass;
            }
            if((theRobotPose.translation-theLibCheck.getJollyPosition()).norm() < 100.f)
            {
                goto turnToBall;
            }
        }
        action
        {
            theLookLeftAndRightSkill();
            // lookLeftAndRight();
            theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f),theLibCheck.getJollyPosition());
            // WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f),theLibCheck.getJollyPosition());
        }
    }
    state(waitBall)
    {
        transition
        {
            // If too long and ball didn't reach, that means the pass failed
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                    break;
                }
            }
            if(state_time > 5000)
                goto stand;
        }
        action
        {
            // initialize ball position to the target in case we don't know where the ball is
            std::tuple<int,int,Pose2f> strikerPS = theLibCheck.strikerPassShare();
            Vector2f glob_ballPos = std::get<2>(strikerPS).translation;
            Vector2f ballPos = theLibCheck.glob2Rel(glob_ballPos.x(),glob_ballPos.y()).translation;

            Vector2f goalPos = Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f);
          
            Pose2f angleBallGoal = Pose2f(goalPos.dot(ballPos)*0.25);//intersection point? 0.25???
            // std::cout<<"  waiting sa target pos ball stimate "<<ballPos.transpose()<<std::endl;
            // std::cout<<"  waiting sa angle ball stimate "<<angleBallGoal.translation.transpose()<<std::endl;
            // std::cout<<"  waiting sa target pos ball seen "  <<theBallModel.estimate.position.transpose()<<std::endl;
            theStandSkill();
            
            // theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0));        //   lookAtBall();
            // theLookAtPointSkill(Vector3f(angleBallGoal.translation.x(), angleBallGoal.translation.y(), 0));        //   lookAtBall();
            theLookAtPointSkill(Vector3f(ballPos.x(), ballPos.y(), 0));        //   lookAtBall();

        }
    }

state(turnToBall)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                    break;
                }
            }
            std::tuple<int,int,Pose2f> strikerPS = theLibCheck.strikerPassShare();
            if(std::get<0>(strikerPS) == 1 && std::get<1>(strikerPS) == theRobotInfo.number
            && (std::get<2>(strikerPS).translation-strikerPosition).norm() > 1500 && std::get<2>(strikerPS).translation != Vector2f(theFieldDimensions.xPosOpponentGroundline,0.f))
                goto receivePass;

            if((std::abs(theLibCheck.radiansToDegree(theLibCheck.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y()))) < 10.f))
                goto stand;
        }
        action
        {
            
            theLookForwardSkill();
            if(theLibCheck.radiansToDegree(theLibCheck.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y())) > 0.f)
                theWalkAtRelativeSpeedSkill(Pose2f(20.f, 0.0001f,0.0001f));
            else
                theWalkAtRelativeSpeedSkill(Pose2f(-20.f, 0.0001f,0.0001f));
        }
    }

    
  }
};
MAKE_CARD(JollyCoreCardSentinelAttack);