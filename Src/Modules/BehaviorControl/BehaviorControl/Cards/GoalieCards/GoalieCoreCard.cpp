// TODO FIX 2020 EMANUELE (SOLVED?)

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Tools/Math/BHMath.h"
#include <iostream>
using namespace std;

CARD(GoalieCoreCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(LookLeftAndRight),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(SpecialAction),
  CALLS(WalkAtAbsoluteSpeed),
  CALLS(WalkToTargetPathPlannerStraight),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(BallModel),
  REQUIRES(TeamBallModel),
  REQUIRES(GameInfo),
  REQUIRES(LibCheck),
  DEFINES_PARAMETERS(
  {,
  }),
});

class GoalieCoreCard : public GoalieCoreCardBase
{

  bool preconditions() const override
  {
    Vector2f velocity = theBallModel.estimate.velocity;
    Vector2f position = theBallModel.estimate.position;
    #ifdef PENALTY_STRIKER_GOALIE  
    return true;
    #endif
    return (theGameInfo.state == STATE_PLAYING  && velocity.x() < 0 && velocity.norm()/position.norm() >= 0.7);  
    /*Other conditions in previous version: 
      common_transition: theGameInfo.state == STATE_PLAYING && velocity.x() < 100 && velocity.norm()/position.norm()>1.8)
      in mainLoop: theGameInfo.state == STATE_PLAYING && velocity.x()<0 && velocity.norm()/position.norm()>0.7) //& theBallModel.estimate.position.x() < 1500)*/

  }

  bool postconditions() const override
  {
    Vector2f velocity = theBallModel.estimate.velocity;
    Vector2f position = theBallModel.estimate.position;

    return (((velocity.norm()/position.norm() < 0.7) && (theFieldBall.teamPositionRelative.norm() < 1000.f ||
      theLibCheck.distance(theRobotPose, Pose2f(theFieldDimensions.xPosOwnGroundline, 0.f)) > 1500.f  ))  || (state_time > 3000));
  }

  option
  {
    theActivitySkill(BehaviorStatus::goalieCore);
    initial_state(start)
    {
      transition
      {
        #ifdef PENALTY_STRIKER_GOALIE            
          goto goaliePose;
        #endif
          goto loop;               
      }
      action
      {
        theStandSkill();
        //theLookAtGlobalBallSkill()
        Pose2f globalBall = theLibCheck.glob2Rel(theTeamBallModel.position.x(),theTeamBallModel.position.y());
        theLookAtPointSkill(Vector3f(globalBall.translation.x(), globalBall.translation.y(), 0));
        
      }
    }

    state(goaliePose)
    {
      transition
      {
        if( state_time>1000)
          goto loop;
      }
      action
      {
        theLookForwardSkill();

        theWalkAtRelativeSpeedSkill(Pose2f(0.f, 1.f,0.f));
      }
    }

    state(loop)
    {  
      transition{
        Vector2f velocity = theBallModel.estimate.velocity;
        Vector2f position = theBallModel.estimate.position;

        /*
        if( !theLibCheck.isGoalieInAngle )
          goto turnToOpponentGoal;
        
        if( theLibCheck.distance(theRobotPose, Pose2f(theFieldDimensions.xPosOwnGroundline + 200.f, 0.f)) > 300.f ||
          theRobotPose.translation.x() < theFieldDimensions.xPosOwnGroundline + 100.f ){
            goto gotoGoaliePosition;
          }
        */

        #ifdef PENALTY_STRIKER_GOALIE  
        if(theGameInfo.state == STATE_PLAYING  && velocity.x()<0 && velocity.norm()/position.norm()>0.7){ 
        #endif     
        double teta = atan(velocity.y()/velocity.x());
        float l = position.x()* (float) tan(teta);
        float side = position.y()-l;
        
        if( position.x() < 1200.f){
          if(side<=200.f && side >= -200.f)
              goto stopBall;
          else if(side>200.f && side < 1500.f)
              goto goalieDiveLeft;
          else if(side<-200.f && side>-1500.f)
              goto goalieDiveRight;
        
        }
        #ifdef PENALTY_STRIKER_GOALIE  
        }
        #endif     
      }
        
      action{
        if(theLibCheck.timeSinceBallWasSeen < 2000){
          theStandSkill();
          theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0));
        }
        #ifndef PENALTY_STRIKER_GOALIE       
        else if(theLibCheck.timeSinceBallWasSeen < 5000){
          theStandSkill();
          //theLookAtGlobalBallSkill()
          Pose2f globalBall = theLibCheck.glob2Rel(theTeamBallModel.position.x(),theTeamBallModel.position.y());
          theLookAtPointSkill(Vector3f(globalBall.translation.x(), globalBall.translation.y(), 0));
        }
        #endif              
        else{
          theStandSkill();
          theLookLeftAndRightSkill();
        }
      }
    }

    /*
    state(turnToOpponentGoal) 
    {
        transition
        {
            #ifdef SPQR_DEBUG_GOALIE
            std::cerr << "turntoopponentgoal" << std::endl;
            #endif
            if( theLibCheck.isGoalieInAngle )
                goto loop;
        }
        action
        {
            theLookLeftAndRightSkill();

            if( theRobotPose.rotation >= 0.17f)
                theWalkAtAbsoluteSpeedSkill(Pose2f(-.6f, 0.f, 0.f));
            else if ( theRobotPose.rotation < -0.17f)
                theWalkAtAbsoluteSpeedSkill(Pose2f(.6f, 0.f, 0.f));
        }
    }

    state( gotoGoaliePosition){
        transition{
            if( theLibCheck.distance(theRobotPose, Pose2f(theFieldDimensions.xPosOwnGroundline + 200.f, 0.f)) < 290.f  ){
                goto loop;

            }
            Vector2f velocity = theBallModel.estimate.velocity;
            Vector2f position = theBallModel.estimate.position;
            double teta = atan(velocity.y()/velocity.x());
            float l = position.x()* (float) tan(teta);
            float side = position.y()-l;

            if( position.x() < 1200.f){
                if(side<=200.f && side >= -200.f)
                    goto stopBall;
                else if(side>200.f && side < 1500.f)
                    goto goalieDiveLeft;
                else if(side<-200.f && side>-1500.f)
                    goto goalieDiveRight;

            }
        }action{
            if(theLibCheck.timeSinceBallWasSeen < 2000){
                //theLookAtGlobalBallSkill()
                Pose2f globalBall = theLibCheck.glob2Rel(theTeamBallModel.position.x(),theTeamBallModel.position.y());
                theLookAtPointSkill(Vector3f(globalBall.translation.x(), globalBall.translation.y(), 0));
            } else {
                theLookLeftAndRightSkill();
            }


            theWalkToTargetPathPlannerStraightSkill(Pose2f(0.8f,0.8f,0.8f), Pose2f(theFieldDimensions.xPosOwnGroundline + 200.f, 0.f));
        }
    }
    */

    state(goalieDiveLeft)
    {
        action
        {
          theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0));
          theSpecialActionSkill(SpecialActionRequest::goalieFastDiveLeft);
        }
    }

    state(goalieDiveRight)
    {
        action
        {
          theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0));
          theSpecialActionSkill(SpecialActionRequest::goalieFastDiveLeft, true);
        }
    }

    state(stopBall)
    {
        action
        {
          theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0));
          theSpecialActionSkill(SpecialActionRequest::stopBall);
        }
    }
  }
};

MAKE_CARD(GoalieCoreCard);
