/**
 * @file ReadyCard.cpp
 * 
 * This Card represent a behavior for robots during the ready state
 * 
 * @author Emanuele Antonioni
 */

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Modeling/RobotPose.h"

CARD(ReadyCard,
{,
  CALLS(WalkToTargetPathPlanner),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(Activity),
  CALLS(WalkToTarget),
  CALLS(WalkAtAbsoluteSpeed),
  CALLS(LookLeftAndRight),
  CALLS(WalkToTargetPathPlannerStraight),
  REQUIRES(GameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(LibCheck),
  REQUIRES(RobotPose),
  USES(TeamData),
  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) ownKickoff,
    (DeckOfCards<CardRegistry>) opponentKickoff,
    (DeckOfCards<CardRegistry>) ownFreeKick,
    (DeckOfCards<CardRegistry>) opponentFreeKick,
    (DeckOfCards<CardRegistry>) normalPlay,
    (DeckOfCards<CardRegistry>) dummyCards,
    }),
});

class ReadyCard : public ReadyCardBase
{

  //This parameter set the distance that triggers a robot to wait for the movement of a teammate to avoid collisions
  const float MIN_ROBOT_DISTANCE = 500.f;

  //This parameter set a treshold for the max length of the ready state for a robot, is used to avoid the motion in set
  const int MAX_READY_TIME = 42000;

  //Time to wait for collision avoidance 
  const int WAIT_TIME = 3000;

  //support variable for taking total time
  long long int total_time;

  bool preconditions() const override
  {
    return theGameInfo.state == STATE_READY;
  }

  bool postconditions() const override
  {
    return theGameInfo.state != STATE_READY;
  }

  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }

  option{
    theActivitySkill(BehaviorStatus::ready);
    //Start state
    initial_state(start)
    {
      transition
      {
        if(state_time >  10)
          goto goToPosition;
      }

      action
      {
        total_time = 0;
        theLookForwardSkill();
        theStandSkill();
      }
    }//end of start

    //This state uses the pathplanner to go to the desired ready position for the robot
    state(goToPosition)
    {
      transition
      {
        
        for(auto teammate : theTeamData.teammates){
          if(theLibCheck.distance(theRobotPose, teammate.theRobotPose) < MIN_ROBOT_DISTANCE){
            if(theRobotInfo.number > teammate.number){
              total_time += state_time;
              goto wait;
            }
          }
        }//end of for of teammates

        if(theLibCheck.distance(theRobotPose, theLibCheck.myReadyPosition()) < 750.f){
          total_time += state_time;
          goto finalGo;
        }

        if(total_time + state_time > MAX_READY_TIME){
          goto stand;
        }
      }

      action
      {
        
        theLookLeftAndRightSkill();
        theWalkToTargetPathPlannerSkill(Pose2f(0.8f,0.8f,0.8f), theLibCheck.myReadyPosition());
        }
    }//end of goToPosition

    state(finalGo){
      transition{
        for(auto teammate : theTeamData.teammates){
          if(theLibCheck.distance(theRobotPose, teammate.theRobotPose) < MIN_ROBOT_DISTANCE){
            if(theRobotInfo.number > teammate.number){
              total_time += state_time;
              goto wait;
            }
          }
        }//end of for of teammates

        if(total_time + state_time > MAX_READY_TIME){
          goto stand;
        }

      }action{
        theLookLeftAndRightSkill();
        theWalkToTargetPathPlannerStraightSkill(Pose2f(0.5f,0.5f,0.5f), theLibCheck.myReadyPosition());
      }
    }

    //this state stops the robot for a certain time to wait for a close teammate to move
    state(wait){
      transition{
        if(state_time >  WAIT_TIME){
          total_time += state_time;
          goto goToPosition;
        }

        if(total_time + state_time > MAX_READY_TIME){
          goto stand;
        }
          
      }action{
        theLookForwardSkill();
        theStandSkill();
      }
    }
    
    // pit state when the ready state time length is reached
    state(stand){
      transition{

      }action{
        theLookForwardSkill();
        theStandSkill();
      }
    }

  }// end of option

};


MAKE_CARD(ReadyCard);
