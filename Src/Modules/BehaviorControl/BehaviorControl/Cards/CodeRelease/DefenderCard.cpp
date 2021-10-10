/**
 * @file DefenderCard.cpp
 *
 * This file implements a behavior for covering the goal, hopefully in an intelligent way.
 *
 * @author Graziano Specchi & Tommaso Carlini
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Role.h"
#include "Tools/Math/BHMath.h"
#include <iostream>
CARD(DefenderCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(Stand),
  CALLS(WalkToBallControllerDefender),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(Kick),
  CALLS(WalkToTargetPathPlanner),
  CALLS(WalkToTargetPathPlannerStraight),
  CALLS(WalkToApproach),
  USES(TeamData),
  REQUIRES(FieldBall),
  REQUIRES(LibCheck),
  REQUIRES(RobotModel),
  REQUIRES(TeamBallModel),
  REQUIRES(BallModel),
  REQUIRES(ObstacleModel),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(Role),

  REQUIRES(GameInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(50) initialWaitTime,
    (int)(3500) ballNotSeenTimeout,
    (float)(400.f) positionTh,
    (float)(300.f) smallPositionTh,
    }),
});


class DefenderCard : public DefenderCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return false;
  }

  option
  {
    theActivitySkill(BehaviorStatus::defender);

    initial_state(start)
    {
      transition
      {
        if(theRole.role == Role::searcher_1 || theRole.role == Role::searcher_2 || theRole.role == Role::searcher_4)
          goto searchForBall;
        // If I am too far from my defender area, use pathplanner
        if((theRobotPose.translation - Vector2f(-3500,0)).norm()>1200)
          goto usePathPlanner;
        else
        // If I am already close to my defender area, let's use Akshay Controlled Walk
          goto useAkshayWalk;
          
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(usePathPlanner)
    {
      transition
      {
        if(!theTeamBallModel.isValid)
          goto searchForBall;
        //std::cout<<" NORMA  "<<(theRobotPose.translation - Vector2f(-3500,0)).norm()<<"\n";
        if(!thereIsSomeoneCloseToMe(650) && (theRobotPose.translation - Vector2f(-3500,0)).norm()<1000)
          goto useAkshayWalk;
      }

      action
      {
        int y_offset_target = 0;
        if(theTeamBallModel.position.y()>0){
          //case in which the ball is on the right
          y_offset_target=600;
        }else{
          //case in which the ball is on the left
          y_offset_target=-600;
        }
        
        int goalieY = 0;
        for (auto const& teammate : theTeamData.teammates){
          if (teammate.theRobotPose.translation.x()<-4000){
            goalieY = teammate.theRobotPose.translation.y();
          }  
        }

        if(goalieY>-300 && goalieY<300){
          if(theTeamBallModel.position.y()>0){
            //case in which the ball is on the right
            y_offset_target=600;
          }else{
            //case in which the ball is on the left
            y_offset_target=-600;
          }
        }
        int x_offset_obstacle = 0;
        if(thereIsSomeoneCloseToMe(650)) x_offset_obstacle = -300;
        //std::cout<<"STO CACCHIO DE NORMA È "<<(theRobotPose.translation - Vector2f(-3500+x_offset_obstacle,y_offset_target)).norm()<<"\n";
        
        if ((theRobotPose.translation - Vector2f(-3500+x_offset_obstacle,y_offset_target)).norm() <400)//theLookAtPointSkill(Vector3f(-3500,y_offset_target, 0.f));
          theStandSkill();
        else if((theRobotPose.translation - Vector2f(-3500+x_offset_obstacle,y_offset_target)).norm() <1000)
          theWalkToTargetPathPlannerSkill(Pose2f(0.2f,0.2f,0.2f), Pose2f(-3500+x_offset_obstacle,y_offset_target));
        else
          theWalkToTargetPathPlannerSkill(Pose2f(0.8f,0.8f,0.8f), Pose2f(-3500+x_offset_obstacle,y_offset_target));
        theLookForwardSkill();
      }
    }
     
    state(useAkshayWalk)
    {
      transition
      {
        if(!theTeamBallModel.isValid)
          goto searchForBall;
        // If I am too far from my defender area, use pathplanner
        //if(theRobotPose.translation.x()>-2700 || (theRobotPose.translation.x()>-3500 && (theRobotPose.translation.y()>1100 || theRobotPose.translation.y()<-1100)))
        if(thereIsSomeoneCloseToMe(550) || (theRobotPose.translation - Vector2f(-3500,0)).norm()>1200)
          goto usePathPlanner;

          // QUESTE RIGHE SEGUENTI NON C'ENTRANO NULLA MA LE HO LASCIATE COSÌ SE VOGLIAMO IMPLEMENTARE ANCHE L'OBS. AVOID. ABBIAMO GIA LE VARIABILI 
        /*float obstacleDistance = std::numeric_limits<float>::infinity();
        for(auto obs : theObstacleModel.obstacles)
        {
          float dist = theBallModel.estimate.position.norm();
          if (dist < 300) {
            obstacleDistance =  dist;
          }
        }

        if (obstacleDistance > 300.0) {
          //goto kickBall;
        }*/

      }
      action
      {
        int goalieY = 0;
        for (auto const& teammate : theTeamData.teammates){
          if (teammate.theRobotPose.translation.x()<-4000){
            goalieY = teammate.theRobotPose.translation.y();
          }  
        }

        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        int y_offset_from = 0;
        int y_offset_to = 0;
        int x_offset_additional = 0;
        int y_goalie = 0;
        bool goalie_isFallen = false;

          // We want to cover the "internal" pole
        if(goalieY>-300 && goalieY<300){
          if(theTeamBallModel.position.y()>0){
            //case in which the ball is on the right
            y_offset_from=600;
          }else{
            //case in which the ball is on the left
            y_offset_from=-600;
          }
        }

                  
        if(goalieY>400){
          //case in which the ball is on the right
          y_offset_from=-600;
        }else if (goalieY<-400){
          //case in which the ball is on the left
          y_offset_from=600;
        }
        int x_offset_so_that_defender_stands_more_ahead_or_backward = 0;
        if(theTeamBallModel.position.x()>0){
          x_offset_so_that_defender_stands_more_ahead_or_backward = 400;
        }

        // if the ball is really close to the goal, then try to help the goalie 
        //std::cout<<" diffferenza palla " <<theTeamBallModel.position.x()+ 3500<<"\n" ;
        if(theTeamBallModel.position.x()+ 3500 <500){
          x_offset_so_that_defender_stands_more_ahead_or_backward = -400;
        }
        /* let's find the goalie
        for (auto const& teammate : theTeamData.teammates){ //Teammate utilities
            if (teammate.number==1 and theTeamData.teammates.size()>0){ // Robot #1 is the goalie
              y_goalie = teammate.theRobotPose.translation.y();
              if(teammate.theBehaviorStatus.activity == BehaviorStatus::fallen){
                // add part abou fallen robot. mo devo anna e continuo stanotte o domani
              }
            }
        }*/
        /*/ but if the ball is close to the goal..
        if(theTeamBallModel.position.y()>-300 && theTeamBallModel.position.y()<300){
          if (theTeamData.teammates.at(0).theRobotPose.translation.y()>-300 && theTeamData.teammates.at(0).theRobotPose.translation.y()<300){
            y_offset_from=700;
            x_offset_additional = 300;
          }
        }*/
        float yOffset = 0;
        Pose2f fromTarget = Vector2f(theFieldDimensions.xPosOwnGoalPost, 0.0);
        Pose2f toTarget = theTeamBallModel.position;
        fromTarget = Pose2f(fromTarget.translation.x(),fromTarget.translation.y()+y_offset_from);
        
        float xOffset;
        xOffset = theTeamBallModel.position.x() + 3500.f - x_offset_so_that_defender_stands_more_ahead_or_backward;
        //std::cout<<"From target x:"<<fromTarget.translation.x()<<" y:"<<fromTarget.translation.y()<<"\n";
        //std::cout<<"To   target x:"<<toTarget.translation.x()<<" y:"<<toTarget.translation.y()<<"\n";
                /*
                SKILL_INTERFACE(WalkToBallControllerDefender,
                  (const Pose2f&) fromTarget,
                  (const Pose2f&) toTarget,
                  (float)(0.0) offsetX,
                  (float)(0.0) offsetY,
                  (bool)(true) useLeftFoot,
                  (float)(1.0) gainWhenBallOnSide,
                  (float)(10.0) forwardThreshold,
                  (float)(50.0) sidewaysThreshold,
                  (float)(7.0) rotationThreshold,
                  (float)(0.05) kp1,
                  (float)(-0.003) kp2,
                  (float)(0.003) kp3 );
                */
        // NOTE: This skill includes also the Stand(). The 3 thresholds can be used to give different weight to the 3 types of errors
        theWalkToBallControllerDefenderSkill(fromTarget, //fromTarget
                                    toTarget, //toTarget
                                    xOffset, //offsetX
                                    yOffset, //offsetY
                                    false,  // useLeftFoot
                                    2.5, //gainWhenBallOnSide
                                    100, //forwardThreshold
                                    100, //sidewaysThreshold
                                    50); //rotationThreshold
      }
    }
    state(searchForBall)
    {
      transition
      {
        if(theTeamBallModel.isValid)
          goto start;
      }

      action
      {
        theLookForwardSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }
  }

  bool thereIsSomeoneCloseToMe(float threshold){
    float minimum_distance = 10000;
    if (theTeamData.teammates.size()==0) return false;
    for (auto const& teammate : theTeamData.teammates){ //Teammate utilities
      float current_distance = (teammate.theRobotPose.translation - theRobotPose.translation).norm();
      // If it is the goalie, I want to give less weight 
      if (teammate.theRobotPose.translation.x()<-4000) current_distance = current_distance + 400;
      if (current_distance<minimum_distance){
        minimum_distance = current_distance;
      }  
    }
    for(auto obs : theObstacleModel.obstacles)
    {
      float current_distance = obs.center.norm();
      if(obs.isOpponent()){
        if (current_distance<minimum_distance){
          minimum_distance = current_distance;
        }  
      }
    }
    if(minimum_distance<threshold)  return true;
    return false;
       /* TOMMYX AGGIUNGERE PARTE PER CONSIDERAERE ANCHE GLI OPPONENTS */

      
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

MAKE_CARD(DefenderCard);


/*TOMMYX
1) FATTO Coprire meglio la porta se portiere al centro  DA AGGIUNGERE LA DIPENDENZA DALLA POSIZIONE DEL PORTIERE E ANCHE CONSIDERARE SE È CADUTO COSI LO COPRE IL DIFENSORE 
2) FATTO Se sto lontano, devo approcciare con il PathPlanner 
3) FATTO Avanzo se la palla sta lontano FATTO
4) PIUOMENOFATTO se c'è lo striker non gli rompe le scatole 
  */
