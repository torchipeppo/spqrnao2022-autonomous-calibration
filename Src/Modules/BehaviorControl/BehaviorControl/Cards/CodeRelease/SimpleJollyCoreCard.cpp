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

CARD(SimpleJollyCoreCard,
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
    (Pose2f) strikerPos,
    (float)(1.f) jollyYPos,
    (float)(1.f) jollyXpos,
    (bool) turnOnce,
    }),
});

class SimpleJollyCoreCard : public SimpleJollyCoreCardBase
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
    theActivitySkill(BehaviorStatus::simpleJollyCoreCard);
    initial_state(start)
    {
        transition
        {
            strikerPos = Pose2f(-500,0);
            if(state_time > 10)
                goto follow;
        }
        action
        {
            theLookLeftAndRightSkill();
            theStandSkill();
        }
    }

    common_transition
    {
        for(unsigned i = 0; i < theTeamData.teammates.size(); i++){
                if(theTeamData.teammates.at(i).thePassShare.role == 5){

                  strikerPos = theTeamData.teammates.at(i).theRobotPose;


                }
            }
        if(strikerPos.translation.x() > -800){
          if(strikerPos.translation.x() < 0){
            jollyXpos = -400;
          }else{
            jollyXpos = std::min((strikerPos.translation.x() - 1000), 1000.f);
          }
          jollyXpos = std::min((strikerPos.translation.x() - 1000), 1000.f);
          if(strikerPos.translation.y() > 0){

            jollyYPos = strikerPos.translation.y() - 700;
          }else{
            jollyYPos = strikerPos.translation.y() + 700;

          }
        }else{
          jollyYPos = strikerPos.translation.y();
          if(strikerPos.translation.y() > 0){

            jollyYPos = strikerPos.translation.y() - 700;
          }else{
            jollyYPos = strikerPos.translation.y() + 700;

          }
        }
        if(jollyYPos > 2000){
          jollyYPos = 2000;
        }else if(jollyYPos < -2000){
          jollyYPos = -2000;
        }
    }

    state(follow)
    {
        transition
        {
            if(theLibCheck.norm((theRobotPose.translation.x() - jollyXpos ),
                                            (theRobotPose.translation.y() - jollyYPos)) < 450 ){
                turnOnce = true;
                goto turnToGlobalBall;
            }
        }
        action 
        {
            theLookLeftAndRightSkill();
            theWalkToTargetPathPlannerSkill(Pose2f(0.9f,0.9f,0.9f), Pose2f(jollyXpos,jollyYPos));
            theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
            // theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::keyFrame;
            // theArmMotionRequest.armKeyFrameRequest.arms[Arms::left].motion = ArmKeyFrameRequest::back;
            // theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::keyFrame;
            // theArmMotionRequest.armKeyFrameRequest.arms[Arms::right].motion = ArmKeyFrameRequest::back;
        }
    }
    state(turnToGlobalBall)
    {
        transition
        {
            if(state_time > 10000 ||
               theLibCheck.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y() ) < Angle::fromDegrees(10.f)){
                    goto wait;
            }
        }
        action 
        {
            theTurnSkill(Pose2f(theTeamBallModel.position));
        }
    }

    state(wait)
    {
        transition
        {
            if(state_time > 10000 || theLibCheck.norm((theRobotPose.translation.x() - jollyXpos ),
                                            (theRobotPose.translation.y() - jollyYPos)) > 500){
                goto follow;
            }
        }
        action
        {
            theStandSkill();
            theLookAtPointSkill(Vector3f(theFieldBall.teamPositionRelative.x(), theFieldBall.teamPositionRelative.y(), 0.f));
        }
    }   
  }
};

MAKE_CARD(SimpleJollyCoreCard);