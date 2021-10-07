/**
 * @file GoBehindStriker.cpp
 *
 * This file implements the implementation of the GoBehindStriker skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Modeling/ObstacleModel.h"

// #include ""
SKILL_IMPLEMENTATION(GoBehindStrikerImpl,
{,
  IMPLEMENTS(GoBehindStriker),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(LookLeftAndRight),
  CALLS(LookAtGlobalBall),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(WalkQuarterCircleDownLeft),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(FieldDimensions),

  REQUIRES(RobotPose),
  // REQUIRES(WalkAtAbsoluteSpeed),

  USES(TeamData),

  MODIFIES(MotionRequest),
});

class GoBehindStrikerImpl : public GoBehindStrikerImplBase
{
    float setTR;
    option(GoBehindStriker)
    {
    initial_state(get_behind_striker)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }

            if(std::abs(theRobotPose.translation.y() - strikerPosition.y()) < 800.f){
                if(theRobotPose.translation.y() < strikerPosition.y()){
                    if(theRobotPose.translation.y() < 700.f - theFieldDimensions.yPosLeftSideline){
                        if(std::abs(theRobotPose.translation.y() - strikerPosition.y()) < 500.f){
                            setTR = (float)0.16;
                            goto do_a_semicircle_up;
                        }
                        else{
                            goto walk_straight_back;
                        }
                    }
                    else{
                        setTR = (float)0.3;
                        goto do_a_semicircle_down;
                    }
                }
                else{
                    if(theRobotPose.translation.y() > theFieldDimensions.yPosLeftSideline - 700.f){
                        if(std::abs(theRobotPose.translation.y() - strikerPosition.y()) < 500.f){
                            setTR = (float)0.16;
                            goto do_a_semicircle_down;
                        }
                        else
                            goto walk_straight_back;
                    }
                    else{
                        setTR = (float)0.3;
                        goto do_a_semicircle_up;
                    }
                }
            }
            else {
               goto walk_straight_back;
            }
        }
        action
        {
            ;
        }
    }
     // These movements need to check for collision with opponent
    state(do_a_semicircle_down)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }
            if(theRobotPose.translation.x() < p.striker_offset_pos.x() +strikerPosition.x())
                goto stand;

            for(const auto& obs : theObstacleModel.obstacles){
                if(obs.center.x() > 0 && obs.center.x() < 500.f){
                    goto walk_straight_back;
                }
            }

            if(state_time > 3000.f)
                goto get_behind_striker;
        }
        action
        {
            
            theWalkQuarterCircleDownLeftSkill(setTR);
            theLookForwardSkill();
        }
    }
        state(do_a_semicircle_up)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }
            if(theRobotPose.translation.x() < p.striker_offset_pos.x() +strikerPosition.x())
                goto stand;

            for(const auto& obs : theObstacleModel.obstacles){
                if(obs.center.x() > 0.f && obs.center.x() < 500.f){
                    goto walk_straight_back;
                }
            }

            if(state_time > 3000.f)
                goto get_behind_striker;
        }
        action
        {
            
            theWalkQuarterCircleDownLeftSkill(setTR);
            theLookForwardSkill();
        }
    }
    state(walk_straight_back)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }
            if(theRobotPose.translation.x() < p.striker_offset_pos.x() +strikerPosition.x())
                goto stand;

            if(state_time > 3000.f)
                goto get_behind_striker;
        }
        action
        {
            

            bool ObsAvoid = false;
            for(const auto& obs : theObstacleModel.obstacles){
                Vector2f globalObsPos = theLibCheck.rel2Glob(obs.center.x(),obs.center.y()).translation;
                if(globalObsPos.x() < theRobotPose.translation.x() && std::abs(globalObsPos.x()-theRobotPose.translation.x()) < 500.f ) {
                    if(std::abs(globalObsPos.y()-theRobotPose.translation.y()) < 500.f){
                        if(globalObsPos.y() < theRobotPose.translation.y())
                            theWalkToTargetSkill(Pose2f(1,1,1),theLibCheck.glob2Rel(theRobotPose.translation.x(),theRobotPose.translation.y()+500.f));
                        else
                            theWalkToTargetSkill(Pose2f(1,1,1),theLibCheck.glob2Rel(theRobotPose.translation.x(),theRobotPose.translation.y()-500.f));
                        ObsAvoid = true;
                        break;
                    }
                }
            }

            if(!ObsAvoid){
                Vector2f targetPosition(theRobotPose.translation.x()-500.f,theRobotPose.translation.y());
                Vector2f relTargetPos = theLibCheck.glob2Rel(targetPosition.x(),targetPosition.y()).translation;
                theWalkToTargetSkill(Pose2f(1.f,1.f,1.f),Pose2f(relTargetPos.x(),relTargetPos.y()));
            }
            theLookForwardSkill();
        }
    }

    state(stand)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }

            if(theRobotPose.translation.x() > p.striker_offset_pos.x() +strikerPosition.x())
                goto get_behind_striker;
        }
        action
        {
            
            theStandSkill();
            theLookLeftAndRightSkill();
        }
    }
}
};

MAKE_SKILL_IMPLEMENTATION(GoBehindStrikerImpl);