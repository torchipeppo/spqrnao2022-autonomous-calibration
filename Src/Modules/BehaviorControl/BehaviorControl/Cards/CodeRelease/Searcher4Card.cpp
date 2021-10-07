/**
 * @file Searcher4Card.cpp
 *
 * This file implements a behavior for searching the ball on the field
 *
 * @author Emanuele Antonioni
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/spqr_representations/PassShare.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Role.h"
#include "Tools/Math/BHMath.h"
#include "Platform/SystemCall.h"
#include <string>
CARD(Searcher4Card,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(WalkToTargetPathPlanner),
  CALLS(WalkToTargetPathPlannerStraight),
  CALLS(WalkToApproach),
  CALLS(LookLeftAndRight),
  CALLS(ParametricLookLeftAndRight),
  
  REQUIRES(PassShare),
  REQUIRES(FieldBall),
  REQUIRES(LibCheck),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  USES(FieldCoverage),
  REQUIRES(Role),
  REQUIRES(GameInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    
  }),
});

class Searcher4Card : public Searcher4CardBase
{
    
  
  bool preconditions() const override
  {
    return theRole.role == Role::searcher_4;
  }

  bool postconditions() const override
  {
    return true;
  }

  option
  {
    theActivitySkill(BehaviorStatus::searcher4);

    initial_state(start)
    {
      transition
      {
        srand(time(NULL));
        int r_point = (rand()%4) + 1;
        switch (r_point)
        {
        case 1:
            goto walkToPoint1;
            break;
        case 2:
            goto walkToPoint2;
            break;
        case 3:
            goto walkToPoint3;
            break;
        case 4:
            goto walkToPoint4;
            break;
        
        default:
            goto walkToPoint1;
            break;
        }
      }

      action
      {
        theLookLeftAndRightSkill();
        theStandSkill();
      }
    }

    state(walkToPoint1)
    {
      transition
      {
        Pose2f point1 = Pose2f(-theFieldDimensions.yPosRightFieldBorder*(1/2), theFieldDimensions.xPosOpponentGroundline*(3/4));
        if(state_time > 8000 || theLibCheck.distance(theRobotPose.translation, point1) < 300.f)
          goto turnAround;
          
      }

      action
      {
        Pose2f point1 = Pose2f(-theFieldDimensions.yPosRightFieldBorder*(1/2), theFieldDimensions.xPosOpponentGroundline*(3/4));
        
        theLookLeftAndRightSkill();
        theWalkToTargetPathPlannerSkill(Pose2f(walkSpeed,walkSpeed,walkSpeed), point1);
      }
    }//end of poin 1

    state(walkToPoint2)
    {
      transition
      {
        Pose2f point2 = Pose2f(theFieldDimensions.yPosRightFieldBorder*(1/2), theFieldDimensions.xPosOpponentGroundline*(3/4));
        if(state_time > 8000 || theLibCheck.distance(theRobotPose.translation, point2) < 300.f)
          goto turnAround;
          
      }

      action
      {
        Pose2f point2 = Pose2f(theFieldDimensions.yPosRightFieldBorder*(1/2), theFieldDimensions.xPosOpponentGroundline*(3/4));
        
        theLookLeftAndRightSkill();
        theWalkToTargetPathPlannerSkill(Pose2f(walkSpeed,walkSpeed,walkSpeed), point2);
      }
    }// endo of point 2

    state(walkToPoint3)
    {
      transition
      {
        Pose2f point3 = Pose2f(-theFieldDimensions.yPosRightFieldBorder*(1/2), theFieldDimensions.xPosOpponentGroundline*(1/4));
        if(state_time > 8000 || theLibCheck.distance(theRobotPose.translation, point3) < 300.f)
          goto turnAround;
          
      }

      action
      {
          Pose2f point3 = Pose2f(-theFieldDimensions.yPosRightFieldBorder*(1/2), theFieldDimensions.xPosOpponentGroundline*(1/4));
        
        theLookLeftAndRightSkill();
        theWalkToTargetPathPlannerSkill(Pose2f(walkSpeed,walkSpeed,walkSpeed), point3);
      }
    }//end of point 3

    state(walkToPoint4)
    {
      transition
      {
        Pose2f point4 = Pose2f(theFieldDimensions.yPosRightFieldBorder*(1/2), theFieldDimensions.xPosOpponentGroundline*(1/4));
        if(state_time > 8000 || theLibCheck.distance(theRobotPose.translation, point4) < 300.f)
          goto turnAround;
          
      }

      action
      {
        Pose2f point4 = Pose2f(theFieldDimensions.yPosRightFieldBorder*(1/2), theFieldDimensions.xPosOpponentGroundline*(1/4));
        
        theLookLeftAndRightSkill();
        theWalkToTargetPathPlannerSkill(Pose2f(walkSpeed,walkSpeed,walkSpeed), point4);
      }
    }//end of point 4

    state(turnAround)
    {
      transition
      {
        if(state_time > 5000.f){
            goto start;
        }      
      }

      action
      {
        theParametricLookLeftAndRightSkill(-30, 30, 20, 80);
        if(theRobotPose.translation.y() > 0){
            theWalkAtRelativeSpeedSkill(Pose2f(-0.5f,0.f,0.f));
        }else{
            theWalkAtRelativeSpeedSkill(Pose2f(0.5f,0.f,0.f));
        }
      }
    }

  }
};

MAKE_CARD(Searcher4Card);
