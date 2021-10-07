/**
 * @file BallController.cpp
 *
 *
 * @author Akshay Dhonthi
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/spqr_representations/BallPath.h"

#include "Tools/Math/BHMath.h"
#include <iostream>

using namespace std;

CARD(BallControllerCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(LookLeftAndRight),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(WalkAtAbsoluteSpeed),
  CALLS(WalkToTargetPathPlannerStraight),
  CALLS(InWalkKick),
  CALLS(Kick),
  REQUIRES(RobotModel),
  REQUIRES(FieldBall),
  REQUIRES(RobotPose),
  REQUIRES(BallModel),
  REQUIRES(BallPath),

  DEFINES_PARAMETERS(
  {,
  (int)(3500) ballNotSeenTimeout,
  (float)(180.f) ballOffsetX,
  (float)(45.f) ballOffsetY,

  (float)(0.f) theta,
  (float)(0.f) disty,
  (float)(0.f) distx,
  (float)(0.05) kp1,
  (float)(-0.008) kp2,
  (float)(0.008) kp3,

  (bool)(false) oppKickFlag,
  
  }),
});

class BallControllerCard : public BallControllerCardBase
{
  


  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  option
  {
    theActivitySkill(BehaviorStatus::goalieCore);
    initial_state(start)
    {
      transition
      {
        if(theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto coltroller_ID1;
          // goto behindTheBall;
      }
      action
      {
        theStandSkill();
        theLookAtPointSkill(Vector3f(theFieldBall.teamPositionRelative.x(), theFieldBall.teamPositionRelative.y(), 0.f));
        
      }
    }

    state(coltroller_ID1)
    {
      transition
      {
        if (disty > 1.5 && oppKickFlag == true) {
          goto kickBall_ID2;
        } 
        
        if ( (theBallModel.estimate.position).norm() < 150.0) 
          oppKickFlag = true;

        // if ((theBallModel.estimate.velocity).norm() == 0.0 && (theBallModel.estimate.position).norm() <=100.0)
        //   goto kickBall_ID2;

      }
      action
      {
        theLookAtPointSkill(Vector3f(theFieldBall.teamPositionRelative.x(), theFieldBall.teamPositionRelative.y(), 0.f));  
        if (isnan(theBallPath.errorTheta)) {theta = 0.0;} else {theta = kp1*theBallPath.errorTheta;}    
        if (isnan(theBallPath.errorDistY)) {disty = 0.0;} else {disty = kp2*theBallPath.errorDistY;}
        if (isnan(theBallPath.errorDistX)) {distx = 0.0;} else if (theBallPath.errorDistX<0) {distx = kp3*2.0*theBallPath.errorDistX;} else {distx = kp3*theBallPath.errorDistX;}
        // std::cout << theBallPath.errorTheta << ", " << theta  <<  std::endl;
        // std::cout << theBallPath.errorDistY << ", " << disty  <<  std::endl;
        // std::cout << theta << ", " << distx  << ", " << disty << std::endl;
        
        theWalkAtRelativeSpeedSkill(Pose2f(theta, distx, disty));
        // theWalkAtRelativeSpeedSkill(Pose2f(0.f, 1.f,0.f));
      }
    }

    state(kickBall_ID2)
    {
      transition
      {
        // if (oppKickFlag == false)
        //   goto coltroller_ID1;



      }
      action
      {
        theLookAtPointSkill(Vector3f(theFieldBall.teamPositionRelative.x(), theFieldBall.teamPositionRelative.y(), 0.f));  
        if (isnan(theBallPath.errorTheta)) {theta = 0.0;} else {theta = kp1*theBallPath.errorTheta;}    
        if (isnan(theBallPath.errorDistY)) {disty = 0.0;} else {disty = kp2*theBallPath.errorDistY;}
        if (isnan(theBallPath.errorDistX)) {distx = 0.0;} else if (theBallPath.errorDistX<0) {distx = kp3*2.0*theBallPath.errorDistX;} else {distx = kp3*theBallPath.errorDistX;}
        // std::cout << theBallPath.errorTheta << ", " << theta  <<  std::endl;
        // std::cout << theBallPath.errorDistY << ", " << disty  <<  std::endl;
        // std::cout << (sqrt( pow(theta,2) + pow(disty,2) + pow(distx,2) )) << std::endl;

        if ((sqrt( pow(theta,2) + pow(disty,2) + pow(distx,2) )) > 0.2 )
          theWalkAtRelativeSpeedSkill(Pose2f(theta, distx, disty));
        else {
          // theKickSkill(true, 9000.f, false);
          theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(Angle::fromDegrees(0.f), theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
          // theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(Angle::fromDegrees(0.f), theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y()));
          // theKickSkill(true, 9000.f, false);
          oppKickFlag = false;
        }

      }
    }

  }
};

MAKE_CARD(BallControllerCard);
