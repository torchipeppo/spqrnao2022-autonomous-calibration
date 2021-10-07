/**
 * @file BallCarrierCard.cpp
 *
 * This file implements a behavior to carry the ball forward in the field, avoiding obstacles.
 *
 * @author Emanuele Musumeci (based on Emanuele Antonioni's basic approacher behavior structure)
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/BallCarrierModel/BallCarrierModel.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Tools/Math/BHMath.h"
#include "Platform/SystemCall.h"
#include <string>

#define __STRINGIFY_I(arg) #arg
#define __STRINGIFY(arg) __STRINGIFY_I(arg)
#define DEBUG_CODE(code) \
  std::cout<<__STRINGIFY(code)<<": "<<std::to_string(code)<<std::endl;

//TODO: Make this a skill and move all stuff here from BallCarrierModel to allow selecting a custom target
//TODO: Might also make a skill that returns a path and use that in the ball carrier

CARD(BallCarrierCard,
{,
  CALLS(Activity),

  CALLS(LookForward),
  CALLS(LookAtPoint),
  
  CALLS(ParametricLookLeftAndRight),

  CALLS(Stand),
  //CALLS(CarryBall),
  CALLS(CarryBallWithController),
  
  REQUIRES(FieldBall),
  REQUIRES(BallModel),
  REQUIRES(BallCarrierModel),
  REQUIRES(LibCheck),
  
  REQUIRES(FieldDimensions),
  
  REQUIRES(RobotPose),
  REQUIRES(BallSpecification),
  REQUIRES(ObstacleModel),

  USES(BehaviorStatus),
  

  LOADS_PARAMETERS(
  {,
    
    (int) initialWaitTime,
    
    (float) normalWalkSpeed,
    (float) slowerWalkSpeed,

    (bool) SHOW_CONDITIONS_DEBUG,
    (bool) DEBUG_MODE,
  }),
});

class BallCarrierCard : public BallCarrierCardBase
{

  bool preconditions() const override
  {
    
    if(SHOW_CONDITIONS_DEBUG)
    {}

    return true;
  }

  bool postconditions() const override
  {

    if(SHOW_CONDITIONS_DEBUG)
    {}
        
    return true;
  }

  option
  {
    
    initial_state(start)
    {
      std::cout<<"BALL_CARRIER: start"<<std::endl;
      transition
      {
        if(state_time > initialWaitTime)
          goto carry_ball;
      }

      action
      {
        theActivitySkill(BehaviorStatus::ball_carrier_behavior_start);
        
        theLookForwardSkill();
        theStandSkill();
      }
    } 

    state(carry_ball)
    {
      //std::cout<<"BALL_CARRIER: carry_ball"<<std::endl;
      transition
      {
      }

      action
      {
        
        theCarryBallWithControllerSkill(normalWalkSpeed, slowerWalkSpeed, true);
      }
    }
    
    state(idle)
    {
      //std::cout<<"BALL_CARRIER: idle"<<std::endl;
      transition
      {
      }

      action
      {
        theActivitySkill(BehaviorStatus::ball_carrier_idle);

        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        
        theStandSkill();
      }
    }
  }
};

MAKE_CARD(BallCarrierCard);
