#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Tools/Math/BHMath.h"
#include <iostream>
using namespace std;

CARD(SundayWalkCard,
{,
  CALLS(Activity),
  CALLS(LookLeftAndRight),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(SpecialAction),
  CALLS(WalkAtAbsoluteSpeed),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(GameInfo),
  REQUIRES(LibCheck),
  DEFINES_PARAMETERS(
  {,
  }),
});

class SundayWalkCard : public SundayWalkCardBase
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
    theActivitySkill(BehaviorStatus::sundayWalk);
    
    initial_state(start)
    {
      transition
      {
          goto orientamento_iniziale;               
      }
      action
      {
        theStandSkill();
        theLookLeftAndRightSkill();
        
      }
    }

    state(orientamento_iniziale)
    {
      transition
      {
        if( state_time>2000)
          goto primo_tratto;
      }
      action
      {
        theLookLeftAndRightSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(-.6f, 0.f, 0.f));
      }
    }
    
    state(primo_tratto)
    {
      transition
      {
        if( state_time>20000)
          goto orientamento_pre_curva1;
      }
      action
      {
        theLookLeftAndRightSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(0.f, 0.6f, 0.0f));
      }
    }

    state(orientamento_pre_curva1)
    {
      transition
      {
        if( state_time>1000)
          goto curva1;
      }
      action
      {
        theLookLeftAndRightSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(0.6f, 0.f, 0.f));
      }
    }
    
    state(curva1)
    {
      transition
      {
        if( state_time>23900)
          goto orientamento_post_curva1;
      }
      action
      {
        theLookLeftAndRightSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(0.11f, 1.0f, 0.0f));
      }
    }

    state(orientamento_post_curva1)
    {
      transition
      {
        if( state_time>1225)
          goto tratto1;
      }
      action
      {
        theLookLeftAndRightSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(0.6f, 0.f, 0.f));
      }
    }
    
    state(tratto1)
    {
      transition
      {
        if( state_time>40000)
          goto orientamento_pre_curva2;
      }
      action
      {
        theLookLeftAndRightSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(0.f, 0.6f, 0.0f));
      }
    }
    
    state(orientamento_pre_curva2)
    {
      transition
      {
        if( state_time>1000)
          goto curva2;
      }
      action
      {
        theLookLeftAndRightSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(-0.6f, 0.f, 0.f));
      }
    }
    
    state(curva2)
    {
      transition
      {
        if( state_time>23900)
          goto orientamento_post_curva2;
      }
      action
      {
        theLookLeftAndRightSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(-0.11f, 1.0f, 0.0f));
      }
    }

    state(orientamento_post_curva2)
    {
      transition
      {
        if( state_time>1230)
          goto tratto2;
      }
      action
      {
        theLookLeftAndRightSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(-0.6f, 0.f, 0.f));
      }
    }
    
    state(tratto2)
    {
      transition
      {
        if( state_time>40000)
          goto orientamento_pre_curva1;
      }
      action
      {
        theLookLeftAndRightSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(0.0f, 0.6f, 0.0f));
      }
    }
  }
};

MAKE_CARD(SundayWalkCard);