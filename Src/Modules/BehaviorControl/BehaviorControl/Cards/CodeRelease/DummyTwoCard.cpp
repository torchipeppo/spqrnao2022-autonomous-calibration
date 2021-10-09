/**
 * @file DummyCardNumberOne.cpp
 *
 * This file implements nothing.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(DummyTwoCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Say),
  CALLS(Stand),
  REQUIRES(GameInfo),
});

class DummyTwoCard : public DummyTwoCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::codeReleasePositionForKickOff);
    theLookForwardSkill();
    theStandSkill();
    // Not implemented in the Code Release.
    //theSaySkill("I am The Dummy Number Two, I am the son of the Dummy Number One");
  }
};

MAKE_CARD(DummyTwoCard);

