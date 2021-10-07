/**
 * @file DummyCardNumberOne.cpp
 *
 * This file implements nothing.
 *
 * @author Emanuele Antonioni
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"


CARD(DummyOneCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Say),
  CALLS(Stand),
  REQUIRES(GameInfo),
  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) ownKickoff,
    (DeckOfCards<CardRegistry>) opponentKickoff,
    (DeckOfCards<CardRegistry>) ownFreeKick,
    (DeckOfCards<CardRegistry>) opponentFreeKick,
    (DeckOfCards<CardRegistry>) normalPlay,
    (DeckOfCards<CardRegistry>) dummyDummyCards,
  }),
});

class DummyOneCard : public DummyOneCardBase
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

MAKE_CARD(DummyOneCard);

