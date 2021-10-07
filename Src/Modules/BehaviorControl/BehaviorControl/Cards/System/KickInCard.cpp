/**
 * @file KickInCard.cpp
 *
 * This file specifies the behavior for a robot in a free kick (kick in) situation.
 * 
 * TODO check the deck for each role (except the strikers)
 *
 * @author Elisa Foderaro
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"
#include "Representations/BehaviorControl/Role.h"
#include <iostream>

CARD(KickInCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(SpecialAction),
  REQUIRES(GameInfo),
  REQUIRES(Role),
  REQUIRES(OwnTeamInfo),
  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) offensiveStriker,
    (DeckOfCards<CardRegistry>) defensiveStriker,
    (DeckOfCards<CardRegistry>) jolly,
    (DeckOfCards<CardRegistry>) defender,
    (DeckOfCards<CardRegistry>) supporter,
    (DeckOfCards<CardRegistry>) goalie,
  }),
});

class KickInCard : public KickInCardBase
{
  bool preconditions() const override
  {
    return theGameInfo.state == STATE_PLAYING && theGameInfo.setPlay == SET_PLAY_KICK_IN;
  }

  bool postconditions() const override
  {
    return theGameInfo.state != STATE_PLAYING || theGameInfo.setPlay != SET_PLAY_KICK_IN;
  }

  void execute() override
  {
    if(theRole.role == Role::striker)
    {
      // Own Free Kick -> offensive
      if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
      {
        dealer.deal(offensiveStriker)->call();
        setState("offensiveStriker");
      }
      
      // Opponent Free Kick -> defensive
      else
      {
        dealer.deal(defensiveStriker)->call();
        setState("defensiveStriker");
      }
    }

    // Has to get the ball
    else if(theRole.role == Role::jolly)
    {
      dealer.deal(jolly)->call();
      setState("jolly");  
    }

    else if(theRole.role == Role::goalie)
    {
      dealer.deal(goalie)->call();
      setState("goalie");
    }

    else if(theRole.role == Role::supporter)
    {
      dealer.deal(supporter)->call();
      setState("supporter");  
    }

    else if(theRole.role == Role::defender)
    {
      dealer.deal(defender)->call();
      setState("defender");  
    }
  }

  void reset() override
  {
    dealer.reset();
  }

  PriorityListDealer dealer; /**< The dealer which selects the card to play. */
};

MAKE_CARD(KickInCard);
