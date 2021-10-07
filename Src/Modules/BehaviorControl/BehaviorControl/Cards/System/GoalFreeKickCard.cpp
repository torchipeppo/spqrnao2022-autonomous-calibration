/**
 * @file GoalFreeKickCard.cpp
 *
 * This file specifies the behavior for a robot in the a free kick (goalie) situation.
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

CARD(GoalFreeKickCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(SpecialAction),
  REQUIRES(GameInfo),
  REQUIRES(Role),
  REQUIRES(OwnTeamInfo),
  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) defensiveStriker,
    (DeckOfCards<CardRegistry>) offensiveStriker,
    (DeckOfCards<CardRegistry>) defender,
    (DeckOfCards<CardRegistry>) supporter,
    (DeckOfCards<CardRegistry>) jolly,
    (DeckOfCards<CardRegistry>) goalie,
    (DeckOfCards<CardRegistry>) offensiveGoalie,
  }),
});

class GoalFreeKickCard : public GoalFreeKickCardBase
{
  bool preconditions() const override
  {
    return theGameInfo.state == STATE_PLAYING && theGameInfo.setPlay == SET_PLAY_GOAL_FREE_KICK;
  }

  bool postconditions() const override
  {
    return theGameInfo.state != STATE_PLAYING || theGameInfo.setPlay != SET_PLAY_GOAL_FREE_KICK;
  }

  void execute() override
  {
    if(theRole.role == Role::striker)
    {
      // Own Free Kick -> offensive
      // Can't go to the ball
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
      // Own Free Kick -> offensive
      if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
      {
        dealer.deal(offensiveGoalie)->call();
        setState("offensiveGoalie");
      }
      
      // Opponent Free Kick -> defensive
      else
      {
        dealer.deal(goalie)->call();
        setState("goalie");
      }
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

MAKE_CARD(GoalFreeKickCard);
