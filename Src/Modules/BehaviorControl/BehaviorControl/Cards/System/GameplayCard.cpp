/**
 * @file GameplayCard.cpp
 *
 * This file implements a card that represents the behavior of a robot in states in which it is not forced to do nothing.
 *
 * @author Arne Hasselbring
 */

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"
#include "Representations/BehaviorControl/Role.h"

CARD(GameplayCard,
{,
  REQUIRES(GameInfo),
  REQUIRES(Role),
  REQUIRES(OwnTeamInfo),
  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) ownKickoff,
    (DeckOfCards<CardRegistry>) opponentKickoff,
    (DeckOfCards<CardRegistry>) ownFreeKick,
    (DeckOfCards<CardRegistry>) opponentFreeKick,
    (DeckOfCards<CardRegistry>) normalPlay,
    (DeckOfCards<CardRegistry>) striker,
    (DeckOfCards<CardRegistry>) defender,
    (DeckOfCards<CardRegistry>) supporter,
    (DeckOfCards<CardRegistry>) jolly,
    (DeckOfCards<CardRegistry>) goalie,
    (DeckOfCards<CardRegistry>) searcher,
  }),
});

class GameplayCard : public GameplayCardBase
{
  bool preconditions() const override
  {
    return theGameInfo.state == STATE_PLAYING;
  }

  bool postconditions() const override
  {
    return theGameInfo.state != STATE_PLAYING;
  }

  void execute() override
  {
    
    // ASSERT(theGameInfo.state == STATE_PLAYING);
    // if(theGameInfo.setPlay != SET_PLAY_NONE)
    // {
    //   if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
    //   {
    //     dealer.deal(ownFreeKick)->call();
    //     setState("ownFreeKick");
    //   }
    //   else
    //   {
    //     dealer.deal(opponentFreeKick)->call();
    //     setState("opponentFreeKick");
    //   }
    // }
    // else
    // {
    //   dealer.deal(normalPlay)->call();
    //   setState("normalPlay");
    // }
//  std::cout<<theRole.role <<std::endl;
    if(theRole.role == Role::striker){
      dealer.deal(striker)->call();
      setState("striker");
    }else if(theRole.role == Role::goalie){
      dealer.deal(goalie)->call();
      setState("goalie");
    }else if(theRole.role == Role::supporter){
      dealer.deal(supporter)->call();
      setState("supporter");  
    }else if(theRole.role == Role::jolly){
      dealer.deal(jolly)->call();
      setState("jolly");  
    }else if(theRole.role == Role::defender){
      dealer.deal(defender)->call();
      setState("defender");  
    }else if(theRole.role == Role::searcher_1){
      dealer.deal(searcher)->call();
      setState("searcher1");
    }else if(theRole.role == Role::searcher_2){
      dealer.deal(searcher)->call();
      setState("searcher2");
    }else if(theRole.role == Role::searcher_3){
      dealer.deal(searcher)->call();
      setState("searcher3");
    }else if(theRole.role == Role::searcher_4){
      dealer.deal(searcher)->call();
      setState("searcher4");
    }
  }

  void reset() override
  {
    dealer.reset();
  }

  PriorityListDealer dealer; /**< The dealer which selects the card to play. */
};

MAKE_CARD(GameplayCard);
