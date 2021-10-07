/**
 * @file Representations/BehaviorControl/BehaviorStatus.h
 * The file declares a struct that contains data about the current behavior state.
 * @author Andreas Stolpmann
 */

#pragma once

#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"

/**
 * @struct BehaviorStatus
 * A struct that contains data about the current behavior state.
 */
STREAMABLE(BehaviorStatus, COMMA public BHumanMessageParticle<idBehaviorStatus>
{
  ENUM(Activity,
  {,
    unknown,

    fallen,
    finished,
    initial,
    set,
    ready,
    sundayWalk,
    codeReleaseKickAtGoal,
    searcher1,
    searcher2,
    searcher3,
    searcher4,

    //OLD
    approachAndCarry,
    approachAndKick,
    //

    ball_carrier_behavior_start,
    ball_carrier_carrying,
    ball_carrier_idle,

    approach_and_kick_start,
    choosing_target,
    aligning_to_ball,
    kicking_to_goal,

    approach_and_carry_start,
    realigning_to_ball,
    realigning_to_target,
    kicking_to_dynamic_target,
    searching_for_ball,

    escapeFromObstacles_start,
    move_away_from_obstacle,

    reaching_ball,
    debug_standing,

    defender,
    defenderCore,
    defenderCoreAU,
    supporterCore,
    supporterCoreAU,
    supporterCoreSentinel,
    supporterCoreCardFixed,
    jollyCoreAU,
    jollyCoreSentinelDefense,
    jollyCoreCardSentinelAttack,
    jollyCoreCardFixed,
   JollyCoreCardSentinelDefense,
    simpleJollyCoreCard,
    goalieCore,
    goalie2020,
    codeReleasePositionForKickOff,
    passing, // new for passShareProvider

  });

  /**
   * Checks whether the activity indicates that the robot is going to the ball.
   * @return Whether the activity indicates that the robot is going to the ball.
   */
  bool goesToBall() const
  {
    return activity == codeReleaseKickAtGoal;
  }

  /** BHumanMessageParticle functions */
  void operator>>(BHumanMessage& m) const override;
  void operator<<(const BHumanMessage& m) override,

  (Activity)(unknown) activity, /**< What is the robot doing in general? */
  (int)(-1) passTarget,
  (Vector2f)(Vector2f::Zero()) walkingTo,
  (Vector2f)(Vector2f::Zero()) shootingTo,
  (Vector2f)(Vector2f::Zero()) goalTarget,
});

inline void BehaviorStatus::operator>>(BHumanMessage& m) const
{
  m.theMixedTeamHeader.wantsToPlayBall = goesToBall();
  m.theMixedTeamHeader.someBehaviorNumber = -1; // TODO
  m.theBHumanStandardMessage.activity = static_cast<unsigned char>(activity);
  m.theBHumanStandardMessage.passTarget = passTarget;
  m.theBHumanStandardMessage.walkingTo = walkingTo;
  m.theBHumanStandardMessage.shootingTo = shootingTo;
  m.theBHumanStandardMessage.goalTarget = goalTarget;
}

inline void BehaviorStatus::operator<<(const BHumanMessage& m)
{
  if(!m.hasBHumanParts)
  {
    activity = BehaviorStatus::unknown;
    const GameInfo& theGameInfo = static_cast<GameInfo&>(Blackboard::getInstance()["GameInfo"]);
    if(theGameInfo.state == STATE_INITIAL)
      activity = BehaviorStatus::initial;
    else if(theGameInfo.state == STATE_SET)
      activity = BehaviorStatus::set;
    else if(theGameInfo.state == STATE_FINISHED)
      activity = BehaviorStatus::finished;
    walkingTo = Vector2f::Zero();
    shootingTo = Vector2f::Zero();
    goalTarget = Vector2f::Zero();
    return;
  }
  activity = static_cast<Activity>(m.theBHumanStandardMessage.activity);
  passTarget = m.theBHumanStandardMessage.passTarget;
  walkingTo = m.theBHumanStandardMessage.walkingTo;
  shootingTo = m.theBHumanStandardMessage.shootingTo;
  goalTarget = m.theBHumanStandardMessage.goalTarget;
}
