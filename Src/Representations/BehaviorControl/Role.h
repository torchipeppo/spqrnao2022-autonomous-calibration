/**
 * @file Role.h
 *
 * This file declares a representation of a player's role in the 2019 behavior.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Pose2f.h"
#include <string>

STREAMABLE(Role, COMMA public BHumanMessageParticle<undefined>
{
  /**
   * Returns a human readable description of this role.
   * @return A description of the role.
   */

   // ****** SPQR ********
   ENUM(RoleType,
   {,
    undefined,
    goalie,
    striker,
    defender,
    supporter,
    jolly,
    searcher_1,
    searcher_2,
    searcher_3,
    searcher_4,
    penaltyStriker,
    penaltyKeeper,
    planStriker,
    planJolly,
    none,

   });


   ENUM(Context,
    {,
        no_context, //TODO CAPIRE COME SOSTITUIRE L' =1 che c'era
        playing,
        search_for_ball,
    });

   // ****** END SPQR ********

  std::string getName() const;

  /** BHumanMessageParticle functions */
  void operator>>(BHumanMessage& m) const override;
  void operator<<(const BHumanMessage& m) override,

  (bool)(false) isGoalkeeper, /**< Whether I shall be the goalkeeper. */
  (bool)(true) playBall, /**< Whether I shall play the ball. */
  (int)(-1) supporterIndex, /**< My index in the supporter set (equivalent to the number of supporters behind me). */
  (int)(0) numOfActiveSupporters, /**< The number of not penalized supporters (i.e. robots that have a supporterIndex >= 0). */
  
  // ******* SPQR *******  
  (RoleType)(undefined) role,
  (RoleType)(undefined) lastRole,
  (Context)(no_context) current_context,
  (std::vector<int>)(5,0) utility_vector,
  (std::vector<Pose2f>)(std::vector<Pose2f>(5,Pose2f())) robots_poses,
  //********END SPQR ****
});
