/**
 * @file WalkKickGenerator.h
 *
 * This file declares a representation that generates walk kicks (should be used from a walking engine).
 *
 * @author Alexis Tsogias
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/RobotParts/Legs.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Streams/EnumIndexedArray.h"

namespace WalkKicks
{
  ENUM(Type,  //NOTE of JULY 2020: The official kick is the forward kick.
  {,
    none,    // The robot doesn't kick, like an empty subroutine.

    forward, // NOTE: THIS IS THE PREFERRED AND TESTED OFFICIAL KICK.
             // SIMULATOR: It performs well, it covers not so much distance ( like 10% of the field ) . 
             // REAL:      It performs like in the simulator.

    forwardShort, 
    
    //Select in the call the leg to use, select right to move the ball left, select left to move the ball right
    lateral, // SIMULATOR: It is not so much precise, but it is quite stable
             // REAL: To be tested.
  });
}

STREAMABLE(WalkKickVariant,
{
  WalkKickVariant() = default;
  WalkKickVariant(WalkKicks::Type kickType, Legs::Leg kickLeg);

  bool operator==(const WalkKickVariant& other) const,

  (WalkKicks::Type)(WalkKicks::none) kickType,
  (Legs::Leg)(Legs::left) kickLeg,
});

inline WalkKickVariant::WalkKickVariant(WalkKicks::Type kickType, Legs::Leg kickLeg) : kickType(kickType), kickLeg(kickLeg) {}

inline bool WalkKickVariant::operator==(const WalkKickVariant& other) const
{
  return kickType == other.kickType && kickLeg == other.kickLeg;
}

STREAMABLE(WalkKickGenerator,
{
  /**
   * Starts an in walk kick and sets the other members of this representation.
   * @param walkKickVariant The walk kick variant to execute.
   */
  FUNCTION(void(const WalkKickVariant& walkKickVariant)) start;
  /**
   * Calculates the swing foot offset during the kick step.
   * @param phase The phase in the kick step (0: beginning of the kick, 1: end of the kick).
   * @param position The translation offset of the swing foot.
   * @param rotation The rotation offset of the swing foot (axis angles?).
   */
  FUNCTION(void(float phase, Vector3f& position, Vector3f& rotation)) getState,

  (bool) requiresPreStep, /**< Whether the kick that has been started last requires a pre step. */
  (Pose2f) preStepSize, /**< The pre step size of the kick that has been started last. */
  (Pose2f) stepSize, /**< The step size of the kick that has been started last. */
});
