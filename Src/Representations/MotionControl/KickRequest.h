/**
 * @file Representations/MotionControl/KickRequest.h
 * @author <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#pragma once
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(DynPoint,
{
  DynPoint() = default;
  DynPoint(int limb, int phaseNumber, int duration, const Vector3f& translation, const Vector3f& angle, const Vector3f& odometryOffset);
  DynPoint(int limb, int phaseNumber, const Vector3f& translationl, const int duration = -1);

  bool operator==(const DynPoint& other) const,

  (int) limb,
  (int) phaseNumber,
  (int)(0) duration,
  (Vector3f)(Vector3f::Zero()) translation,
  (Vector3f)(Vector3f::Zero()) angle,
  (Vector3f)(Vector3f::Zero()) odometryOffset,
});

inline bool DynPoint::operator==(const DynPoint& other) const
{
  return limb == other.limb &&
         phaseNumber == other.phaseNumber &&
         duration == other.duration &&
         translation == other.translation &&
         angle == other.angle &&
         odometryOffset == other.odometryOffset;
};

inline DynPoint::DynPoint(int limb, int phaseNumber, int duration, const Vector3f& translation,
                          const Vector3f& angle, const Vector3f& odometryOffset) :
  limb(limb), phaseNumber(phaseNumber), duration(duration),
  translation(translation), angle(angle), odometryOffset(odometryOffset)
{}

inline DynPoint::DynPoint(int limb, int phaseNumber, const Vector3f& translation, const int duration) :
  limb(limb), phaseNumber(phaseNumber), duration(duration), translation(translation)
{}

STREAMABLE(KickRequest,
{
  ENUM(KickMotionID,  // NOTE of JULY 2020: The official preferred kick is the InWalkKick forward in the file WalkKickGenerator.h.
  {,
    kickForward, // SIMULATOR: Very very strong kick ( it covers more than half of the field ). 
		 // REAL: it is too imprecise, because the robot doesn't control the ball position when he turn its head before the kick.

    //powerKick,   // It raises runtime error.

    lobKick,     // SIMULATOR: 10% of the times the robot falls when he is trying to kick the ball. 
		 // REAL: to be tested
    kick_0_1,
    kick_1_2,
    kick_2_25,
    kick_25_3,
    kick_3_4,
    kick_4_5,
    kick_5_6,
    kick_6_7,
    kick_7_9,
    kick_8_10_goal,
    // kicks up to here are loaded by the KickEngine

    strongKick,  // SIMULATOR: Very soft kick. 
		 // REAL: It performs better than the simulator, covering much more distance.
    
    ottoMetriKick,    // an old overpowered kick that can shoot at very long distances

    newKick,     // It raises runtime error.

    none,        // The robot doesn't kick, like an empty subroutine.
  });

  static KickMotionID getKickMotionFromName(const char* name),

  (KickMotionID)(none) kickMotionType,
  (bool)(false) mirror,
  (bool)(false) armsBackFix,
  (bool)(false) autoProceed,
  (bool)(false) boost,
  (std::vector<DynPoint>) dynPoints,
});

STREAMABLE(Continuation,
{,
  (KickRequest::KickMotionID)(KickRequest::none) kickType,
  (bool)(false) mirror,
});

using stdVectorContinuation = std::vector<Continuation>;
