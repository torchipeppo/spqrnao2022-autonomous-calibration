/**
 * @file TeamData.h
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/TeamTalk.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/spqr_representations/PassShare.h" //TODO VINCENZO 2020 CHECK
#include "Representations/BehaviorControl/Role.h"//Added by david 2021
#include "Representations/spqr_representations/RoleAndContext.h" //Added by david 2021
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Function.h"
#include "Tools/Streams/Enum.h"
namespace RoboCup
{
#include <SPLStandardMessage.h>
}

#include "Tools/Communication/BNTP.h"

STREAMABLE(Teammate, COMMA public MessageHandler
{
  const SynchronizationMeasurementsBuffer* bSMB = nullptr;

  unsigned toLocalTimestamp(unsigned remoteTimestamp) const
  {
    if(bSMB)
      return bSMB->getRemoteTimeInLocalTime(remoteTimestamp);
    else
      return 0u;
  };

  /** MessageHandler function */
  bool handleMessage(InMessage& message) override;

  ENUM(Status,
  {,
    PENALIZED,                        /** OK   : I receive packets, but robot is penalized */
    FALLEN,                           /** GOOD : Robot is playing but has fallen or currently no ground contact */
    PLAYING,                          /** BEST : Teammate is standing/walking and has ground contact :-) */
  });

  ENUM(TeamOrigin,
  {,
    otherTeamRobot,
    BHumanRobot,
  });

  FieldCoverage theFieldCoverage, /**< Do not log this huge representation! */

  (int)(-1) number,
  (TeamOrigin) mateType,
  (bool)(false) isGoalkeeper,
  (bool)(true) isPenalized,
  (bool)(true) isUpright,
  (unsigned)(0) timeWhenLastPacketSent,
  (unsigned)(0) timeWhenLastPacketReceived,
  (unsigned)(0) timeOfLastGroundContact,
  (bool)(true) hasGroundContact,
  (Status)(PENALIZED) status,
  (unsigned)(0) timeWhenStatusChanged,

  (RobotPose) theRobotPose,
  (BallModel) theBallModel,
  (ObstacleModel) theObstacleModel,
  (BehaviorStatus) theBehaviorStatus,
  (Whistle) theWhistle,
  (TeamBehaviorStatus) theTeamBehaviorStatus,
  (SideConfidence) theSideConfidence,

  (RobotHealth) theRobotHealth,
  (TeamTalk) theTeamTalk,

  (Role::RoleType)(Role::RoleType::undefined) role,              //CHECK VINCENZO 2020
  (int)(0) role_id,              //CHECK VINCENZO 2020
  (PassShare) thePassShare,           //VINCENZO 2020 CHECK
  (RoleAndContext) theRoleAndContext, //David 2021
  (Role) theRole,                     //David 2021
});

/**
 * @struct TeammateData
 * Collection of teammate information
 */
STREAMABLE(TeamData,
{
  void draw() const;
  FUNCTION(void(const RoboCup::SPLStandardMessage* const)) generate,

  (std::vector<Teammate>) teammates, //< An unordered(!) list of all teammates that are currently communicating with me */
  (int)(0) numberOfActiveTeammates,   //< The number of teammates (in the list) that are at not INACTIVE */
  (unsigned)(0) receivedMessages,     //< The number of received (not self) team messages
});
