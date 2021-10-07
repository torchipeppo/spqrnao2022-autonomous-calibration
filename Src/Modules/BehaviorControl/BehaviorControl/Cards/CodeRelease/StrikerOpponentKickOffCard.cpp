/**
 * @file StrikerOpponentKickOffCard.cpp
 *
 * Handle opponent kickoff by standing still
 *
 * @author Francesco Petri
 */

//disabling warnings while importing so I don't see irrelevant messages when compiling
//these are only meant to be here while I work
//TODO remove asap
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#pragma GCC diagnostic ignored "-Wimplicit-int-float-conversion"
#pragma GCC diagnostic ignored "-Wreorder-ctor"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/BallCarrierModel/BallCarrierModel.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/spqr_representations/PassShare.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Settings.h"
#include "Platform/SystemCall.h"
#include <string>

//see above
#pragma GCC diagnostic pop

CARD(StrikerOpponentKickOffCard,
{,
  CALLS(Activity),
  
  CALLS(LookForward),
  CALLS(LookAtPoint),

  CALLS(LookLeftAndRight),

  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(Kick),
  CALLS(WalkToTargetPathPlanner),

  CALLS(SetTarget),

  CALLS(KeyFrameArms),

  CALLS(Approacher2021),

  REQUIRES(FieldBall),
  REQUIRES(BallModel),

  REQUIRES(LibCheck),

  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(BallSpecification),
  REQUIRES(FrameInfo),

  USES(BehaviorStatus),

  REQUIRES(GameInfo),

  DEFINES_PARAMETERS(
  {,
    (float)(200.0f) ballThresholdPre,
    (float)(200.0f) ballThresholdPost,
    (int)(10000) waitTime,
  }),
});

class StrikerOpponentKickOffCard : public StrikerOpponentKickOffCardBase
{

  bool preconditions() const override
  {
    // opponent is kicking off...
    return theGameInfo.kickingTeam != Global::getSettings().teamNumber &&
        // and the ball is at the center of the field.
        theFieldBall.positionOnField.squaredNorm() <= ballThresholdPre*ballThresholdPre &&
        theGameInfo.secsRemaining > (600-10);
  }

  bool postconditions() const override
  {
    // exit if the kicking team changes...
    return theGameInfo.kickingTeam == Global::getSettings().teamNumber ||
        // ...or the kickoff conditions are no longer true...
        theFieldBall.positionOnField.squaredNorm() <= ballThresholdPost*ballThresholdPost ||
        // !theLibCheck.isKickoffStriker(ballThresholdPost) ||
        // ...or the timeout has passed
        theGameInfo.secsRemaining < (600-10);
  }

  option
  {
    theActivitySkill(BehaviorStatus::codeReleasePositionForKickOff);

    initial_state(wait)
    {
      // std::cout<<"opponent kickoff: wait"<<std::endl;
      transition
      {
        ;
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }
  }
};

MAKE_CARD(StrikerOpponentKickOffCard);
