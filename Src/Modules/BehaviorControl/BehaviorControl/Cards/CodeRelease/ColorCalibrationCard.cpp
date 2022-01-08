/**
 * @file ColorCalibrationCard.cpp
 *
 * Card to... TODO
 *
 * @author Francesco Petri
 */

//disabling warnings while importing so I don't see irrelevant messages when compiling
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#pragma GCC diagnostic ignored "-Wimplicit-int-float-conversion"
#pragma GCC diagnostic ignored "-Wreorder-ctor"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#pragma GCC diagnostic ignored "-Wenum-enum-conversion"
#pragma GCC diagnostic ignored "-Wenum-float-conversion"

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
#include "Representations/Communication/TeamData.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Tools/Math/BHMath.h"
#include "Platform/SystemCall.h"
#include <string>

#include "Modules/FieldColorsCalibrator/FieldColorsBoard.h"

//see above
#pragma GCC diagnostic pop

CARD(ColorCalibrationCard,
{,
  CALLS(Activity),
  CALLS(Say),
  
  CALLS(LookForward),
  CALLS(LookAtAngles),
  CALLS(LookAtPoint),
  CALLS(LookLeftAndRight),

  CALLS(Stand),

  REQUIRES(FieldBall),
});

#define TILT_PHASE1 -0.65f
#define TILT_PHASE2 -1.5f

class ColorCalibrationCard : public ColorCalibrationCardBase
{


  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return false;
  }

  option
  {
    initial_state(start)
    {
      transition
      {
        goto begin_calibration;
      }

      action
      {
        // framework wants these to be set at all times
        theActivitySkill(BehaviorStatus::unknown);
        theLookAtAnglesSkill(0.f, TILT_PHASE1, 150_deg, HeadMotionRequest::lowerCamera);
        theStandSkill();
      }
    }

    state(begin_calibration)
    {
      transition
      {
        if (state_time) {
          goto wait_calibration;
        }
      }
      action
      {
        theActivitySkill(BehaviorStatus::unknown);
        theLookAtAnglesSkill(0.f, TILT_PHASE1, 150_deg, HeadMotionRequest::lowerCamera);
        theStandSkill();
        FieldColorsBoard::set_cognition_to_lower(FLDCOLBRD__C2L_BEGIN_CALIBRATION);
      }
    }

    state(wait_calibration)
    {
      transition
      {
        if (FieldColorsBoard::get_lower_to_cognition() == FLDCOLBRD__L2C_CALIBRATION_DONE) {
          FieldColorsBoard::del_lower_to_cognition();
          goto move_head;
        }
      }
      action
      {
        theActivitySkill(BehaviorStatus::unknown);
        theLookAtAnglesSkill(0.f, TILT_PHASE1, 150_deg, HeadMotionRequest::lowerCamera);
        theStandSkill();
      }
    }

    state(move_head)
    {
      transition
      {
        // wait 3 seconds to reposition the head
        if (state_time > 3000) {
          goto begin_phase2;
        }
      }
      action
      {
        theActivitySkill(BehaviorStatus::unknown);
        theLookAtAnglesSkill(0.f, TILT_PHASE2, 150_deg, HeadMotionRequest::lowerCamera);
        theStandSkill();
      }
    }

    state(begin_phase2)
    {
      transition
      {
        if (state_time) {
          goto wait_phase2;
        }
      }
      action
      {
        theActivitySkill(BehaviorStatus::unknown);
        theLookAtAnglesSkill(0.f, TILT_PHASE2, 150_deg, HeadMotionRequest::lowerCamera);
        theStandSkill();
        FieldColorsBoard::set_cognition_to_lower(FLDCOLBRD__C2L_BEGIN_PHASE2);
      }
    }

    state(wait_phase2)
    {
      transition
      {
        if (FieldColorsBoard::get_lower_to_cognition() == FLDCOLBRD__L2C_CALIBRATION_DONE) {
          FieldColorsBoard::del_lower_to_cognition();
          goto done;
        }
      }
      action
      {
        theActivitySkill(BehaviorStatus::unknown);
        theLookAtAnglesSkill(0.f, TILT_PHASE2, 150_deg, HeadMotionRequest::lowerCamera);
        theStandSkill();
      }
    }

    state(done)
    {
      transition
      {
        ;
      }
      action
      {
        theActivitySkill(BehaviorStatus::unknown);
        theLookForwardSkill();
        theStandSkill();
        theSaySkill("The whole calibration procedure has been completed successfully. The calibration has been saved to my configuration files.");
      }
    }

  }

};

MAKE_CARD(ColorCalibrationCard);
