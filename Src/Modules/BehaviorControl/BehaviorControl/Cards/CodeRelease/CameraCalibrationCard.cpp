/**
 * @file CameraCalibrationCard.cpp
 *
 * Card that defines the camera calibration behavior.
 *
 * @author Amila Sikalo
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

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Communication/GameInfo.h"

//#include "Modules/Configuration/CameraCalibrator/AutomaticCameraCalibrator.h"
#include "Modules/Configuration/CameraCalibrator/CameraCalibratorMain.h"

//see above
#pragma GCC diagnostic pop

CARD(CameraCalibrationCard,
{,
  CALLS(Activity),
  CALLS(Say),
  
  CALLS(LookAtAngles),
  //currently unused, but might turn out useful
  // CALLS(LookAtPoint),
  // CALLS(LookForward),
  CALLS(Stand),
});

#define TILT_PHASE1 -0.65f
#define TILT_PHASE2 -1.4f

class CameraCalibrationCard : public CameraCalibrationCardBase
{

  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  option
  {
    initial_state(start)
    {
      transition
      {
        if (state_time > 3000)
          goto calibration_state;
      }
      action
      {
        //std::cout << "state start\n";
        theActivitySkill(BehaviorStatus::unknown);
        theLookAtAnglesSkill(0.f, TILT_PHASE1, 150_deg, HeadMotionRequest::lowerCamera);
        theStandSkill();
      }
    }

    state(calibration_state)
    {
      transition
      {
        if (CameraCalibrationMain::getCurrentStateCard() == 10)
          goto optimization_state;
      }
      action
      {
        //std::cout << "state calib\n";
        theActivitySkill(BehaviorStatus::unknown);
        theLookAtAnglesSkill(0.f, TILT_PHASE1, 150_deg, HeadMotionRequest::lowerCamera);
        CameraCalibrationMain::setCurrentStateCard(1);
      }
    }

    state(optimization_state)
    {
      transition
      {
        if (CameraCalibrationMain::getCurrentStateCard() == 0)
          goto finished;
      }
      action
      {
        //std::cout << "state opt\n";
        theActivitySkill(BehaviorStatus::unknown);
        theLookAtAnglesSkill(0.f, TILT_PHASE1, 150_deg, HeadMotionRequest::lowerCamera);
        CameraCalibrationMain::setCurrentStateCard(9);
      }
    }

    state(finished)
    {
      transition
      {
        ;
      }
      action
      {
        //std::cout << "Finished!\n";
        theActivitySkill(BehaviorStatus::unknown);
        theLookAtAnglesSkill(0.f, TILT_PHASE1, 150_deg, HeadMotionRequest::lowerCamera);
      }
    }

  }

};

MAKE_CARD(CameraCalibrationCard);