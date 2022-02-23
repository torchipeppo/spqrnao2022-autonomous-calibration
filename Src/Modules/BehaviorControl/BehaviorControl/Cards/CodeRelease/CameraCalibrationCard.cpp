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
#include "Modules/Configuration/CameraCalibrator/CameraCalibratorNewMain.h"

//see above
#pragma GCC diagnostic pop

CARD(CameraCalibrationCard,
{,
  CALLS(Activity),
  CALLS(Say),
  CALLS(Stand),
});

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
        if (state_time > 3000){
          theSaySkill("Starting calibration procedure");
          goto calibration_state;
        }
      }
      action
      {
        //std::cout << "state start\n";
        theActivitySkill(BehaviorStatus::unknown);
        theStandSkill();
        //theSaySkill("The camera calibration procedure has begun.");

      }
    }

    state(calibration_state)
    {
      transition
      {
        if (CameraCalibratorNewMain::getCurrentCameraCalibratorState() == 10){
          theSaySkill("Accumulated enough points. Starting optimizer");
          goto optimization_state;
        }
      }
      action
      {
        //std::cout << "state calib\n";
        theActivitySkill(BehaviorStatus::unknown);
        theStandSkill();
        CameraCalibratorNewMain::setCurrentCameraCalibratorState(1);
        //theSaySkill("The whole calibration procedure has been completed successfully. The calibration has been saved to my configuration files.");
      }
    }

    state(optimization_state)
    {
      transition
      {
        if (CameraCalibratorNewMain::getCurrentCameraCalibratorState() == 0){
          theSaySkill("Optimization finished.");
          goto finished;
          }
      }
      action
      {
        //std::cout << "state opt\n";
        theActivitySkill(BehaviorStatus::unknown);
        theStandSkill();
        CameraCalibratorNewMain::setCurrentCameraCalibratorState(9);
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
        std::cout << "Finished!\n";
        theActivitySkill(BehaviorStatus::unknown);
        theStandSkill();
        theSaySkill("The whole calibration procedure has been completed successfully. The calibration has been saved to my configuration files.");
      }
    }

  }

};

MAKE_CARD(CameraCalibrationCard);