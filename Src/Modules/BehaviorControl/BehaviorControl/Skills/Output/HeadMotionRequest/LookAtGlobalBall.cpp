/**
 * @file LookLeftAndRight.cpp
 *
 * This file implements the implementation of the LookLeftAndRight skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Communication/GameInfo.h"


SKILL_IMPLEMENTATION(LookAtGlobalBallImp,
{,
  IMPLEMENTS(LookAtGlobalBall),
  CALLS(LookAtPoint),
  REQUIRES(GetUpEngineOutput),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(HeadMotionRequest),
  REQUIRES(TeamBallModel),
  REQUIRES(BallModel),
  REQUIRES(GameInfo),

});

class LookAtGlobalBallImp : public LookAtGlobalBallImpBase
{
  option(LookAtGlobalBall)
  {
    initial_state(start)
    {
      action
      {
        Pose2f globalBall;
            if(theGameInfo.state == STATE_SET)
                globalBall = theLibCheck.glob2Rel(0.f, 0.f);
            else
                globalBall = theLibCheck.glob2Rel(theTeamBallModel.position.x(),theTeamBallModel.position.y());
  
        theLookAtPointSkill(Vector3f(globalBall.translation.x(), globalBall.translation.y(), 0));
      }
    }

 }

//   void setRequest(const LookLeftAndRight& p, HeadMotionRequest::CameraControlMode camera, Angle pan, Angle tilt, Angle speed, bool stopAndGoMode = false)
//   {
//     theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
//     theHeadMotionRequest.cameraControlMode = camera;
//     theHeadMotionRequest.pan = pan;
//     theHeadMotionRequest.tilt = tilt;
//     theHeadMotionRequest.speed = speed;
//     theHeadMotionRequest.stopAndGoMode = stopAndGoMode;
//     theLibCheck.inc(LibCheck::headMotionRequest);
//   }
};

MAKE_SKILL_IMPLEMENTATION(LookAtGlobalBallImp);
