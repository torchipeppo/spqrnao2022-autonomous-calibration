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


SKILL_IMPLEMENTATION(LookLeftAndRightImpl,
{,
  IMPLEMENTS(LookLeftAndRight),
  REQUIRES(GetUpEngineOutput),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(HeadMotionRequest),
});

class LookLeftAndRightImpl : public LookLeftAndRightImplBase
{
  option(LookLeftAndRight)
  {
    initial_state(s_1)
    {
      transition
      {
        if(state_time > 450.f){
            goto s_2;
        }
      }
      action
      {
        setRequest(p, HeadMotionRequest::upperCamera, 0.f, 0.38f, 100_deg, false);
      }
    }

    state(s_2)
    {
      transition
      {
        if(state_time > 450.f){
            goto s_3;
        }
      }
      action
      {
        setRequest(p, HeadMotionRequest::upperCamera, 0.5f, 0.38f, 100_deg, false);
      }
    }

    state(s_3)
    {
        transition{
          if(state_time > 450.f){
            goto s_4;
        }  
        }
      action
      {
       setRequest(p, HeadMotionRequest::upperCamera, 1.f, 0.38f, 100_deg, false);
      }
    }

    state(s_4)
    {
      transition
      {
        if(state_time > 450.f){
            goto s_5;
        }
      }
      action
      {
        setRequest(p, HeadMotionRequest::upperCamera, 1.5f, 0.38f, 100_deg, false);
      }
    }

    state(s_5)
    {
      transition
      {
        if(state_time > 450.f){
            goto s_6;
        }
      }
      action
      {
        setRequest(p, HeadMotionRequest::upperCamera, 1.f, 0.38f, 100_deg, false);
      }
    }

    state(s_6)
    {
        transition{
          if(state_time > 450.f){
            goto s_7;
        }  
        }
      action
      {
       setRequest(p, HeadMotionRequest::upperCamera, 0.5f, 0.38f, 100_deg, false);
      }
    }

    state(s_7)
    {
      transition
      {
        if(state_time > 450.f){
            goto s_8;
        }
      }
      action
      {
        setRequest(p, HeadMotionRequest::upperCamera, 0.f, 0.38f, 100_deg, false);
      }
    }

    state(s_8)
    {
      transition
      {
        if(state_time > 450.f){
            goto s_9;
        }
      }
      action
      {
        setRequest(p, HeadMotionRequest::upperCamera, -0.5f, 0.38f, 100_deg, false);
      }
    }

    state(s_9)
    {
        transition{
          if(state_time > 450.f){
            goto s_10;
        }  
        }
      action
      {
       setRequest(p, HeadMotionRequest::upperCamera, -1.f, 0.38f, 100_deg, false);
      }
    }

    state(s_10)
    {
      transition
      {
        if(state_time > 450.f){
            goto s_11;
        }
      }
      action
      {
        setRequest(p, HeadMotionRequest::upperCamera, -1.5f, 0.38f, 100_deg, false);
      }
    }

    state(s_11)
    {
      transition
      {
        if(state_time > 450.f){
            goto s_12;
        }
      }
      action
      {
        setRequest(p, HeadMotionRequest::upperCamera, -1.f, 0.38f, 100_deg, false);
      }
    }

    state(s_12)
    {
        transition{
          if(state_time > 450.f){
            goto s_1;
        }  
        }
      action
      {
       setRequest(p, HeadMotionRequest::upperCamera, -0.5f, 0.38f, 100_deg, false);
      }
    }

    


  }

  void setRequest(const LookLeftAndRight& p, HeadMotionRequest::CameraControlMode camera, Angle pan, Angle tilt, Angle speed, bool stopAndGoMode = false)
  {
    theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
    theHeadMotionRequest.cameraControlMode = camera;
    theHeadMotionRequest.pan = pan;
    theHeadMotionRequest.tilt = tilt;
    theHeadMotionRequest.speed = speed;
    theHeadMotionRequest.stopAndGoMode = stopAndGoMode;
    theLibCheck.inc(LibCheck::headMotionRequest);
  }
};

MAKE_SKILL_IMPLEMENTATION(LookLeftAndRightImpl);
