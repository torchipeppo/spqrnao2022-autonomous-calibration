/**
 * @file Esorcista.cpp
 *
 * This file implements the implementation of the Esorcista skill.
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
#include "Tools/Math/Angle.h"

SKILL_IMPLEMENTATION(EsorcistaImpl,
{,
  IMPLEMENTS(Esorcista),
  CALLS(Stand),
  REQUIRES(GetUpEngineOutput),
  // REQUIRES(Angle),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(HeadMotionRequest),
});

class EsorcistaImpl : public EsorcistaImplBase
{
  option(Esorcista)
  {
    
    initial_state(zero)
    {
      transition
      {
        if(state_time > 800.f){
            goto one;
        }
      }
      action
      {
        setRequest(p, HeadMotionRequest::autoCamera, 0.f, 40.f, pi, false);
        // theStandSkill();
      }
    }

    state(one)
    {
      transition
      {
        if(state_time > 800.f){
            goto twoFinal;
        }
      }
      action
      {
        setRequest(p, HeadMotionRequest::autoCamera, p.direction*22, 40.f, pi, false);
      }
    }

    state(twoFinal)
    {
        transition{
          if(state_time > 800.f){
            goto two;
        }  
        }
      action
      {
       setRequest(p, HeadMotionRequest::autoCamera, p.direction*60, 40.f, pi, false);
      }
    }

    state(two)
    {
        transition{
          if(state_time > 100.f){
            goto three;
        }  
        }
      action
      {
       setRequest(p, HeadMotionRequest::autoCamera, p.direction*120, 40.f, pi, false);
      }
    }
      state(three)
    {
        transition{
          if(state_time > 800.f){
            goto four;
        }  
        }
      action
      {
       setRequest(p, HeadMotionRequest::autoCamera, p.direction*22, 40.f, pi, false);
      }
    }
      state(four)
    {
        transition{
          if(state_time > 800.f){
            goto five;
        }  
        }
      action
      {
       setRequest(p, HeadMotionRequest::autoCamera, 0, 40.f, pi, false);
      }
    }
  state(five)
    {
        transition{
          if(state_time > 800.f){
            goto sexfinal;
        }  
        }
      action
      {
       setRequest(p, HeadMotionRequest::autoCamera, -p.direction*22, 40.f, pi, false);
      }
    }
    
  state(sexfinal)
    {
        transition{
          if(state_time > 800.f){
            goto six;
        }  
        }
      action
      {
       setRequest(p, HeadMotionRequest::autoCamera, -p.direction*60, 40.f, pi, false);
      }
    }
  state(six)
    {
        transition{
          if(state_time > 1000.f){
            goto seven;
        }  
        }
      action
      {
       setRequest(p, HeadMotionRequest::autoCamera, -p.direction*120, 40.f, pi, false);
      }
    }
      state(seven)
    {
        transition{
          if(state_time > 1500.f){
            goto seven;
        }  
        }
      action
      {
       setRequest(p, HeadMotionRequest::autoCamera, -p.direction*40, 40.f, pi, false);
      }
    }
  }

  void setRequest(const Esorcista& p, HeadMotionRequest::CameraControlMode camera, Angle pan, Angle tilt, Angle speed, bool stopAndGoMode = false)
  {
    theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
    theHeadMotionRequest.cameraControlMode = camera;
    theHeadMotionRequest.pan = Angle::fromDegrees( pan);
    theHeadMotionRequest.tilt = Angle::fromDegrees( tilt);
    theHeadMotionRequest.speed = speed;
    theHeadMotionRequest.stopAndGoMode = stopAndGoMode;
    theLibCheck.inc(LibCheck::headMotionRequest);
    theStandSkill();//it is the same that the "stopAndGoMode" parameter?
  }
};

MAKE_SKILL_IMPLEMENTATION(EsorcistaImpl);
