/**
 * @file LookLeftAndRightParametric.cpp
 *
 * This file implements the implementation of the LookLeftAndRight skill.
 *
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include <iostream>

SKILL_IMPLEMENTATION(ParametricLookLeftAndRightImpl,
{,
  IMPLEMENTS(ParametricLookLeftAndRight),
  REQUIRES(GetUpEngineOutput),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(HeadMotionRequest),
});

#define __STRINGIFY_I(arg) #arg
#define __STRINGIFY(arg) __STRINGIFY_I(arg)
#define DEBUG_CODE(code) \
  std::cout<<__STRINGIFY(code)<<": "<<std::to_string(code)<<std::endl;

class ParametricLookLeftAndRightImpl : public ParametricLookLeftAndRightImplBase
{
    option(ParametricLookLeftAndRight)
    {

        initial_state(rotateToLeftAngle)
        {
            transition
            {
                //DEBUG_CODE(getTotalAngle(p));
                //DEBUG_CODE(getTotalTime(p));
                //DEBUG_CODE(getLeftAngleTime(p));
                //DEBUG_CODE(getRightAngleTime(p));
                if(state_time > getLeftAngleTime(p)){
                    
                    goto rotateToRightAngle;
                }
            }
            action
            {
                setRequest(p, HeadMotionRequest::upperCamera, Angle::fromDegrees(p.leftAngle), Angle::fromDegrees(p.headTilt), Angle::fromDegrees(p.speed), false);
            }
        }

        state(rotateToRightAngle)
        {
            transition
            {
                //DEBUG_CODE(getLeftAngleTime(p) + getRightAngleTime(p));
                if(state_time > getLeftAngleTime(p) + getRightAngleTime(p)){
                    
                    goto rotateBackToCenter;
                }
            }
            action
            {
                setRequest(p, HeadMotionRequest::upperCamera, -Angle::fromDegrees(p.rightAngle), Angle::fromDegrees(p.headTilt), Angle::fromDegrees(p.speed), false);
            }
        }

        state(rotateBackToCenter)
        {
            transition
            {
                //DEBUG_CODE(getRightAngleTime(p));
                if(state_time > getRightAngleTime(p)){
                    
                    goto rotateToLeftAngle;
                }
            }
            action
            {
                setRequest(p, HeadMotionRequest::upperCamera, 0.f, Angle::fromDegrees(p.headTilt), Angle::fromDegrees(p.speed), false);
            }
        }
    }

    float getTotalAngle(const ParametricLookLeftAndRight& p)
    {
        return (p.leftAngle + p.rightAngle) * 2;
    }

    //In milliseconds
    float getTotalTime(const ParametricLookLeftAndRight& p)
    {
        //Speed is in degress/sec so we divide by 1000 to obtain degrees/millisec
        return getTotalAngle(p) / (p.speed/1000.f);
    }

    //In milliseconds
    float getLeftAngleTime(const ParametricLookLeftAndRight& p)
    {
        //std::cout<<"Initial angle: "<<std::to_string(theHeadMotionRequest.pan)<<std::endl;
        return p.leftAngle / (p.speed/1000.f);
    }

    //In milliseconds
    float getRightAngleTime(const ParametricLookLeftAndRight& p)
    {
        return p.rightAngle / (p.speed/1000.f);
    }

    void setRequest(const ParametricLookLeftAndRight& p, HeadMotionRequest::CameraControlMode camera, Angle pan, Angle tilt, Angle speed, bool stopAndGoMode = false)
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

MAKE_SKILL_IMPLEMENTATION(ParametricLookLeftAndRightImpl);
