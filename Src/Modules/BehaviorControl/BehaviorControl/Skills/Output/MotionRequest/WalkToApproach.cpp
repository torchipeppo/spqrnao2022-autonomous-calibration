/**
 * @file WalkToApproach.cpp
 *
 * This file implements the implementation of the WalkToApproach skill.
 * his skill allows a robot to move to the ball and approach it in the direction of a given target
 * 
 * @author Emanuele Antonioni
 */
 
  // *******      NOTE: THIS FILE HAS BEEN MERGED WITH WalkToPass.cpp

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/Configuration/FieldDimensions.h"

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/spqr_representations/PassShare.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Math/BHMath.h"
#include <iostream>

SKILL_IMPLEMENTATION(WalkToApproachImpl,
{,
  IMPLEMENTS(WalkToApproach),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  REQUIRES(FieldBall),
  REQUIRES(PassShare),
  REQUIRES(RobotPose),
  REQUIRES(FieldDimensions),
  CALLS(Activity),

  MODIFIES(MotionRequest),
  MODIFIES(BehaviorStatus),

  DEFINES_PARAMETERS(
  {,
    (Angle)(10_deg) angleToShootThreshold,
    
  }),
});

class WalkToApproachImpl : public WalkToApproachImplBase
{
  void execute(const WalkToApproach& p) override
  {
    if(p.target.translation.x()>=theFieldDimensions.xPosOpponentGroundline){ // KICK TO GOAL CASE
        //std::cout<<"KICK\n";
        Angle target_angle = ( theRobotPose.inversePose * Vector2f(p.target.translation.x(), p.target.translation.y() ) ).angle();
        theMotionRequest.motion = MotionRequest::walk;
        theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
        theMotionRequest.walkRequest.target = Pose2f( target_angle, theFieldBall.positionRelative.x() - p.offsetX,
                                                 theFieldBall.positionRelative.y() - p.offsetY );
        theMotionRequest.walkRequest.speed = Pose2f(1.f,1.f,1.f);
        theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
        theLibCheck.inc(LibCheck::motionRequest);
    }else{                                // PASS CASE
        //std::cout<<"PASS\n";
        Vector2f target = p.target.translation;
        if(thePassShare.readyPass == 0) return;

        theBehaviorStatus.passTarget = thePassShare.passingTo;
        theBehaviorStatus.shootingTo = target;
        theLibCheck.inc(LibCheck::passTarget);
        theActivitySkill(BehaviorStatus::passing);

        Angle target_angle = ( theRobotPose.inversePose * Vector2f(target.x(), target.y() ) ).angle();
        theMotionRequest.motion = MotionRequest::walk;
        theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
        theMotionRequest.walkRequest.target = Pose2f( target_angle, theFieldBall.positionRelative.x() - p.offsetX,
                                                 theFieldBall.positionRelative.y() - p.offsetY );
        theMotionRequest.walkRequest.speed = Pose2f(1.f,1.f,1.f);
        theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
        theLibCheck.inc(LibCheck::motionRequest);
    }
  }

  bool isDone(const WalkToApproach&) const override
  {
    return theMotionInfo.motion == MotionRequest::walk;
  }
};

MAKE_SKILL_IMPLEMENTATION(WalkToApproachImpl);
