/**
 * @file Skills.h
 *
 * This file declares all skills that are used by the current behavior.
 *
 * @author Probably many people who will not add themselves to this declaration.
 */

#pragma once

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/MotionControl/ArmKeyFrameRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/KickRequest.h"
#include "Representations/MotionControl/SpecialActionRequest.h"
#include "Representations/MotionControl/WalkKickGenerator.h"
#include "Tools/BehaviorControl/Framework/Skill/Skill.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/RobotParts/Arms.h"
#include <string>

namespace Skills
{
  /** This skill executes the get up engine. */
  SKILL_INTERFACE(GetUpEngine);

  /**
   * This skill executes an in walk kick.
   * @param walkKick The walk kick variant that should be executed
   * @param kickPose The pose at which the kick should be executed in robot-relative coordinates
   */
  SKILL_INTERFACE(InWalkKick, (const WalkKickVariant&) walkKick, (const Pose2f&) kickPose);

  /**
   * This skill executes a kick.
   * @param mirror Whether the kick should be mirrored
   * @param length The desired length of the kick (i.e. distance that the ball moves)
   * @param armsBackFix Use inverse elbow yaw if backLikeDevil is active (motion must be designed for that)
   */
  SKILL_INTERFACE(Kick, (bool) mirror, (float) length, (bool)(true) armsBackFix);

  /**
   * This skill walks to a target using a path planner.
   * @param speed The walking speed as ratio of the maximum speed in [0, 1]
   * @param target The target pose in absolute field coordinates
   */
  SKILL_INTERFACE(PathToTarget, (float) speed, (const Pose2f&) target);

  /**
   * This skill lets the robot execute a special action.
   * @param id The ID of the motion
   * @param mirror Whether the motion should be mirrored
   */
  SKILL_INTERFACE(SpecialAction, (SpecialActionRequest::SpecialActionID) id, (bool)(false) mirror);

  /** This skill makes the robot stand. */
  SKILL_INTERFACE(Stand);

  /**
   * This skill walks with a specified speed.
   * @param speed The walking speed in radians/s for the rotation and mm/s for the translation
   */
  SKILL_INTERFACE(Turn360);

  /**
   * This skill turns the robot.
   * @param target to turn
   */
  SKILL_INTERFACE(Turn, (const Pose2f&) target);

  /**
   * This skill go behind thestriker position.
   */
  
  SKILL_INTERFACE(GoBehindStriker, (const Vector2f&) striker_offset_pos);

  /**
   * This skill Turns 2pi radians.
   */
  
  SKILL_INTERFACE(WalkAtAbsoluteSpeed, (const Pose2f&) speed);

  /**
   * This skill walks with a specified speed relative to the configured maximum.
   * @param speed The walking speed as ratio of the maximum speed in [0, 1]
   */
  SKILL_INTERFACE(WalkAtRelativeSpeed, (const Pose2f&) speed);

  /**
   * This skill walks to a (relative) target.
   * @param speed The walking speed as ratio of the maximum speed in [0, 1]
   * @param target The target pose in robot-relative coordinates
   */
  SKILL_INTERFACE(WalkToTarget, (const Pose2f&) speed, (const Pose2f&) target);

 
/**
   * This skill walks to the ball to approach a global target and kicks to goal or passes.
   * @param target The target to shoot in global coordinates
   */
  SKILL_INTERFACE(WalkToApproach, (const Pose2f&) target, (float) offsetX, (float) offsetY, (bool)(true) useLeft);
 
 /**
   * This skill position the robot behind the ball using controller based approach.
   * @param fromTarget The first target point
   * @param toTarget The second target point to create a vector
   * @param offsetX Offset in x-axis
   * @param offsetY Offset in y-axis
   * @param useLeftFoot The choose left or right foot
   */
  SKILL_INTERFACE(WalkToBallController,
                  (const Pose2f&) fromTarget,
                  (const Pose2f&) toTarget,
                  (float)(0.0) offsetX,
                  (float)(0.0) offsetY,
                  (bool)(true) useLeftFoot,
                  (float)(1.0) gainWhenBallOnSide,
                  (float)(0.05) kp1,
                  (float)(-0.003) kp2,
                  (float)(0.003) kp3
);


 /** TO BE MERGED WITH THE ORIGINAL SKILL 
   * This skill position the robot behind the ball using controller based approach.
   * @param fromTarget The first target point
   * @param toTarget The second target point to create a vector
   * @param offsetX Offset in x-axis
   * @param offsetY Offset in y-axis
   * @param useLeftFoot The choose left or right foot
   */
  SKILL_INTERFACE(WalkToBallControllerDefender,
                  (const Pose2f&) fromTarget,
                  (const Pose2f&) toTarget,
                  (float)(0.0) offsetX,
                  (float)(0.0) offsetY,
                  (bool)(true) useLeftFoot,
                  (float)(1.0) gainWhenBallOnSide,
                  (float)(10.0) forwardThreshold,
                  (float)(50.0) sidewaysThreshold,
                  (float)(7.0) rotationThreshold,
                  (float)(0.05) kp1,
                  (float)(-0.003) kp2,
                  (float)(0.003) kp3
);

  /**
   * Implementation of the approaching behavior employed in Challenge 3 of RoboCup 2021
   * @param target The target to shoot in global coordinates
   * @param offsetX Offset from the ball
   * @param offsetY Offset from the ball
   * @param normalWalkSpeed Normal walk speed
   * @param slowerWalkSpeed Walk speed while finalizing approach
   * @param allowLookAround If true, the robot will LookLeftAndRight some of the time. If false, it will always look at the ball.
   */
  SKILL_INTERFACE(
    Approacher2021,
    (const Pose2f&) target,
    (float)(230.0) offsetX,
    (float)(50.0) offsetY,
    (float)(1.0) normalWalkSpeed,
    (float)(0.5) slowerWalkSpeed,
    (bool)(false) allowLookAround
  );
 
 
    /**
   * Skill that has the robot carry the ball to a certain position.
   * NOT USED@param target The position where the ball has to be carried (GLOBAL COORDINATES)
   * @param normalWalkSpeed Normal walk speed
   * @param slowerWalkSpeed Walk speed while finalizing approach
   * @param putArmsOnBack If true, the robot will put arms on its back if there are obstacles nearby.
   */
  SKILL_INTERFACE(
    CarryBall,
    //(const Vector2f&) destination,
    (float)(0.7) normalWalkSpeed,
    (float)(0.6) slowerWalkSpeed,
    (bool)(false) putArmsOnBack//,
  );

  /**
   * Skill that has the robot carry the ball to a certain position.
   * NOT USED@param target The position where the ball has to be carried (GLOBAL COORDINATES)
   * @param normalWalkSpeed Normal walk speed
   * @param slowerWalkSpeed Walk speed while finalizing approach
   * @param putArmsOnBack If true, the robot will put arms on its back if there are obstacles nearby.
  */ 
  SKILL_INTERFACE(
    CarryBallWithController,
    //(const Vector2f&) destination,
    (float)(0.7) normalWalkSpeed,
    (float)(0.6) slowerWalkSpeed,
    (bool)(false) putArmsOnBack
  );


  /**
   * This skill walks to an absolute target using path planner.
   * @param speed The walking speed as ratio of the maximum speed in [0, 1]
   * @param target The target pose in global coordinates
   */
  SKILL_INTERFACE(WalkToTargetPathPlanner, (const Pose2f&) speed, (const Pose2f&) target);

 
/**
   * This skill walks to an absolute target using path planner and facing always the opponent goal.
   * @param speed The walking speed as ratio of the maximum speed in [0, 1]
   * @param target The target pose in global coordinates
   */
  SKILL_INTERFACE(WalkToTargetPathPlannerStraight, (const Pose2f&) speed, (const Pose2f&) target);


  /**
   * This skill executes a key frame motion with both arms.
   * @param motion The motion that the arm should execute
   * @param fast Whether states should not be interpolated
   */
  SKILL_INTERFACE(KeyFrameArms, (ArmKeyFrameRequest::ArmKeyFrameId) motion, (bool)(false) fast);

  /**
   * This skill executes a key frame motion with a single arm.
   * @param motion The motion that the arm should execute
   * @param arm The arm that should execute the motion
   * @param fast Whether states should not be interpolated
   */
  SKILL_INTERFACE(KeyFrameSingleArm, (ArmKeyFrameRequest::ArmKeyFrameId) motion, (Arms::Arm) arm, (bool)(false) fast);

  /**
   * This skill lets one arm point at some point.
   * @param localPoint The point in robot-relative coordinates
   */
  SKILL_INTERFACE(PointAt, (const Vector3f&) localPoint);

  /**
   * This skill lets a specific arm point at some point.
   * @param localPoint The point in robot-relative coordinates
   * @param arm The arm that shall be used for pointing
   */
  SKILL_INTERFACE(PointAtWithArm, (const Vector3f&) localPoint, (Arms::Arm) arm);
/*
 * @param TurnRate
   */
  SKILL_INTERFACE(WalkQuarterCircleDownLeft,(float) TurnRate)
  /**
   * This skill moves the head so that a camera looks at specified angles.
   * @param pan The target pan angle
   * @param tilt The target tilt angle
   * @param speed The speed with which to move the head
   * @param camera The camera which should have the specified angles
   */
  SKILL_INTERFACE(LookAtAngles, (Angle) pan, (Angle) tilt, (float)(180_deg) speed, (HeadMotionRequest::CameraControlMode)(HeadMotionRequest::autoCamera) camera);

  /**
   * This skill moves the head such that a specified (robot-relative) point is focused by one camera.
   * @param target The point to look at in robot-relative coordinates
   * @param camera The camera which should look at the point
   * @param speed The speed with which to move the head
   */
  SKILL_INTERFACE(LookAtPoint, (const Vector3f&) target, (HeadMotionRequest::CameraControlMode)(HeadMotionRequest::autoCamera) camera, (Angle)(180_deg) speed);

  /** This skill moves the head so that it looks forward.  */
  SKILL_INTERFACE(LookForward);

  /**
   * This skill sets the passTarget member of the BehaviorStatus.
   * @param passingTo The number of the teammate the ball is passed to
   * @param passTarget The Vector2f representing the final desired position of the ball
   * @param offsetX Ball approach offset X
   * @param offsetY Ball approach offset Y
   */
  SKILL_INTERFACE(WalkToPassForced, (int) passingTo, (float) offsetX, (float) offsetY, (bool)(true) useLeft, (const Vector2f&)(Vector2f::Zero()) passTarget);

  /**
   * This skill sets the goalTarget member of the BehaviorStatus.
   * @param goalTarget The position in the opponent goal where the ball should end up in global coordinates
   */
  SKILL_INTERFACE(GoalTarget, (const Vector2f&)(Vector2f::Zero()) goalTarget);

  /**
   * This skill sets the shootingTo member of the BehaviorStatus.
   * @param shootingTo The target where the ball should end up in global coordinates
   */
  SKILL_INTERFACE(SetTarget, (const Vector2f&)(Vector2f::Zero()) chosenTarget);



  //LOGGING SKILLS
  /**
   * These dummy skills are only used to show the value of their arguments on the "behavior" View of any robot in SimRobot.
   * 
   * @param Param String that represents the parameter name
   * @param Value Parameter value (string)
   */
  SKILL_INTERFACE(LogStringParameter, (const std::string&) Param, (const std::string&) Value);

  /**
   * This skill sets the goalTarget member of the BehaviorStatus.
   * @param Param String that represents the parameter name
   * @param Value Parameter value (float)
   */
  SKILL_INTERFACE(LogFloatParameter, (const std::string&) Param, (float) Value);

  //--------------



  /**
   * This skill sets the activity member of the BehaviorStatus.
   * @param activity The activity to set
   */
  SKILL_INTERFACE(Activity, (BehaviorStatus::Activity) activity);

  /**
   * This skill adds an annotation if it differs from the one that has been added in the last frame.
   * @param annotation The annotation message
   */
  SKILL_INTERFACE(Annotation, (const std::string&) annotation);

  /**
   * This skill plays a sound file if it differs from the one that has been played in the last frame.
   * @param name The name of the sound file
   */
  SKILL_INTERFACE(PlaySound, (const std::string&) name);

  /**
   * This skill makes the Nao say something if it differs from what was said in the last frame.
   * @param name The text to be synthesized and pronounced
   */
  SKILL_INTERFACE(Say, (const std::string&) text);

  SKILL_INTERFACE(LookLeftAndRight);
  SKILL_INTERFACE(LookRightAndLeft);

  /**
   * This skill moves the head left and right between a leftAngle and a rightAngle BOTH POSITIVE, using a certain headTilt, at a certain speed.
   * @param leftAngle pan angle reached by the head when moving left
   * @param rightAngle pan angle reached by the head when moving right
   * @param headTilt tilt angle of the head through the whole movement
   * @param speed pan rotation speed 
   */
  SKILL_INTERFACE(ParametricLookLeftAndRight, (float) leftAngle, (float) rightAngle, (float) headTilt, (float) speed);

  /**
   * Same as ParametricLookLeftAndRight but looks only once
   * @param leftAngle pan angle reached by the head when moving left
   * @param rightAngle pan angle reached by the head when moving right
   * @param headTilt tilt angle of the head through the whole movement
   * @param speed pan rotation speed 
   */
  SKILL_INTERFACE(ParametricLookLeftAndRightOnce, (float) leftAngle, (float) rightAngle, (float) headTilt, (float) speed);


   /**
   * This skill makes the Nao see to the global ball position.
   * @param none
   */
  SKILL_INTERFACE(LookAtGlobalBall);

     /**
   * This skill implementes the Esorcista from the previous code.
   * @param none
   */
  SKILL_INTERFACE(Esorcista, (const int) direction);

}
