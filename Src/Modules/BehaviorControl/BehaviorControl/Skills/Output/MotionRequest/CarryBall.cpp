/**
 * @file CarryBall.cpp
 *
 * Skill that moves and aligns the robot to the ball in a position such that
 * it's possible to carry it just by walking through it. Based on the behavior
 * used in Challenge 1 of RoboCup 2021, developed by Emanuele Musumeci, Elisa Foderaro
 * and Amila Sikalo.
 * 
 * @author Emanuele Musumeci (with previous work of Elisa Foderaro and Amila Sikalo)
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/Configuration/BallSpecification.h"

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Math/BHMath.h"
#include <iostream>

#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

#define __STRINGIFY_I(arg) #arg
#define __STRINGIFY(arg) __STRINGIFY_I(arg)
#define DEBUG_CODE(code) \
  std::cout<<__STRINGIFY(code)<<": "<<std::to_string(code)<<std::endl;

//POTENTIAL TODOs:
//1) Move logic regarding position from the BallCarrierModel to the skill
//2) Allow custom setting of ball-robot offsets
//3) More control on path change, threshold etc.


SKILL_IMPLEMENTATION(CarryBallImpl,
{,
  IMPLEMENTS(CarryBall),

  REQUIRES(ObstacleModel),
  REQUIRES(LibCheck),
  REQUIRES(RobotPose),

  REQUIRES(FieldDimensions),
  REQUIRES(BallSpecification),
  
  REQUIRES(BallModel),
  REQUIRES(FieldBall),

  REQUIRES(BallCarrierModel),
  

  CALLS(Activity),
  
  CALLS(Stand),
  CALLS(WalkToTarget),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTargetPathPlanner),
  CALLS(WalkToBallController),

  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(LookLeftAndRight),
  CALLS(LookRightAndLeft),
  CALLS(ParametricLookLeftAndRight),

  CALLS(KeyFrameArms),
  CALLS(InWalkKick),

  CALLS(SetTarget),
  CALLS(GoalTarget),

  LOADS_PARAMETERS(
  {,
    (int) initialWaitTime,
    (int) realignmentTimeout,
    

    (int) pathChangeTimeoutInsideCenterCircle,
    (int) pathChangeTimeoutOutsideCenterCircle,


    (int) ballNotSeenTimeout,
    (float) ballPositionChangedThreshold,
    

    (Rangef) approachXRange,
    (Rangef) approachYRange,
    (Rangef) largeApproachYRange,
    (Rangef) smallBallAlignmentRange,
    (Rangef) largeBallAlignmentRange,
    (Rangef) ANGLE_TO_BALL_RANGE_FOR_ROTATION_DURING_REALIGNMENT,

    (bool) USE_DYNAMIC_APPROACH_POINT_WHEN_INSIDE_APPROACH_AREA,

    (float) nearbyObstacleRadius, //Radius inside which an obstacle is considered "nearby"

    (int) maxKickWaitTime,

    //NOT USED RIGHT NOW (bool) goToCenterWhenSearchingForBall, //Recovery behavior, when ball is lost, the robot performs a 360° turn hoping to find it again. 
    //                                       //If it doesn't find it it might have gone out so the robot goes back to the center of the field

    (float) safetyObstacleRadius,         // default = 500; used to move hands for stability when not around obstacles
    (float) dangerObstacleRadius,         //At and below this radius, the robot will walk at slowerWalkSpeed, between this and safetyObstacleRadius, the speed will be proportional to the distance, above safetyObstacleRadius, it will be normalWalkSpeed

    (float) robotArmsRange,               //Clearance radius of an arm movement

    (float) MAX_OBSTACLE_LOCALIZATION_DRIFT, //Maximum drift of an obstacle after which the path is recomputed

    (bool) USE_FOOT_OFFSET,               //Use an offset added to the approach point to align the ball to the foot when InWalkKicking
    (float) BALL_KICK_OFFSET_Y,           //Foot offset along the Y coordinate

    (bool) LOOK_LEFT_AND_RIGHT_WHILE_ALIGNING,
    (bool) LOOK_LEFT_AND_RIGHT_WHILE_WALKING_TO_BALL,
    (bool) LOOK_LEFT_AND_RIGHT_WHILE_KICKING,
    (float) MAX_LOOKING_ANGLE,

    (float) LEFT_AND_RIGHT_LOOKING_SPEED,

    (float) START_LOOKING_LEFT_AND_RIGHT_AT_X_COORDINATE,

    (bool) WALK_ON_BALL,
    //CODE IS COMMENTED (float) USE_IN_WALK_KICKS_FOR_DISTANCE_MORE_THAN,

    (bool) USE_FIGHTER_FOR_REALIGNMENT,

    //Debug info
    (bool) SHOW_CONDITIONS_DEBUG,
    (bool) DEBUG_MODE,

    //PARAMETER (bool) putArmsOnBack,
  }),
});

class CarryBallImpl : public CarryBallImplBase
{

  Vector2f getNearestObstacle(bool global = false) const
  {
    Vector2f nearestObstacle;
    float nearestObstacleDistance = -1;
    for(auto obs : theObstacleModel.obstacles)
    {
      float currentObsDistance = obs.center.norm();
      if(nearestObstacleDistance == -1 || obs.center.norm() < nearestObstacleDistance)
      {
          nearestObstacle = Vector2f(obs.center.x(), obs.center.y());
          nearestObstacleDistance = currentObsDistance;     
      }
    }

    if(global) 
      return theLibCheck.rel2Glob(nearestObstacle.x(), nearestObstacle.y()).translation;
    else 
      return nearestObstacle;
  }
  
  float decideWalkSpeed(float minObstacleRadius, float maxObstacleRadius, float minWalkSpeed, float maxWalkSpeed)
  {
    float distanceFromNearestObs = theLibCheck.distance(theRobotPose.translation, getNearestObstacle(true));
    if(distanceFromNearestObs > maxObstacleRadius)
    {
      return maxWalkSpeed;
    }
    else if(distanceFromNearestObs < maxObstacleRadius && distanceFromNearestObs > minObstacleRadius)
    {
      return theLibCheck.mapToInterval(distanceFromNearestObs - minObstacleRadius, minObstacleRadius, maxObstacleRadius, minWalkSpeed, maxWalkSpeed);
    }
    else
    {
      return minWalkSpeed;
    }
  }

  ArmKeyFrameRequest::ArmKeyFrameId decideArmsPosition(float maxObstacleRadius, bool putArmsOnBack)
  {
    Vector2f nearestObs = getNearestObstacle(true);
    if(theLibCheck.distance(theRobotPose.translation, nearestObs) > maxObstacleRadius
      || theRobotPose.translation.x() > nearestObs.x() + robotArmsRange
      || !putArmsOnBack)
    {
      return ArmKeyFrameRequest::useDefault;
    }
    else 
    {
      return ArmKeyFrameRequest::back;
    }
  }

  bool isRightFootUsedToKick(bool kickWithRightFoot)
  {
    if(getNearestObstacle().y() < -300.f && !kickWithRightFoot)
    {
      return true;
    }
    else if(getNearestObstacle().y() > 300.f && kickWithRightFoot)
    {
      return false;
    }
    else
    {
      return kickWithRightFoot;
    }
  }

  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }

  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }

  int countNearbyObstacles(float distanceRadius) const
  {
    int obstacleCount = 0;
    for(auto obs : theObstacleModel.obstacles)
    {
      if(theLibCheck.distance(obs.center, theRobotPose) < distanceRadius)
      {
        obstacleCount++;
      }
    }  

    return obstacleCount;
  }

  bool isRobotInCenterCircle()
  {
    return theRobotPose.translation.norm() < theFieldDimensions.centerCircleRadius;
  }

  /*bool useLongKick()
  {
    return false;
  }*/

  void callParametricLook()
  {
    /*
    Vector2f relativePenaltyMark = theLibCheck.glob2Rel(theFieldDimensions.xPosOpponentPenaltyMark, 0.f).translation;
    if(relativePenaltyMark.x() >= 0)
    {
      if(relativePenaltyMark.y() < 0)
      {
        theParametricLookLeftAndRightSkill(0, abs(Angle(relativePenaltyMark.angle()).toDegrees()), 20, LEFT_AND_RIGHT_LOOKING_SPEED);
      }
      else
      {
        theParametricLookLeftAndRightSkill(abs(Angle(relativePenaltyMark.angle()).toDegrees()), 0, 20, LEFT_AND_RIGHT_LOOKING_SPEED);
      }
    }
    */

    float lookingAngle;
    if(theRobotPose.translation.x() <= theFieldDimensions.xPosOpponentPenaltyMark)
    {
        lookingAngle = theLibCheck.mapToInterval(theRobotPose.translation.x(), 0, theFieldDimensions.xPosOpponentPenaltyMark, 0, MAX_LOOKING_ANGLE);
    }
    else
    {
        lookingAngle = MAX_LOOKING_ANGLE - theLibCheck.mapToInterval(theRobotPose.translation.x(), theFieldDimensions.xPosOpponentPenaltyMark, theFieldDimensions.xPosOpponentGroundline, 0, 90);
    }

    /*if(theRobotPose.translation.y() > 0)
    {
        theParametricLookLeftAndRightSkill(0, lookingAngle, 20, LEFT_AND_RIGHT_LOOKING_SPEED);
    }
    else
    {
        theParametricLookLeftAndRightSkill(lookingAngle, 0, 20, LEFT_AND_RIGHT_LOOKING_SPEED);
    }*/

    Vector2f relativePenaltyMark = theLibCheck.glob2Rel(theFieldDimensions.xPosOpponentPenaltyMark, 0.f).translation;
    if(relativePenaltyMark.x() >= 0)
    {
      //if(relativePenaltyMark.y() > 0)
      //{
        theParametricLookLeftAndRightSkill(0, lookingAngle, 20, LEFT_AND_RIGHT_LOOKING_SPEED);
      //}
      //else
      //{
      //  theParametricLookLeftAndRightSkill(lookingAngle, 0, 20, LEFT_AND_RIGHT_LOOKING_SPEED);
      //}
    }
    else
    {
      theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
    }
  }

  Vector2f chooseGoalTarget()
  {
    bool useHeuristic = true;
    
    Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

    if(globalBall.x() > theFieldDimensions.xPosOpponentGroundline - theBallCarrierModel.xRangeDistanceFromGoalToUseKicks.max)
    {
      useHeuristic = false;
    }

    return theLibCheck.goalTarget(false, useHeuristic);
  }

  bool pathChosen = false;

  //To avoid oscillation in front of the goal
  bool footChosen = false;
  bool kickWithRightFoot = true;


  float pathChangeTime = 0.f;
  float pathChangeTimeout;


  Vector2f chosenDynamicTarget;
  Vector2f currentDestination;
  Vector2f previousBallPosition;
  int previousNearbyObstaclesCount = 0;
  Vector2f nearestObstacleGlobalPositionWhenPathWasComputed = getNearestObstacle(true);
  

  //Ball searcher (DISABLED -> USING SEARCHERS)
  /*bool isSearchingToggle = false;
  float rotationVel;
  Angle startingRobotAngleWhenSearching;
  bool fullLoopRevolution = false;*/


  option(CarryBall)
  {
    common_transition
    {
      if(isRobotInCenterCircle())
      {
        pathChangeTimeout = pathChangeTimeoutInsideCenterCircle;
      } 
      else
      {
        pathChangeTimeout = pathChangeTimeoutOutsideCenterCircle;
      }

      //Number of obstacles changed
      if(/*!isSearchingToggle && */ countNearbyObstacles(nearbyObstacleRadius) != previousNearbyObstaclesCount && (option_time - pathChangeTime) > pathChangeTimeout)
      {
        std::cout<<"common transition -> choose_path: obstacle number changed: "<<std::to_string(previousNearbyObstaclesCount)<<" before, "<<std::to_string(countNearbyObstacles(nearbyObstacleRadius))<<"now"<<std::endl;
        goto choose_path;
      }

      //Same number of obstacles but big position drift due to localization
      if(/*!isSearchingToggle && */ theLibCheck.distance(getNearestObstacle(true), nearestObstacleGlobalPositionWhenPathWasComputed) > MAX_OBSTACLE_LOCALIZATION_DRIFT && (option_time - pathChangeTime) > pathChangeTimeout)
      {
        std::cout<<"common transition -> choose_path: obstacle position drift too high"<<std::endl;
        goto choose_path;
      }

      Vector2f lastPercievedBallPosition = theLibCheck.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y() ).translation;
      float angleToBall = Angle(theLibCheck.angleToTarget( lastPercievedBallPosition.x(), lastPercievedBallPosition.y())).toDegrees();

      //if(theBallModel.estimate.position.x() < approachXRange.min)
      //{
        if(Rangef(-theBallSpecification.radius*3, theBallSpecification.radius*3).isInside(theBallModel.estimate.position.x())
        && !ANGLE_TO_BALL_RANGE_FOR_ROTATION_DURING_REALIGNMENT.isInside(angleToBall))
        {
          if(!approachYRange.isInside(theBallModel.estimate.position.y()))
          {
            std::cout<<"common_transition -> centerBallInView"<<std::endl;
            goto centerBallInView; //go back
          }
        }
      //}

    }

    initial_state(start)
    {
      std::cout<<"CarryBallSkill: start"<<std::endl;
      transition
      {
        if(state_time > initialWaitTime)
          goto choose_path;
      }

      action
      {
        theActivitySkill(BehaviorStatus::approach_and_carry_start);
        
        theLookForwardSkill();
        theStandSkill();
      }
    }


    state(turnToBall)
    {
      
      transition
      {
        /*if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          std::cout<<"turnToBall -> searchForBall: ball not found"<<std::endl;
          goto searchForBall;
        }*/

        if(smallBallAlignmentRange.isInside(calcAngleToTarget(chosenDynamicTarget).toDegrees()))
        {
          std::cout<<"turnToBall -> choose_path: turned to ball"<<std::endl;
          goto choose_path;
        }
      }

      action
      {
        theActivitySkill(BehaviorStatus::realigning_to_ball);
        
        float walkSpeed = decideWalkSpeed(dangerObstacleRadius, safetyObstacleRadius, p.slowerWalkSpeed, p.normalWalkSpeed);
        theKeyFrameArmsSkill(decideArmsPosition(safetyObstacleRadius, p.putArmsOnBack), false);

        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(1.0f, walkSpeed, walkSpeed), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));

      }
    }

    state(choose_path)
    {
      footChosen = false;

      transition
      {

        if(DEBUG_MODE)
        {
          goto debug_state;
        }

        if(pathChosen)
        {
          pathChosen = false;

          std::cout<<"choose_path -> secondStepAlignment: target chosen"<<std::endl;
          goto secondStepAlignment;
        }
      }
      action
      {

        theActivitySkill(BehaviorStatus::choosing_target);

        float walkSpeed = decideWalkSpeed(dangerObstacleRadius, safetyObstacleRadius, p.slowerWalkSpeed, p.normalWalkSpeed);
        theKeyFrameArmsSkill(decideArmsPosition(safetyObstacleRadius, p.putArmsOnBack), false);

        currentDestination = chooseGoalTarget();
        chosenDynamicTarget = theBallCarrierModel.dynamicTarget.translation;
        previousBallPosition = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        previousNearbyObstaclesCount = countNearbyObstacles(nearbyObstacleRadius);
        nearestObstacleGlobalPositionWhenPathWasComputed = getNearestObstacle(true);
      
        
        pathChosen = true;
        pathChangeTime = option_time;
        
        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        theStandSkill();

//NOTICE: the foot is chosen (based on the nearest obstacle position) in choose_path because choose_path is visited every time 
//the number or position of nearby obstacles changes
        footChosen = true;
        kickWithRightFoot = isRightFootUsedToKick(kickWithRightFoot);
        //kickWithRightFoot = false;

      }
    }

    state(debug_state)
    {
      action
      {
        theActivitySkill(BehaviorStatus::debug_standing);
        //theLookForwardSkill();          
        Vector2f relativePenaltyMark = theLibCheck.glob2Rel(theFieldDimensions.xPosOpponentPenaltyMark, 0.f).translation;
        if(relativePenaltyMark.x() >= 0)
        {
          if(relativePenaltyMark.y() > 0)
          {
            theParametricLookLeftAndRightSkill(0, abs(Angle(relativePenaltyMark.angle()).toDegrees()), 20, LEFT_AND_RIGHT_LOOKING_SPEED);
          }
          else
          {
            theParametricLookLeftAndRightSkill(abs(Angle(relativePenaltyMark.angle()).toDegrees()), 0, 20, LEFT_AND_RIGHT_LOOKING_SPEED);
          }
        }
        theStandSkill();
      }
    }

    state(walkToBall)
    {
      transition
      {

        /*if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          std::cout<<"walkToBall -> searchForBall: ball not found"<<std::endl;
          goto searchForBall;
        }*/

        //if the LOCAL ball is inside the approach on the x axis 
        //AND the LOCAL ball is inside the approach range on the y axis 
        //AND the robot is aligned with the target
        float offsetBallYPosition;
        if(USE_FOOT_OFFSET)
        {
          if(kickWithRightFoot)
          {
            offsetBallYPosition = theBallModel.estimate.position.y() + BALL_KICK_OFFSET_Y;
          }
          else
          {
            offsetBallYPosition = theBallModel.estimate.position.y() - BALL_KICK_OFFSET_Y;
          }
        }
        else
        {
          offsetBallYPosition = theBallModel.estimate.position.y();
        }

        if(approachXRange.isInside(theBallModel.estimate.position.x())
        && approachYRange.isInside(offsetBallYPosition)
        && smallBallAlignmentRange.isInside(calcAngleToTarget(chosenDynamicTarget).toDegrees()))
        {
          std::cout<<"walkToBall -> kick: OK to kick"<<std::endl;
          goto kick;
        }
        
        if(theBallModel.estimate.position.norm() < approachXRange.max
        && approachXRange.isInside(theBallModel.estimate.position.x())
        && !approachYRange.isInside(offsetBallYPosition))
        {
          //Case B: Robot is in the X range but not in the Y range
          std::cout<<"walkToBall -> secondStepAlignment: Case B"<<std::endl;
          goto secondStepAlignment;
        }
        
      }

      action
      {
        theActivitySkill(BehaviorStatus::reaching_ball);

        float walkSpeed = decideWalkSpeed(dangerObstacleRadius, safetyObstacleRadius, p.slowerWalkSpeed, p.normalWalkSpeed);
        theKeyFrameArmsSkill(decideArmsPosition(safetyObstacleRadius, p.putArmsOnBack), false);

        Vector2f ballPositionRelative = theBallModel.estimate.position;

        //Look for landmarks while walking to the ball (far). If the robot is in the left side of the field start looking right, else left
        
        if(
          LOOK_LEFT_AND_RIGHT_WHILE_WALKING_TO_BALL && 
          (theRobotPose.translation.x() > START_LOOKING_LEFT_AND_RIGHT_AT_X_COORDINATE || theRobotPose.translation.x() < -START_LOOKING_LEFT_AND_RIGHT_AT_X_COORDINATE)
        )
        {
          callParametricLook();
        }
        else
        {
          theLookAtPointSkill(Vector3f(ballPositionRelative.x(), ballPositionRelative.y(), 0.f));
        }
        
        theWalkToTargetPathPlannerSkill(Pose2f(1.0f, walkSpeed, walkSpeed), theBallCarrierModel.staticApproachPoint());
      }
    }
    
//TOTEST
    state(centerBallInView)
    {
      transition
      {
        
        Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

        Vector2f lastPercievedBallPosition = theLibCheck.rel2Glob(theBallModel.lastPerception.x(),theBallModel.lastPerception.y() ).translation;
        float angleToBall = Angle(theLibCheck.angleToTarget( lastPercievedBallPosition.x(), lastPercievedBallPosition.y())).toDegrees();

        if(
          theBallModel.estimate.position.x() > theBallSpecification.radius*3
          && ANGLE_TO_BALL_RANGE_FOR_ROTATION_DURING_REALIGNMENT.isInside(angleToBall)
        )
        {
          std::cout<<"centerBallInView -> walkAroundBall"<<std::endl;
          goto walkAroundBall;
        }


      }
      action
      {
        theActivitySkill(BehaviorStatus::realigning_to_ball);

        Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

        float walkSpeed = decideWalkSpeed(dangerObstacleRadius, safetyObstacleRadius, p.slowerWalkSpeed, p.normalWalkSpeed);
        theKeyFrameArmsSkill(decideArmsPosition(safetyObstacleRadius, p.putArmsOnBack), false);

        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));


        float firstStepXPos = (approachXRange.max - approachXRange.min)/2;

        theWalkAtRelativeSpeedSkill(Pose2f(theLibCheck.angleToTarget(globalBall.x(), globalBall.y()), -1.0, 0.0));
      }
    }

    state(walkAroundBall)
    {
      transition
      {

        Vector2f lastPercievedBallPosition = theLibCheck.rel2Glob(theBallModel.lastPerception.x(),theBallModel.lastPerception.y() ).translation;
        float angleToBall = Angle(theLibCheck.angleToTarget( lastPercievedBallPosition.x(), lastPercievedBallPosition.y())).toDegrees();
        
        /*if(
          smallBallAlignmentRange.isInside(Angle(theLibCheck.angleToTarget(theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y())).toDegrees())
          && ANGLE_TO_BALL_RANGE_FOR_ROTATION_DURING_REALIGNMENT.isInside(angleToBall)
        )
        {
          std::cout<<"walkAroundBall -> secondStepAlignment"<<std::endl;
          goto secondStepAlignment;
        }*/

        float offsetBallYPosition;
        if(USE_FOOT_OFFSET)
        {
          if(kickWithRightFoot)
          {
            offsetBallYPosition = theBallModel.estimate.position.y() + BALL_KICK_OFFSET_Y;
          }
          else
          {
            offsetBallYPosition = theBallModel.estimate.position.y() - BALL_KICK_OFFSET_Y;
          }
        }
        else
        {
          offsetBallYPosition = theBallModel.estimate.position.y();
        }

        //DEBUG_CODE(offsetBallYPosition);
        //Case B: Robot is in the X range but not in the Y range
        //if(approachXRange.isInside(theBallModel.estimate.position.x()))
        //{
          if(largeBallAlignmentRange.isInside(Angle(theLibCheck.angleToTarget(theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y())).toDegrees()))
          {
            std::cout<<"walkAroundBall -> secondStepAlignment"<<std::endl;
            goto secondStepAlignment;
          }
        //}
      }

      action
      {

        Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        float angleToDynamicTarget = Angle(theLibCheck.angleToTarget(theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y())).toDegrees();

        float lateralSpeed = p.normalWalkSpeed;
        if(angleToDynamicTarget > 0.f) lateralSpeed *= -1;
        theWalkAtRelativeSpeedSkill(Pose2f(theLibCheck.angleToTarget(globalBall.x(), globalBall.y()), 0.0, lateralSpeed));
      }
    }


    state(secondStepAlignment)
    {
      transition
      {

        float offsetBallYPosition;
        if(USE_FOOT_OFFSET)
        {
          if(kickWithRightFoot)
          {
            offsetBallYPosition = theBallModel.estimate.position.y() + BALL_KICK_OFFSET_Y;
          }
          else
          {
            offsetBallYPosition = theBallModel.estimate.position.y() - BALL_KICK_OFFSET_Y;
          }
        }
        else
        {
          offsetBallYPosition = theBallModel.estimate.position.y();
        }

        if(theBallModel.estimate.position.norm() > approachXRange.max)
        {
          goto walkToBall;
        }

        //DEBUG_CODE(approachYRange.isInside(offsetBallYPosition));
        //DEBUG_CODE(offsetBallYPosition);
        if(approachXRange.isInside(theBallModel.estimate.position.x())
        && approachYRange.isInside(offsetBallYPosition)
        && smallBallAlignmentRange.isInside(Angle(theLibCheck.angleToTarget(theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y())).toDegrees()))
        {
          std::cout<<"secondStepAlignment -> kick: OK to kick"<<std::endl;
          goto kick;
        }

        if(state_time > realignmentTimeout)
        {
          std::cout<<"secondStepAlignment -> kick: TIMEOUT"<<std::endl;
          goto kick;
        }

        //Vector2f lastPercievedBallPosition = theLibCheck.rel2Glob(theBallModel.lastPerception.x(),theBallModel.lastPerception.y() ).translation;
        Vector2f lastPercievedBallPosition = theLibCheck.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y() ).translation;
        float angleToBall = Angle(theLibCheck.angleToTarget( lastPercievedBallPosition.x(), lastPercievedBallPosition.y())).toDegrees();
        
//BELOW copied from the KICK state
        //Case E: Robot not in the X range but is behind the ball and is not in the Y range 
        if(theBallModel.estimate.position.x() < approachXRange.min)
        {
          if(Rangef(-theBallSpecification.radius*3, theBallSpecification.radius*3).isInside(theBallModel.estimate.position.x())
          || !ANGLE_TO_BALL_RANGE_FOR_ROTATION_DURING_REALIGNMENT.isInside(angleToBall))
          {
            if(!largeApproachYRange.isInside(offsetBallYPosition))
            {
              std::cout<<"kick -> centerBallInView: Case E"<<std::endl;
              goto centerBallInView; //go back
            }
          }
        }

        /*if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          std::cout<<"secondStepAlignment -> searchForBall:"<<std::endl;
          goto searchForBall;
        }*/
        
        //Case F: Robot is between the target and the ball
        if(theBallModel.estimate.position.x() < -theBallSpecification.radius*2)
        {
          if(approachYRange.isInside(offsetBallYPosition))
          {
            std::cout<<"secondStepAlignment -> walkToBall: Case F"<<std::endl;
            goto walkToBall;
          }
        }
        
        //Case G: the robot is between the target and the ball and to the side wrt the ball

        if(theBallModel.estimate.position.x() < -theBallSpecification.radius*3)
        {
          if(!largeApproachYRange.isInside(offsetBallYPosition))
          {  
            std::cout<<"secondStepAlignment -> centerBallInView: Case G"<<std::endl;
            goto centerBallInView;
          }
        }
        

      }
      action
      {
        theActivitySkill(BehaviorStatus::realigning_to_ball);
        
        float walkSpeed = decideWalkSpeed(dangerObstacleRadius, safetyObstacleRadius, p.slowerWalkSpeed, p.normalWalkSpeed);
        theKeyFrameArmsSkill(decideArmsPosition(safetyObstacleRadius, p.putArmsOnBack), false);
        
        float localGoalYCoordinate = theLibCheck.glob2Rel(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal).translation.y();

        if(
          LOOK_LEFT_AND_RIGHT_WHILE_ALIGNING && 
          (theRobotPose.translation.x() > START_LOOKING_LEFT_AND_RIGHT_AT_X_COORDINATE || theRobotPose.translation.x() < -START_LOOKING_LEFT_AND_RIGHT_AT_X_COORDINATE)
        )
        {
          callParametricLook();
        }
        else
        {
          theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        }
        
        Vector2f approachPoint;
        if(USE_DYNAMIC_APPROACH_POINT_WHEN_INSIDE_APPROACH_AREA)
        {
          approachPoint = theBallCarrierModel.dynamicApproachPoint().translation;
        }
        else
        {
          approachPoint = theBallCarrierModel.staticApproachPoint().translation;
        }
        

        Pose2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
        
        if(USE_FIGHTER_FOR_REALIGNMENT)
        {
          float xOffset = std::abs(globalBall.translation.x() - approachPoint.x()), yOffset = std::abs(globalBall.translation.y() - approachPoint.y());

          Pose2f fromTarget = globalBall;
          Pose2f toTarget = theBallCarrierModel.dynamicTarget;
          theWalkToBallControllerSkill(fromTarget, toTarget, xOffset, yOffset, USE_FOOT_OFFSET && !kickWithRightFoot, 4.0, 0.05, -0.003, 0.003);  
        }
        else
        {

          Pose2f secondStepXPos = theLibCheck.glob2Rel(approachPoint.x(), approachPoint.y());
          Pose2f secondStepPoseRelative = Pose2f(theLibCheck.angleToTarget(theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y()), secondStepXPos.translation.x(), secondStepXPos.translation.y());
          
          if(USE_FOOT_OFFSET)
          {
            if(kickWithRightFoot)
            {
              secondStepPoseRelative.translation.y() += BALL_KICK_OFFSET_Y;
            }
            else
            {
              secondStepPoseRelative.translation.y() -= BALL_KICK_OFFSET_Y;
            }
          }
          theWalkToTargetSkill(Pose2f(1.0f, walkSpeed, walkSpeed), secondStepPoseRelative);
        }
      }
    }


    state(kick)
    {
      transition
      {

        float offsetBallYPosition;
        if(USE_FOOT_OFFSET)
        {
          if(kickWithRightFoot)
          {
            offsetBallYPosition = theBallModel.estimate.position.y() + BALL_KICK_OFFSET_Y;
          }
          else
          {
            offsetBallYPosition = theBallModel.estimate.position.y() - BALL_KICK_OFFSET_Y;
          }
        }
        else
        {
          offsetBallYPosition = theBallModel.estimate.position.y();
        }

        /*if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          std::cout<<"kick -> searchForBall: ball not seen TIMEOUT"<<std::endl;
          goto searchForBall;
        }*/

        if(theLibCheck.distance(previousBallPosition, theLibCheck.rel2Glob(theBallModel.estimate.position.x(), offsetBallYPosition)) > ballPositionChangedThreshold)
        {
          std::cout<<"kick -> choose_path: ball moved too much"<<std::endl;
          goto choose_path;
        }

        Vector2f ballPositionGlobal = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

        //Refer to the drawing BallCarrier Alignment chart for case letters:
        //https://docs.google.com/drawings/d/1FcS2yrCbGkUmbWM1GRGHuXTYnEhcrbkovdwFayTGWkc/edit?usp=sharing

        //Case 0: if the robot is too far away from the ball, just walkToBall
        if(theLibCheck.distance(theRobotPose, ballPositionGlobal) >= approachXRange.max)
        {
          std::cout<<"kick -> walkToBall: Case 0"<<std::endl;
          goto walkToBall;
        }
        
        //DEBUG_CODE(offsetBallYPosition);
        //Case B: Robot is in the X range but not in the Y range
        if(approachXRange.isInside(theBallModel.estimate.position.x()))
        {
          if(!approachYRange.isInside(offsetBallYPosition))
          {
            std::cout<<"kick -> secondStepAlignent: Case B"<<std::endl;
            goto secondStepAlignment;
          }
        }
        

        //Vector2f lastPercievedBallPosition = theLibCheck.rel2Glob(theBallModel.lastPerception.x(),theBallModel.lastPerception.y() ).translation;
        Vector2f lastPercievedBallPosition = theLibCheck.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y() ).translation;
        float angleToBall = Angle(theLibCheck.angleToTarget( lastPercievedBallPosition.x(), lastPercievedBallPosition.y())).toDegrees();

        //Case E: Robot not in the X range but is behind the ball and is not in the Y range 
        if(theBallModel.estimate.position.x() < approachXRange.min)
        {
          if(Rangef(-theBallSpecification.radius*3, theBallSpecification.radius*3).isInside(theBallModel.estimate.position.x())
          || !ANGLE_TO_BALL_RANGE_FOR_ROTATION_DURING_REALIGNMENT.isInside(angleToBall))
          {
            if(!largeApproachYRange.isInside(offsetBallYPosition))
            {
              std::cout<<"kick -> centerBallInView: Case E"<<std::endl;
              goto centerBallInView; //go back
            }
          }
        }
        
        //Case D + wrong angle: Robot not in the X range but is behind the ball and is in the Y range but is not aligned (angle) to the ball
        if(theBallModel.estimate.position.x() < approachXRange.min)
        {
          if(theBallModel.estimate.position.x() > theBallSpecification.radius*2)
          {
            if(approachYRange.isInside(offsetBallYPosition))
            {
              if(!smallBallAlignmentRange.isInside(calcAngleToTarget(chosenDynamicTarget).toDegrees()))
              {
                //If it is not aligned to the target first turn to the target, then the conditions will be checked again to perform further necessary alignments
                std::cout<<"kick -> turnToBall: Case D + wrong angle"<<std::endl;
                goto turnToBall;
              }
            }
          }
        }

//NOTICE: I've put the transition to the search state before the transitions to the G and F states because it is unlikely that 
//we know that the ball is behind us
        /*if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          std::cout<<"kick -> searchForBall:"<<std::endl;
          goto searchForBall;
        }*/
        
        //Case F: Robot is between the target and the ball
        if(theBallModel.estimate.position.x() < -theBallSpecification.radius*3)
        {
          if(approachYRange.isInside(offsetBallYPosition))
          {
            std::cout<<"kick -> walkToBall: Case F"<<std::endl;
            goto walkToBall;
          }
        }
        
        //Case G: the robot is between the target and the ball and to the side wrt the ball
        if(theBallModel.estimate.position.x() < -theBallSpecification.radius*3)
        {
          if(!largeApproachYRange.isInside(offsetBallYPosition))
          {  
            std::cout<<"kick -> centerBallInView: Case G"<<std::endl;
            goto centerBallInView;
          }
        }

        if(state_time > maxKickWaitTime)
        {  
          std::cout<<"kick -> choose_path: TIMEOUT"<<std::endl;
          goto choose_path;
        }

//If we get here, the robot is in case A of the diagram aka perfectly aligned. Now we check the alignment angle
        //Case A but WRONG ANGLE: Robot is perfectly aligned but with a wrong angle
        /*if(!smallBallAlignmentRange.isInside(calcAngleToTarget(chosenDynamicTarget).toDegrees()) && option_time - timeFromLastRealignment > angleRealignmentTimeout)
        {
          goto secondStepAlignment;
        }*/
      }

      action
      {
        theActivitySkill(BehaviorStatus::kicking_to_dynamic_target);
        
        Vector2f nearestObs = getNearestObstacle();
        

        float walkSpeed = decideWalkSpeed(dangerObstacleRadius, safetyObstacleRadius, p.slowerWalkSpeed, p.normalWalkSpeed);
        theKeyFrameArmsSkill(decideArmsPosition(safetyObstacleRadius, p.putArmsOnBack), false);

        Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

        //Kick with the foot that is nearest to the obstacle that is nearest to the ball
        if(WALK_ON_BALL /*&& theLibCheck.distance(globalBall, chosenDynamicTarget) > USE_IN_WALK_KICKS_FOR_DISTANCE_MORE_THAN*/)
        {
          theWalkToTargetSkill(Pose2f(1.0f, walkSpeed, walkSpeed), theBallModel.estimate.position);
        }
        else
        {
          if(kickWithRightFoot)
          {
            theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::right), Pose2f(Angle::fromDegrees(0.f), theBallModel.estimate.position.x(), theBallModel.estimate.position.y() + BALL_KICK_OFFSET_Y));
            //theKickSkill(false, (float) 9000.f, false); // parameters: (kick_type, mirror, distance, armsFixed)
          }
          else
          {
            theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(Angle::fromDegrees(0.f), theBallModel.estimate.position.x(), theBallModel.estimate.position.y() - BALL_KICK_OFFSET_Y));
            //theKickSkill(false, (float) 9000.f, false); // parameters: (kick_type, mirror, distance, armsFixed)
          }
        }
        
        if(
          LOOK_LEFT_AND_RIGHT_WHILE_KICKING && 
          (theRobotPose.translation.x() > START_LOOKING_LEFT_AND_RIGHT_AT_X_COORDINATE || theRobotPose.translation.x() < -START_LOOKING_LEFT_AND_RIGHT_AT_X_COORDINATE)
        )
        {
          callParametricLook();
        }
        else
        {
          theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        }
        theGoalTargetSkill(currentDestination);
        theSetTargetSkill(chosenDynamicTarget);
      }
    }

    //USING SEARCHERS
    /*state(searchForBall)
    {
      footChosen = false;

      transition
      {
        if(theFieldBall.ballWasSeen(ballNotSeenTimeout)) 
        {
          isSearchingToggle = false;
          fullLoopRevolution = false;
          
          std::cout<<"searchForBall -> choose_path: ball found"<<std::endl;
          goto choose_path;
        }

        if (goToCenterWhenSearchingForBall
                && fullLoopRevolution 
                    && !theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          fullLoopRevolution = false;

          std::cout<<"searchForBall -> lookForBallAtCenter: ball not found"<<std::endl;
          goto lookForBallAtCenter;
        }
      }

      action
      {
        
        theActivitySkill(BehaviorStatus::searching_for_ball);
        Pose2f robotPose = theRobotPose;
        previousBallPosition = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        Vector2f lastPercievedBallPosition = theLibCheck.rel2Glob(theBallModel.lastPerception.x(),theBallModel.lastPerception.y() ).translation;

        float walkSpeed = decideWalkSpeed(dangerObstacleRadius, safetyObstacleRadius, p.slowerWalkSpeed, p.normalWalkSpeed);
        theKeyFrameArmsSkill(decideArmsPosition(safetyObstacleRadius, p.putArmsOnBack), false);

        float angleToBall = Angle(theLibCheck.angleToTarget( lastPercievedBallPosition.x(), 
                                                       lastPercievedBallPosition.y())).toDegrees();

        if(angleToBall > 0)
        {
          theLookLeftAndRightSkill();
        }
        else
        {
          theLookRightAndLeftSkill();
        }

        if (!isSearchingToggle)
        {

          startingRobotAngleWhenSearching = Angle(theRobotPose.rotation);
          rotationVel = angleToBall > 0 ? walkSpeed : -walkSpeed;
          isSearchingToggle = true;          
        }
        theWalkAtRelativeSpeedSkill(Pose2f(rotationVel, 0.f, 0.f));
             
        
        float angleDiff = theRobotPose.rotation.toDegrees() - startingRobotAngleWhenSearching.toDegrees();

        if(rotationVel > 0)
        {
          fullLoopRevolution = angleDiff < 0 
                                  && Rangef(-60,60).isInside(angleDiff)
                                      && !Rangef(-10,10).isInside(angleDiff);
        }
        else
        {
          fullLoopRevolution = angleDiff > 0
                                  && Rangef(-60,60).isInside(angleDiff)
                                      && !Rangef(-10,10).isInside(angleDiff);
        }
        
        
      }
    }*/

    //USING SEARCHERS
    /*state (lookForBallAtCenter)
    {
      transition 
      {
        if(theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          isSearchingToggle = false;

          std::cout<<"lookForBallAtCenter -> walkToBall: ball found"<<std::endl;
          goto walkToBall;
        }

      }
      action
      {
        theActivitySkill(BehaviorStatus::reaching_ball);


        theLookLeftAndRightSkill();
        if(theLibCheck.distance(theRobotPose, Pose2f(0.f, -850.f, 0.f)) > 300.f)
        {
          theWalkToTargetPathPlannerSkill(Pose2f(1.0f, p.normalWalkSpeed, p.normalWalkSpeed), Pose2f(0.f, -850.f, 0.f));
        }
        else
        {
          float walkSpeed = decideWalkSpeed(dangerObstacleRadius, safetyObstacleRadius, p.slowerWalkSpeed, p.normalWalkSpeed);
          theKeyFrameArmsSkill(decideArmsPosition(safetyObstacleRadius, p.putArmsOnBack), false);
          theWalkAtRelativeSpeedSkill(Pose2f(-theRobotPose.rotation,0.f,0.f));
        }
      }
    }*/
  }
};

MAKE_SKILL_IMPLEMENTATION(CarryBallImpl);
