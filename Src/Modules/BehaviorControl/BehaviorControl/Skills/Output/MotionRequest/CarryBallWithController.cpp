/**
 * @file CarryBallWithController.cpp
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
//32 More control on path change, threshold etc.


SKILL_IMPLEMENTATION(CarryBallWithControllerImpl,
{,
  IMPLEMENTS(CarryBallWithController),

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
  CALLS(ParametricLookLeftAndRightOnce),

  CALLS(KeyFrameArms),
  CALLS(InWalkKick),

  CALLS(SetTarget),
  CALLS(GoalTarget),

  LOADS_PARAMETERS(
  {,
    (int) initialWaitTime,
    (int) realignmentTimeout,
    (int) pathChangeTimeout,
    
    (float) ballPositionChangedThreshold,
    

    (Rangef) approachXRange,
    (Rangef) approachYRange,
    (Rangef) smallBallAlignmentRange,

    (float) gainWhenBallOnSide,
    (float) kp1,
    (float) kp2,
    (float) kp3,

    (bool) USE_DYNAMIC_APPROACH_POINT_WHEN_INSIDE_APPROACH_AREA,
    (float) DISTANCE_FROM_APPROACH_POINT_TO_START_REALIGNING,

    (float) nearbyObstacleRadius, //Radius inside which an obstacle is considered "nearby"

    (int) maxKickWaitTime,

    (float) safetyObstacleRadius,         // default = 500; used to move hands for stability when not around obstacles
    (float) dangerObstacleRadius,         //At and below this radius, the robot will walk at slowerWalkSpeed, between this and safetyObstacleRadius, the speed will be proportional to the distance, above safetyObstacleRadius, it will be normalWalkSpeed

    (float) robotArmsRange,               //Clearance radius of an arm movement
    (bool) armsAlwaysOnBack,              //Forcs arms position on back

    (float) MAX_OBSTACLE_LOCALIZATION_DRIFT, //Maximum drift of an obstacle after which the path is recomputed

    (bool) USE_FOOT_OFFSET,               //Use an offset added to the approach point to align the ball to the foot when InWalkKicking
    (float) BALL_KICK_OFFSET_Y,           //Foot offset along the Y coordinate

    (bool) LOOK_LEFT_AND_RIGHT_WHILE_ALIGNING,
    (bool) LOOK_LEFT_AND_RIGHT_WHILE_WALKING_TO_BALL,
    (bool) LOOK_LEFT_AND_RIGHT_WHILE_KICKING,
    (float) MAX_LOOKING_ANGLE,
    (int) LOOK_AT_LANDMARK_EVERY,

    (float) LEFT_AND_RIGHT_LOOKING_SPEED,

    (float) START_LOOKING_LEFT_AND_RIGHT_AT_X_COORDINATE,

    (bool) WALK_ON_BALL,
    (float) minKickSpeed, //Min and Max speed for the walk kick (minimum if there are opponets in front of us, other wise fast)
    (float) maxKickSpeed,
    (float) safetyRadiusAroundBallForFastKick, //radius around the ball to allow a faster kick

    //Debug info
    (bool) SHOW_CONDITIONS_DEBUG,
    (bool) DEBUG_MODE,
  }),
});

class CarryBallWithControllerImpl : public CarryBallWithControllerImplBase
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

  
  float decideKickSpeed(float minKickSpeed, float maxKickSpeed, float safetyRadius)
  {
    Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
    if(theLibCheck.areThereOpponentsNearby(globalBall, safetyRadius))
    {
      return minKickSpeed;
    }
    else
    {
      return maxKickSpeed;
    }
  }

  ArmKeyFrameRequest::ArmKeyFrameId decideArmsPosition(float maxObstacleRadius, bool putArmsOnBack)
  {
    if(armsAlwaysOnBack) return ArmKeyFrameRequest::back;

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

  //Looks at the penaltyMark (with an angle that is directly proportional to the x coordinate of the robot) every LOOK_AT_LANDMARK_EVERY seconds
  //otherwise looks at the ball
  int lookAtBallAndLandmark(int lastLookAtLandmarkTime, int current_time)
  {
    float leftLookingAngle;
    float rightLookingAngle;
    float lookingSpeed;
    float headTilt = 20;
    if(theRobotPose.translation.x() <= theFieldDimensions.xPosOpponentPenaltyMark - 500)
    {
      lookingSpeed = LEFT_AND_RIGHT_LOOKING_SPEED;
      if(theRobotPose.translation.y() >= 0.f)
      {
        leftLookingAngle = 0.f;
        rightLookingAngle = theLibCheck.mapToInterval(theRobotPose.translation.x(), 0, theFieldDimensions.xPosOpponentPenaltyMark, 0, MAX_LOOKING_ANGLE);
      }
      else
      {
        leftLookingAngle = theLibCheck.mapToInterval(theRobotPose.translation.x(), 0, theFieldDimensions.xPosOpponentPenaltyMark, 0, MAX_LOOKING_ANGLE);
        rightLookingAngle = 0.f;
      }
    }
    else
    {
      lookingSpeed = LEFT_AND_RIGHT_LOOKING_SPEED*1.5;
      headTilt = 40;
      leftLookingAngle = theLibCheck.mapToInterval(theRobotPose.translation.x(), 0, theFieldDimensions.xPosOpponentPenaltyMark, 0, MAX_LOOKING_ANGLE/3);
      rightLookingAngle = theLibCheck.mapToInterval(theRobotPose.translation.x(), 0, theFieldDimensions.xPosOpponentPenaltyMark, 0, MAX_LOOKING_ANGLE/3);
    }

    Vector2f relativePenaltyMark = theLibCheck.glob2Rel(theFieldDimensions.xPosOpponentPenaltyMark, 0.f).translation;
    if(current_time - lastLookAtLandmarkTime > LOOK_AT_LANDMARK_EVERY)
    {
      if(theParametricLookLeftAndRightOnceSkill.isDone())
      {
        lastLookAtLandmarkTime = current_time;
      }
      else
      {
        if(theRobotPose.translation.y() >= 0.f)
        {
          theParametricLookLeftAndRightOnceSkill(leftLookingAngle, rightLookingAngle, headTilt, LEFT_AND_RIGHT_LOOKING_SPEED);
        }
        else
        {
          theParametricLookLeftAndRightOnceSkill(leftLookingAngle, rightLookingAngle, headTilt, LEFT_AND_RIGHT_LOOKING_SPEED);
        }
      }
    }
    else
    {
      theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
    }
    return lastLookAtLandmarkTime;
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

  float lastLookAtLandmarkTime = 0.f;


  Vector2f chosenDynamicTarget;
  Vector2f currentDestination;
  Vector2f previousBallPosition;
  int previousNearbyObstaclesCount = 0;
  Vector2f nearestObstacleGlobalPositionWhenPathWasComputed = getNearestObstacle(true);
  

  option(CarryBallWithController)
  {
    common_transition
    {
      //Number of obstacles changed
      if(countNearbyObstacles(nearbyObstacleRadius) != previousNearbyObstaclesCount && (option_time - pathChangeTime) > pathChangeTimeout)
      {
        std::cout<<"common transition -> choose_path: obstacle number changed: "<<std::to_string(previousNearbyObstaclesCount)<<" before, "<<std::to_string(countNearbyObstacles(nearbyObstacleRadius))<<"now"<<std::endl;
        goto choose_path;
      }

      //Same number of obstacles but big position drift due to localization
      if(theLibCheck.distance(getNearestObstacle(true), nearestObstacleGlobalPositionWhenPathWasComputed) > MAX_OBSTACLE_LOCALIZATION_DRIFT && (option_time - pathChangeTime) > pathChangeTimeout)
      {
        std::cout<<"common transition -> choose_path: obstacle position drift too high"<<std::endl;
        goto choose_path;
      }
    }

    initial_state(start)
    {
      std::cout<<"CarryBallWithControllerSkill: start"<<std::endl;
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

          std::cout<<"choose_path -> realignWithController: target chosen"<<std::endl;
          goto walkToBall;
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

      }
    }

    state(debug_state)
    {
      action
      {
        theActivitySkill(BehaviorStatus::debug_standing);

        Vector2f relativePenaltyMark = theLibCheck.glob2Rel(theFieldDimensions.xPosOpponentPenaltyMark, 0.f).translation;
        /*if(relativePenaltyMark.x() >= 0)
        {
          if(relativePenaltyMark.y() > 0)
          {
            theParametricLookLeftAndRightSkill(0, abs(Angle(relativePenaltyMark.angle()).toDegrees()), 20, LEFT_AND_RIGHT_LOOKING_SPEED);
          }
          else
          {
            theParametricLookLeftAndRightSkill(abs(Angle(relativePenaltyMark.angle()).toDegrees()), 0, 20, LEFT_AND_RIGHT_LOOKING_SPEED);
          }
        }*/
        lastLookAtLandmarkTime = lookAtBallAndLandmark(lastLookAtLandmarkTime, option_time);
        theStandSkill();
      }
    }

    state(walkToBall)
    {
      transition
      {
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
        
        if(theLibCheck.distance(theRobotPose, approachPoint) < DISTANCE_FROM_APPROACH_POINT_TO_START_REALIGNING 
          &&
          theLibCheck.distance(theRobotPose, chosenDynamicTarget) > theLibCheck.distance(globalBall, chosenDynamicTarget))
        {
          //Case B: Robot is in the X range but not in the Y range
          std::cout<<"walkToBall -> realignWithController: Case B"<<std::endl;
          goto realignWithController;
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
          lastLookAtLandmarkTime = lookAtBallAndLandmark(lastLookAtLandmarkTime, option_time);
        }
        else
        {
          theLookAtPointSkill(Vector3f(ballPositionRelative.x(), ballPositionRelative.y(), 0.f));
        }
        
        theWalkToTargetPathPlannerSkill(Pose2f(1.0f, walkSpeed, walkSpeed), theBallCarrierModel.staticApproachPoint());
      }
    }

    state(realignWithController)
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
        //DEBUG_CODE(offsetBallYPosition);

        if(theBallModel.estimate.position.norm() > approachXRange.max)
        {
          std::cout<<"realignWithController -> walkToBall: ball too far"<<std::endl;
          goto walkToBall;
        }
        
        //Robot is between the target and the ball
        if(theBallModel.estimate.position.x() < -theBallSpecification.radius*2)
        {
          if(approachYRange.isInside(offsetBallYPosition))
          {
            std::cout<<"realignWithController -> walkToBall: ball behind us"<<std::endl;
            goto walkToBall;
          }
        }

        //DEBUG_CODE(approachXRange.isInside(theBallModel.estimate.position.x()));
        //DEBUG_CODE(approachYRange.isInside(offsetBallYPosition));
        //DEBUG_CODE(smallBallAlignmentRange.isInside(Angle(theLibCheck.angleToTarget(theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y())).toDegrees()));
        if(approachXRange.isInside(theBallModel.estimate.position.x())
        && approachYRange.isInside(offsetBallYPosition)
        && smallBallAlignmentRange.isInside(Angle(theLibCheck.angleToTarget(theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y())).toDegrees()))
        {
          std::cout<<"realignWithController -> kick: OK to kick"<<std::endl;
          goto kick;
        }

        if(state_time > realignmentTimeout)
        {
          std::cout<<"realignWithController -> kick: TIMEOUT"<<std::endl;
          goto kick;
        }
      }        

      action
      {
        theActivitySkill(BehaviorStatus::realigning_to_ball);
        
        theKeyFrameArmsSkill(decideArmsPosition(safetyObstacleRadius, p.putArmsOnBack), false);
        
        if(
          LOOK_LEFT_AND_RIGHT_WHILE_ALIGNING && 
            (theRobotPose.translation.x() > START_LOOKING_LEFT_AND_RIGHT_AT_X_COORDINATE 
            || theRobotPose.translation.x() < -START_LOOKING_LEFT_AND_RIGHT_AT_X_COORDINATE)
        )
        {
          lastLookAtLandmarkTime = lookAtBallAndLandmark(lastLookAtLandmarkTime, option_time);
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
        
        float xOffset = std::abs(globalBall.translation.x() - approachPoint.x());
        float yOffset;
        if(USE_FOOT_OFFSET)
        {
          yOffset = 0;
        }
        else
        {
          yOffset = theBallModel.estimate.position.y();
        }
        
        Pose2f fromTarget = Vector2f(globalBall.translation.x(), globalBall.translation.y());
        Pose2f toTarget = Vector2f(theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y());
        
        theWalkToBallControllerSkill(fromTarget, toTarget, xOffset, yOffset, USE_FOOT_OFFSET && !kickWithRightFoot, gainWhenBallOnSide, kp1, kp2, kp3);
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
        DEBUG_CODE(offsetBallYPosition);

        if(theLibCheck.distance(previousBallPosition, theLibCheck.rel2Glob(theBallModel.estimate.position.x(), offsetBallYPosition)) > ballPositionChangedThreshold)
        {
          std::cout<<"kick -> choose_path: ball moved too much"<<std::endl;
          goto choose_path;
        }

        if(state_time > maxKickWaitTime)
        {  
          std::cout<<"kick -> choose_path: TIMEOUT"<<std::endl;
          goto choose_path;
        }

        Vector2f ballPositionGlobal = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;


    //PATH PLANNER NEEDED -> walkToBall
        if(theBallModel.estimate.position.norm() > approachXRange.max)
        {
          std::cout<<"kick -> walkToBall: Ball too far"<<std::endl;
          goto walkToBall;
        }
        
        Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

        if(approachYRange.isInside(offsetBallYPosition))
        {
          if(theBallModel.estimate.position.x() < theBallSpecification.radius * 2)
          {
            std::cout<<"kick -> walkToBall: Ball behind us"<<std::endl;
            goto walkToBall;
          }
        }

    //REALIGNMENT NEEDED -> realignWithController
        DEBUG_CODE(approachXRange.isInside(theBallModel.estimate.position.x()));
        DEBUG_CODE(approachYRange.isInside(offsetBallYPosition));
        //DEBUG_CODE(smallBallAlignmentRange.isInside(Angle(theLibCheck.angleToTarget(theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y())).toDegrees()));
        if(
          !approachXRange.isInside(theBallModel.estimate.position.x())
          ||
          !approachYRange.isInside(offsetBallYPosition)
          //||
          //!smallBallAlignmentRange.isInside(Angle(theLibCheck.angleToTarget(theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y())).toDegrees())
        )
        {
          std::cout<<"kick -> realignWithController: realignment needed"<<std::endl;
          goto realignWithController;
        }
      }        

      action
      {
        theActivitySkill(BehaviorStatus::kicking_to_dynamic_target);
        
        float walkSpeed = decideKickSpeed(minKickSpeed, maxKickSpeed, safetyRadiusAroundBallForFastKick);
        theKeyFrameArmsSkill(decideArmsPosition(safetyObstacleRadius, p.putArmsOnBack), false);

        Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

        //Kick with the foot that is nearest to the obstacle that is nearest to the ball
        theWalkAtRelativeSpeedSkill(Pose2f(0.f, decideKickSpeed(minKickSpeed, maxKickSpeed, safetyRadiusAroundBallForFastKick), 0.f));
        
        if(
          LOOK_LEFT_AND_RIGHT_WHILE_KICKING && 
          (theRobotPose.translation.x() > START_LOOKING_LEFT_AND_RIGHT_AT_X_COORDINATE || theRobotPose.translation.x() < -START_LOOKING_LEFT_AND_RIGHT_AT_X_COORDINATE)
        )
        {
          lastLookAtLandmarkTime = lookAtBallAndLandmark(lastLookAtLandmarkTime, option_time);
        }
        else
        {
          theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        }
        theGoalTargetSkill(currentDestination);
        theSetTargetSkill(chosenDynamicTarget);
      }
    }
  }
};

MAKE_SKILL_IMPLEMENTATION(CarryBallWithControllerImpl);
