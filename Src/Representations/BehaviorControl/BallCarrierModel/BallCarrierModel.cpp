/**
 * @file Representations/BehaviorControl/BallCarrierModel.cpp
 * 
 * Implementation of the drawing functions of the BallCarrierModel
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "BallCarrierModel.h"
#include "Platform/Time.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Representations/Modeling/BallModel.h"
#include <iostream>

void BallCarrierModel::draw() const
{
  const BallCarrierModel& theBallCarrierModel = static_cast<const BallCarrierModel&>(Blackboard::getInstance()["BallCarrierModel"]);
  if(theBallCarrierModel.graphicalDebug)
  {
    const Role& theRole = static_cast<const Role&>(Blackboard::getInstance()["Role"]);
    const GameInfo& theGameInfo = static_cast<const GameInfo&>(Blackboard::getInstance()["GameInfo"]);
    const BallModel& theBallModel = static_cast<const BallModel&>(Blackboard::getInstance()["BallModel"]);
    const LibCheck& theLibCheck = static_cast<const LibCheck&>(Blackboard::getInstance()["LibCheck"]);
    const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
    const RobotPose& theRobotPose = static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]);
    
    #ifdef TARGET_SIM
    //STOLEN FROM PathPlannerProvider
    /**
     * Draw a 3D arc parallel to the field plane.
     * @param id The drawing id.
     * @param xCenter The x coordinate of the center.
     * @param yCenter The y coordinate of the center.
     * @param zCenter The z coordinate of the center.
     * @param radius The radius.
     * @param fromAngle The start angle.
     * @param angleSize The angular size of the arc. The arc is in the range
     *                  [fromAngle ... fromAngle + angleSize].
     * @param thickness The line thickness.
     * @param color The color of the circle.
     */
    #define ARC3D(id, xCenter, yCenter, zCenter, radius, fromAngle, angleSize, thickness, color) \
      do \
      { \
        constexpr Angle _angleStep = pi2 / 32.f; \
        Vector2f _from((xCenter) + std::cos(fromAngle) * (radius), (yCenter) + std::sin(fromAngle) * (radius)); \
        for(Angle _angle = _angleStep; _angle <= (angleSize) - _angleStep; _angle += _angleStep) \
        { \
          Vector2f _to((xCenter) + std::cos(_angle + (fromAngle)) * (radius), (yCenter) + std::sin(_angle + (fromAngle)) * (radius)); \
          LINE3D(id, _from.x(), _from.y(), (zCenter), _to.x(), _to.y(), (zCenter), thickness, color); \
          _from = _to; \
        } \
        Vector2f _to((xCenter) + std::cos((fromAngle) + (angleSize)) * (radius), (yCenter) + std::sin((fromAngle) + (angleSize)) * (radius)); \
        LINE3D(id, _from.x(), _from.y(), (zCenter), _to.x(), _to.y(), (zCenter), thickness, color); \
      } \
      while(false)

    Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
    float angleToBall = theLibCheck.angleToBall;
    DEBUG_DRAWING3D("representation:BallCarrierModel", "field")
    {
      if(theGameInfo.state == STATE_PLAYING && theRole.role==Role::RoleType::striker)
      {
        // drawing of the model in the field view
        
        for(auto edge: theBallCarrierModel.obstacleAvoidancePlan)
        {

          ARC3D("representation:BallCarrierModel",
                edge.fromNode.center.x(), edge.fromNode.center.y(), 10, 
                edge.fromNode.radius, 
                0, pi2, 5, ColorRGBA(255,0,0,255)
                );
          /*SPHERE3D("representation:BallCarrierModel", 
            edge.fromNode.center.x(), edge.fromNode.center.y(), 10, edge.fromNode.radius,
            ColorRGBA(255,0,0,100));*/
          LINE3D("representation:BallCarrierModel", 
            edge.fromNode.center.x(), edge.fromNode.center.y(), 10, 
            edge.toNode.center.x(), edge.toNode.center.y(), 10, 
            5, ColorRGBA(255,0,0,255));
        }
        if(!theBallCarrierModel.isFallbackPath)
        {
          BallCarrierModel::Node prevStep = theBallCarrierModel.ballPath[0];
          SPHERE3D("representation:BallCarrierModel", 
            prevStep.center.x(), prevStep.center.y(), 10, 30,
            ColorRGBA(0,255,0,100));

          for(int i=1; i<theBallCarrierModel.ballPath.size(); i++)
          {
            BallCarrierModel::Node step = theBallCarrierModel.ballPath[i];
            SPHERE3D("representation:BallCarrierModel", 
              step.center.x(), step.center.y(), 10, 30,
              ColorRGBA(0,255,0,100));

            LINE3D("representation:BallCarrierModel", 
              prevStep.center.x(), prevStep.center.y(), 10, 
              step.center.x(), step.center.y(), 10, 
              5, ColorRGBA(0,255,0,255));
              prevStep = step;
          }
          //Draw the dynamic target (kick target chosen from the path)
        }
        else
        //If there is no path, use the fallback target
        {
          SPHERE3D("representation:BallCarrierModel", 
            globalBall.x(), globalBall.y(), 10, 
            30, ColorRGBA(255,0,0,100));

          SPHERE3D("representation:BallCarrierModel", 
            theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y(), 10, 
            30, ColorRGBA(255,0,0,100));

          LINE3D("representation:BallCarrierModel", 
            globalBall.x(), globalBall.y(), 10, 
            theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y(), 10, 
            5, ColorRGBA(200,200,0,255));
        }
        float ARROW_LENGTH = 100;
        Pose2f staticApproachPoint = theBallCarrierModel.staticApproachPoint();
        Pose2f dynamicApproachPoint = theBallCarrierModel.dynamicApproachPoint();
        CYLINDERARROW3D("representation:BallCarrierModel", 
              Vector3f(theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y(), 10),
              Vector3f(theBallCarrierModel.dynamicTarget.translation.x()+ARROW_LENGTH*cos(theBallCarrierModel.dynamicTarget.rotation), theBallCarrierModel.dynamicTarget.translation.y()+ARROW_LENGTH*sin(theBallCarrierModel.dynamicTarget.rotation), 10),
              10, 50, 20, ColorRGBA(0,0,255,255));
        SPHERE3D("representation:BallCarrierModel", 
              theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y(), 10, 
              30, ColorRGBA(0,0,255,255));
        
        //Draw the static approach point (farthest entry point to the approach area, determined by the chosen target)
        CYLINDERARROW3D("representation:BallCarrierModel", 
              Vector3f(staticApproachPoint.translation.x(), staticApproachPoint.translation.y(), 10),
              Vector3f(staticApproachPoint.translation.x()+ARROW_LENGTH*cos(staticApproachPoint.rotation), staticApproachPoint.translation.y()+ARROW_LENGTH*sin(staticApproachPoint.rotation), 10),
              10, 50, 20, ColorRGBA(255,0,0,255));
        SPHERE3D("representation:BallCarrierModel", 
              staticApproachPoint.translation.x(), staticApproachPoint.translation.y(), 10, 30,
              ColorRGBA(255,0,0,255));
                      
        //Draw the dynamic approach point (entry point to the approach area, determined by the chosen target and by the distance of the robot from the ball)
        CYLINDERARROW3D("representation:BallCarrierModel", 
              Vector3f(dynamicApproachPoint.translation.x(), dynamicApproachPoint.translation.y(), 10),
              Vector3f(dynamicApproachPoint.translation.x()+ARROW_LENGTH*cos(dynamicApproachPoint.rotation), dynamicApproachPoint.translation.y()+ARROW_LENGTH*sin(dynamicApproachPoint.rotation), 10),
              10, 50, 20, ColorRGBA(0,255,0,255));
        SPHERE3D("representation:BallCarrierModel", 
              dynamicApproachPoint.translation.x(), dynamicApproachPoint.translation.y(), 10, 30,
              ColorRGBA(0,0,255,255));
        
        //Draw the approach area
        ARC3D("representation:BallCarrierModel",
              globalBall.x(), globalBall.y(), 10, theBallCarrierModel.maximumApproachDistance, 0, pi2, 5, ColorRGBA(0,0,0,255)
              );
        ARC3D("representation:BallCarrierModel",
              globalBall.x(), globalBall.y(), 10, theBallCarrierModel.minimumApproachDistance, 0, pi2, 5, ColorRGBA(0,0,0,255)
              );


        bool useHeuristic = true;
        Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        if(globalBall.x() > theFieldDimensions.xPosOpponentGroundline - theBallCarrierModel.xRangeDistanceFromGoalToUseKicks.max)
        {
            useHeuristic = false;
        }
        Vector2f goalTarget = theLibCheck.goalTarget(false, useHeuristic);
        if(theBallCarrierModel.useEscapeTarget(goalTarget))
        {
          Vector2f nearestObstacle = theBallCarrierModel.nearestObstacleToBall();
          Vector2f globalNearestObstacle = theLibCheck.rel2Glob(nearestObstacle.x(), nearestObstacle.y()).translation;
          //Draw the nearest obstacle escape danger zone
          ARC3D("representation:BallCarrierModel",
                globalNearestObstacle.x(), globalNearestObstacle.y(), 10, theBallCarrierModel.nearestObstacleEscapeRadius, 0, pi2, 5, ColorRGBA(255,0,255,255)
                );
        }

        if(theBallCarrierModel.useLongKicksToCarry) 
        {
          SPHERE3D("representation:BallCarrierModel", 
              theBallCarrierModel.firstTargetWithOneObstacleLeft.x(), theBallCarrierModel.firstTargetWithOneObstacleLeft.y(), 10, 30,
              ColorRGBA(100,100,100,255));
          
          SPHERE3D("representation:BallCarrierModel", 
              theBallCarrierModel.firstTargetWithOneObstacleRight.x(), theBallCarrierModel.firstTargetWithOneObstacleRight.y(), 10, 30,
              ColorRGBA(100,100,100,255));
          
          SPHERE3D("representation:BallCarrierModel", 
              theBallCarrierModel.firstTargetWithTwoObstacles.x(), theBallCarrierModel.firstTargetWithTwoObstacles.y(), 10, 30,
              ColorRGBA(100,100,100,255));
        }


        if(theBallCarrierModel.useLongKicksToCarry) 
        {
          SPHERE3D("representation:BallCarrierModel", 
              theBallCarrierModel.firstTargetWithOneObstacleLeft.x(), theBallCarrierModel.firstTargetWithOneObstacleLeft.y(), 10, 30,
              ColorRGBA(100,100,100,255));
          
          SPHERE3D("representation:BallCarrierModel", 
              theBallCarrierModel.firstTargetWithOneObstacleRight.x(), theBallCarrierModel.firstTargetWithOneObstacleRight.y(), 10, 30,
              ColorRGBA(100,100,100,255));
          
          SPHERE3D("representation:BallCarrierModel", 
              theBallCarrierModel.firstTargetWithTwoObstacles.x(), theBallCarrierModel.firstTargetWithTwoObstacles.y(), 10, 30,
              ColorRGBA(100,100,100,255));
        }
        
        if(theBallCarrierModel.USE_TRAJECTORY)
        {
          for(int i=0; i<theBallCarrierModel.TRAJECTORY.size(); i++)
          {
            ColorRGBA trajNodeColor = ColorRGBA(100,100,100,255);
            if(i == theBallCarrierModel.currentTrajectoryNode)
            {
              trajNodeColor = ColorRGBA(0,255,0,255);
            }

            BallCarrierModel::TrajectoryNode currentNode =  theBallCarrierModel.TRAJECTORY.at(i);
            SPHERE3D("representation:BallCarrierModel", 
                currentNode.destination.x(), currentNode.destination.y(), 10, 30,
                trajNodeColor);
            LINE3D("representation:BallCarrierModel", 
              currentNode.reachedAtXCoordinate, theFieldDimensions.yPosLeftSideline, 10, 
              currentNode.reachedAtXCoordinate, theFieldDimensions.yPosRightSideline, 10, 
              5, ColorRGBA(100,100,100,255));

            if(i==0)
            {
              LINE3D("representation:BallCarrierModel", 
                0.f, 0.f, 10, 
                currentNode.destination.x(), currentNode.destination.y(), 10, 
                20, ColorRGBA(100,100,100,255));

            }
            else if(i>0)
            {
              BallCarrierModel::TrajectoryNode prevNode =  theBallCarrierModel.TRAJECTORY.at(i-1);
              LINE3D("representation:BallCarrierModel", 
                prevNode.destination.x(), prevNode.destination.y(), 10, 
                currentNode.destination.x(), currentNode.destination.y(), 10, 
                20, ColorRGBA(100,100,100,255));

            }
          }
        }

        //Draw the nearest obstacle escape route
        LINE3D("representation:BallCarrierModel", 
          globalBall.x(), globalBall.y(), 10, 
          theBallCarrierModel.nearestObstacleEscapeTarget.x(), theBallCarrierModel.nearestObstacleEscapeTarget.y(), 10, 
          5, ColorRGBA(255,0,255,255));
          
        SPHERE3D("representation:BallCarrierModel", 
          theBallCarrierModel.nearestObstacleEscapeTarget.x(), theBallCarrierModel.nearestObstacleEscapeTarget.y(), 10, 
          30, ColorRGBA(255,0,255,255));
                
        //Kicking area
          //Sides
            //Outer
            /*LINE3D("representation:BallCarrierModel", 
              theFieldDimensions.xPosOpponentGroundline - theBallCarrierModel.xRangeDistanceFromGoalToUseKicks.max, theBallCarrierModel.yRangeDistanceFromGoalToUseKicksInner.max, 10, 
              theFieldDimensions.xPosOpponentGroundline, theBallCarrierModel.yRangeDistanceFromGoalToUseKicksOuter.max, 10, 
              6, ColorRGBA(200,200,200,255));
            LINE3D("representation:BallCarrierModel", 
              theFieldDimensions.xPosOpponentGroundline - theBallCarrierModel.xRangeDistanceFromGoalToUseKicks.max, theBallCarrierModel.yRangeDistanceFromGoalToUseKicksInner.min, 10, 
              theFieldDimensions.xPosOpponentGroundline, theBallCarrierModel.yRangeDistanceFromGoalToUseKicksOuter.min, 10, 
              6, ColorRGBA(200,200,200,255));
              
            //Inner
            LINE3D("representation:BallCarrierModel", 
              theFieldDimensions.xPosOpponentGroundline - theBallCarrierModel.xRangeDistanceFromGoalToUseKicks.min, theBallCarrierModel.yRangeDistanceFromGoalToUseKicksOuter.min, 10, 
              theFieldDimensions.xPosOpponentGroundline, theBallCarrierModel.yRangeDistanceFromGoalToUseKicksInner.min, 10, 
              6, ColorRGBA(0,0,150,255));
            LINE3D("representation:BallCarrierModel", 
              theFieldDimensions.xPosOpponentGroundline - theBallCarrierModel.xRangeDistanceFromGoalToUseKicks.min, theBallCarrierModel.yRangeDistanceFromGoalToUseKicksOuter.max, 10, 
              theFieldDimensions.xPosOpponentGroundline, theBallCarrierModel.yRangeDistanceFromGoalToUseKicksInner.max, 10, 
              6, ColorRGBA(0,0,150,255));

          //Front
          LINE3D("representation:BallCarrierModel", 
            theFieldDimensions.xPosOpponentGroundline - theBallCarrierModel.xRangeDistanceFromGoalToUseKicks.min, theBallCarrierModel.yRangeDistanceFromGoalToUseKicksInner.min, 10, 
            theFieldDimensions.xPosOpponentGroundline - theBallCarrierModel.xRangeDistanceFromGoalToUseKicks.min, theBallCarrierModel.yRangeDistanceFromGoalToUseKicksInner.max, 10, 
            6, ColorRGBA(0,0,150,255));
          LINE3D("representation:BallCarrierModel", 
            theFieldDimensions.xPosOpponentGroundline - theBallCarrierModel.xRangeDistanceFromGoalToUseKicks.max, theBallCarrierModel.yRangeDistanceFromGoalToUseKicksOuter.max, 10, 
            theFieldDimensions.xPosOpponentGroundline - theBallCarrierModel.xRangeDistanceFromGoalToUseKicks.max, theBallCarrierModel.yRangeDistanceFromGoalToUseKicksOuter.min, 10, 
            6, ColorRGBA(200,200,200,255));*/

          //Front
          LINE3D("representation:BallCarrierModel", 
            theFieldDimensions.xPosOpponentGroundline - theBallCarrierModel.xRangeDistanceFromGoalToUseKicks.min, theFieldDimensions.yPosLeftSideline, 10, 
            theFieldDimensions.xPosOpponentGroundline - theBallCarrierModel.xRangeDistanceFromGoalToUseKicks.min, theFieldDimensions.yPosRightSideline, 10, 
            6, ColorRGBA(0,0,150,255));
          LINE3D("representation:BallCarrierModel", 
            theFieldDimensions.xPosOpponentGroundline - theBallCarrierModel.xRangeDistanceFromGoalToUseKicks.max, theFieldDimensions.yPosLeftSideline, 10, 
            theFieldDimensions.xPosOpponentGroundline - theBallCarrierModel.xRangeDistanceFromGoalToUseKicks.max, theFieldDimensions.yPosRightSideline, 10, 
            6, ColorRGBA(200,200,200,255));
      }
    }
    #endif
  }
}
