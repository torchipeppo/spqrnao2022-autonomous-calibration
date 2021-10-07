/**
 * @file Representations/BehaviorControl/BallCarrierModel.h
 *
 * Declaration of struct BallCarrierModel, that provides a model of the opponent goal
 * from the point of view of the striker, including obstacle coverage and
 * a score for the utility of each discretized targetable segment on the goal line
 *
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include "Representations/BehaviorControl/Libraries/LibPathPlanner.h"

/**
 * @struct BallCarrierModel
 *
 * This representation is used only for debugging purposes
 * 
 * Struct containing various modeling info useful to the ball carrier
 */


STREAMABLE(BallCarrierModel,
{

  /** Draws model on the field */
  void draw() const;
  
  STREAMABLE(Node,
  {
    Node() = default;
    Node(Vector2f center, float radius);
    ,
    (Vector2f) center,
    (float) radius,
  });

  STREAMABLE(Edge,
  {
    Edge() = default;
    Edge(BallCarrierModel::Node fromNode, BallCarrierModel::Node toNode);
    ,
    (BallCarrierModel::Node) fromNode,
    (BallCarrierModel::Node) toNode,
  });

  STREAMABLE(TrajectoryNode,
  {
    TrajectoryNode() = default;
    TrajectoryNode(Vector2f destination, float reachedAtXCoordinate);
    ,
    (Vector2f) destination,
    (float) reachedAtXCoordinate,
  });

  FUNCTION(Pose2f(float radius, float angle)) computeApproachPoint;

  FUNCTION(Pose2f()) dynamicApproachPoint;  /** Entry point for the approach area, aligned with the ball and the dynamicTarget, 
                                            at a distance from the ball equal to the MAX_APPROACH_DISTANCE */
  
  FUNCTION(Pose2f()) staticApproachPoint; /** Entry point for the approach area, aligned with the ball and the dynamicTarget, 
                                          at a distance from the ball equal to the cuX_APPROACH_DISTANCE] */

  FUNCTION(Pose2f()) escapeApproachPoint; /** Entry point for the approach area, aligned with the ball and the escapeTarget,
                                          to be used to kick the ball away from the nearest obstacle "danger zone" (to unstuck the ball), 
                                          at a distance from the ball equal to the MAX_APPROACH_DISTANCE] */

  /** Given the line passing through the ball and the obstacle, provides a direction that is perpendicular to this line
   *  (in clockwise or counter-clockwise rotation) */
  //NOTICE: requires GLOBAL obstacle coordinates
  FUNCTION(Vector2f(Vector2f obstacle, bool clockwise)) getTangentDirectionToObstacle;

  /** Gives a target along the tanget direction to the obstacle, that can be followed to escape the obstacle area */
  //NOTICE: requires GLOBAL obstacle coordinates
  FUNCTION(Vector2f(Vector2f obstacle, float clearanceRadius)) getObstacleEscapeTarget;

  /** Return the nearest obstacle to the ball */
  FUNCTION(Vector2f()) nearestObstacleToBall;

  /** Is the ball too near to its nearest obstacle? (inside the nearestObstacleEscapeRadius?) */
  FUNCTION(bool()) isBallInDangerZone; 

  /** Is the robot too near to its nearest obstacle? (inside the nearestObstacleEscapeRadius?) */
  FUNCTION(bool()) isRobotInDangerZone; 

  /** Conditions that determine whether the ball uses the escape target or the dynamic target (the target parameter refers to the final target of the path)*/
  FUNCTION(bool(Vector2f target)) useEscapeTarget; 

  ,

  (bool) graphicalDebug,                              /** Toggle for the graphical debug in SimRobot */
  (float) obstacleAvoidanceArcAngleStep,              /** When computing a path around an obstacle, determines the arc increase around it */
  (std::vector<Edge>) obstacleAvoidancePlan,          /** Filled with a list of obstacles, ordered as they would be encountered along the path */
  (std::vector<Node>) ballPath,                       /** Filled with a list of waypoints for the path */
  (bool) isFallbackPath,                              /** Determines whether the current path is a "safe" path or a fallback one (a fake/unsafe one, because a safe one could not be found)*/
  (bool) isTargetOnGoal,                              /** Determines whether the current dynamic target is on the goal line*/
  (bool) isTargetInPenaltyArea,                       /** Determines whether the current dynamic target is in the penalty area*/
  (bool) isTargetAPass,                               /** Determines whether the current dynamic target is a pass target*/

  (float) dynamicGoalPostObstacleRadius,
  (float) dynamicUprightRobotObstacleRadius,
  (float) dynamicReadyRobotObstacleRadius,
  (float) dynamicFallenRobotObstacleRadius,
  (float) dynamicRadiusControlOffset,

  (Pose2f) dynamicTarget,                             /** Next immediate target for the ball along the path */
  (float) minimumApproachDistance,                    /** Minimum radius around the ball delimiting the "approach area" */
  (float) maximumApproachDistance,                    /** Maximum radius around the ball delimiting the "approach area" */
  (float) staticApproachRadius,                       /** Radius of the static approach point from the ball */
  (float) dynamicApproachRadius,                      /** Radius of the static approach point from the ball */

  (bool)(false) usingRightTrajectory,                       /** Use left or right hard-coded path */
  (float) OFFSET_FROM_Y_CENTER_TO_CHANGE_TRAJECTORY,   /** If using the right traj and the ball y coordinate goes above this offset, we switch to left traj and vice-versa */
  
  /*
  Range
  - MIN: Minimum distance from the opponent groundline above which kicks can be used (otherwise use InWalkKick to avoid losing time) 
  - MAX: (As per Challenge 1 rules) Distance from the opponent groundline below which kicks can be used
  */  
  (Rangef) xRangeDistanceFromGoalToUseKicks,

  /*These ranges on the y coordinate contribute to creating a "kicking area" all around the goal, inside which kicking to goal is allowed*/
  (Rangef) yRangeDistanceFromGoalToUseKicksInner,
  (Rangef) yRangeDistanceFromGoalToUseKicksOuter,

  (Vector2f) nearestObstacleEscapeTarget,             /** Provides a dynamicTarget to follow along the tangent to the nearest obstacle, in the 
                                                          direction bringing the robot closer to the dynamicTarget */
  (float) nearestObstacleEscapeRadius,                /** Radius around the nearest obstacle to the ball where the escape target is preferred*/

  (bool) ESCAPE_OBSTACLES,                            /** (EXPERIMENTAL FEATURE) Use the obstacle escape routine */
  
  (bool)(false) USE_TRAJECTORY,                              /** (EXPERIMENTAL FEATURE) Follow a hard-coded trajectory */
  (std::vector<BallCarrierModel::TrajectoryNode>) TRAJECTORY, /** The hard-coded trajectory */
  (int)(0) currentTrajectoryNode, 

  //Only for challenge1
  (bool) useLongKicksToCarry,                         /** Use a long kick as the first kick */
  
  (Vector2f) firstTargetWithOneObstacleLeft,          /** Target used for the first long kick in challenge1 in case there is only one obstacle in front of the center circle, to the right */
  (Vector2f) firstTargetWithOneObstacleRight,         /** Target used for the first long kick in challenge1 in case there is only one obstacle in front of the center circle, to the left */
  (Vector2f) firstTargetWithTwoObstacles,             /** Target used for the first long kick in challenge1 in case there are two obstacles in front of the center circle */

  //(std::vector<TrajectoryNode>) trajectory,        /** Trajectory to follow in case the hard-coded path is chosen */

});

inline BallCarrierModel::Node::Node(Vector2f center, float radius) : center(center), radius(radius) {};
inline BallCarrierModel::Edge::Edge(BallCarrierModel::Node fromNode, BallCarrierModel::Node toNode) : fromNode(fromNode), toNode(toNode) {};
