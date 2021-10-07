/**
 * @file Modules/BehaviorControl/BallCarrierModel/BallCarrierModelProvider.h
 *
 * This file implements a module that provides a model of the opponent goal
 * from the point of view of the striker, including obstacle coverage and
 * a score for the utility of each discretized targetable segment on the goal line
 *  
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/BallCarrierModel/BallCarrierModel.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Libraries/LibPathPlanner.h"
#include <iostream>

MODULE(BallCarrierModelProvider,
{,
    REQUIRES(LibCheck),
    REQUIRES(LibPathPlanner),
    REQUIRES(FieldDimensions),
    REQUIRES(BallModel),
    REQUIRES(Role),
    REQUIRES(GameInfo),
    REQUIRES(RobotPose),
    REQUIRES(ObstacleModel),

    PROVIDES(BallCarrierModel),
    LOADS_PARAMETERS(
    {,
      (bool) GRAPHICAL_DEBUG,                                      /** Shows a graphical debug render in SimRobot */
      (float) OBSTACLE_AVOIDANCE_ARC_ANGLE_STEP_FACTOR,            /** Fraction of 2*pi used as angle increments to compute segments of arc for going around obstacles */
      (float) MAXIMUM_APPROACH_DISTANCE,
      (float) MINIMUM_APPROACH_DISTANCE,

      /*Range
      - MIN: Minimum distance from the opponent groundline above which kicks can be used (otherwise use InWalkKick to avoid losing time) 
      - MAX: (As per Challenge 1 rules) Distance from the opponent groundline below which kicks can be used */  
      (Rangef) X_RANGE_DISTANCE_FROM_GOAL_TO_USE_KICKS,

      /*These ranges on the y coordinate contribute to creating a "kicking area" all around the goal, inside which kicking to goal is allowed*/
      (Rangef) Y_RANGE_DISTANCE_FROM_GOAL_TO_USE_KICKS_INNER,
      (Rangef) Y_RANGE_DISTANCE_FROM_GOAL_TO_USE_KICKS_OUTER,

      (float) STATIC_APPROACH_RADIUS,

      (float) BASE_UPRIGHT_ROBOT_OBSTACLE_RADIUS,                     /** Base radius used for upright robot as obstacles when the ball is outside their influence radius (other wise the distance between the ball and the obstacle is used*/
      (float) MINIMUM_UPRIGHT_ROBOT_OBSTACLE_RADIUS,                  /** Minimum obstacle radius, used as a lower bound for the upright robots obstacle radius */ 
      (float) MINIMUM_UPRIGHT_ROBOT_OBSTACLE_RADIUS_IN_KICKING_AREA,   /** Minimum obstacle radius, used as a lower bound for the upright robots obstacle radius, when nearing the goal */ 
      (float) OBSTACLE_DANGER_ZONE_RADIUS,                            /** If the ball is inside this radius from its nearest obstacle, an escape target, along the tangent to this obstacle, is preferred instead of the usual ball path */

      //Only for challenge1
      (float) FIRST_TARGET_POS_X,                                     /** X coordinate of the first target used in the first long kick for challenge1 */
      (float) FIRST_TARGET_WITH_ONE_OBSTACLE_POS_Y,                   /** Y coordinate of the first target used in the first long kick for challenge1, when only one obstacle is in front of the center circle */
      (float) FIRST_TARGET_WITH_TWO_OBSTACLES_POS_Y,                  /** Y coordinate of the first target used in the first long kick for challenge1, when two obstacle are in front of the center circle */

      (bool) USE_LONG_KICKS_TO_CARRY,                                 /** Use a long kick as the first kick */    

      (bool) ESCAPE_OBSTACLES,                                        /** (EXPERIMENTAL) Use the obstacle escape routine */

      (bool) ALWAYS_USE_GOAL_TARGET_HEURISTIC,                        /** Force using the heuristic to compute every goal target */

      (float) OBSTACLE_RADIUS_MULTIPLIER_CHANGE_X_COORDINATE,         /** After this x coordinate the max of the obstacleRadiusMultiplier is multiplied to the obstacle radius, while before the min is used */
      (Rangef) OBSTACLE_RADIUS_MULTIPLIER,                            /** This multiplier is applied to the radius of obstacles before (the min of the range) and after (the max) the x coordinate specified in OBSTACLE_RADIUS_MULTIPLIER_CHANGE_X_COORDINATE*/

      (bool) USE_TRAJECTORY,
      (std::vector<BallCarrierModel::TrajectoryNode>) TRAJECTORY,
      
      (bool) ONLY_LEFT_PATH,

      (float) OFFSET_FROM_Y_CENTER_TO_CHANGE_TRAJECTORY,

    }),
});


/**
 * @class BallCarrierModelProvider
 * A module that provides the model of the opponent goal
 */
class BallCarrierModelProvider: public BallCarrierModelProviderBase
{
public:
  /** Constructor*/
  BallCarrierModelProvider();

private:
  void update(BallCarrierModel& ballCarrierModel);

};
