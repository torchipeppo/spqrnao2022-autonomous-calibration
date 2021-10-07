/**
 * @file LibPotentialFieldsProvider.h
 *
 * Performs some checks of the behavior control module such as
 * - How often is a MotionRequest set?
 * - How often is a HeadMotionRequest set?
 *
 * @author Tim Laue
 */

#pragma once

#include "Representations/BehaviorControl/Libraries/LibPotentialFields.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/NodePF.h"

#include "Representations/spqr_representations/ConfigurationParameters.h"

#include "Tools/Module/Module.h"
#include <math.h>

MODULE(LibPotentialFieldsProvider,
{,
  REQUIRES(BallModel),
  REQUIRES(FieldBall),
  REQUIRES(FrameInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamBallModel),
  REQUIRES(TeamPlayersModel),
  REQUIRES(ObstacleModel),

  USES(TeamBehaviorStatus),
  USES(TeamData),
  USES(Role),
  USES(PassShare),

  PROVIDES(LibPotentialFields),
});

class LibPotentialFieldsProvider : public LibPotentialFieldsProviderBase
{
  /**
   * Updates LibPotentialFields
   * @param libCheck The representation provided
   */
  void update(LibPotentialFields& libCheck) override;

  /** Resets all status information */
  void reset();

  /**
   * Provides the distance between two pose2f
   * @param p1 the first point
   * @param p2 the second point
   * @return the distance between p1 and p2
   */
  float distance(Pose2f p1, Pose2f p2) const;

  /**
   * Provides the distance between two points (x1,y1) and (x2,y2)
   * @param x1 the first point
   * @param y1 the second point
   * @param x2 the first point
   * @param y2 the second point
   * @return the distance between p1 and p2
   */
  float distance(float x1, float y1, float x2, float y2) const;

 /** Computes the attractive field for the striker
  * @param goal The 2D position of the goal (that generates the attractive field)
  * @return std::vector of PFCell structures, containing various info about the potential field cell
  * **/
 std::vector<NodePF> computeStrikerAttractivePF(std::vector<NodePF>& potential_field, Vector2f goal, float RO = 1000.f, float Kap = 0.1f, float Kbp = 100.f,
                                                  float Kr = 100.f, float TEAMMATE_CO = 500.f, float ETA = 1000.f, float GAMMA = 2.f);

 /** Computes the repulsive field for the striker
  * @param goal The 2D position of the goal (that generates the attractive field)
  * @return std::vector of PFCell structures, containing various info about the potential field cell
  * **/
 std::vector<NodePF> computeStrikerRepulsivePF(std::vector<NodePF>& potential_field, Vector2f source_pos, bool navigateAroundBall = false, 
                                                  float RO = 1000.f, float Kap = 0.1f, float Kbp = 100.f, float Kr = 100.f,
                                                  float TEAMMATE_CO = 500.f, float ETA = 1000.f, float GAMMA = 2.f, float POW = 1000.f);

 /** Computes the repulsive field for the striker with custom specified obstacles
  * @param source_pos The 2D position of the center of the field
  * @param repulsive_obstacles A list of 2D positions of the obstacles (that generate the repulsive field)
  * @return std::vector of PFCell structures, containing various info about the potential field cell
  * **/
 std::vector<NodePF> computeStrikerRepulsivePFWithCustomObstacles(std::vector<NodePF>& potential_field, Vector2f source_pos, std::vector<Vector2f>& repulsive_obstacles,
                                                  float RO = 1000.f, float Kap = 0.1f, float Kbp = 100.f, float Kr = 100.f,
                                                  float TEAMMATE_CO = 500.f, float ETA = 1000.f, float GAMMA = 2.f, float POW = 1000.f);

 /** Initializes an empty PF spanning a circle given a center point and a radius
  * @param cell_size Side length of a discretized potential field cell
  * @return std::vector of NodePF with null potential
  * **/
std::vector<NodePF> initializePFAroundPoint(float cell_size, Vector2f field_center, float field_radius, float FIELD_BORDER_OFFSET = 300);

 /** Initializes an empty PF spanning the whole field
  * @param cell_size Side length of a discretized potential field cell
  * @return std::vector of NodePF with null potential
  * **/
std::vector<NodePF> initializePFAllField(float cell_size, float FIELD_BORDER_OFFSET = 300);

 /** Based on a previous implementation by Vincenzo Suriani, computes the an artificial potential field given a precomputed attractive and
  * repulsive field respectively. Allows specifying a maximum cell update radius around the player.
  * @param obstacles A vector of obstacles that generate the repulsive field
  * @param attractive_field An std::vector containing the precomputed attractive field, it has to be of the same size of repulsive_field
  * @param attractive_field An std::vector containing the precomputed repulsive field, it has to be of the same size of attractive_field
  * @param radius Maximum distance of the computed cells (cells that further away will not be updated). If left to 0.f, the whole field will be computed
  * @return std::vector of PFCell structures, containing various info about the potential field cell
  * **/
 std::vector<NodePF> computePFAllField (std::vector<NodePF>& potential_field, std::vector<NodePF> attractive_field, std::vector<NodePF> repulsive_field);

 /** Based on a previous implementation by Vincenzo Suriani, computes the an artificial potential field given a precomputed attractive and
  * repulsive field respectively. The field is computed in a certain radius around a given point
  * @param obstacles A vector of obstacles that generate the repulsive field
  * @param attractive_field An std::vector containing the precomputed attractive field, it has to be of the same size of repulsive_field
  * @param attractive_field An std::vector containing the precomputed repulsive field, it has to be of the same size of attractive_field
  * @param radius Maximum distance of the computed cells (cells that further away will not be updated). If left to 0.f, the whole field will be computed
  * @return std::vector of PFCell structures, containing various info about the potential field cell
  * **/
 std::vector<NodePF> computePFAroundPoint (std::vector<NodePF>& potential_field, std::vector<NodePF> attractive_field, std::vector<NodePF> repulsive_field);

 Pose2f glob2Rel(float x, float y);
 Pose2f rel2Glob(float x, float y);
 bool obstacleExistsAroundPoint(Vector2f point);

 bool isValueBalanced(float currentValue, float target, float bound);

 float radiansToDegree(float x);

 float angleToTarget(float x, float y);   // TODO This is to check orientation wrt to target x = 4500 y = 3000 to left and -3000 to right
 float norm(float x, float y);

 //Maps value from interval [fromIntervalMin, fromIntervalMax] to interval [toIntervalMin, toIntervalMax]
 float mapToInterval(float value, float fromIntervalMin, float fromIntervalMax, float toIntervalMin, float toIntervalMax);

 public: LibPotentialFieldsProvider();
};
