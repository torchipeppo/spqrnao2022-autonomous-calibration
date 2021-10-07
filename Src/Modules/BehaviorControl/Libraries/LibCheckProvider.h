/**
 * @file LibCheckProvider.h
 *
 * Performs some checks of the behavior control module such as
 * - How often is a MotionRequest set?
 * - How often is a HeadMotionRequest set?
 *
 * @author Tim Laue
 */

#pragma once

#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/BallCarrierModel/BallCarrierModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/TeamBallModel.h"

//Goal targeting system
#include "Representations/Modeling/OpponentGoalModel.h"
#include "Representations/Configuration/BallSpecification.h"

#include "Representations/spqr_representations/RoleAndContext.h"

#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/spqr_representations/FreeCorridors.h"
#include "Representations/spqr_representations/OurDefinitions.h"

#include "Representations/spqr_representations/PassShare.h"

#include "Tools/Module/Module.h"
#include <math.h>

MODULE(LibCheckProvider,
{,
  USES(ActivationGraph),
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
  REQUIRES(FreeCorridors),
  //RECENTLY ADDED
  //Goal targeting system
  USES(OpponentGoalModel),
  USES(BallSpecification),
  //BallCarrierModel
  USES(BallCarrierModel),
  //
  USES(Role),
  USES(RoleAndContext),
  USES(TeamActivationGraph),
  USES(TeamBehaviorStatus),
  USES(TeamData),
  USES(PassShare),
  PROVIDES(LibCheck),
  LOADS_PARAMETERS(
  {,
    (std::vector<int>) notSetCheck,       /** Assert that a request has been set at least once */
    (std::vector<int>) multipleSetCheck,  /** Print a warning if an assert has not been set at least once */
    (bool) assertValidWalkRequest,        /** Asserts that there are no strange walk parameters */
  }),
});

class LibCheckProvider : public LibCheckProviderBase
{
private:
  int callCounters[LibCheck::numOfCheckedOutputs]; /**< The counters for different checks */
  bool setArmsInThisFrame[Arms::numOfArms]; /**< This arm was set in this frame */

  /**
   * Updates LibCheck
   * @param libCheck The representation provided
   */
  void update(LibCheck& libCheck) override;

  /** Resets all status information */
  void reset();

  /**
   * Checks whether a behavior part set all its outputs
   * @param activationGraph The activation graph of the behavior part
   * @param start The first output ID to be checked
   * @param end The first output ID not to be checked after \c start
   */
  void checkOutputs(const ActivationGraph& activationGraph, LibCheck::CheckedOutput start, LibCheck::CheckedOutput end) const;

  /**
   * Checks whether the motion request is valid
   * @param activationGraph The activation graph of the individual behavior
   * @param theMotionRequest The motion request to check for validity
   */
  void checkMotionRequest(const ActivationGraph& activationGraph, const MotionRequest& theMotionRequest) const;

  /** Increments one counter */
  void inc(LibCheck::CheckedOutput outputToCheck);

  /**
   * Serializes an activation graph to a string
   * @param activationGraph The activation graph to convert
   * @return The compressed string that represents the activation graph
   */
  std::string getActivationGraphString(const ActivationGraph& activationGraph) const;

  /**
   * Provides the Pose to reach in ready state for each robot
   * @return the target pose
   */
  Pose2f myReadyPosition() const;

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

  /** Provides a target, on the line between the robot and t, at a defined distance d
   * @param t the target to be refined (global)
   * @param d the desired distance
   * @return a target with distance d from the robot
   */
 Pose2f refineTarget(Pose2f t, float d);

 /** Returns the global y coord point we are looking at on the opponent groundline
  * @return Global y coord of the point we are looking at
  * **/
 float projectGazeOntoOpponentGroundline();

 /** Given a certain point in input proivdes its projection on the opponent ground line by the robot's perspective
  * @param x x global coordinate of point to be projected
  * @param y y global coordinate of point to be projected
  * @return Global y coord of the projected point
  * **/
 float projectPointOntoOpponentGroundline(float x, float y);

 /** Return the distance between my gaze projection and the obstacle
     left/right point onto groundline or 0 if the gaze is directly inside
     an obstacle
  * @param x x global coordinate of point to be projected
  * @param y y global coordinate of point to be projected
  * @return Global y coord of the projected point
  * **/
 float gazeToObstacleProjectionDistanceOntoOpponentGroundLine(Obstacle obs);

 /** Provides a float value representing a score for each FreeGoalTargetableArea Determined by computeFreeAreas
  * @param leftLimit left limit of the free targetable area
  * @param rightLimit right limit of the free targetable area
  * @param poles_weight relative weight in the final utility for the poles
  * @param opponents_weight relative weight in the final utility for the opponents
  * @param teammates_weight relative weight in the final utility for the teammates
  * @return value assigned after area evaluation
  * **/
 float areaValueHeuristic(float leftLimit, float rightLimit, float poles_weight = 1, float opponents_weight = 1, float teammates_weight = 1);

  /** Tells whether two segments are overlapping or not
  * @param l1 left limit of first segment
  * @param r1 right limit of the first segment
  * @param l2 left limit of the second segment
  * @param r2 right limit of the second segment
  * @return bool value
  * **/
 bool areOverlappingSegmentsOnYAxis (float l1, float r1, float l2, float r2);

 /** Provides a vector with the point of beginning and finish of goal areas free from opponent coverage
  * @param myPose pose of the robot
  * @param opponents the opponent vector (global coordinates)
  * @return vector of free areas
  * **/
 std::vector<FreeGoalTargetableArea> computeFreeAreas (float minimumDiscretizedAreaSize);

 /** Based on a previous implementation by Emanuele Antonioni, provides the best point to shoot at inside the goal. If the opponent goal is completely occluded
  * returns the field center (exactly (0,0))
  * @param shootASAP If set to true, if the robot is near the goal, shoot in the spot nearest to where you're looking at ("As Soon As Possible"), else use the heuristic to decide
  * @param forceHeuristic If se to true, always use the heuristic to decide where to shoot
  * @return the Vector2f of the position selected to shoot
  * **/
Vector2f goalTarget(bool shootASAP, bool forceHeuristic);

/** Variant of goalTarget that also return the FreeGoalTargetableArea associated to the target point.
  * @param shootASAP If set to true has the robot will shoot to the nearest accessible point, located inside the nearest targetable area
  * @return the Vector2f of the position selected to shoot
  * **/
 std::pair<Vector2f, FreeGoalTargetableArea> goalTargetWithArea (bool shootASAP, bool forceHeuristic);


 int isTargetToPass;

 float sqrDistanceOfClosestOpponentToPoint(Vector2f p);


 Pose2f glob2Rel(float x, float y);
 Pose2f rel2Glob(float x, float y);
 Vector2f getSupporterMarkPosition();
 Vector2f getSupportStrikerPosition();
 Vector2f getSupporterPosition();
 Vector2f getJollyPosition();
 Vector2f updateDefender();
 Vector2f updateSupporter();
 Vector2f updateGoalie();
 float distanceToLine(Vector2f objectToCheck, Vector2f linePoint1, Vector2f linePoint2);
 bool obstacleExistsAroundPoint(Vector2f point);

 bool opponentOnOurField();
 std::tuple<int,int,Pose2f> strikerPassShare();
 bool isValueBalanced(float currentValue, float target, float bound);

 bool isGoalieInStartingPosition();
 bool isBallInKickAwayRange();
 bool isGoalieInKickAwayRange();
 bool isBallInArea();
 bool isGoalieInArea();
 bool isGoalieInAngle();
 float goalie_displacement;
 float angleToGoal;
 float angleToBall;
 float angleToMyGoal;
 float penaltyAngle;
 float kickAngle;
 float correctionKickAngle;
 bool ballOutOnLeft;
 float radiansToDegree(float x);


 float angleToTarget(float x, float y);   // TODO This is to check orientation wrt to target x = 4500 y = 3000 to left and -3000 to right
 float norm(float x, float y);

 //Maps value from interval [fromIntervalMin, fromIntervalMax] to interval [toIntervalMin, toIntervalMax]
 float mapToInterval(float value, float fromIntervalMin, float fromIntervalMax, float toIntervalMin, float toIntervalMax);

 public: LibCheckProvider();
};
