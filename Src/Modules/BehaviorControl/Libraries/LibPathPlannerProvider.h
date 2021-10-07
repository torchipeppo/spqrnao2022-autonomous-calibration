/**
 * @file LibPathPlannerProvider.h
 *
 * 
 * 
 * @author Emanuele Musumeci (based on LibCheckProvider.h)
 */

#pragma once

#include "Representations/BehaviorControl/Libraries/LibPathPlanner.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/BehaviorControl/BallCarrierModel/BallCarrierModel.h"

#include "Tools/Module/Module.h"
#include <math.h>

//Every time we add a module here, check in LibCheck if it's USED or REQUIRED
MODULE(LibPathPlannerProvider,
{,
  //REQUIRES(FieldBall),
  REQUIRES(BallModel),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(TeamPlayersModel),
  REQUIRES(TeamBehaviorStatus),
  REQUIRES(OwnTeamInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(LibCheck),
  
  PROVIDES(LibPathPlanner),
});


using Rotation = PathPlannerUtils::Rotation;
using BlockedSector = PathPlannerUtils::BlockedSector;
using Node = PathPlannerUtils::Node;
using Edge = PathPlannerUtils::Edge;
using Candidate = PathPlannerUtils::Candidate;
using Tangents = PathPlannerUtils::Tangents;
using Tangent = PathPlannerUtils::Tangent;
using Barrier = PathPlannerUtils::Barrier;
using Tangents = std::array<std::vector<Tangent>, PathPlannerUtils::numOfRotations>;

class LibPathPlannerProvider : public LibPathPlannerProviderBase
{
private:

  /**
   * Updates LibPathPlanner
   * @param libPathPlanner The representation provided
   */
  void update(LibPathPlanner& libPathPlanner) override;


  float defaultGoalPostRadius = 350.f; 
  float defaultRadiusControlOffset = 100.f; 
  float defaultFallenRobotRadius = 550.f; 
  float defaultUprightRobotRadius = 500.f; 
  float defaultReadyRobotRadius = 550.f;

  //std::vector<Node> nodes; 
  //std::vector<Candidate> candidates; 
  std::vector<Geometry::Line> borders; /**< The border of the field plus a tolerance. */
  Rotation lastDir = Rotation::cw; /**< Last direction selected when walking around first obstacle. */
  float turnAngleIntegrator = 0.f; /**< An integrator over the angle to the next node. Unclear which unit this has. */
  unsigned timeWhenLastPlayedSound = 0; /**< Used to limit frequency of sound playback. */
  bool pathPlannerWasActive = false; /**< Was the path planner active in previous frame? */

 public: 
  
  LibPathPlannerProvider(
                        float fieldBorderLimit = 350 /**< Distance outside the side lines that is still used for walking (in mm). */
                        );


  /**
   * Clip penalty area barriers to make a position reachable.
   * @param position The position that should be reachable.
   * @param left The y coordinate of the left barrier.
   * @param right The y coordinate of the right barrier.
   * @param front The x coordinate of the front barrier.
   */
  void clipPenaltyArea(const Vector2f& position, float& left, float& right, float& front) const;

  /**
   * Compute barrier lines that cannot be crossed during planning.
   * @param target The target the robot tries to reach.
   * @param excludePenaltyArea Also generate barriers for the own penalty area.
   * @param penaltyAreaRadius Radius to walk around a corner of the own penalty area (in mm).
   * @param radiusControlOffset Plan closer to obstacles by this offset, but keep original distance when executing plan (in mm).
   * @param wrongBallSideCostFactor How much of a full circle is it more expensive to pass the ball on the wrong side?
   * @param ballRadius Radius to walk around the ball (in mm).
   * @param wrongBallSideRadius How far from the ball is passing it on the wrong side penalized?
   */
  void createBarriers(std::vector<Barrier>& barriers, const Pose2f& source, const Pose2f& target, bool excludePenaltyArea,  
                                      float penaltyAreaRadius = 150, /**< Radius to walk around a corner of the own penalty area (in mm). */
                                      float radiusControlOffset = 100, /**< Plan closer to obstacles by this offset, but keep original distance when executing plan (in mm). */
                                      float wrongBallSideCostFactor = 0.25, /**< How much of a full circle is it more expensive to pass the ball on the wrong side? */
                                      float ballRadius = 250, /**< Radius to walk around the ball (in mm). */
                                      float wrongBallSideRadius = 400 /**< How far from the ball is passing it on the wrong side penalized? */
                                      );

  /**
   * Create the nodes from obstacles.
   * @param target The target the robot tries to reach.
   * @param excludePenaltyArea Filter out obstacles inside the own penalty area and the own goal.
   */
  void createNodes(std::vector<Node>& nodes, std::vector<Barrier>& barriers, const Pose2f& source, const Pose2f& target, bool excludePenaltyArea,
                                  float goalPostRadius = 350, /**< Radius to walk around a goal post (in mm). */
                                  float uprightRobotRadius = 500, /**< Radius to walk around an upright robot (in mm). */
                                  float fallenRobotRadius = 550, /**< Radius to walk around a fallen robot (in mm). */
                                  float readyRobotRadius = 550, /**< Radius to walk around a robot in ready state (in mm). */
                                  float radiusControlOffset = 100, /**< Plan closer to obstacles by this offset, but keep original distance when executing plan (in mm). */
                                  bool useObstacles = true, /**< Use TeamPlayersModel or ObstacleModel? */
                                  float penaltyAreaRadius = 150, /**< Radius to walk around a corner of the own penalty area (in mm). */
                                  float freeKickRadius = 850, /**< Radius to walk around the ball when defending a free kick (in mm). */
                                  float ballRadius = 250, /**< Radius to walk around the ball (in mm). */
                                  float centerCircleRadius = 0 /**< If != 0: Radius to walk around a the center circle in ready state in defensive kickoff (in mm). */
                                  );

  /**
   * Determine the radius of an obstacle.
   * @param type The type of the obstacle.
   * @return The radius with which the obstacle can be surrounded in mm. 0 if the obstacle should be ignored.
   */
  float getDefaultObstacleRadius(Obstacle::Type type);
  float getObstacleRadius(Obstacle::Type type, float goalPostRadius, float uprightRobotRadius, float fallenRobotRadius, float readyRobotRadius, float radiusControlOffset);

  /**
   * Adds a node for an obstacle if it is inside the field and valid.
   * @param center The center of the obstacle.
   * @param radius The radius of the obstacle. If 0, it is ignored.
   */
  void addObstacle(std::vector<Node>& nodes, const Vector2f& center, float radius);

  /**
   * Plan a shortest path. The result can be tracked backwards from the target node.
   * @param from The starting node. It is implicitly assumed that this is also the first entry in the vector "nodes".
   * @param to The target node.
   * @param speedRatio The ratio between forward speed and turn speed.
   */
  void plan(std::vector<Node>& nodes, std::vector<Barrier>& barriers, const Pose2f& source, Node& from, Node& to, float speedRatio);

  /**
   * Expand a node during the A* search and add all suitable outgoing edges to the set of open edges.
   * @param node The node that is expanded.
   * @param to The overall target node. Required to calculate the heuristic.
   * @param rotation Only the outgoing edges with the same rotation are expanded.
   * @param speedRatio The ratio between forward speed and turn speed.
   */
  void expand(std::vector<Node>& nodes, std::vector<Candidate>& candidates, std::vector<Barrier>& barriers, const Pose2f& source,
              Node& node, const Node& to, Rotation rotation, float speedRatio,
              float rotationPenalty = 150, /**< Penalty factor for rotating towards first intermediate target in mm/radian. Stabilizes path selection. */
              float switchPenalty = 400 /**< Penalty for selecting a different turn direction around first obstacle in mm. */
             );

  /**
   * Find all nodes reachable from this node without intersecting with other nodes, i.e. determine the outgoing edges.
   * @param node The node the outgoing edges of which are determined.
   */
  void findNeighbors(std::vector<Node>& nodes, std::vector<Barrier>& barriers, Node& node);

  /**
   * Create all tangents from one node to all other nodes. The number of tangents created per other node depends
   * on whether the nodes are circles or points (one for point to point, two for a point and a circle, four for two
   * circles) and whether they overlap (none if one node is inside the other one, two if they intersect, four if two
   * circles do not overlap). If another circle overlaps, the angular range of the overlap is also marked as being
   * blocked in the node passed, i.e. no tangents can start from this ranges.
   * @param node The node from which the tangents to all neighbors are created.
   * @param tangents The tangents found are returned here. Must be empty when passed. There are two sets of tangents,
   *                 i.e. the ones that start in clockwise direction and the ones that start in counter clockwise
   *                 direction. In addition, some tangents might be marked as dummies, because they are copies of
   *                 tangents in the other direction, but are needed by the sweepline algorithm that is later used.
   */
  void createTangents(std::vector<Node>& nodes, std::vector<Barrier>& barriers, Node& node, Tangents& tangents);

  /**
   * Add all outgoing edges of a node to that node based on the tangents to all other nodes. Do not add edges that
   * intersect with other nodes in between. This is determined using a sweepline algorithm that go through all
   * tangents in ascending angular direction and keeps track of all nodes in the current direction ordered by their
   * distance. Only the tangents to the closest node in each direction are accepted as outgoing edges. The is done
   * separately for outgoing edges in clockwise and counterclockwise directions.
   * @param node The starting node of the edges that are created.
   * @param tangents The tangents as produced by the method "createTangents".
   */
  void addNeighborsFromTangents(Node& node, Tangents& tangents);

  /** Computes possible plans (at each node two possible directions are considered, clock-wise and counter-clock-wise, and the best
   * outgoing branch is selected), to reach a certain target from a certain source (uses the framework's native A* path planner)
   * @param source the Pose2f origin point of the plan
   * @param target the Pose2f destination point of the plan
   * @param speed the desired speed for the plan
   * @param excludePenaltyArea a bool specifying whether we want to have path segments inside the penalty areass
   *
   * @return a std::vector<Node> containing all the nodes of the plan
   */
  std::vector<Node> populatePlan(Pose2f source, Pose2f target, Pose2f speed, bool excludePenaltyArea);

  /** Computes possible plans (at each node two possible directions are considered, clock-wise and counter-clock-wise, and the best
   * outgoing branch is selected), to reach a certain target from a certain source (uses the framework's native A* path planner). 
   * Allows specifying custom radiuses for each obstacle type
   * @param source the Pose2f origin point of the plan
   * @param target the Pose2f destination point of the plan
   * @param speed the desired speed for the plan
   * @param excludePenaltyArea a bool specifying whether we want to have path segments inside the penalty areass
   *
   * @return a std::vector<Node> containing all the nodes of the plan
   */
  std::vector<Node> populatePlanWithCustomObstacleRadius(const Pose2f source, const Pose2f target, const Pose2f speed, bool excludePenaltyArea,
                                                        float customGoalPostRadius, /**< Radius to walk around a goal post (in mm). */
                                                        float customUprightRobotRadius, /**< Radius to walk around an upright robot (in mm). */
                                                        float customFallenRobotRadius, /**< Radius to walk around a fallen robot (in mm). */
                                                        float customReadyRobotRadius, /**< Radius to walk around a robot in ready state (in mm). */
                                                        float customRadiusControlOffset /**< Plan closer to obstacles by this offset, but keep original distance when executing plan (in mm). */
                                                        );

  /** Given a populated plan, selects the best "avoidance plan", where each hop is the obstacle to be avoided that is nearest to the optimal path.
   *
   * @return a std::vector containing all the nodes of the plan
   */
  std::vector<Edge*> createAvoidancePlan(std::vector<Node>& nodes);

  /** Given a center, a radius, a starting and ending angle and an angle step, adds to vector path a series of hops arranged on a circumference sector around the center
   *
   * @param path the (ref) std::vector<Vector2f> sequence of "hops" in the circular path
   * @param center the (ref) Vector2f to the center of the circle
   * @param radius the radius of the circle
   * @param angleStep the angle increment to create all hops
   * @param fromAngle the beginning angle for the circumference segment
   * @param angleSize the size of the circumference segment
   * @return void (hops are added to path as a side-effect)
   */
  void createArc(std::vector<Vector2f>& path, Vector2f center, float radius, float angleStep, float fromAngle, float angleSize, float reversed = false);
 
  /** Given an avoidance plan, computes the shortest path, going around obstacles
   * @param avoidancePlan the (ref) std::vector<Edge*>& obstacle avoidance plan for the path
   * @param angleStep the float angleStep to determine hops on circular segments that "go around" obstacles
   * @return a std::vector<Vector2f> containing nodes of the path in sequence from source to destination
   */
  std::vector<Vector2f> computePath(std::vector<Node>& nodes, float angleStep);
  
};
