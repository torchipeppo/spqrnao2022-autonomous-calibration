/**
 * @file LibPathPlanner.h
 *
 * This file defines a representation that provides a library for planning paths from a custom source
 *
 * @author Emanuele Musumeci (based on LibCheck.h)
 */

#pragma once

#include "Representations/MotionControl/MotionRequest.h"
#include "Modules/BehaviorControl/PathPlannerProvider/PathPlannerProvider.h"
#include "Tools/Function.h"
#include "Tools/RobotParts/Arms.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"

namespace PathPlannerUtils {
 ENUM(Rotation,
  {,
    cw,
    ccw,
  });

  struct Node;

/** The edges of the visibility graph. */
  struct Edge
  {
    Node* fromNode; /**< The node from which this edge starts. */
    Node* toNode; /**< The node at which this edge ends. */
    float fromAngle; /**< The angle where this edge touches the circle around fromNode. */
    Vector2f toPoint; /**< The point where this edge touches the circle of toNode. */
    Rotation fromRotation; /**< The rotation with which fromNode was surrounded. */
    Rotation toRotation;  /**< The rotation with which toNode will be surrounded. */
    float length; /**< The length of this edge. */
    float pathLength; /**< The overall length of the path until arriving at toNode. Will be set by A* search. */

    /**
     * Constructor.
     * @param fromNode The node from which this edge starts.
     * @param toNode The node at which this edge ends.
     * @param fromAngle The angle where this edge touches the circle around fromNode.
     * @param toPoint The point where this edge touches the circle of toNode.
     * @param fromRotation The rotation with which fromNode was surrounded.
     * @param toRotation The rotation with which toNode will be surrounded.
     * @param length The length of this edge.
     */
    Edge(Node* fromNode, Node* toNode, float fromAngle, const Vector2f& toPoint, Rotation fromRotation, Rotation toRotation, float length)
      : fromNode(fromNode), toNode(toNode), fromAngle(fromAngle), toPoint(toPoint), fromRotation(fromRotation), toRotation(toRotation), length(length) {}
  };

  /**
   * A sector of a circle surrounding an obstacle that creates higher costs when
   * it is traversed.
   */
  struct BlockedSector : public Rangef
  {
    float costs; /**< costs for passing this circle segment. */
    BlockedSector(float min, float max, float costs = std::numeric_limits<float>::infinity()) : Rangef(min, max), costs(costs) {}
  };

  /** The nodes of the visibility graph, i.e. the obstacles. */
  struct Node : public Geometry::Circle
  {
    std::vector<Edge> edges[numOfRotations]; /** The outgoing edges per rotation. */
    std::vector<BlockedSector> blockedSectors; /**< Angular sectors that are blocked by overlapping other obstacles. */
    Edge* fromEdge[numOfRotations]; /**< From which edge was this node reached first (per rotation) during the A* search? */
    bool expanded = false; /**< Were the outgoing edges of this node already expanded? */
    int allowedClones = 0; /**< The number of times this node can be cloned. */
    float originalRadius; /**< The original radius of this node before it was reduced (in mm). */

    /**
     * Constructor.
     * @param center The center of the obstacle.
     * @param radius The radius of the obstacle.
     */
    Node(const Vector2f& center, float radius) : Circle(center, radius), originalRadius(radius)
    {
      fromEdge[cw] = fromEdge[ccw] = nullptr;
    }

    /**
     * The copy constructor copies all edges, but sets this node as their origin.
     * The new node has not been reached by from an edge yet.
     * @param other The other node that is copied.
     */
    Node(const Node& other) : Node(other.center, other.radius)
    {
      FOREACH_ENUM(Rotation, rotation)
        for(auto& edge : other.edges[rotation])
        {
          edges[rotation].emplace_back(this, edge.toNode, edge.fromAngle, edge.toPoint, edge.fromRotation, edge.toRotation, edge.length);
          edges[rotation].back().pathLength = edge.pathLength;
        }
      blockedSectors = other.blockedSectors;
      expanded = other.expanded;
      originalRadius = other.originalRadius;
    }
  };

  /** A structure to manage the open edges during the A* search. */
  struct Candidate
  {
    Edge* edge; /**< The corresponding edge. */
    float estimatedPathLength; /**< The estimated path length including the heuristic. */

    /**
     * Constructor.
     * @param edge The corresponding edge.
     * @param heuristic The estimated path length from the end of the edge to the target position.
     */
    Candidate(Edge* edge, float heuristic)
      : edge(edge), estimatedPathLength(edge->pathLength + heuristic) {}

    /**
     * Comparison operator for the heap that manages the open edges.
     * @param other The other candidate that is compared with.
     * @param Which one should be taken out of the heap later?
     */
    bool operator<(const Candidate& other) const
    {
      return estimatedPathLength > other.estimatedPathLength;
    }
  };

  /** Barrier lines that cannot be crossed during planning. */
  struct Barrier
  {
    Vector2f from; /**< First end point of barrier. */
    Vector2f to; /**< Second end point of barrier. */
    float costs; /**< Costs for crossing this barrier. */

    /**
     * Constructor.
     * @param x1 The x coordinate of first end point of the barrier.
     * @param y1 The y coordinate of first end point of the barrier.
     * @param x2 The x coordinate of second end point of the barrier.
     * @param y2 The y coordinate of second end point of the barrier.
     */
    Barrier(float x1, float y1, float x2, float y2, float costs = std::numeric_limits<float>::infinity())
      : from(x1, y1), to(x2, y2), costs(costs) {}

    /**
     * Does a line intersect this barrier?
     * @param p1 The first end point of the line.
     * @param p2 The second end point of the line.
     * @return Do they intersect?
     */
    bool intersects(const Vector2f& p1, const Vector2f& p2) const
    {
      return Geometry::checkIntersectionOfLines(from, to, p1, p2);
    }
  };

  struct Tangent : public Edge
  {
    ENUM(Side,
    {,
      none,
      left,
      right,
    });

    Side side; /**< Is this the left or right side of the corridor to the other node? */
    float circleDistance; /**< The closest distance between the borders of the two node connected by this tangent. */
    bool dummy; /**< Is this just a helper and should not be transformed into a real edge? */
    bool ended = false; /**< Has the matching left tangent already processed for this right tangent? */
    int matchingRightTangent = -1; /**< The index of the matching right tangent for this left tangent. */

    /**
     * Constructor.
     * @param edge The edge that might be added to the graph if is not blocked by obstacles.
     * @param side Is this the left or right side of the corridor to the other node?
     * @param circleDistance The closest distance between the borders of the two node connected by this tangent.
     * @param dummy Is this just a helper and should not be transformed into a real edge?
     */
    Tangent(const Edge& edge, Side side, float circleDistance, bool dummy)
    : Edge(edge), side(side), circleDistance(circleDistance), dummy(dummy) {}
  };

  using Tangents = std::array<std::vector<PathPlannerUtils::Tangent>, numOfRotations>;
};

STREAMABLE(LibPathPlanner,
{
  /** Computes the attractive field for the striker **/
  FUNCTION(std::vector<PathPlannerUtils::Node>(Pose2f source, Pose2f target, Pose2f speed, bool excludePenaltyArea)) populatePlan;
  FUNCTION(std::vector<PathPlannerUtils::Node>(Pose2f source, Pose2f target, Pose2f speed, bool excludePenaltyArea, float customGoalPostRadius, float customUprightRobotRadius, float customFallenRobotRadius, float customReadyRobotRadius, float customRadiusControlOffset)) populatePlanWithCustomObstacleRadius;
  FUNCTION(float(Obstacle::Type type)) getDefaultObstacleRadius;
  FUNCTION(std::vector<PathPlannerUtils::Edge*>(std::vector<PathPlannerUtils::Node>& nodes)) createAvoidancePlan;
  FUNCTION(std::vector<Vector2f>(std::vector<PathPlannerUtils::Node>& nodes, float angleStep)) computePath,

});
