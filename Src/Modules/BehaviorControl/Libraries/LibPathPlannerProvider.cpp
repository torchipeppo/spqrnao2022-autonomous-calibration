/**
 * @file LibPathPlannerProvider.cpp
 *
 * Provides a library of streamable functions for path planning
 *
 * @author Emanuele Musumeci (based on LibCheckProvider)
 */
#include "Platform/Nao/SoundPlayer.h"
#include "LibPathPlannerProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/Annotation.h"
#include <iostream>
#include <algorithm>
#define SQ(x) x*x

MAKE_MODULE(LibPathPlannerProvider, behaviorControl);

static const float epsilon = 0.1f; // Small offset in mm.

LibPathPlannerProvider::LibPathPlannerProvider(
                                              float fieldBorderLimit
                                              ) : pathPlannerWasActive(true), turnAngleIntegrator(0.f)
{
  borders.emplace_back(theFieldDimensions.xPosOpponentGroundline + fieldBorderLimit, 0.f, 0.f, -1.f);
  borders.emplace_back(theFieldDimensions.xPosOwnGroundline - fieldBorderLimit, 0.f, 0.f, 1.f);
  borders.emplace_back(0.f, theFieldDimensions.yPosLeftSideline + fieldBorderLimit, 1.f, 0.f);
  borders.emplace_back(0.f, theFieldDimensions.yPosRightSideline - fieldBorderLimit, -1.f, 0.f);
}

void LibPathPlannerProvider::update(LibPathPlanner& libPathPlanner)
{

  if(!pathPlannerWasActive)
  {
    turnAngleIntegrator = 0.f;
    lastDir = PathPlannerUtils::numOfRotations;
  }
  else
    pathPlannerWasActive = false;
  
  libPathPlanner.populatePlan = [this](Pose2f source, Pose2f target, Pose2f speed, bool excludePenaltyArea) -> std::vector<PathPlannerUtils::Node>
  {
    return populatePlan(source, target, speed, excludePenaltyArea);
  };

  libPathPlanner.populatePlanWithCustomObstacleRadius = [this](Pose2f source, Pose2f target, Pose2f speed, bool excludePenaltyArea, float goalPostRadius = 350, float radiusControlOffset = 100, float fallenRobotRadius = 550, float uprightRobotRadius = 500, float readyRobotRadius = 550) -> std::vector<PathPlannerUtils::Node>
  {
    return populatePlanWithCustomObstacleRadius(source, target, speed, excludePenaltyArea, goalPostRadius, radiusControlOffset, fallenRobotRadius, uprightRobotRadius, readyRobotRadius);
  };

  libPathPlanner.createAvoidancePlan = [this](std::vector<PathPlannerUtils::Node>& nodes) -> std::vector<PathPlannerUtils::Edge*>
  {
    return createAvoidancePlan(nodes);
  };

  libPathPlanner.computePath = [this](std::vector<Node>& nodes, float angleStep) -> std::vector<Vector2f>
  {
    return computePath(nodes, angleStep);
  };

  libPathPlanner.getDefaultObstacleRadius = [this](Obstacle::Type type) -> float
  {
    return getDefaultObstacleRadius(type);
  };
}

void LibPathPlannerProvider::clipPenaltyArea(const Vector2f& position, float& left, float& right, float& front) const
{
  if(position.x() <= front && position.y() >= right && position.y() <= left)
  {
    // If the robot is inside the penalty area, move the closest barrier so the robot is still outside
    const float distanceLeft = left - position.y();
    const float distanceRight = position.y() - right;
    const float distanceFront = front - position.x();
    if(distanceLeft < std::min(distanceRight, distanceFront))
      left = position.y() - epsilon;
    else if(distanceRight < std::min(distanceLeft, distanceFront))
      right = position.y() + epsilon;
    else
      front = position.x() - epsilon;
  }
}

void LibPathPlannerProvider::createNodes(std::vector<Node>& nodes, std::vector<Barrier>& barriers, const const Pose2f& source, const Pose2f& target, bool excludePenaltyArea,
                                         float goalPostRadius,
                                         float uprightRobotRadius,
                                         float fallenRobotRadius,
                                         float readyRobotRadius,
                                         float radiusControlOffset,
                                         bool useObstacles,
                                         float penaltyAreaRadius,
                                         float freeKickRadius,
                                         float ballRadius,
                                         float centerCircleRadius
                                         )
{
  // Reserve enough space that prevents any reallocation, because the addresses of entries are used.
  nodes.reserve(sqr((excludePenaltyArea ? 8 : 6) +
                    (useObstacles ? theObstacleModel.obstacles.size() : theTeamPlayersModel.obstacles.size())));

  // Insert start and target
  nodes.emplace_back(source.translation, 0.f);
  nodes.emplace_back(target.translation, 0.f);
  Node& from(nodes.front());
  Node& to(nodes.back());

  // Insert goalposts
  nodes.emplace_back(Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal), goalPostRadius - radiusControlOffset);
  nodes.emplace_back(Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal), goalPostRadius - radiusControlOffset);
  nodes.emplace_back(Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal), goalPostRadius - radiusControlOffset);
  nodes.emplace_back(Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal), goalPostRadius - radiusControlOffset);

  if(excludePenaltyArea)
  {
    // The nodes around the penalty area will be intersected by barriers. Therefore, they can
    // be reached from two sides and must be cloneable once.
    nodes.emplace_back(Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea), penaltyAreaRadius - radiusControlOffset + epsilon);
    nodes.back().allowedClones = 1;
    nodes.emplace_back(Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea), penaltyAreaRadius - radiusControlOffset + epsilon);
    nodes.back().allowedClones = 1;
    nodes.emplace_back(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea), penaltyAreaRadius - radiusControlOffset + epsilon);
    nodes.back().allowedClones = 1;
    nodes.emplace_back(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightPenaltyArea), penaltyAreaRadius - radiusControlOffset + epsilon);
    nodes.back().allowedClones = 1;
  }

  // Insert obstacles if they are on the field.
  if(useObstacles)
  {
    for(const auto& obstacle : theObstacleModel.obstacles)
      if(obstacle.type != Obstacle::goalpost)
        //addObstacle(nodes, theRobotPose * obstacle.center, getDefaultObstacleRadius(obstacle.type));
        addObstacle(nodes, theRobotPose * obstacle.center, getObstacleRadius(obstacle.type, goalPostRadius, uprightRobotRadius, fallenRobotRadius, readyRobotRadius, radiusControlOffset));
  }
  else
  {
    for(const auto& obstacle : theTeamPlayersModel.obstacles)
      if(obstacle.center != theRobotPose.translation && obstacle.type != Obstacle::goalpost)
        //addObstacle(nodes, obstacle.center, getDefaultObstacleRadius(obstacle.type));
        addObstacle(nodes, obstacle.center, getObstacleRadius(obstacle.type, goalPostRadius, uprightRobotRadius, fallenRobotRadius, readyRobotRadius, radiusControlOffset));
  }

  // Add ball in playing if it is not the target
  /*if(theGameInfo.state == STATE_PLAYING)
  {
    const Vector2f& ballPosition = theTeamBehaviorStatus.role.playBall ? theFieldBall.recentBallEndPositionOnField() : theFieldBall.recentBallPositionOnField();
    if(theGameInfo.setPlay != SET_PLAY_NONE && theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber)
      addObstacle(nodes, ballPosition, freeKickRadius);
    else if((ballPosition - to.center).norm() >= 1.f)
      addObstacle(nodes, ballPosition, ballRadius);
  }*/

  if(centerCircleRadius != 0.f && theGameInfo.state == STATE_READY && theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber)
    addObstacle(nodes, Vector2f::Zero(), centerCircleRadius);

  // If other nodes surround start or target, shrink them.
  for(auto node = nodes.begin(); node != nodes.begin() + 2; ++node)
    for(auto other = nodes.begin() + 2; other != nodes.end();)
    {
      if((other->center - node->center).squaredNorm() <= sqr(other->radius))
      {
        Vector2f diff = other->center - node->center;
        const float distance = diff.norm();
        other->radius = (other->radius + distance - epsilon) / 2.f;
        other->center = node->center + (other->center - node->center).normalized(other->radius + epsilon);
        if(other->radius <= 0.f)
        {
          other = nodes.erase(other);
          continue; // skip ++
        }
      }
      ++other;
    }

  // If start and target are both inside the field, prevent passing obstacles outside the field.
  Boundaryf border(Rangef(borders[1].base.x(), borders[0].base.x()),
                   Rangef(borders[3].base.y(), borders[2].base.y()));
  if(border.isInside(from.center) && border.isInside(to.center))
  {
    Vector2f p1;
    Vector2f p2;
    for(auto node = nodes.begin() + 2; node != nodes.end(); ++node)
      for(const auto& border : borders)
        if(Geometry::getIntersectionOfLineAndCircle(border, *node, p1, p2) == 2)
          node->blockedSectors.emplace_back((p1 - node->center).angle(), (p2 - node->center).angle());
  }

  // Whenever a barrier intersects a node, add a blocking sector.
  for(auto& node : nodes)
  {
    for(auto& barrier : barriers)
    {
      Geometry::Line line(barrier.from, barrier.to - barrier.from);
      Vector2f p1;
      Vector2f p2;
      if(Geometry::getIntersectionOfLineAndCircle(line, node, p1, p2) == 2)
      {
        if(Geometry::getDistanceToEdge(line, p1) == 0.f)
        {
          const float angle = (p1 - node.center).angle();
          node.blockedSectors.emplace_back(angle, angle, barrier.costs);
        }
        if(Geometry::getDistanceToEdge(line, p2) == 0.f)
        {
          const float angle = (p2 - node.center).angle();
          node.blockedSectors.emplace_back(angle, angle, barrier.costs);
        }
      }
    }
  }
}

void LibPathPlannerProvider::addObstacle(std::vector<Node>& nodes, const Vector2f& center, float radius)
{
  if(radius != 0.f &&
     borders[0].base.x() > center.x() - radius &&
     borders[1].base.x() < center.x() + radius &&
     borders[2].base.y() > center.y() - radius &&
     borders[3].base.y() < center.y() + radius)
     {
        //std::cout<<"Obstacle: ("<<center.x()<<", "<<center.y()<<")"<<std::endl;
        nodes.emplace_back(center, radius);
     }
    
}

void LibPathPlannerProvider::createBarriers(std::vector<Barrier>& barriers, const Pose2f& source, const Pose2f& target, bool excludePenaltyArea, 
                                            float penaltyAreaRadius, float radiusControlOffset,
                                            float wrongBallSideCostFactor, float ballRadius, float wrongBallSideRadius)
{

    //Barrier lines that cannot be crossed during planning.
    barriers.reserve(8);

    // Add sides of the goal nets
    barriers.emplace_back(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal,
                            theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftGoal);
    barriers.emplace_back(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal,
                            theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosRightGoal);
    barriers.emplace_back(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal,
                            theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosLeftGoal);
    barriers.emplace_back(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal,
                            theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosRightGoal);

    if(excludePenaltyArea)
    {
        float left = theFieldDimensions.yPosLeftPenaltyArea + penaltyAreaRadius - radiusControlOffset;
        float right = theFieldDimensions.yPosRightPenaltyArea - penaltyAreaRadius + radiusControlOffset;
        float front = theFieldDimensions.xPosOwnPenaltyArea + penaltyAreaRadius - radiusControlOffset;

        clipPenaltyArea(source.translation, left, right, front);
        clipPenaltyArea(target.translation, left, right, front);

        barriers.emplace_back(theFieldDimensions.xPosOwnPenaltyArea, left,
                            theFieldDimensions.xPosOwnGroundline, left);
        barriers.emplace_back(theFieldDimensions.xPosOwnPenaltyArea, right,
                            theFieldDimensions.xPosOwnGroundline, right);
        barriers.emplace_back(front, theFieldDimensions.yPosLeftPenaltyArea,
                            front, theFieldDimensions.yPosRightPenaltyArea);
    }

    if(wrongBallSideCostFactor > 0.f)
    {
        //const Vector2f& ballPosition = theFieldBall.recentBallPositionOnField();
        const Vector2f& ballPosition = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        
        Vector2f end = ballPosition + (ballPosition - Vector2f(theFieldDimensions.xPosOwnGoal, 0)).normalized(wrongBallSideRadius);
        barriers.emplace_back(ballPosition.x(), ballPosition.y(),end.x(), end.y(), ballRadius * pi2 * wrongBallSideCostFactor);
    }

}

void LibPathPlannerProvider::plan(std::vector<Node>& nodes, std::vector<Barrier>& barriers, const Pose2f& source, Node& from, Node& to, float speedRatio)
{
  std::vector<Candidate> candidates; /**< All open edges during the A* search. */
  candidates.reserve(nodes.size() * nodes.size() * 4);

  expand(nodes, candidates, barriers, source, from, to, Rotation::cw, speedRatio);
  expand(nodes, candidates, barriers, source, from, to, Rotation::ccw, speedRatio);

  //NOTICE: INVESTIGATE WHY CANDIDATES IS EMPTY (MAYBE SPEED OR SOMETHING ELSE)

  //std::cout<<"Candidates: "<<candidates.size()<<std::endl;
  // Do A* search
  while(!candidates.empty())
  {
    Candidate candidate = candidates.front();

    std::pop_heap(candidates.begin(), candidates.end());
    candidates.pop_back();

    //std::cout<<"Candidate: from ("<<candidate.edge->fromNode->center.x()<<", "<<candidate.edge->fromNode->center.y()<<") to ("<<candidate.edge->toNode->center.x()<<", "<<candidate.edge->toNode->center.y()<<")"<<std::endl;

    // Clone target node if it was already reached and clones are allowed.
    if(candidate.edge->toNode->fromEdge[candidate.edge->toRotation] &&
       candidate.edge->toNode->allowedClones > 0)
    {
      nodes.emplace_back(*candidate.edge->toNode);
      --candidate.edge->toNode->allowedClones;
      candidate.edge->toNode = &nodes.back();
    }
    if(!candidate.edge->toNode->fromEdge[candidate.edge->toRotation])
    {
      candidate.edge->toNode->fromEdge[candidate.edge->toRotation] = candidate.edge;
      if(candidate.edge->toNode == &to)
      {
        //std::cout<<"DESTINATION REACHED"<<std::endl;
        break;
      }
      else
        expand(nodes, candidates, barriers, source, *candidate.edge->toNode, to, candidate.edge->toRotation, speedRatio);
    }
  }
}

void LibPathPlannerProvider::expand(std::vector<Node>& nodes, std::vector<Candidate>& candidates, std::vector<Barrier>& barriers, 
                                    const Pose2f& source, Node& node, const Node& to, Rotation rotation, float speedRatio,                                
                                    float rotationPenalty, float switchPenalty)
{
  if(!node.expanded)
  {
    findNeighbors(nodes, barriers, node);
    node.expanded = true;
  }

  for(auto& edge : node.edges[rotation])
  {
    edge.pathLength = edge.length;
    if(node.fromEdge[rotation])
    {
      // This is not the first node, i.e. we arrived here at fromEdge->toPoint.
      edge.pathLength += node.fromEdge[rotation]->pathLength;
      const float toAngle = (node.fromEdge[rotation]->toPoint - node.center).angle();

      // The sector used on the circle is from toAngle of the incoming edge
      // to fromAngle of the outgoing edge.
      Rangef interval;
      if(rotation == Rotation::cw)
      {
        interval.min = edge.fromAngle;
        interval.max = toAngle;
      }
      else
      {
        interval.min = toAngle;
        interval.max = edge.fromAngle;
      }

      // If an blocking sector overlaps with this interval, at least one limit
      // of one interval must be inside the other interval.
      // If it is, reaching the outgoing edge is not possible.
      for(const auto& sector : node.blockedSectors)
        if(interval.isInside(sector.min) || interval.isInside(sector.max) ||
           sector.isInside(interval.min) || sector.isInside(interval.max))
        {
          if(sector.costs == std::numeric_limits<float>::infinity())
            goto continueOuterLoop;
          else
            edge.pathLength += sector.costs;
        }

      // Compute the positive angle from incoming to outgoing edge in the fixed direction (cw/ccw).
      float angle = interval.max - interval.min;
      if(angle < 0.f)
        angle += pi2;

      edge.pathLength += angle * node.radius;
    }
    else
    {
      // This is the first node. Add penalty for rotating to outgoing edge.
      const float toRotate = std::abs((source.translation - edge.toNode->center).norm() > edge.toNode->radius
                                      ? (Pose2f(edge.toPoint) - source).translation.angle()
                                      : Angle::normalize((edge.toPoint - edge.toNode->center).angle() + (edge.toRotation == Rotation::cw ? -pi_2 : pi_2) - source.rotation));
      const float distanceRatio = toRotate * speedRatio;
      edge.pathLength += distanceRatio * rotationPenalty + (lastDir == edge.toRotation ? 0.f : switchPenalty);
    }

    candidates.emplace_back(&edge, (to.center - edge.toPoint).norm());
    std::push_heap(candidates.begin(), candidates.end());

  continueOuterLoop:
    ;
  }
}

void LibPathPlannerProvider::findNeighbors(std::vector<Node>& nodes, std::vector<Barrier>& barriers, Node& node)
{
  Tangents tangents;
  createTangents(nodes, barriers, node, tangents);
  addNeighborsFromTangents(node, tangents);
  //FOREACH_ENUM(PathPlannerUtils::Rotation, i)
  //  std::cout<<"Edges for rotation "<<i<<": "<<node.edges[i].size()<<std::endl;
  //std::cout<<std::endl;
}

void LibPathPlannerProvider::createTangents(std::vector<Node>& nodes, std::vector<Barrier>& barriers, Node& node, Tangents& tangents)
{
  // search all nodes except for start node
  for(auto neighbor = nodes.begin() + 1; neighbor != nodes.end(); ++neighbor)
  {
    Vector2f v = neighbor->center - node.center;
    const float d2 = v.squaredNorm();
    if(d2 > (node.radius - neighbor->radius) * (node.radius - neighbor->radius))
    {
      const float d = std::sqrt(d2);
      v /= d;

      // http://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Tangents_between_two_circles
      //
      // Let A, B be the centers, and C, D be points at which the tangent
      // touches first and second circle, and n be the normal vector to it.
      //
      // We have the system:
      //   n * n = 1          (n is a unit vector)
      //   C = A + r1 * n
      //   D = B +/- r2 * n
      //   n * CD = 0         (common orthogonality)
      //
      // n * CD = n * (AB +/- r2*n - r1*n) = AB*n - (r1 -/+ r2) = 0,  <=>
      // AB * n = (r1 -/+ r2), <=>
      // v * n = (r1 -/+ r2) / d,  where v = AB/|AB| = AB/d
      // This is a linear equation in unknown vector n.
      FOREACH_ENUM(PathPlannerUtils::Rotation, i)
      {
        const float sign1 = i ? -1.f : 1.f;
        const float c = (node.radius - sign1 * neighbor->radius) / d;

        if(c * c <= 1.f)
        {
          // If one of the circles is just a point, the second pair of tangents is skipped,
          // because they would be duplicates of the first pair.
          if(i && (node.radius == 0.f || neighbor->radius == 0.f))
          {
            // However, if the current node is a point (and the other one is not),
            // the other one still hides nodes further away. Therefore,
            // it needs a second (dummy) tangent for both rotations.
            if(node.radius == 0.f)
            {
              tangents[0].emplace_back(tangents[1].back());
              tangents[0].back().dummy = true;
              tangents[1].emplace_back(tangents[0][tangents[0].size() - 2]);
              tangents[1].back().dummy = true;
            }
          }
          else
          {
            // Now we're just intersecting a line with a circle: v*n=c, n*n=1
            const float h = std::sqrt(std::max(0.f, 1.f - c * c));
            FOREACH_ENUM(PathPlannerUtils::Rotation, j)
            {
              float sign2 = j ? -1.f : 1.f;
              const Vector2f n(v.x() * c - sign2 * h * v.y(), v.y() * c + sign2 * h * v.x());
              const Vector2f p1 = node.center + n * node.radius;
              const Vector2f p2 = neighbor->center + n * sign1 * neighbor->radius;
              float distance = (p2 - p1).norm();
              const float fromAngle = node.radius == 0.f ? (v * d + n * sign1 * neighbor->radius).angle() : n.angle();
              bool dummy = neighbor->fromEdge[i ^ j] != nullptr;
              if(dummy)
              {
                // Clone target node if it was already reached and clones are allowed.
                if(neighbor->allowedClones > 0)
                {
                  nodes.push_back(*neighbor);
                  --neighbor->allowedClones;
                }
              }
              else
                for(const auto& barrier : barriers)
                  if(barrier.intersects(p1, p2))
                  {
                    if(barrier.costs == std::numeric_limits<float>::infinity())
                    {
                      dummy = true;
                      break;
                    }
                    else
                      distance += barrier.costs;
                  }

              tangents[j].emplace_back(Edge(&node, &*neighbor, fromAngle, p2, static_cast<Rotation>(j), static_cast<Rotation>(i ^ j), distance),
                                       neighbor->radius == 0.f ? Tangent::none : i ^ j ? Tangent::right : Tangent::left, d - neighbor->radius, dummy);

              // If both nodes are points, there is only a single connection. Skip the rest.
              if(node.radius == 0.f && neighbor->radius == 0.f)
                goto exitBothLoops;
            }
          }
        }
        else if(i)
        {
          // The circles overlap and no second pair can be computed.
          // However, the other node still hides all other nodes within an angular range.
          // Add (dummy) tangents as end points for these ranges.
          for(auto& t : tangents)
          {
            const float d1 = 0.5f * (d + (sqr(node.radius) - sqr(neighbor->radius)) / d);
            const float a = std::acos(d1 / node.radius);
            const float dir = v.angle();
            t.emplace_back(t.back());
            BlockedSector blocked(Angle::normalize(dir - a), Angle::normalize(dir + a));
            if(t.back().side == Tangent::left)
            {
              node.blockedSectors.emplace_back(blocked);
              if(neighbor->radius < node.radius)
                ++node.allowedClones;
              t.back().side = Tangent::right;
              t.back().fromAngle = blocked.min;
              t.back().dummy = true;
            }
            else
            {
              t.back().side = Tangent::left;
              t.back().fromAngle = blocked.max;
              t.back().dummy = true;
            }
          }
        }

        // If this is the second tangent for a rotation, check for wraparound.
        // If right tangent is on the wrong side of left tangent, add a second
        // (dummy) right tangent 2pi earlier.
        // For each left tangent, set the index of the matching right tangent.
        if(neighbor->radius != 0.f && i)
        {
          for(auto& t : tangents)
          {
            ASSERT(t.size() >= 2);
            if(t.back().side == Tangent::left)
            {
              if(t.back().fromAngle < t[t.size() - 2].fromAngle)
              {
                t.emplace_back(t[t.size() - 2]);
                t.back().fromAngle -= pi2;
                t.back().dummy = true;
                t[t.size() - 2].matchingRightTangent = static_cast<int>(t.size() - 1);
              }
              else
                t.back().matchingRightTangent = static_cast<int>(t.size() - 2);
            }
            else
            {
              if(t.back().fromAngle > t[t.size() - 2].fromAngle)
              {
                t.emplace_back(t.back());
                t.back().fromAngle -= pi2;
                t.back().dummy = true;
                t[t.size() - 3].matchingRightTangent = static_cast<int>(t.size() - 1);
              }
              else
                t[t.size() - 2].matchingRightTangent = static_cast<int>(t.size() - 1);
            }
          }
        }
      }
    exitBothLoops:
      ;
    }
  }
}

void LibPathPlannerProvider::addNeighborsFromTangents(Node& node, Tangents& tangents)
{
  FOREACH_ENUM(PathPlannerUtils::Rotation, rotation)
  {
    auto& t = tangents[rotation];
    // Create index for tangents sorted by angle.
    // Since indices are used to reference between tangents,
    // the original vector of tangents must stay unchanged.
    std::vector<Tangent*> index;
    index.reserve(t.size());
    for(auto& tangent : t)
      index.push_back(&tangent);
    std::sort(index.begin(), index.end(),
              [](const Tangent* t1, const Tangent* t2) -> bool
              {
                return t1->fromAngle < t2->fromAngle;
              });

    // Sweep through all tangents, managing a set of current nodes sorted by their distance.
    std::vector<Tangent*> sweepline;
    sweepline.reserve(t.size());
    const auto byDistance = [](const Tangent* t1, const Tangent* t2) -> bool
    {
      return t1->circleDistance > t2->circleDistance;
    };
    for(auto& tangent : index)
    {
      // In general, if the node of the current tangent is not further away than the closest node
      // in the sweepline, it is a neighbor. However, since the obstacles are modeled as circles,
      // all obstacles must be checked until one is found that is actually further away or the target
      // point is actually inside another obstacle.
      if(!tangent->dummy)
      {
        for(const auto& s : sweepline)
          if(!s->ended)
          {
            if(s->circleDistance >= tangent->circleDistance)
              break;
            else if(s->circleDistance + 2.f * s->toNode->radius < tangent->circleDistance)
              goto doNotAddTangent;
            else
            {
              Vector2f fromPoint = Pose2f(tangent->fromAngle, node.center) * Vector2f(node.radius, 0.f);
              Geometry::Line line(fromPoint, tangent->toPoint - fromPoint);
              Vector2f p1;
              Vector2f p2;
              if(Geometry::getIntersectionOfLineAndCircle(line, *s->toNode, p1, p2) &&
                 line.direction.squaredNorm() >= (p1 - fromPoint).squaredNorm())
                goto doNotAddTangent;
            }
          }
        node.edges[rotation].emplace_back(*tangent);

      doNotAddTangent:
        ;
      }

      if(tangent->side == Tangent::right)
      {
        // If the current tangent is a right edge, add it to the sweepline
        sweepline.push_back(tangent);
        std::push_heap(sweepline.begin(), sweepline.end(), byDistance);
      }
      else if(tangent->side == Tangent::left)
      {
        // If the current tangent is a left edge, mark it as to be deleted
        // from the sweepline. Delete all closest entries from the sweepline
        // as long as they can be deleted.
        if(tangent->matchingRightTangent != -1)
          t[tangent->matchingRightTangent].ended = true;
        else
        {
          ANNOTATION("PathPlannerProvider", "ASSERT(tangent->matchingRightTangent != -1)");
/*#ifndef NDEBUG
          {
            OutBinaryFile stream("PathPlannerProvider.log");
            stream << theRobotPose << theTeamPlayersModel << lastDir;
          }
          FAIL("Send Config/PathPlannerProvider.log to Thomas.Roefer@dfki.de.");
#endif*/
        }
        while(!sweepline.empty() && sweepline.front()->ended)
        {
          std::pop_heap(sweepline.begin(), sweepline.end(), byDistance);
          sweepline.pop_back();
        }
      }
    }
  }
}

std::vector<Node> LibPathPlannerProvider::populatePlan(const Pose2f source, const Pose2f target, const Pose2f speed, bool excludePenaltyArea) 
{

    LibPathPlannerProvider::pathPlannerWasActive = true;

    //MotionRequest motionRequest;
    
//TODO: REMEMBER THAT WHEN USING THIS TO PLAN A PATH FOR THE BALL, THE wrongBallSideCostFactor SHOULD BE 0.f
    std::vector<Barrier> barriers;
    createBarriers(barriers, source, target, excludePenaltyArea);

    /*std::cout<<"2"<<std::endl;
    std::cout.flush();

    std::cout<<"Barriers size: "<<barriers.size()<<std::endl;
    for(auto barrier : barriers)
    {
      std::cout<<"Barrier: From: ("<<barrier.from.x()<<", "<<barrier.from.y()<<"), To: ("<<barrier.to.x()<<", "<<barrier.to.y()<<")"<<std::endl;
    }*/

    std::vector<Node> nodes; 
    createNodes(nodes, barriers, source, target, excludePenaltyArea); /**< All nodes of the visibility graph, i.e. all obstacles, and starting point (1st entry) and target (2nd entry). */
    /*std::cout<<"BEFORE PLANNING\nLength:"<<nodes.size();
    for(auto node : nodes)
    {
      std::cout<<"\nNode: Center: ("<<node.center.x()<<", "<<node.center.y()<<")"<<std::endl;
    }*/

    plan(nodes, barriers, source, nodes[0], nodes[1], speed.translation.x() / speed.rotation); 
    /*std::cout<<"AFTER PLANNING\nLength:"<<nodes.size();
    for(auto node : nodes)
    {
      std::cout<<"\nNode: Center: ("<<node.center.x()<<", "<<node.center.y()<<")"<<std::endl;
    }*/

    return nodes;
}

std::vector<Node> LibPathPlannerProvider::populatePlanWithCustomObstacleRadius(const Pose2f source, const Pose2f target, const Pose2f speed, bool excludePenaltyArea,
                                                                            float customGoalPostRadius, /**< Radius to walk around a goal post (in mm). */
                                                                            float customUprightRobotRadius, /**< Radius to walk around an upright robot (in mm). */
                                                                            float customFallenRobotRadius, /**< Radius to walk around a fallen robot (in mm). */
                                                                            float customReadyRobotRadius, /**< Radius to walk around a robot in ready state (in mm). */
                                                                            float customRadiusControlOffset /**< Plan closer to obstacles by this offset, but keep original distance when executing plan (in mm). */
                                                                            ) 
{

    LibPathPlannerProvider::pathPlannerWasActive = true;

    //MotionRequest motionRequest;
    
//TODO: REMEMBER THAT WHEN USING THIS TO PLAN A PATH FOR THE BALL, THE wrongBallSideCostFactor SHOULD BE 0.f
    std::vector<Barrier> barriers;
    createBarriers(barriers, source, target, excludePenaltyArea);

    /*std::cout<<"2"<<std::endl;
    std::cout.flush();

    std::cout<<"Barriers size: "<<barriers.size()<<std::endl;
    for(auto barrier : barriers)
    {
      std::cout<<"Barrier: From: ("<<barrier.from.x()<<", "<<barrier.from.y()<<"), To: ("<<barrier.to.x()<<", "<<barrier.to.y()<<")"<<std::endl;
    }*/

    std::vector<Node> nodes; 
    createNodes(nodes, barriers, source, target, excludePenaltyArea, customGoalPostRadius, customUprightRobotRadius, customFallenRobotRadius, customReadyRobotRadius, customRadiusControlOffset); /**< All nodes of the visibility graph, i.e. all obstacles, and starting point (1st entry) and target (2nd entry). */
    /*std::cout<<"BEFORE PLANNING\nLength:"<<nodes.size();
    for(auto node : nodes)
    {
      std::cout<<"\nNode: Center: ("<<node.center.x()<<", "<<node.center.y()<<")"<<std::endl;
    }*/

    plan(nodes, barriers, source, nodes[0], nodes[1], speed.translation.x() / speed.rotation); 
    /*std::cout<<"AFTER PLANNING\nLength:"<<nodes.size();
    for(auto node : nodes)
    {
      std::cout<<"\nNode: Center: ("<<node.center.x()<<", "<<node.center.y()<<")"<<std::endl;
    }*/

    return nodes;
}
/** Provides a path from a source to a goal on the field, using the RRT-A* native path planner
 * @param source the Vector2f origin point of the plan
 * @param goal the Vector2f destination point of the plan
 * @return a std::vector containing all the nodes of the plan
 */
std::vector<Edge*> LibPathPlannerProvider::createAvoidancePlan(std::vector<Node>& nodes) 
{

    std::vector<Edge*> avoidancePlan;
    //motionRequest.motion = MotionRequest::stand; // fall back if no path is found
    FOREACH_ENUM(PathPlannerUtils::Rotation, rotation)
    {
      //NOTICE: each node will have only ONE ingoing edge (we don't know the direction though), therefore we use this loop to "collect" the path backwards
      if(nodes[1].fromEdge[rotation])
      {
        Edge* edge = nodes[1].fromEdge[rotation];
        avoidancePlan.push_back(edge);
        Edge* nextEdge = nullptr;
        while(edge->fromNode != &nodes[0])
        {
          nextEdge = edge;
          edge = edge->fromNode->fromEdge[edge->fromRotation];
          avoidancePlan.push_back(edge);
        }
        lastDir = edge->toRotation;
        //calcMotionRequest(target, speed, edge, nextEdge, motionRequest);
        break;
      }
    }

    std::reverse(avoidancePlan.begin(), avoidancePlan.end());

    /*for(auto edge : finalPlan)
    {
        std::cout<<"Edge: from ("<<edge->fromNode->center.x()<<", "<<edge->fromNode->center.y()<<") to ("<<edge->toNode->center.x()<<", "<<edge->toNode->center.y()<<")"<<std::endl;
    }*/

    //if(motionRequest.motion == MotionRequest::stand)
    //{
      // Walk straight to target
      //PathPlannerUtils::Edge edge(&nodes[0], &nodes[1], 0.f, nodes[1].center, cw, cw, (nodes[0].center - nodes[1].center).norm());
      //calcMotionRequest(target, speed, &edge, nullptr, motionRequest);

    //  if(theFrameInfo.getTimeSince(timeWhenLastPlayedSound) > 5000)
    //  {
    //    ANNOTATION("PathPlannerProvider", "No path to target");
    //    SystemCall::playSound("theValidityDuck.wav");
    //    timeWhenLastPlayedSound = theFrameInfo.time;
    //  }
    //}
    //draw();

    //return motionRequest;
    return avoidancePlan;
}

void LibPathPlannerProvider::createArc(std::vector<Vector2f>& path, Vector2f center, float radius, float angleStep, float fromAngle, float angleSize, float reversed)
{
    //const Angle angleStep = pi2 / 32.f;
    Vector2f from;
    Vector2f to;

    //Beginning point around node
    to = Vector2f(center.x() + std::cos(fromAngle + angleSize) * radius, center.y() + std::sin(fromAngle + angleSize) * radius); 
    from = Vector2f(center.x() + std::cos(fromAngle) * radius, center.y() + std::sin(fromAngle) * radius);
    
    if(reversed)
    {
      //path.push_back(to);
      //std::cout<<"Pushed: ("<<to.x()<<", "<<to.y()<<")"<<std::endl;
      //Beginning point around node
      for(Angle angle = angleSize - angleStep; angle >= angleStep; angle -= angleStep) 
      { 
        to = Vector2f(center.x() + std::cos(angle + fromAngle) * radius, center.y() + std::sin(angle + fromAngle) * radius);
        path.push_back(to); 
      //std::cout<<"Pushed: ("<<to.x()<<", "<<to.y()<<")"<<std::endl;
        
        //LINE3D(id, from.x(), from.y(), (zCenter), to.x(), to.y(), (zCenter), thickness, color); 
        //from = to; 
      } 
      path.push_back(from);
      //std::cout<<"Pushed: ("<<from.x()<<", "<<from.y()<<")"<<std::endl;
    }
    else
    {
      //path.push_back(from);
      //std::cout<<"Pushed: ("<<from.x()<<", "<<from.y()<<")"<<std::endl;
      //Beginning point around node
      for(Angle angle = angleStep; angle <= angleSize - angleStep; angle += angleStep) 
      { 
        to = Vector2f(center.x() + std::cos(angle + fromAngle) * radius, center.y() + std::sin(angle + fromAngle) * radius);
        path.push_back(to); 
        //std::cout<<"Pushed: ("<<to.x()<<", "<<to.y()<<")"<<std::endl;
        
        //LINE3D(id, from.x(), from.y(), (zCenter), to.x(), to.y(), (zCenter), thickness, color); 
        //from = to; 
      } 
      path.push_back(to);
      //std::cout<<"Pushed: ("<<to.x()<<", "<<to.y()<<")"<<std::endl;
    }
        
    //LINE3D(id, from.x(), from.y(), (zCenter), to.x(), to.y(), (zCenter), thickness, color); 
}

std::vector<Vector2f> LibPathPlannerProvider::computePath(std::vector<Node>& nodes, float angleStep)
{
  std::vector<Vector2f> path;
  FOREACH_ENUM(PathPlannerUtils::Rotation, rotation)
    if(nodes[1].fromEdge[rotation])
    {
      path.push_back(nodes[1].center);
      for(const Edge* edge = nodes[1].fromEdge[rotation]; edge; edge = edge->fromNode->fromEdge[edge->fromRotation])
      {
        Vector2f p1 = Pose2f(edge->fromAngle, edge->fromNode->center) * Vector2f(edge->fromNode->radius, 0.f);
        //const Vector2f& p2 = edge->toPoint;
        
        path.push_back(p1);
        //std::cout<<"Pushed: ("<<p1.x()<<", "<<p1.y()<<")"<<std::endl;
        //LINE3D("module:PathPlannerProvider:path", p1.x(), p1.y(), 3.f, p2.x(), p2.y(), 3.f, 10, ColorRGBA::green);

        if(edge->fromNode->fromEdge[edge->fromRotation])
        {
          const Node& node = *edge->fromNode;
          // This is not the first node, i.e. we arrived here at fromEdge->toPoint.
          const float toAngle = (node.fromEdge[edge->fromRotation]->toPoint - node.center).angle();

          // The sector used on the circle is from toAngle of the incoming edge
          // to fromAngle of the outgoing edge.
          Rangef interval;
          bool reversed;
          if(edge->fromRotation == PathPlannerUtils::Rotation::cw)
          {
            interval.min = edge->fromAngle;
            interval.max = toAngle;
            reversed = false; 
          }
          else
          {
            interval.min = toAngle;
            interval.max = edge->fromAngle;
            reversed = true; 
          }

          // Compute the positive angle from incoming to outgoing edge in the fixed direction (cw/ccw).
          float angle = interval.max - interval.min;
          if(angle < 0.f)
            angle += pi2;

          //float angleStep = pi2 / 32.f;
          //std::cout<<"createArc"<<std::endl;
          createArc(path, node.center, node.radius, angleStep, interval.min, angle, reversed);
          //ARC3D("module:PathPlannerProvider:path", node.center.x(), node.center.y(), 3.f, node.radius, interval.min, angle, 10, ColorRGBA::green);
        }
      }
    }
  
  //ADD Ball position statically
  //path.push_back(theFieldBall.positionOnField);
  
  std::reverse(path.begin(), path.end());
  return path;
}

float LibPathPlannerProvider::getDefaultObstacleRadius(Obstacle::Type type)
{
    switch(type)
    {
      case Obstacle::goalpost: /**< Radius to walk around a goal post (in mm). */
        return defaultGoalPostRadius - defaultRadiusControlOffset;
      case Obstacle::fallenSomeRobot:
      case Obstacle::fallenOpponent:
      case Obstacle::fallenTeammate:
        if(theGameInfo.state != STATE_READY) /**< Radius to walk around a fallen robot (in mm). */
          return defaultFallenRobotRadius - defaultRadiusControlOffset;
      default:
        if(theGameInfo.state != STATE_READY) /**< Radius to walk around an upright robot (in mm). */
          return defaultUprightRobotRadius - defaultRadiusControlOffset;
        else
          return defaultReadyRobotRadius - defaultRadiusControlOffset; /**< Radius to walk around a robot in ready state (in mm). */
    }
}

float LibPathPlannerProvider::getObstacleRadius(Obstacle::Type type, float goalPostRadius, float uprightRobotRadius, float fallenRobotRadius, float readyRobotRadius, float radiusControlOffset)
{
    switch(type)
    {
      case Obstacle::goalpost: /**< Radius to walk around a goal post (in mm). */
        return goalPostRadius - radiusControlOffset;
      case Obstacle::fallenSomeRobot:
      case Obstacle::fallenOpponent:
      case Obstacle::fallenTeammate:
        if(theGameInfo.state != STATE_READY) /**< Radius to walk around a fallen robot (in mm). */
          return fallenRobotRadius - radiusControlOffset;
      default:
        if(theGameInfo.state != STATE_READY) /**< Radius to walk around an upright robot (in mm). */
          return uprightRobotRadius - radiusControlOffset;
        else
          return readyRobotRadius - radiusControlOffset; /**< Radius to walk around a robot in ready state (in mm). */
    }
}