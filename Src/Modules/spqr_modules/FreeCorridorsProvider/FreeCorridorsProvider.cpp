#include "FreeCorridorsProvider.h"
/*
 * custom function used to order the obstacles according to the y coordinate
 */
bool orderObstaclesByY(const Obstacle& o1, const Obstacle& o2) {
  return (o1.center.y() < o2.center.y());
}

/*
 * Find if the obstacle is lying on the semiplane above the rect passing through start and target,
 * below or on the rect itself
 * @return integer code for which side of the line ab c is on.
 * 1 means left turn, -1 means right turn.
 * Returns 0 if all three are on a line
 */
int findObstacleSide(Vector2f start, Vector2f target, Vector2f point) {
  if(std::abs(target(0)-start(0))<0.05) { // almost vertical line
    if(point(0) < target(0))
      return target(1) > start(1) ? 1 : -1;
    if (point(0) > target(0))
      return target(1) > start(1) ? -1 : 1;
    return 0;
  }

  if(std::abs(target(1)-start(1))<0.05) { // almost horizontal line
    if(point(1) < target(1))
      return target(0) > start(0) ? -1 : 1;
    if(point(1) > target(1))
      return target(0) > start(0) ? 1 : -1;
    return 0;
  }

  // all other cases
  float slope = (target(1) - start(1))/(target(0) - start(0));
  float yIntercept = start(1)-start(0)*slope;
  float cSolution = (slope*point(0)) + yIntercept;
  if (slope != 0) {
    if (point(1) > cSolution) {
      return target(0) > start(0) ? 1 : -1;
    }
    if (point(1) < cSolution) {
      return target(0) > start(0) ? -1 : 1;
    }
    return 0;
  }
  return 0;
}

void FreeCorridorsProvider::update(FreeCorridors& freeCorridors) {
  // reset the representation
  freeCorridors.maxThreshold = 0;
  freeCorridors.corridors.clear();

  // generate and fill an array of obstacles
  std::vector<Obstacle> orderedObstacles;
  // pre-filter the obstacles
  for(const auto& obstacle : theTeamPlayersModel.obstacles) {
    if(obstacle.type == Obstacle::opponent) {
      // skip obstacles that are behind
      if((obstacle.center.x()+theFieldDimensions.xPosOpponentGroundline) < (theRobotPose.translation.x()+theFieldDimensions.xPosOpponentGroundline)) {
        continue;
      }
      orderedObstacles.push_back(obstacle);
     }
    }

  // add two fake obstacles representing the projection of last and first on the sidelines
  Obstacle side1;
  Obstacle side2;
  if(orderedObstacles.empty()) {
    side1.center = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline);
    side2.center = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline);
    orderedObstacles.push_back(side1);
    orderedObstacles.push_back(side2);
  } else {
    side1.center = Vector2f(orderedObstacles.back().center.x(), theFieldDimensions.yPosRightSideline);
    side2.center = Vector2f(orderedObstacles.front().center.x(), theFieldDimensions.yPosLeftSideline);
    orderedObstacles.insert(orderedObstacles.begin(), side1);
    orderedObstacles.push_back(side2);
  }

  // sort according to y
  std::sort(orderedObstacles.begin(), orderedObstacles.end(), orderObstaclesByY);

  // generate segments between the obstacles
  for(unsigned index = 0; index < orderedObstacles.size()-1; index++){
    Vector2f midPoint((float)(orderedObstacles.at(index+1).center.x()+orderedObstacles.at(index).center.x())/2,
                      (float)(orderedObstacles.at(index+1).center.y()+orderedObstacles.at(index).center.y())/2);

    // fill the corridors
    Corridor corridor;
    corridor.start = theRobotPose.translation;
    corridor.target = midPoint;
    corridor.line = ParametrizedLine<float, 2>::Through(corridor.start, corridor.target);
    corridor.threshold = corridor.line.distance(orderedObstacles.at(index).center);

    // discard if too small, avoid this computations
    if(corridor.threshold > 50.f) {
      // is poiting toward the goal line?
      if(findObstacleSide(corridor.start,
                          Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline),
                          corridor.target) == -1 ||
         findObstacleSide(corridor.start,
                          Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline),
                          corridor.target) == 1) {
        corridor.toGoalLine = true;
        if(freeCorridors.maxThreshold < corridor.threshold) {
          freeCorridors.maxThreshold = corridor.threshold;
          freeCorridors.maxIndex++;
        }
      } else {
        corridor.toGoalLine = false;
        if(freeCorridors.maxThresholdSide < corridor.threshold) {
          freeCorridors.maxThresholdSide = corridor.threshold;
          freeCorridors.maxIndexSide++;
        }
      }

      // update the list
      freeCorridors.corridors.push_back(corridor);
    }
  }

  // print on world model
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:rectangle", "drawingOnField");
  for (const auto& corridor : freeCorridors.corridors) {
    Pose2f startRel = theLibCheck.glob2Rel(corridor.start.x(), corridor.start.y());
    Pose2f targetRel = theLibCheck.glob2Rel(corridor.target.x(), corridor.target.y());

    LINE("representation:ObstacleModel:rectangle",
         startRel.translation.x(),
         startRel.translation.y(),
         targetRel.translation.x(),
         targetRel.translation.y(),
         50,
         Drawings::solidPen,
         corridor.toGoalLine? ColorRGBA::red : ColorRGBA::black);
    DRAWTEXT("representation:ObstacleModel:rectangle",
             targetRel.translation.x()+5,
             targetRel.translation.y()+5,
             150, ColorRGBA::black, corridor.threshold
             );
  }
}

MAKE_MODULE(FreeCorridorsProvider, perception)
