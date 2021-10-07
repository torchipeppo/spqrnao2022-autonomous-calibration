/**
 * Declare the representation for a free corridor where a corridor is a portion of the field
 * that does not contain opponents and is identified by a line starting at current robot postion
 * and ending at a target.
 * The width of the corridor is identified by the variable threshold
 *
 * @author Dario Albani
 */
#pragma once

#include <vector>
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Streams/AutoStreamable.h"


struct Corridor{
  Eigen::ParametrizedLine<float, 2> line; // line passing between start and target
  Eigen::Vector2f start; // start of the line
  Eigen::Vector2f target; // end of the line
  bool toGoalLine; // is intersecting with the goal line?
  float threshold; // width of the corridor
};

STREAMABLE(FreeCorridors,
           {
             void draw() const;
             FreeCorridors() = default;
             std::vector<Corridor> corridors,
             (float)(0.f) maxThreshold, // the width of the biggest corridor
             (float)(0.f) maxThresholdSide, // the width of the biggest corridor to the side
             (int)(0) maxIndex, // the index of the corridor with the biggest width
             (int)(0) maxIndexSide, // the index of the corridor with the biggest width to the side
           });
