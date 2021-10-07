/**
 * @file Representations/Modeling/NodePF.h
 *
 *
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"

/**
 * @struct NodePF
 *
 */

STREAMABLE(NodePF,
{

  /**< Default constructor */
  NodePF() = default;

  /** Constructor with field initialization
   * @param begin 
   */
  
  NodePF(Vector2f position, Vector2f potential, float cell_size);

  /** Verifies that the model contains valid values. */
  void verify() const,

  (Vector2f)(Vector2f::Zero()) potential,       /** Node computed potential */
  (Vector2f)(Vector2f::Zero()) position,        /** Node position */
  (float) cell_size,                       /** PF cell size */

});

//Inline Constructor
inline NodePF::NodePF(Vector2f position, Vector2f potential, float cell_size):
position(position),
potential(potential) {}