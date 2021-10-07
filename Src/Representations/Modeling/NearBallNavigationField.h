/**
 * @file Representations/Modeling/NearBallNavigationField.h
 *
 * This file implements a structure holding info about an artificial potential field used by the striker to navigate
 * around the ball
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "Representations/Modeling/NodePF.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"

/**
 * @struct NearBallNavigationField
 *  
 * Struct containing modeling info about computed artificial potential field used by the striker to navigate around
 * the ball
 */

STREAMABLE(NearBallNavigationField,
{
  /** Draws model on the field */
  void draw() const,

  (bool) graphical_debug,
  (bool) show_tiles,
  (float) graphical_norm_factor,
  (float) graphical_potential_upper_bound,
  (float) graphical_potential_lower_bound,

  (float) graphical_min_mesh_height,
  (float) graphical_max_mesh_height,

  (float) graphical_arrow_length_as_norm,
  (float) graphical_arrow_length,

  (bool) angle_based_color,
  
  (float) attractive_field_delay,
  (float) repulsive_field_delay,
  (float) potential_field_delay,
  
  (float) RO,
  (float) Kap,
  (float) Kbp,
  (float) Kr,
  (float) TEAMMATE_CO,
  (float) ETA,
  (float) GAMMA,
  (float) POW,

  (std::vector<NodePF>) attractive_field,
  (std::vector<NodePF>) repulsive_field,
  (std::vector<NodePF>) potential_field,

  (float) field_border_offset,

  (float) max_cell_size,
  (float) min_cell_size,
  (float) current_cell_size,
    
  (float) min_field_radius,                             /** Max. distance radius of field cells from field center */                     
  (float) max_field_radius,                             /** Max. distance radius of field cells from field center */
  (float) current_field_radius,

  (bool) use_dynamic_approach_point,                    /** Use the ball carrier dynamic approach point as an attractor, otherwise use the static one */

  (Vector2f)(Vector2f::Zero()) current_field_center,

  (Vector2f)(Vector2f::Zero()) current_field_source,
});
