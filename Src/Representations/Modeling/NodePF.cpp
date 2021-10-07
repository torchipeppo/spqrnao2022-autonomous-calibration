/**
 * @file Representations/Modeling/NodePF.cpp
 * 
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "NodePF.h"
#include "Platform/Time.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"


void NodePF::verify() const
{
  ASSERT(std::isfinite(potential.x()));
  ASSERT(std::isfinite(potential.y()));
}