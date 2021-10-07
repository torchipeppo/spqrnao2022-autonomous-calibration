#pragma once

#include "Tools/Module/Module.h"

#include "Tools/Math/Eigen.h"
#include "Tools/Math/Geometry.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/spqr_representations/FreeCorridors.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Tools/Debugging/DebugDrawings.h"

#include <iostream>
#include <algorithm>
#include <vector>

using namespace Eigen;

MODULE(FreeCorridorsProvider,
       {,
         REQUIRES(FieldDimensions),
         REQUIRES(RobotPose),
         REQUIRES(TeamPlayersModel),
         USES(LibCheck),
         PROVIDES(FreeCorridors),
        });

class FreeCorridorsProvider : public FreeCorridorsProviderBase {
 public:
  void update(FreeCorridors& freeCorridors);
};
