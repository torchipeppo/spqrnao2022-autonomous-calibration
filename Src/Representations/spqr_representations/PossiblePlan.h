#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Modeling/Obstacle.h"
#include "Representations/Modeling/ObstacleModel.h"


#include <vector>
#include "Tools/Streams/Enum.h"

STREAMABLE(PossiblePlan,
{
	public:

		ENUM(Action,    //PossiblePlanningActions
		  {,
		    Move,
		    Stand,
		    ForwardKick,
		    Turn,
		    none,
		  });


		float x,        	/**< The X position of the filtered robot pose */
		(Action) plan,
		(int) movements,
		(std::vector<Pose2f>) possibleTargets,
		(Pose2f) betterTarget,
		(float) receiveUtil,
        PossiblePlan() = default;
});

STREAMABLE(PossiblePlanCompressed,
{
public:
  PossiblePlanCompressed() = default;
  PossiblePlanCompressed(const PossiblePlan& myPossiblePlan);
  operator PossiblePlan() const,
	(float) x,
	
});
