#include "PossiblePlan.h"

PossiblePlanCompressed::PossiblePlanCompressed(const PossiblePlan& myPossiblePlan)
: x(myPossiblePlan.x){}

PossiblePlanCompressed::operator PossiblePlan() const
{
  PossiblePlan pp;
  pp.x = 0.0;
  return pp;
}
