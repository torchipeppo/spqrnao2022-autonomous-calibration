/**
 * @file SetTarget.cpp
 *
 * This file contains the implementation of the SetTarget skill, that just updates the BehaviorStatus shootingTo target to the chosenTarget.
 *
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"

SKILL_IMPLEMENTATION(SetTargetImpl,
{,
  IMPLEMENTS(SetTarget),
  REQUIRES(LibCheck),
  MODIFIES(BehaviorStatus),
});

class SetTargetImpl : public SetTargetImplBase
{
  void execute(const SetTarget& p) override
  {
    theBehaviorStatus.shootingTo = p.chosenTarget;
  }
};

MAKE_SKILL_IMPLEMENTATION(SetTargetImpl);
