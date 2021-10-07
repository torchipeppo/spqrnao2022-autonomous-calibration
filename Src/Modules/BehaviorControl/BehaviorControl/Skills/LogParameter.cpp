/**
 * @file LogParameter.cpp
 *
 * This "dummy" skill exploits the .behavior View in SimRobot (accessible in the "Scene Graph" panel by opening "robotN" (N is the robot number)
 * and double-clicking behavior). Until a better solution is found, this skill has to be called wherever we want to show the parameter value in SimRobot,
 * for each parameter we want to show.
 *
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include <iostream>

SKILL_IMPLEMENTATION(LogStringParameterImpl,
{,
  IMPLEMENTS(LogStringParameter),
});

class LogStringParameterImpl : public LogStringParameterImplBase
{
  void execute(const LogStringParameter& p) override
  {
      //std::cout<<"You should see: \nParam="<<p.Param<<"\nValue="<<p.Value<<std::endl;
  }
};

MAKE_SKILL_IMPLEMENTATION(LogStringParameterImpl);

SKILL_IMPLEMENTATION(LogFloatParameterImpl,
{,
  IMPLEMENTS(LogFloatParameter),
});

class LogFloatParameterImpl : public LogFloatParameterImplBase
{
  void execute(const LogFloatParameter& p) override
  {
      //std::cout<<"You should see: \nParam="<<p.Param<<"\nValue="<<p.Value<<std::endl;
  }
};

MAKE_SKILL_IMPLEMENTATION(LogFloatParameterImpl);
