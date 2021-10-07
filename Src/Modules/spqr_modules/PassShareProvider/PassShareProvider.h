#pragma once

#include <iostream>
#include <set>
#include "Tools/Module/Module.h"
#include "Tools/Math/Transformation.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/spqr_representations/OurDefinitions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/spqr_representations/PassShare.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
//#include "Representations/spqr_representations/SPQRInfoDWK.h"
#include "Representations/spqr_representations/PossiblePlan.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Tools/Math/Pose2f.h"


#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/spqr_representations/RoleAndContext.h" //Added by david 2021

#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include <mutex>

MODULE(PassShareProvider,
{,
 USES(BehaviorStatus),
 REQUIRES(BallModel),
 REQUIRES(RobotInfo),
 REQUIRES(LibCheck),
 REQUIRES(TeamData),
 REQUIRES(RobotPose),
 //REQUIRES(SPQRInfoDWK),
 //REQUIRES(PossiblePlan),
 REQUIRES(Role),
 REQUIRES(RoleAndContext),
 REQUIRES(FieldDimensions),
 PROVIDES(PassShare),

 LOADS_PARAMETERS(
    {,
      (bool) GRAPHICAL_DEBUG,                                   /** shows a graphical debug in SimRobot */
      (float) MAX_ALLOWED_OPPONENT_DISTANCE_FROM_PASS_TARGET,   /** if there are opponents within this distance from the pass target, set readyPass to 0 */
    }),
});

class PassShareProvider : public PassShareProviderBase
{
private:


public:
    void update(PassShare& ps);
    PassShareProvider();
};
