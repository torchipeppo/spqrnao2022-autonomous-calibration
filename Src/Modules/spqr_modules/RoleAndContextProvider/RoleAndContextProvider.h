
#pragma once

#include <iostream>
#include <set>
#include "Tools/Module/Module.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/spqr_representations/OurDefinitions.h"
#include "Representations/spqr_representations/RoleAndContext.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Role.h"
//#include "Representations/spqr_representations/GCData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Communication/TeamData.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include <mutex>

MODULE(RoleAndContextProvider, 
{,
 REQUIRES(Role),
 REQUIRES(BallModel),
 REQUIRES(ObstacleModel),
 REQUIRES(LibCheck),
 REQUIRES(OpponentTeamInfo),
 REQUIRES(OwnTeamInfo),
 USES(TeamPlayersModel),
 USES(TeamBallModel),
 USES(TeamData),
 PROVIDES(RoleAndContext),
});

class RoleAndContextProvider : public RoleAndContextProviderBase
{
private:
    

public:
    std::tuple<float,int> minOpponent_BallDistance();
    std::tuple<float,int, int> minTeammate_BallDistance();
    std::tuple<int,int,int,int> score_andActivePlayers();
    
    void update(RoleAndContext& rac);
    RoleAndContextProvider();
};
