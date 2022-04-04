/**
 * @file AlwaysPlayingStateProvider.h
 * 
 * A dummy module that makes it so that the robot is ALWAYS IN PLAYING STATE.
 * And I mean ALWAYS. Be careful.
 * Originally created to automatically start autonomous calibration,
 * but it might turn out to be useful in other contexts too,
 * such as small tests w/o gamecontroller
 * (tests prove that chest button to penalize still works).
 * Note that the robot will still connect to the gamecontroller,
 * this module just overrides the RawGameData.
 * 
 * @author Francesco Petri
 */

#pragma once

#include "Representations/Communication/GameInfo.h"
#include "Tools/Module/Module.h"

MODULE(AlwaysPlayingStateProvider,
{,
  PROVIDES(GameInfo),
});

class AlwaysPlayingStateProvider : public AlwaysPlayingStateProviderBase
{
  void update(GameInfo& gameInfo) override;
};
