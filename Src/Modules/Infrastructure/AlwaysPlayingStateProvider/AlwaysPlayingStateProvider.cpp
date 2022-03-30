/**
 * @file AlwaysPlayingStateProvider.cpp
 * Implementation for AlwaysPlayingStateProvider.h
 * @author Francesco Petri
 */

#include "AlwaysPlayingStateProvider.h"

MAKE_MODULE(AlwaysPlayingStateProvider, infrastructure)

void AlwaysPlayingStateProvider::update(GameInfo& gameInfo)
{
    gameInfo.state = STATE_PLAYING;
}