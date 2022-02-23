/**
 * @file BallSpotsWithoutPreviousGuessProvider.h
 * 
 * A variant of BallSpotsProvider that does NOT include the previous BallModel
 * as a possible spot, so that ball perception depends entirely
 * on the segmented image.
 * Used in the autonomous color calibration.
 * 
 * @author Francesco Petri
 */

#pragma once

/**
 * Make an exact copy of BallSpotsProvider, except the name is changed.
 * A questionable hack, but necessary because it is impossible to make
 * a module that's child of another.
 */

#define BallSpotsProvider BallSpotsWithoutPreviousGuessProvider
#define BallSpotsProviderBase BallSpotsWithoutPreviousGuessProviderBase

#include "Modules/Perception/BallPerceptors/BallSpotsProvider.h"

#undef BallSpotsProviderBase
#undef BallSpotsProvider
