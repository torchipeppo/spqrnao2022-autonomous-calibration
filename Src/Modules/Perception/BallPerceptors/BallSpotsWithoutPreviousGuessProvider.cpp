/**
 * @file BallSpotsWithoutPreviousGuessProvider.cpp
 * 
 * A variant of BallSpotsProvider that does NOT include the previous BallModel
 * as a possible spot, so that ball perception depends entirely
 * on the segmented image.
 * Used in the autonomous color calibration.
 * 
 * @author Francesco Petri
 */

#pragma once

#include "Modules/Perception/BallPerceptors/BallSpotsWithoutPreviousGuessProvider.h"

#define BallSpotsProvider BallSpotsWithoutPreviousGuessProvider
#define BallSpotsProviderBase BallSpotsWithoutPreviousGuessProviderBase

// makes it so that the #imported code from BallSpotsProvider.cpp is slightly edited
// in order not to include the previous ball model into the candidate ball spots.
// see BallSpotsProvider.cpp for its use.
#define HACK_FOR_CALIBRATION__NO_PREVIOUS_GUESS

/**
 * Yes, this is a very questionable hack to make this module
 * an exact copy of the other one except for one little detail,
 * but in such a way that code from BallSpotsProvider is reused,
 * rather than making a separate copy that needs to be updated separately.
 */

#include "Modules/Perception/BallPerceptors/BallSpotsProvider.cpp"

#undef HACK_FOR_CALIBRATION__NO_PREVIOUS_GUESS
#undef BallSpotsProviderBase
#undef BallSpotsProvider

/**
 * Things I [Francesco] tried before resorting to the above hack:
 * - Manually copying the BallSpotsProvider code here.
 *   Hypothesis discarded a priori since any future modifications to the
 *   original BallSpotsProvider would desync the behavior of this and that module
 *   if we forget to port that edit here too.
 * - Making a module class that's child of BallSpotsProvider.
 *   The bhuman framework doesn't seem to allow that.
 * - Addding a cfg parameter to BallSpotsProvider.
 *   Would work if the cfg file was in the scenario (or even a location),
 *   in this way it could be (de)activated in settings.cfg, which is already
 *   used to set the ColorCalibration scenario.
 *   Unfortunately, the ballSpotsProvider.cfg file is also in a robot,
 *   which takes priority over locations and scenarios.
 *   Therefore this solution has the same problems as a direct edit of the
 *   BallSpotsProvider code: we'd have to remember to manually activate
 *   it in a remote cfg file before calibrating, and then remember
 *   to deactivate it the same way after calibrating.
 * 
 * By contrast, this hack can be turned on and off simply by changing the
 * scenario in settings.cfg, which in turn selects a threads.cfg file
 * that decides which module (this or the original) provides BallSpots.
 * 
 * Unfortunately, the cfg file has to be manually copied,
 * with the desync problem discussed in the bullet list above.
 * (It might be possible to avoid this by making this module's cfg file
 *  nothing more than a link to the original module's cfg file,
 *  assuming such link still works on the robot after the copyfiles.)
 * (Actually, TODO try this link thing on a real robot.)
 * 
 * If someone with more experience with the framework (or C++ in general)
 * comes up with a cleaner solution that doesn't require too much bookkeeping
 * to turn on and off, I'd be glad to hear that.
 */
