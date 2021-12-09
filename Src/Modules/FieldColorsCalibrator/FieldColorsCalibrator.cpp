/**
 * Implementation file for module FieldColorsCalibrator.
 * 
 * @author Francesco Petri
 */

#include "FieldColorsCalibrator.h"
#include "Tools/Debugging/Debugging.h"
#include "Platform/SystemCall.h"
#include "Tools/Module/Module.h"
#include <iostream>

MAKE_MODULE(FieldColorsCalibrator, infrastructure)

bool breakCalibration = false;
bool loaded = false;

void FieldColorsCalibrator::update(FieldColors& fc) {
  // start with the initial guess provided in the cfg (for now, at least)
  if (!loaded) {
    loadModuleParameters(fc, "FieldColors", nullptr);
    loaded=true;
  }

  // just a test of the debug request/response feature of the simulator
  DEBUG_RESPONSE_ONCE("module:FieldColorsCalibrator:hello")
  {
    OUTPUT_TEXT("Hello world");      // outputs to the simulator console
    SystemCall::say("Hello world");     // activates the robot's test-to-speech feature
    std::cout << "Hello world" << std::endl;    // prints to terminal if robot is simulated, or to log file if robot is real
  }

  // a test to make sure of how I can edit the caibration parameters
  // the robot should be (and is) unable to see the ball or anything else after this command.
  DEBUG_RESPONSE_ONCE("module:FieldColorsCalibrator:breakCalibration")
  {
    breakCalibration = true;
  }

  if (breakCalibration) {
    fc.blackWhiteDelimiter = 0;
    fc.maxNonColorSaturation = 0;
    fc.fieldHue.min = 255;
    fc.fieldHue.max = 255;
    OUTPUT_TEXT("NAO is affected by Blindness!");
    breakCalibration = false;
  }
}