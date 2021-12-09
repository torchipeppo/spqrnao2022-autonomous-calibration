/**
 * Implementation file for module FieldColorsCalibrator.
 * 
 * @author Francesco Petri
 */

//disabling warnings while importing so I don't see irrelevant messages when compiling
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#pragma GCC diagnostic ignored "-Wimplicit-int-float-conversion"
#pragma GCC diagnostic ignored "-Wreorder-ctor"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"

#include "FieldColorsCalibrator.h"
#include "Tools/Debugging/Debugging.h"
#include "Platform/SystemCall.h"
#include "Tools/Module/Module.h"
#include <iostream>
#include <stdio.h>
#include <time.h>

//see above
#pragma GCC diagnostic pop

MAKE_MODULE(FieldColorsCalibrator, infrastructure)

bool loaded = false;
bool breakCalibration = false;
bool calibrating = false;

std::vector<Genome> population;

void FieldColorsCalibrator::initCalibration() {
  OUTPUT_TEXT("Beginning color calibration");
  SystemCall::say("Beginning color calibration");
  std::cout << "Beginning color calibration" << std::endl;

  // initialize population at random
  population.clear();
  srand((unsigned) time(NULL));
  for (unsigned i=0; i<POPULATION_SIZE; i++) {
    population.emplace_back(Genome::random());
  }

  // if the Upper and Lower threads are ACTUAL threads, then I should have serious concurrency problems here,
  // and end up w/ a size that's neither POPULATION_SIZE nor 2*POPULATION_SIZE.
  std::cout << population.size() << " little genomes, first: " << population[0].color_delimiter << std::endl;
  // And in fact we DO get concurrency issues! But only if also calibrationStep does something!

  // notify the update loop that calibration has begun
  calibrating = true;
}

void FieldColorsCalibrator::calibrationStep() {
  // testing if the population array is shared b/w threads or not
  std::cout << "And the first is: " << population[0].color_delimiter << std::endl;
}

/**
 * Update cycle, called at each timestep by the rest of the framework.
 */
void FieldColorsCalibrator::update(FieldColors& fc) {
  // Load the (previous) calibration stored in the configuration file, for back-compatibility in case we don't want to calibrate at all
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

  // a test to make sure of how I can edit the caibration parameters.
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

  /**
   * Defines a "Debug Response" for the specified command:
   * if a "Debug Request" containing that command is issued by the simulator,
   * the following event will be fired.
   * (Note that the simulator can be used not only to run a simulation,
   *  but also to connect to a real robot in order to monitor its status
   *  and send commands such as this.)
   */
  DEBUG_RESPONSE_ONCE("module:FieldColorsCalibrator:startCalibration")
  {
    initCalibration();
  }

  // Calibrate
  if (calibrating) {
    calibrationStep();
  }
}