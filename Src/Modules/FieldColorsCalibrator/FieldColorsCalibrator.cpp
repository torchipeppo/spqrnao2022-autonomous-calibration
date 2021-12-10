/**
 * Implementation file for module FieldColorsCalibrator.
 * 
 * This is only executed on the Lower thread, so there are no concurrency issues.
 * 
 * @author Francesco Petri
 */

//NOTE: If we ever need it, the CameraInfo representation knows which camera this instance of the module is working on.

//disabling warnings while importing so I don't see irrelevant messages when compiling
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#pragma GCC diagnostic ignored "-Wimplicit-int-float-conversion"
#pragma GCC diagnostic ignored "-Wreorder-ctor"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"

#include "FieldColorsCalibrator.h"
#include "FieldColorsGenome.h"
#include "Tools/Debugging/Debugging.h"
#include "Platform/SystemCall.h"
#include "Tools/Module/Module.h"
#include <iostream>
#include <stdio.h>
#include <time.h>

//see above
#pragma GCC diagnostic pop

// shorthands
typedef std::vector<Genome> Crowd;
typedef std::pair<Genome, Genome> Couple;

MAKE_MODULE(FieldColorsCalibrator, infrastructure)

bool loaded = false;
bool breakCalibration = false;
bool calibrating = false;

Crowd population;
unsigned generation = 0;

void FieldColorsCalibrator::initCalibration() {
  OUTPUT_TEXT("Beginning color calibration");
  SystemCall::say("Beginning color calibration");
  std::cout << "Beginning color calibration" << std::endl;

  // initialize RNG
  srand((unsigned) time(NULL));

  // initialize population at random
  population.clear();
  for (unsigned i=0; i<POPULATION_SIZE; i++) {
    population.push_back(Genome::random());
  }

  // start counting
  generation = 0;

  // notify the update loop that calibration has begun
  calibrating = true;
}

/**
 * Select ONE PAIR of parents, based on fitness.
 */
Couple select(Crowd population) {
  //TODOOOOO
  return Couple(population[0], population[1]);
}

/**
 * Perform crossover... (details depend on implementation)
 */
Couple crossover(Couple parents) {
  //TODOOOOO
  return parents;
}

/**
 * Mutate both children... (details depend on implementation)
 */
Couple mutate(Couple children) {
  //TODOOOOO
  return children;
}

/**
 * Pick the best POPULATION_SIZE individuals
 * of the parent and children populations together
 * to survive to the next generation.
 */
Crowd pick_elite(Crowd oldpop, Crowd newpop) {
  // TODOOOOOO
  return newpop;
}

/**
 * Perform one iteration of the genetic algorithm.
 */
void FieldColorsCalibrator::calibrationStep() {
  // advance to next generation
  generation++;
  if (generation > MAX_GENERATIONS) {
    SystemCall::say("Generation limit exceeded. Color calibration terminated.");
    OUTPUT_TEXT("Generation limit exceeded. Color calibration terminated.");
    calibrating = false;
    return;
  }

  Crowd child_population;
  for (unsigned i=0; i<POPULATION_SIZE/2; i++) {
    Couple parents = select(population);
    Couple children = crossover(parents);
    Couple mutated = mutate(children);
    child_population.push_back(mutated.first);
    child_population.push_back(mutated.second);
  }

  population = pick_elite(population, child_population);

  // TODO report of some kind
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