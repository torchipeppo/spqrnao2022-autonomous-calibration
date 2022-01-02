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
#include "Tools/Debugging/Debugging.h"
#include "Platform/SystemCall.h"
#include "Tools/Module/Module.h"
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <random>

//see above
#pragma GCC diagnostic pop

// for debugging
#define FIXED_SEED true
#define PRINT_FITNESSES(theCrowd) for (Genome& g : theCrowd) {std::cout << g.fitness << " ";} std::cout << std::endl
                                      // the & is important!

MAKE_MODULE(FieldColorsCalibrator, infrastructure)

// select the specific methods to use here
// (select and combine to come if/when they have more than one alternative)
#define CROSSOVER_FN crossover_blend    // TODO try crossover_sbx in real
#define MUTATION_FN mutate_gaussian   // TODO try mutate_nonuniform in real

bool loaded = false;
bool breakCalibration = false;
bool calibrating = false;

FieldColorsCalibrator::CalibrationState state = FieldColorsCalibrator::CalibrationState::Off;
int fitnessIndex = -1;

Crowd population;
Crowd child_population;
unsigned generation = 0;

std::default_random_engine generator;
std::normal_distribution<float> gaussian;

void FieldColorsCalibrator::initCalibration() {
  OUTPUT_TEXT("Beginning color calibration");
  SystemCall::say("Beginning color calibration");
  std::cout << "Beginning color calibration" << std::endl;

  // initialize RNG
  #if FIXED_SEED
  unsigned seed = (unsigned) 14383421;
  SystemCall::say("with fixed seed");   // fixed seed reminder
  OUTPUT_TEXT("with fixed seed");
  #else
  unsigned seed = (unsigned) time(NULL);
  #endif
  srand(seed);
  generator = std::default_random_engine(seed);
  gaussian = std::normal_distribution<float>(0, MUTATION_SIGMA);


  // initialize population at random
  population.clear();
  for (unsigned i=0; i<POPULATION_SIZE; i++) {
    population.push_back(Genome::random());
  }

  // start counting
  generation = 0;

  // notify the update loop that calibration has begun
  calibrating = true;
  state = CalibrationState::InitFitness;
  fitnessIndex = -1;
}

// evaluate the fitness of one genome per update cycle
void FieldColorsCalibrator::calibrationFitnessStep(FieldColors &fc, Crowd &popul, const CalibrationState &nextState) {
  // evaluate current genome
  if (fitnessIndex >= 0) {
    // TODO might be a good idea to NOT recompute fitness for indivisuals from previous generations, since time is precious now
    //      (may not be so necessary, I have already eliminated the double fitness computation)
    Genome *g = &(popul[fitnessIndex]);
    g->fitness = g->evalFitness(theECImage.colored, theBallPercept);
  }

  // set the color thresholds of the next genome, so in the next update cycle the image will be segmented accordingly
  fitnessIndex++;
  if (fitnessIndex < (int) popul.size()) {
    Genome g = popul[fitnessIndex];
    fc.maxNonColorSaturation = g.color_delimiter;
    fc.blackWhiteDelimiter = g.black_white_delimiter;
    fc.fieldHue.min = g.field_min;
    fc.fieldHue.max = g.field_max;
  }
  else {
    // fitness evaluation terminated
    state = nextState;
    // enforce that the population of each generation is ordered best-to-worst
    sort(popul.begin(), popul.end(), fitness_gt());
  }
}

// auxiliaries to variate certain parameters like crossover probability according to a polynomial schedule.
// lo<=hi, t in [0,1], pow is float to also allow a square-root law w/ pow=0.5
// needless to say, pow=1 is a linear law.
float polynomial_interpolation(float lo, float hi, float t, float pow, bool ascending) {
  if (ascending) {
    return lo + (hi-lo) * powf(t, pow);
  }
  else {
    return hi - (hi-lo) * powf(t, pow);
  }
}
int polynomial_interpolation_rounded(float lo, float hi, float t, float pow, bool ascending) {
  return (int) std::round(polynomial_interpolation(lo, hi, t, pow, ascending));
}

/**
 * Select ONE PAIR of parents, based on fitness.
 */
Couple FieldColorsCalibrator::select() {
  unsigned tournament_size = TOURNAMENT_SIZE;
  if (!IS_TOURNAMENT_SIZE_FIXED) {
    tournament_size = (unsigned) polynomial_interpolation_rounded(
      (float) TOURNAMENT_SIZE_LO,
      (float) TOURNAMENT_SIZE_HI,
      ((float) generation) / ((float) MAX_GENERATIONS),
      TOURNAMENT_SIZE_POLYN_SCHEDULE_POWER,
      true
    );
  }
  return Couple(select_tournament(tournament_size), select_tournament(tournament_size));
}
/**
 * Select ONE genome by tournament.
 */
Genome FieldColorsCalibrator::select_tournament(unsigned tournament_size) {
  // initialize the best genome directly with the first random participant of this tournament
  Genome champion = population[rand() % POPULATION_SIZE];
  // let the other challengers approach
  for (unsigned i=0; i<tournament_size-1; i++) {
    Genome challenger = population[rand() % POPULATION_SIZE];
    if (challenger.fitness > champion.fitness) {
      champion = challenger;
    }
  }
  // may the best genome win
  return champion;
}

/**
 * Auxiliary, produces a random float in [0,1]
 */
float frand() {
  return ((float) rand()) / ((float) RAND_MAX);
}

/**
 * Perform crossover on the two parents to produce two children.
 * Each gene is crossed-over independently of the others.
 */
Couple FieldColorsCalibrator::crossover(Couple parents, float crossover_chance) {
  Genome child1(parents.first);
  Genome child2(parents.second);

  // first gene
  if (frand() < crossover_chance) {
    pair_uc crossed = CROSSOVER_FN(child1.color_delimiter, child2.color_delimiter);
    child1.color_delimiter = crossed.first;
    child2.color_delimiter = crossed.second;
  }

  // second gene
  if (frand() < crossover_chance) {
    pair_uc crossed = CROSSOVER_FN(child1.field_min, child2.field_min);
    child1.field_min = crossed.first;
    child2.field_min = crossed.second;
  }

  // third gene
  if (frand() < crossover_chance) {
    pair_uc crossed = CROSSOVER_FN(child1.field_max, child2.field_max);
    child1.field_max = crossed.first;
    child2.field_max = crossed.second;
  }

  // fourth gene
  if (frand() < crossover_chance) {
    pair_uc crossed = CROSSOVER_FN(child1.black_white_delimiter, child2.black_white_delimiter);
    child1.black_white_delimiter = crossed.first;
    child2.black_white_delimiter = crossed.second;
  }

  return Couple(child1, child2);
}
/**
 * Simple blend crossover for a single gene.
 */
pair_uc FieldColorsCalibrator::crossover_blend(unsigned char x1, unsigned char x2) {
  // work with a wider range to avoid over/under-flow while computing
  int lo = x1<x2 ? x1 : x2;
  int hi = x1>=x2 ? x1 : x2;
  int width = hi - lo;
  // guarantee a minimum width, since this crashes if width==0
  bool move_lo = true;
  while (width < BLX_MIN_WIDTH) {
    if (lo == Genome::MIN && hi < Genome::MAX) {
      hi++;
    }
    else if (hi == Genome::MAX && lo > Genome::MIN) {
      lo--;
    }
    else if (move_lo) {
      lo--;
      move_lo = !move_lo;
    }
    else {
      hi++;
      move_lo = !move_lo;
    }
    width = hi - lo;
  }
  // generate two child genes uniformly in the range [lo - alpha*width, hi + alpha*width]
  lo -= (int) (BLX_ALPHA*(float)width);
  hi += (int) (BLX_ALPHA*(float)width);
  width = hi - lo;
  int y1 = lo + (rand() % width);
  int y2 = lo + (rand() % width);
  // only at the end, clamp them to the unsigned char range
  y1 = clamp255(y1);
  y2 = clamp255(y2);
  return pair_uc((unsigned char) y1, (unsigned char) y2);
}
/**
 * Simulated binary crossover for a single gene.
 */
pair_uc FieldColorsCalibrator::crossover_sbx(unsigned char x1, unsigned char x2) {
  // work with a wider range to avoid over/under-flow while computing
  float xx1 = (float) x1;
  float xx2 = (float) x2;
  // sample beta
  float u = frand();
  float e = 1.0f / (SBX_INDEX + 1.0f);
  float beta;
  if (u <= 0.5) {
    beta = powf(2*u, e);
  }
  else {
    beta = powf(1.0f / (2*(1-u)), e);
  }
  // crossover
  float y1_twice = (1+beta)*xx1 + (1-beta)*xx2;
  float y2_twice = (1-beta)*xx1 + (1+beta)*xx2;
  int y1 = (int)(0.5*y1_twice);
  int y2 = (int)(0.5*y2_twice);
  // only at the end, clamp them to the unsigned char range
  y1 = clamp255(y1);
  y2 = clamp255(y2);
  return pair_uc((unsigned char) y1, (unsigned char) y2);
}

/**
 * Mutate both children indepenently of each other.
 */
Couple FieldColorsCalibrator::mutate(Couple children, float mutation_chance) {
  return Couple(
    mutate_one(children.first, mutation_chance),
    mutate_one(children.second, mutation_chance)
  );
}
/**
 * Mutate one child, each gene independently of the others.
 */
Genome FieldColorsCalibrator::mutate_one(Genome child, float mutation_chance) {
  Genome mutated(child);

  // first gene
  if (frand() < mutation_chance) {
    mutated.color_delimiter = MUTATION_FN(mutated.color_delimiter);
  }

  // second gene
  if (frand() < mutation_chance) {
    mutated.field_min = MUTATION_FN(mutated.field_min);
  }

  // third gene
  if (frand() < mutation_chance) {
    mutated.field_max = MUTATION_FN(mutated.field_max);
  }

  // fourth gene
  if (frand() < mutation_chance) {
    mutated.black_white_delimiter = MUTATION_FN(mutated.black_white_delimiter);
  }

  return mutated;
}
/**
 * Mutate one gene by applying some Gaussian noise.
 */
unsigned char FieldColorsCalibrator::mutate_gaussian(unsigned char x) {
  // work in a wider range to avoid under/over-flow
  int xx = (int) x;
  xx += (int) roundf(gaussian(generator));
  // then clamp back to unsigned char
  return (unsigned char) clamp255(xx);
}
/**
 * Auxiliary for non-uniform mutation.
 * Gives a random number in [0,y] such that the probability of getting a low number
 * increases as t/gmax gets closer to 1.
 */
int delta(int t, int y, int gmax, float b) {
  float r = frand();
  float gratio = ((float) t) / ((float) gmax);
  float e = powf(1 - gratio, b);
  float x = 1 - powf(r, e);
  float d = (float)y * x;
  return (int) d;
}
/**
 * Mutate one gene by non-uniform mutation.
 * It has the property of becoming more likely to produce smaller mutations
 * as generations pass.
 */
unsigned char FieldColorsCalibrator::mutate_nonuniform(unsigned char x) {
  // work in a wider range to avoid under/over-flow
  int xx = (int) x;
  // choose increment or decrement
  char tau = rand() % 2;
  if (!tau) {
    xx += delta(generation, Genome::MAX - xx, MAX_GENERATIONS, NUMUT_B);
  }
  else {
    xx -= delta(generation, xx - Genome::MIN, MAX_GENERATIONS, NUMUT_B);
  }
  // then clamp back to unsigned char
  return (unsigned char) clamp255(xx);
}

/**
 * Pick the best POPULATION_SIZE individuals
 * of the parent and children populations together
 * to survive to the next generation.
 */
Crowd combine_elite(Crowd oldpop, Crowd newpop) {
  // I have changed my mind and don't expect to use this anymore.
  SystemCall::say("Warning: combine elite not implemented. Warning: combine elite not implemented. Warning: combine elite not implemented.");
  std::cout << "combine_elite not implemented." << std::endl;
  return newpop;
}

/**
 * Create a new population that includes all children, then fills up the remaining
 * slots up to POPULATION_SIZE with the best individuals from the previous generation.
 * EXPECTS oldpop (I.E. population) TO BE ALREADY SORTED,
 * which is desirable anyway since it facilitates the end-of-generation report
 * and any other operations I might want to implement in the future.
 */
Crowd FieldColorsCalibrator::combine_allChildrenThenEliteParents(Crowd oldpop, Crowd newpop) {
  // include all children
  Crowd future(newpop);
  // fill up the future population
  unsigned next_oldpop_index = 0;
  while (future.size() < POPULATION_SIZE) {
    future.emplace_back(oldpop[next_oldpop_index]);
    next_oldpop_index++;
  }

  // future population is sorted right outside of this function

  return future;
}

void FieldColorsCalibrator::calibrationSpawningStep() {
  // advance to next generation
  generation++;
  if (generation > MAX_GENERATIONS) {
    SystemCall::say("Generation limit exceeded. Color calibration terminated.");
    OUTPUT_TEXT("Generation limit exceeded. Color calibration terminated.");
    state = CalibrationState::End;
    return;
  }

  float gratio = ((float) generation) / ((float) MAX_GENERATIONS);

  // Set crossover chance to either its fixed value or a variable value according to a polynomial schedule.
  // The selection and all parameters are set in the config file.
  float crossover_chance = CROSSOVER_CHANCE;
  if (!IS_CROSSOVER_CHANCE_FIXED) {
    crossover_chance = polynomial_interpolation(
      CROSSOVER_CHANCE_LO,
      CROSSOVER_CHANCE_HI,
      gratio,
      CROSSOVER_POLYN_SCHEDULE_POWER,
      false
    );
  }

  // Set mutation chance to either its fixed value or a variable value according to a polynomial schedule.
  // The selection and all parameters are set in the config file.
  float mutation_chance = MUTATION_CHANCE;
  if (!IS_MUTATION_CHANCE_FIXED) {
    mutation_chance = polynomial_interpolation(
      MUTATION_CHANCE_LO,
      MUTATION_CHANCE_HI,
      gratio,
      MUTATION_POLYN_SCHEDULE_POWER,
      false
    );
  }

  child_population.clear();
  for (unsigned i=0; i<CHILDPOP_SIZE/2; i++) {
    Couple parents = select();
    Couple children = crossover(parents, crossover_chance);
    Couple mutated = mutate(children, mutation_chance);
    child_population.push_back(mutated.first);
    child_population.push_back(mutated.second);
  }

  // initiate children's fitness evaluation
  state = CalibrationState::ChildrenFitness;
  fitnessIndex = -1;
}

void FieldColorsCalibrator::calibrationGenWrapupStep() {
  // combine parent and child populations in some way
  // in order to create the poputation that will go into the next generation
  population = combine_allChildrenThenEliteParents(population, child_population);

  // enforce that the population of each generation is ordered best-to-worst
  sort(population.begin(), population.end(), fitness_gt());

  // report some stats about this generation
  Genome best = population[0];
  long total_fitness = 0;
  for (Genome g : population) {
    total_fitness += g.fitness;
  }
  double average_fitness = ((double) total_fitness) / ((double) POPULATION_SIZE);

  // report to terminal console or log file
  std::cout << "Generation " << generation << std::endl;
  std::cout << "    Average fitness: " << average_fitness << std::endl;
  std::cout << "    Best fitness: " << best.fitness << std::endl;
  std::cout << "    Best genome: " << (int) best.color_delimiter << " ";
  std::cout << (int) best.field_min << " ";
  std::cout << (int) best.field_max << " ";
  std::cout << (int) best.black_white_delimiter << std::endl;
  std::cout << std::endl;

  // report to simulator console
  std::string line = "Generation " + std::to_string(generation);
  OUTPUT_TEXT(line);
  line = "Best: " + std::to_string(best.fitness);
  OUTPUT_TEXT(line);
  line = std::to_string(best.color_delimiter) + ", ";
  line += std::to_string(best.field_min) + ", ";
  line += std::to_string(best.field_max) + ", ";
  line += std::to_string(best.black_white_delimiter);
  OUTPUT_TEXT(line);
  OUTPUT_TEXT("-------------------");

  // Spoken report
  line = "Generation " + std::to_string(generation) + " completed";
  SystemCall::say(line.c_str());
  line = "Best fitness is " + std::to_string(best.fitness);
  SystemCall::say(line.c_str());


  // begin next spawning phase
  state = CalibrationState::Spawning;
}

void FieldColorsCalibrator::calibrationEnd(FieldColors& fc) {
  // make sure the population is sorted best-first, regardless of how we came to this state
  sort(population.begin(), population.end(), fitness_gt());

  Genome best = population[0];

  // set the color thresholds to the best ones found by the algorithm
  fc.maxNonColorSaturation = best.color_delimiter;
  fc.blackWhiteDelimiter = best.black_white_delimiter;
  fc.fieldHue.min = best.field_min;
  fc.fieldHue.max = best.field_max;

  // save these parameters in the robot's configuration files
  SystemCall::say("Remember that saving needs to be tested in real as well");   // this method MIGHT work even in real as-is, but it's best to check. TODO.

  // search the configuration paths (generic, robot-specific, location-specific, scenario-specific, etc...)
  // for the first existing config file for this representation...
  std::string name = "fieldColors.cfg";
  for(std::string& fullName : File::getFullNames(name))
  {
    File path(fullName, "r", false);
    if(path.exists())
    {
      name = std::move(fullName);
      break;
    }
  }
  // ...and write our newly-calibrated parameters there
  OutMapFile stream(name, false);
  ASSERT(stream.exists());
  stream << fc;

  // inform the simulator user
  OUTPUT_TEXT("Best genome set and saved on the robot:");
  std::string line = std::to_string(best.color_delimiter) + ", ";
  line += std::to_string(best.field_min) + ", ";
  line += std::to_string(best.field_max) + ", ";
  line += std::to_string(best.black_white_delimiter);
  OUTPUT_TEXT(line);
  OUTPUT_TEXT("(its fitness was " + std::to_string(best.fitness) + ")");
  OUTPUT_TEXT("If the robot is real, use this command to save on the computer:");
  OUTPUT_TEXT("save representation:FieldColors");

  // turn of calibration mode
  state = CalibrationState::Off;
  calibrating = false;
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
    SystemCall::say("Hello world");     // activates the robot's text-to-speech feature
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
    // calibrationStep();
    switch (state) {
      case CalibrationState::Init:
        // nothing, initCalibration is called in the debug response event at the moment
        break;
      case CalibrationState::InitFitness:
        DEBUG_COUT("InitFitness " << fitnessIndex);
        calibrationFitnessStep(fc, population, CalibrationState::Spawning);
        break;
      case CalibrationState::Spawning:
        DEBUG_COUT("Spawning");
        calibrationSpawningStep();
        break;
      case CalibrationState::ChildrenFitness:
        DEBUG_COUT("ChildrenFitness " << fitnessIndex);
        calibrationFitnessStep(fc, child_population, CalibrationState::GenWrapup);
        break;
      case CalibrationState::GenWrapup:
        DEBUG_COUT("GenWrapup");
        calibrationGenWrapupStep();
        break;
      case CalibrationState::End:
        DEBUG_COUT("End");
        calibrationEnd(fc);
        break;
    }
  }
}