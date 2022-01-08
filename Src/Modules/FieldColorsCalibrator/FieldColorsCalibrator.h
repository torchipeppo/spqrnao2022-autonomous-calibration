/**
 * Autonomous calibrator for the image segmentation.
 * 
 * @author Francesco Petri
 */

#pragma once

//disabling warnings while importing so I don't see irrelevant messages from all over the framework when compiling
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#pragma GCC diagnostic ignored "-Wimplicit-int-float-conversion"
#pragma GCC diagnostic ignored "-Wreorder-ctor"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#pragma GCC diagnostic ignored "-Wenum-enum-conversion"
#pragma GCC diagnostic ignored "-Wenum-float-conversion"

#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Configuration/FieldColors.h"
#include "Tools/Module/Module.h"

#include "FieldColorsGenome.h"

//see above
#pragma GCC diagnostic pop

// shorthands for common types in the .cpp
typedef std::vector<Genome> Crowd;
typedef std::pair<Genome, Genome> Couple;
typedef std::pair<unsigned char, unsigned char> pair_uc;

MODULE(FieldColorsCalibrator,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CameraImage),
  USES(ECImage),
  USES(BallPercept),
  PROVIDES(FieldColors),

  // these parameters are defined in a separate configuration file.
  LOADS_PARAMETERS(
  {,
    (unsigned) MAX_GENERATIONS,   /** Maximum number of generations before forcefully stopping the search */
    (unsigned) POPULATION_SIZE,   /** Number of genomes in each generation */
    (unsigned) CHILDPOP_SIZE,     /** Number of children produced each generation. Must be even. */


    (bool) IS_TOURNAMENT_SIZE_FIXED,                                   // memento: high tourn. size => high selective pressure
    (unsigned) TOURNAMENT_SIZE,   /** Number of genomes participating in each selection tournament. Only applies if IS_TOURNAMENT_SIZE_FIXED */
    (unsigned) TOURNAMENT_SIZE_LO,  /** Initial tournament if it's NOT fixed. DOES NOT APPLY otherwise. */
    (unsigned) TOURNAMENT_SIZE_HI,  /** Final tournament if it's NOT fixed. DOES NOT APPLY otherwise. */
    (float) TOURNAMENT_SIZE_POLYN_SCHEDULE_POWER,  /** Power of the polynomial law the size follows to increase from LO to HI. ONLY applies if NOT fixed. */


    (bool) IS_CROSSOVER_CHANCE_FIXED,
    (float) CROSSOVER_CHANCE,     /** Fixed probability of crossover for each parent pair. Only applies if IS_CROSSOVER_CHANCE_FIXED. */
    (float) CROSSOVER_CHANCE_HI,  /** Initial probability of crossover if crossover chance is NOT fixed. DOES NOT APPLY otherwise. */
    (float) CROSSOVER_CHANCE_LO,  /** Final probability of crossover if crossover chance is NOT fixed. DOES NOT APPLY otherwise. */
    (float) CROSSOVER_POLYN_SCHEDULE_POWER,   /** Power of the polynomial law the crossover chance follows to decrease from HI to LO. ONLY applies if NOT fixed. */

    (float) BLX_ALPHA,            /** Alpha parameter for blend crossover */
    (int) BLX_MIN_WIDTH,          /** Minimum interval width during blend crossover */
    (float) SBX_INDEX,            /** Parameter for simulated binary crossover. Non-negative, best in [2,5], larger => children closer to parents */


    (bool) IS_MUTATION_CHANCE_FIXED,
    (float) MUTATION_CHANCE,      /** Fixed probability of mutation for each child. Only applies if IS_MUTATION_CHANCE_FIXED. */
    (float) MUTATION_CHANCE_HI,   /** Initial probability of mutation if mutation chance is NOT fixed. DOES NOT APPLY otherwise. */
    (float) MUTATION_CHANCE_LO,   /** Final probability of mutation if mutation chance is NOT fixed. DOES NOT APPLY otherwise. */
    (float) MUTATION_POLYN_SCHEDULE_POWER,   /** Power of the polynomial law the mutation chance follows to decrease from HI to LO. ONLY applies if NOT fixed. */

    (float) MUTATION_SIGMA,       /** Standard deviation of the Gaussian mutation of each gene */
    (float) NUMUT_B,              /** A parameter for non-uniform mutation, simply called "b", which determines the dependency on the number of iterations */
  }),
});

class FieldColorsCalibrator : public FieldColorsCalibratorBase
{
  public:

  enum class CalibrationState {
    Off,
    Init,
    InitFitness,
    Spawning,
    ChildrenFitness,
    GenWrapup,
    End,
  };

  void update(FieldColors& fc) override;

  private:
  void initCalibration();
  // void calibrationStep();
  void calibrationEnd(FieldColors& fc);

  void calibrationFitnessStep(FieldColors &fc, Crowd &popul, const CalibrationState &nextState);
  void calibrationSpawningStep();
  void calibrationGenWrapupStep();

  Couple select();
  Genome select_tournament(unsigned tournament_size);

  Couple crossover(Couple parents, float crossover_chance);
  pair_uc crossover_blend(unsigned char x1, unsigned char x2);
  pair_uc crossover_sbx(unsigned char x1, unsigned char x2);

  Couple mutate(Couple children, float mutation_chance);
  Genome mutate_one(Genome child, float mutation_chance);
  unsigned char mutate_gaussian(unsigned char x);
  unsigned char mutate_nonuniform(unsigned char x);

  Crowd combine_allChildrenThenEliteParents(Crowd oldpop, Crowd newpop);
};
