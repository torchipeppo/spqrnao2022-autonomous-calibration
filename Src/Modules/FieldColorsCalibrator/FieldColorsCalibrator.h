/**
 * Autonomous calibrator for the image segmentation.
 * 
 * @author Francesco Petri
 */

#pragma once

//disabling warnings while importing so I don't see irrelevant messages when compiling
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#pragma GCC diagnostic ignored "-Wimplicit-int-float-conversion"
#pragma GCC diagnostic ignored "-Wreorder-ctor"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"

#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
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
  PROVIDES(FieldColors),

  LOADS_PARAMETERS(
  {,
    (unsigned) MAX_GENERATIONS,   /** Maximum number of generations before forcefully stopping the search */
    (unsigned) POPULATION_SIZE,   /** Number of genomes in each generation */
    (unsigned) CHILDPOP_SIZE,     /** Number of children produced each generation. Must be even. */

    (unsigned) TOURNAMENT_SIZE,   /** Number of genomes participating in each selection tournament */

    (float) CROSSOVER_CHANCE,     /** Probability of crossover for each parent pair */
    (float) BLX_ALPHA,            /** Alpha parameter for blend crossover */

    (float) MUTATION_CHANCE,      /** Probability of mutation for each child */
    (float) MUTATION_SIGMA,       /** Standard deviation of the Gaussian mutation of each gene */
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
  };

  void update(FieldColors& fc) override;

  private:
  void initCalibration();
  // void calibrationStep();

  void calibrationFitnessStep(FieldColors &fc, Crowd &popul, const CalibrationState &nextState);
  void calibrationSpawningStep();
  void calibrationGenWrapupStep();

  Couple select();
  Genome select_tournament();
  Couple crossover(Couple parents);
  pair_uc crossover_blend(unsigned char x1, unsigned char x2);
  Couple mutate(Couple children);
  Genome mutate_one(Genome child);
  unsigned char mutate_gaussian(unsigned char x);
  Crowd combine_allChildrenThenEliteParents(Crowd oldpop, Crowd newpop);
};
