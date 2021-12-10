/**
 * Autonomous calibrator for the image segmentation.
 * 
 * @author Francesco Petri
 */

#pragma once

#include "Representations/Configuration/FieldColors.h"
#include "Tools/Module/Module.h"

MODULE(FieldColorsCalibrator,
{,
  PROVIDES(FieldColors),

  LOADS_PARAMETERS(
  {,
    (unsigned) MAX_GENERATIONS,   /** Maximum number of generations befire forcefully stopping the search */
    (unsigned) POPULATION_SIZE,   /** Number of genomes in each generation */
    (float) CROSSOVER_CHANCE,     /** Probability of crossover for each parent pair */
    (float) MUTATION_CHANCE,      /** Probability of mutation for each child */
    (float) MUTATION_SIGMA,       /** Standard deviation of the Gaussian mutation of each gene */
  }),
});

class FieldColorsCalibrator : public FieldColorsCalibratorBase
{
  void update(FieldColors& fc) override;
  void initCalibration();
  void calibrationStep();
};
