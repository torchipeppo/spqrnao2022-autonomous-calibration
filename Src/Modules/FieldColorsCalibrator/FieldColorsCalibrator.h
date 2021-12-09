/**
 * Autonomous calibrator for the image segmentation.
 * 
 * @author Francesco Petri
 */

#pragma once

#include "Representations/Configuration/FieldColors.h"
#include "Tools/Module/Module.h"
#include <stdio.h>

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

class Genome
{
  public: // this class acts as little more than a struct with some constructors, so it's not going to bother with compartimentalization and other OOP principles

  // arbitrary constructor
  Genome(unsigned short cdel, unsigned short fmin, unsigned short fmax, unsigned short bwdl) {
    color_delimiter = cdel;
    field_min = fmin;
    field_max = fmax;
    black_white_delimiter = bwdl;
  }

  // random constructor. Does NOT initialize srand, that will be a responsibility of the caller (i.e. the .cpp file)
  Genome() {
    color_delimiter = rand() % 256;
    field_min = rand() % 256;
    field_max = rand() % 256;
    black_white_delimiter = rand() % 256;
  }
  // alias
  static Genome random() {
    return Genome();
  }

  unsigned short color_delimiter;
  unsigned short field_min;
  unsigned short field_max;
  unsigned short black_white_delimiter;
};