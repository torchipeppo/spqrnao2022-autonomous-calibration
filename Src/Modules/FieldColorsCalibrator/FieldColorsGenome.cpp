/**
 * Implementation file for the Genome class.
 * 
 * @author Francesco Petri
 */

#include "FieldColorsGenome.h"
#include <stdlib.h>

unsigned short clamp(unsigned short n) {
  const unsigned short t = n < 0 ? 0 : n;
  return t > 255 ? 255 : t;
}

Genome::Genome(unsigned short cdel, unsigned short fmin, unsigned short fmax, unsigned short bwdl) {
  color_delimiter = cdel;
  field_min = fmin;
  field_max = fmax;
  black_white_delimiter = bwdl;
  validateParams();
  fitness = evalFitness();
}

Genome::Genome() {
  color_delimiter = rand() % 256;
  field_min = rand() % 256;
  field_max = rand() % 256;
  black_white_delimiter = rand() % 256;
  validateParams();
  fitness = evalFitness();
}

Genome Genome::random() {
  return Genome();
}

void Genome::validateParams() {
  color_delimiter = clamp(color_delimiter);
  field_min = clamp(field_min);
  field_max = clamp(field_max);
  black_white_delimiter = clamp(black_white_delimiter);
  if (field_min > field_max) {
    const unsigned short actually_max = field_min;
    field_min = field_max;
    field_max = actually_max;
  }
}

int Genome::evalFitness() {
  // TODOOOOOOOOO
  return 0;
}