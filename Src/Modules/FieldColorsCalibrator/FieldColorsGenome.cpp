/**
 * Implementation file for the Genome class.
 * 
 * @author Francesco Petri
 */

#include "FieldColorsGenome.h"
#include <stdlib.h>
#include <iostream>

unsigned char clamp(unsigned char n) {
  const unsigned char t = n < 0 ? 0 : n;
  return t > 255 ? 255 : t;
}

Genome::Genome(unsigned char cdel, unsigned char fmin, unsigned char fmax, unsigned char bwdl) {
  color_delimiter = cdel;
  field_min = fmin;
  field_max = fmax;
  black_white_delimiter = bwdl;
  validateParams();
}

Genome::Genome(const Genome &g) {
  color_delimiter = g.color_delimiter;
  field_min = g.field_min;
  field_max = g.field_max;
  black_white_delimiter = g.black_white_delimiter;
  validateParams();
}

Genome::Genome() {
  color_delimiter = (unsigned char) (rand() % 256);
  field_min = (unsigned char) (rand() % 256);
  field_max = (unsigned char) (rand() % 256);
  black_white_delimiter = (unsigned char) (rand() % 256);
  validateParams();
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
    const unsigned char actually_max = field_min;
    field_min = field_max;
    field_max = actually_max;
  }
}

// HIGH fitness wins.
// For now, it's just the number of green pixels in the image.
// May change in the future, but I prefer sth simple to get the whole algorithm running first.
int Genome::evalFitness(const Image<PixelTypes::ColoredPixel>& coloredImage) {
  DEBUG_COUT("    My field is: " << (int) field_min << " " << (int) field_max);

  int green_counter = 0;
  for (unsigned i=0; i<coloredImage.width; i++) {
    for (unsigned j=0; j<coloredImage.height; j++) {
      if (coloredImage[i][j] == FieldColors::field) {
        green_counter++;
      }
    }
  }

  DEBUG_COUT("    And my fitness is: " << green_counter);

  return green_counter;
}