/**
 * Defines a class that essentially acts as a struct
 * (which is the reason why everything is public, this is not intended to be proper OOP)
 * for the genetic algorithm in FieldColorsCalibrator.
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

#include "Tools/ImageProcessing/Image.h"
#include "Tools/ImageProcessing/PixelTypes.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"

//see above
#pragma GCC diagnostic pop

// debug tools, the Calibrator includes this so they'll be available there as well
#define DEBUG false
#define DEBUG_COUT(msg) if (DEBUG) std::cout << msg << std::endl

template <typename T>
inline T clamp255(T n) {
  const T x = n < 0 ? 0 : n;
  return x > 255 ? 255 : x;
}

class Genome
{
  public:

  // arbitrary constructor
  Genome(unsigned char cdel, unsigned char fmin, unsigned char fmax, unsigned char bwdl);

  // copy constructor
  Genome(const Genome &g);

  // random constructor. Does NOT initialize srand, that will be a responsibility of the caller (i.e. FieldColorsCalibrator)
  Genome();
  // more readably named alias
  static Genome random();

  unsigned char color_delimiter;
  unsigned char field_min;
  unsigned char field_max;
  unsigned char black_white_delimiter;
  int fitness;

  const static unsigned char MIN = 0;
  const static unsigned char MAX = 255;

  // ensures each parameter is in [MIN, MAX] and that field_min <= field_max
  // so that is is a proper interval, and enforces these conditions via side effect.
  void validateParams();

  // computes the fitness of a genome from its parameters.
  int evalFitness(const Image<PixelTypes::ColoredPixel>& coloredImage, const BallPercept& theBallPercept);
};

// fitness comparators
struct fitness_lt {
  inline bool operator()(const Genome &g1, const Genome &g2) const {
    return g1.fitness < g2.fitness;
  }
};
struct fitness_gt {
  inline bool operator()(const Genome &g1, const Genome &g2) const {
    return g1.fitness > g2.fitness;
  }
};
