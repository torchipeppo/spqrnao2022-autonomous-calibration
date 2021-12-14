/**
 * Defines a class that essentially acts as a struct
 * (which is the reason why everything is public, this is not intended to be proper OOP)
 * for the genetic algorithm in FieldColorsCalibrator.
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

#include "Tools/ImageProcessing/Image.h"
#include "Tools/ImageProcessing/PixelTypes.h"

//see above
#pragma GCC diagnostic pop

class Genome
{
  public:

  // arbitrary constructor
  Genome(unsigned char cdel, unsigned char fmin, unsigned char fmax, unsigned char bwdl);

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
  int evalFitness(const Image<PixelTypes::ColoredPixel>& coloredImage);
};
