/**
 * Defines a class that essentially acts as a struct
 * (which is the reason why everything is public, this is not intended to be proper OOP)
 * for the genetic algorithm in FieldColorsCalibrator.
 * 
 * @author Francesco Petri
 */

class Genome
{
  public:

  // arbitrary constructor
  Genome(unsigned short cdel, unsigned short fmin, unsigned short fmax, unsigned short bwdl);

  // random constructor. Does NOT initialize srand, that will be a responsibility of the caller (i.e. FieldColorsCalibrator)
  Genome();
  // more readably named alias
  static Genome random();

  unsigned short color_delimiter;
  unsigned short field_min;
  unsigned short field_max;
  unsigned short black_white_delimiter;
  int fitness;

  const static unsigned short MIN = 0;
  const static unsigned short MAX = 255;

  // ensures each parameter is in [MIN, MAX] and that field_min <= field_max
  // so that is is a proper interval, and enforces these conditions via side effect.
  void validateParams();

  // computes the fitness of a genome from its parameters.
  int evalFitness();
};
