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
  fitness = g.fitness;
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



// fitness function parameters
#define FIELD_REWARD_FORMULA(green_counter) green_counter    // change this to try more complicated forms, like "green_counter*0.8" or something
#define BALL_REWARD 20000
#define FW_LOSS_FORMULA(field_width) (field_width/10)*100
#define BW_THRESHOLD 100
#define BW_LOSS_ONE 5000
#define DELIMITER_LOW_THRESHOLD 64
#define DELIMITER_HIGH_THRESHOLD 255-64
#define DELIMITER_LOSS 100    // very small, just to make a difference betwen multiple already-optimal genomes

// HIGH fitness wins.
int Genome::evalFitness(const Image<PixelTypes::ColoredPixel>& coloredImage, const BallPercept& theBallPercept) {
  DEBUG_COUT("    My field is: " << (int) field_min << " " << (int) field_max);

  // Count the number of pixels of each color
  int green_counter = 0,
      white_counter = 0,
      black_counter = 0,
      none_counter = 0;
  for (unsigned i=0; i<coloredImage.width; i++) {
    for (unsigned j=0; j<coloredImage.height; j++) {
      FieldColors::Color px = coloredImage[i][j];
      if (px == FieldColors::field) {
        green_counter++;
      }
      else if (px == FieldColors::white) {
        white_counter++;
      }
      else if (px == FieldColors::black) {
        black_counter++;
      }
      else if (px == FieldColors::none) {
        none_counter++;
      }
    }
  }
  // a full-green image will give ~57600 points (well, it used to, when fitness was just equal to green_counter)

  // The more green we can see, the better, b/c it's a backdrop for the other objects
  int field_score = FIELD_REWARD_FORMULA(green_counter);

  // The other colors are calibrated with a ball placed in the field:
  // being able to see it will give a large fitness bonus
  int ball_bonus = 0;
  if (theBallPercept.status == BallPercept::Status::seen) {
    ball_bonus = BALL_REWARD;
  }

  // Penalize wide field intervals in order to avoid clustering too much into green
  int field_width = field_max - field_min;
  int field_width_penalty = FW_LOSS_FORMULA(field_width);

  // Images with too little white or black are bad: the ball is made of both colors!
  int bw_penalty = 0;
  if (white_counter < BW_THRESHOLD) {
    bw_penalty += BW_LOSS_ONE;
  }
  if (black_counter < BW_THRESHOLD) {
    bw_penalty += BW_LOSS_ONE;
  }

  // I noticed that the bw delimiter tends to be pretty high for no reason,
  // resulting in the field lines becoming partly black.
  // TODO THIS NEEDS TO BE INVESTIGATED IN REAL, SIMULATION IS TOO PERFECT.
  // As a momentary, hopefully-not-nonsensical fix, however...
  int delimiter_penalty = 0;
  if (black_white_delimiter < DELIMITER_LOW_THRESHOLD || black_white_delimiter > DELIMITER_HIGH_THRESHOLD) {
    delimiter_penalty = DELIMITER_LOSS;
  }

  int fitness = field_score + ball_bonus - field_width_penalty - bw_penalty - delimiter_penalty;

  DEBUG_COUT("    And my fitness is: " << fitness);

  return fitness;
}

/**
 * TODO test in real post-christmas b/c the field interval (predictably)
 * ends up very very narrow in simulation (like, width 5 or 10).
 * Possible solutions I can think of:
 * - Set field width penalty to zero if the interval is narrower than a threshold
 * - Let the genetic algorithm return a narrow interval, then widen it a posteriori
 *   by e.g. 5 per endpoint in order to give ourselves some margin
 * 
 * Also investigate the bw delimiter thing, as above.
 */