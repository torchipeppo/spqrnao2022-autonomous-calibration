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

Genome::Genome(const FieldColors &fc) {
  color_delimiter = fc.maxNonColorSaturation;
  field_min = fc.fieldHue.min;
  field_max = fc.fieldHue.max;
  black_white_delimiter = fc.blackWhiteDelimiter;
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
#define SECOND_BLACK_LOSS_FORMULA(second_black_counter) second_black_counter*10  // relatively small, just to make a difference b/w already-optimal genomes

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
      FieldColors::Color px = coloredImage[Vector2i(i,j)];
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

  // additional penalty to avoid seeing too much black in the image:
  // minimize black pixels outside of the ball.
  // this only makes sense as long as the camera only looks at the field,
  // which is true since we only calibrate the lower camera,
  // but may fail if we start looking around.
  // but if we do, that would be a secondary, fine-tuning phase, so we might as well enforce a
  // "don't stray too much from the previous solution" policy and forget this part.
  int second_black_counter = 0;
  if (theBallPercept.status == BallPercept::Status::seen) {    // of course, "outside the ball" only makes sense if we saw the ball in the first place
    for (unsigned i=0; i<coloredImage.width; i++) {
      for (unsigned j=0; j<coloredImage.height; j++) {
        FieldColors::Color px = coloredImage[Vector2i(i,j)];
        if (px == FieldColors::black) {
          float sqr_dist_from_ball = (Vector2f(i,j) - theBallPercept.positionInImage).squaredNorm();
          if (sqr_dist_from_ball > theBallPercept.radiusInImage*theBallPercept.radiusInImage + 9) {
            second_black_counter++;
          }
        }
      }
    }
  }
  int second_black_penalty = second_black_counter*10;

  int fitness = field_score + ball_bonus - field_width_penalty - bw_penalty - second_black_penalty;

  DEBUG_COUT("    And my fitness is: " << fitness);

  return fitness;
}

/**
 * In simulation, the field interval (predictably) ends up very very narrow (like, width 5 or 10).
 * Possible solutions I can think of:
 * - Set field width penalty to zero if the interval is narrower than a threshold
 * - Let the genetic algorithm return a narrow interval, then widen it a posteriori
 *   by e.g. 5 per endpoint in order to give ourselves some margin
 * 
 * In real things are obviously very different and the interval ends up 60~80 wide.
 * So this might not be such a problem. I'll leave it here just for future reference.
 * 
 * Also investigate the bw delimiter thing, as above.
 */


#define FIELD_REWARD_FORMULA_PHASE2(green_counter_up, green_counter_dn) (green_counter_dn - green_counter_up)
#define STRAY_LOSS(me, reference) (me-reference)*(me-reference)    // REMEMBER TO USE ABSOLUTE VALUE if we decide to change from quadratic

// HIGH fitness still wins.
int Genome::evalFitness_phase2(const Image<PixelTypes::ColoredPixel>& coloredImage, const Genome& reference_calibration) {
  // Separate the image into upper and lower half, and count the color in each half
  int green_counter_up = 0,
      white_counter_up = 0,
      black_counter_up = 0,
      none_counter_up = 0;
  for (unsigned i=0; i<coloredImage.width; i++) {
    for (unsigned j=0; j<coloredImage.height/2; j++) {
      FieldColors::Color px = coloredImage[Vector2i(i,j)];
      if (px == FieldColors::field) {
        green_counter_up++;
      }
      else if (px == FieldColors::white) {
        white_counter_up++;
      }
      else if (px == FieldColors::black) {
        black_counter_up++;
      }
      else if (px == FieldColors::none) {
        none_counter_up++;
      }
    }
  }
  int green_counter_dn = 0,
      white_counter_dn = 0,
      black_counter_dn = 0,
      none_counter_dn = 0;
  for (unsigned i=0; i<coloredImage.width; i++) {
    for (unsigned j = coloredImage.height/2; j<coloredImage.height; j++) {
      FieldColors::Color px = coloredImage[Vector2i(i,j)];
      if (px == FieldColors::field) {
        green_counter_dn++;
      }
      else if (px == FieldColors::white) {
        white_counter_dn++;
      }
      else if (px == FieldColors::black) {
        black_counter_dn++;
      }
      else if (px == FieldColors::none) {
        none_counter_dn++;
      }
    }
  }

  // Main component of this fitness: see as much green as possible in the lower half (where the field is),
  // but also see as little as possible in the upper half (where the rest of the environment is).
  int field_score = FIELD_REWARD_FORMULA_PHASE2(green_counter_up, green_counter_dn);

  // Penalize wide field intervals in order to avoid clustering too much into green
  int field_width = field_max - field_min;
  int field_width_penalty = FW_LOSS_FORMULA(field_width);

  // Since the "actual" calibration was done in the previous calibration phase,
  // and phase 2 does not have a proper viewpoint for a full calibration, but rather it's just a fine-tuning,
  // make sure that the genes do not stray too much from the phase 1 results by imposing a penalty.
  int stray_penalty = 0;
  stray_penalty += STRAY_LOSS(color_delimiter, reference_calibration.color_delimiter);
  stray_penalty += STRAY_LOSS(field_min, reference_calibration.field_min);
  stray_penalty += STRAY_LOSS(field_max, reference_calibration.field_max);
  stray_penalty += STRAY_LOSS(black_white_delimiter, reference_calibration.black_white_delimiter);

  int fitness = field_score - field_width_penalty - stray_penalty;

  // std::cout << (int)color_delimiter << " " << (int)field_min << " " << (int)field_max << " " << (int)black_white_delimiter << std::endl;
  // std::cout << "up: " << green_counter_up << std::endl;
  // std::cout << "dn: " << green_counter_dn << std::endl;
  // std::cout << "---------------" << std::endl;

  return fitness;
}
