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
});

class FieldColorsCalibrator : public FieldColorsCalibratorBase
{
  void update(FieldColors& fc) override;
};