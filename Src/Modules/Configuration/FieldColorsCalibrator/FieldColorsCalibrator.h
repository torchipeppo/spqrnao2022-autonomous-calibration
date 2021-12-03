// in this branch, this is a do-nothing thing.

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