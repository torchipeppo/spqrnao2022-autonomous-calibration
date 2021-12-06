#include "FieldColorsCalibrator.h"
#include "Tools/Debugging/Debugging.h"
#include "Platform/SystemCall.h"
#include "Tools/Module/Module.h"
#include <iostream>

MAKE_MODULE(FieldColorsCalibrator, infrastructure)

int nextArcana = 0;
bool breakCalibration = false;
bool loaded = false;

void FieldColorsCalibrator::update(FieldColors& fc) {
  // start with the initial guess provided in the cfg (for now, at least)
  if (!loaded) {
    loadModuleParameters(fc, "FieldColors", nullptr);
    loaded=true;
  }

  // a needlessly complicated hello world
  DEBUG_RESPONSE_ONCE("module:FieldColorsCalibrator:arcana")
  {
    OUTPUT_TEXT("The Arcana is the means by which all is revealed.");
    SystemCall::say("The Arcana is the means by which all is revealed.");
    std::cout << "The Arcana is the means by which all is revealed." << std::endl;
    switch (nextArcana) {
      case 0: OUTPUT_TEXT("The beginning of the journey is full of possibilities."); break;
      case 1: OUTPUT_TEXT("Birth is a miracle. Beware of illusions."); break;
      case 2: OUTPUT_TEXT("Some secrets aren't meant to be known."); break;
      case 3: OUTPUT_TEXT("Celebrate life... Its grandeur... Its magnificence..."); break;
      case 4: OUTPUT_TEXT("It was full of emperors around here once. Where did they all go?"); break;
      case 5: OUTPUT_TEXT("Learn from your teachers. Especially if there's an exam in a few months."); break;
      case 6: OUTPUT_TEXT("Sometimes there will hard choices. Choose wisely."); break;
      case 7: OUTPUT_TEXT("Victory is fleeting. Be careful."); break;
      case 8: OUTPUT_TEXT("The world is not so black and white."); break;
      case 9: OUTPUT_TEXT("Introspection is necessary sometimes."); break;
      case 10: OUTPUT_TEXT("Luck comes and goes. During the former, prepare for the latter."); break;
      case 11: OUTPUT_TEXT("Inner strength is more important than outer."); break;
      case 12: OUTPUT_TEXT("Sacrifices must be made."); break;
      case 13: OUTPUT_TEXT("Beyond the beaten path lies absolute end. No matter who you are... Death awaits you."); break;
      case 99: OUTPUT_TEXT("I said stop"); break;
      default: OUTPUT_TEXT("I have no more life advice for you."); break;
    }
    nextArcana++;
  }

  DEBUG_RESPONSE_ONCE("module:FieldColorsCalibrator:breakCalibration")
  {
    breakCalibration = true;
  }

  if (breakCalibration) {
    fc.blackWhiteDelimiter = 0;
    fc.maxNonColorSaturation = 0;
    fc.fieldHue.min = 255;
    fc.fieldHue.max = 255;
    OUTPUT_TEXT("NAO is affected by Blindness!");
    breakCalibration = false;
  }
}