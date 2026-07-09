#ifndef TUNING_SKOGESTAD_TUNER_H_
#define TUNING_SKOGESTAD_TUNER_H_

#include "../models.h"

class SkogestadTuner {
 public:
  static PIDParameters calculatePI(const PlantModel& model, double tc);

  static PIDParameters calculatePID(const PlantModel& model, double tc);
};

#endif