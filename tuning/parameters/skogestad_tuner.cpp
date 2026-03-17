#include "skogestad_tuner.h"

#include <algorithm>

PIDParameters SkogestadTuner::calculatePI(const PlantModel& model, double tc) {
  PIDParameters params;
  if (model.K == 0.0) return params;

  params.Kp = (1.0 / model.K) * (model.tau1 / (tc + model.theta));
  params.Ti = std::min(model.tau1, 4.0 * (tc + model.theta));
  params.Td = 0.0;

  return params;
}

PIDParameters SkogestadTuner::calculatePID(const PlantModel& model, double tc) {
  PIDParameters params;
  if (model.K == 0.0) return params;  // return [0,0,0]

  double Kp_serial = (1.0 / model.K) * (model.tau1 / (tc + model.theta));
  double Ti_serial = std::min(model.tau1, 4.0 * (tc + model.theta));
  double Td_serial = model.tau2;

  double fator = (1.0 + (Td_serial / Ti_serial));
  params.Kp = Kp_serial * fator;
  params.Ti = Ti_serial * fator;
  params.Td = Td_serial / fator;

  return params;
}
