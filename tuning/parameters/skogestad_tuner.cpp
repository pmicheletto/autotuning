#include "skogestad_tuner.h"

#include <algorithm>
#include <iostream>

namespace {

// SIMC integral time blends the half-rule table value (tau + theta/2) with a tau_c-dependent
// term 4*(tau_c + theta/2) so Ti is not identical for every tau_c candidate.
constexpr double kSimcIntegralTimeFromTcFactor = 4.0;

}  // namespace

PIDParameters SkogestadTuner::calculatePI(const PlantModel& model, double tc) {
  PIDParameters params;
  if (model.K == 0.0) return params;

  const double tau = model.tau1;
  const double theta = model.theta;
  const double halfTheta = 0.5 * theta;
  const double den = tc + halfTheta;
  if (den <= 0.0) return params;

  params.Kp = (tau + halfTheta) / (model.K * den);
  const double tiFromTable = tau + halfTheta;
  const double tiFromTc = kSimcIntegralTimeFromTcFactor * (tc + halfTheta);
  params.Ti = std::min(tiFromTable, tiFromTc);
  std::cout << "tiFromTable: " << tiFromTable << ", tiFromTc: " << tiFromTc << ", params.Ti: " << params.Ti << std::endl;
  params.Td = 0.0;

  return params;
}

PIDParameters SkogestadTuner::calculatePID(const PlantModel& model, double tc) {
  PIDParameters params;
  if (model.K == 0.0) return params;

  const double tau = model.tau1;
  const double theta = model.theta;
  const double halfTheta = 0.5 * theta;
  const double den = tc + halfTheta;
  if (den <= 0.0) return params;

  params.Kp = (tau + halfTheta) / (model.K * den);
  const double tiFromTable = tau + halfTheta;
  const double tiFromTc = kSimcIntegralTimeFromTcFactor * (tc + halfTheta);
  params.Ti = std::min(tiFromTable, tiFromTc);

  const double twoTauPlusTheta = 2.0 * tau + theta;
  params.Td = (twoTauPlusTheta > 0.0) ? (tau * theta) / twoTauPlusTheta : 0.0;
  std::cout << "tiFromTable: " << tiFromTable << ", tiFromTc: " << tiFromTc << ", params.Ti: " << params.Ti << std::endl;
  return params;
}
