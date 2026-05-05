#ifndef TUNING_MODELS_H_
#define TUNING_MODELS_H_

struct PIDParameters {
  double Kp{0.0};
  double Ti{0.0};
  double Td{0.0};
};

struct PlantModel {
  double K;       // Process gain
  double tau1;    // Dominant time constant
  double tau2;    // Second time constant (second-order models)
  double theta;   // Effective dead time
};

#endif
