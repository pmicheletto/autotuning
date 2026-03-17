#ifndef TUNING_MODELS_H_
#define TUNING_MODELS_H_

struct PIDParameters {
  double Kp{0.0};
  double Ti{0.0};
  double Td{0.0};
};

struct PlantModel {
  double K;      // Ganho
  double tau1;   // Constante de tempo dominante
  double tau2;   // Segunda constante de tempo (para 2nd ordem)
  double theta;  // Atraso total (dead time)
};

#endif
