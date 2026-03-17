#ifndef TUNING_TUNING_TASK_H_
#define TUNING_TUNING_TASK_H_

#include <vector>

#include "controllers/controller_type.h"
#include "models.h"
#include "statistics/WelfordAlgorithm.h"
#include "transfer_function/sundaresan.h"
#include "system/system_interface.h"

enum class TuningState {
  PLANT_IDENTIFICATION,
  CONTROLLER_TUNING,
  TEST_CONTROLLER,  // malha fechada: step 50%, verificar transiente/ruído, analisar overshoot/oscilação/seguimento
  COMPLETED,
  ERROR
};

// Fases do teste de cada candidato PID (estilo Sundaresan: desvio vs ruído, depois análise).
enum class TestPhase {
  Prepare,       // aplicar PID, fechar malha, step 50%
  WaitTransient, // esperar desvio da saída > ruído
  WaitTouchRef,  // esperar saída tocar a referência (50%) para começar análise
  Analyze        // medir overshoot 5%, oscilação e seguimento (desvio)
};

class TuningTask {
 public:
  // system usado para avançar a simulação e pelo Sundaresan na identificação.
  TuningTask(ControllerType controllerType, ISystem& system);

  // Avança a simulação um passo. Deve ser chamado pelo driver a cada iteração (para identificação e teste do controlador).
  void stepSimulation();

  // Um passo da lógica do tuning (sem avançar simulação): Sundaresan, cálculo PID ou recolha para pontuação.
  void update();

  bool isReady() const;
  bool isCompleted() const;
  bool hasError() const;
  PIDParameters getResult();
  // IAE durante TEST_CONTROLLER (menor = melhor). Válido após isCompleted().
  double getTestScore() const { return _testScore; }
  TuningState getState() const { return _state; }

  // Passo de simulação em segundos (ex.: 0.001). Só relevante durante PLANT_IDENTIFICATION.
  void setSimulationStep(double dt) { _simulationStep = dt; }
  double getSimulationStep() const { return _simulationStep; }

 private:
  void tuneController();
  void runTestController();  // um passo da máquina de estados do teste (step 50%, desvio>ruído, análise)
  std::vector<double> buildTcCandidates() const;
  bool isBetterParams(const PIDParameters& current, const PIDParameters& best);
  // Pontuação do candidato atual: penaliza overshoot>5% e oscilação; recompensa seguimento (menor IAE = melhor).
  double scoreCurrentCandidate() const;

  ControllerType _controllerType;
  ISystem* _system;
  Sundaresan _sundaresan;
  double _simulationStep{0.001};
  double _noise{0.0};  // ruído da planta (Sundaresan)

  TuningState _state{TuningState::PLANT_IDENTIFICATION};
  PlantModel _plantModel{0.0, 0.0, 0.0, 0.0};
  PIDParameters _parameters{0.0, 0.0, 0.0};
  double _bestTc{0.0};
  bool _isReady{false};

  std::vector<PIDParameters> _candidates;
  size_t _currentCandidateIndex{0};
  TestPhase _testPhase{TestPhase::Prepare};
  unsigned int _prepareSteps{0};  // passos em Prepare (assentar entre candidatos)
  static constexpr double kReference50 = 50.0;  // step 50% (mesmo que Sundaresan)
  WelfordAlgorithm _testWelford{10};

  bool _touchedRef{false};
  double _maxOutput{0.0};
  bool _hasOvershoot5{false};
  int _oscillationCount{0};
  int _lastSign{0};  // sinal de (output - reference)
  double _analyzeSumAbsError{0.0};
  unsigned int _analyzeStepCount{0};
  static constexpr unsigned int kAnalyzeDurationSteps = 5000;
  static constexpr unsigned int kSettleStepsBetweenCandidates = 2000;
  static constexpr unsigned int kWaitTouchRefTimeoutSteps = 15000;
  static constexpr double kOvershootThreshold = 1.05;  // 5% overshoot
  unsigned int _waitTouchRefSteps{0};

  size_t _bestCandidateIndex{0};
  double _bestScore{-1e9};  // maior = melhor
  double _testScore{0.0};   // IAE do melhor candidato (menor = melhor)
};

#endif
