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
  TEST_CONTROLLER,  // malha fechada: referência = degrau de identificação × K (saída alvo), transiente/IAE etc.
  COMPLETED,
  ERROR
};

// Fases do teste de cada candidato PID (estilo Sundaresan: desvio vs ruído, depois análise).
enum class TestPhase {
  Prepare,       // aplicar PID, fechar malha, referência = K × degrau de entrada (como na identificação)
  WaitTransient, // esperar desvio da saída > ruído
  WaitTouchRef,  // esperar saída tocar a referência alvo para começar análise
  Analyze        // medir overshoot 5%, oscilação e seguimento (desvio)
};

enum class ControllerTestRound {
  INITIAL,    // 4 ganhos espaçados em τ_c
  REFINEMENT  // laterais dos dois melhores τ_c da 1.ª fase
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
  void runTestController();  // um passo da máquina de estados do teste (referência coerente com K, análise)
  std::vector<double> buildInitialFourTc() const;
  void appendLateralCandidatesFromTopTwo();
  bool hasNearbyTc(double tc) const;
  void onCandidateTestFinished(double score, double iaeIfMeaningful);
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
  // Degrau na entrada na fase Sundaresan (0.5×100); manter alinhado a sundaresan.cpp kStepHeightRatio*kMaxReference.
  static constexpr double kIdentificationInputStep = 0.5;
  // Referência em malha fechada = kIdentificationInputStep × K (mesma saída em regime que o degrau aberto).
  double _closedLoopReference{0.0};
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
  // 4 τ_c iniciais: log-uniforme entre kInitialTcMinFactor·base e kInitialTcMaxFactor·base.
  static constexpr double kInitialTcMinFactor = 0.35;
  static constexpr double kInitialTcMaxFactor = 3.2;
  // Laterais: (1±kTcLateralFraction)·τ_c em torno de cada um dos dois melhores da 1.ª fase.
  static constexpr double kTcLateralFraction = 0.22;
  static constexpr double kTcNearRelativeTol = 0.04;  // não repetir τ_c quase iguais
  ControllerTestRound _controllerTestRound{ControllerTestRound::INITIAL};
  size_t _round1Count{0};
  std::vector<double> _candidateTc;
  std::vector<double> _candidateScores;
  unsigned int _waitTouchRefSteps{0};

  size_t _bestCandidateIndex{0};
  double _bestScore{-1e9};  // maior = melhor
  double _testScore{0.0};   // IAE do melhor candidato (menor = melhor)
};

#endif
