#ifndef TUNING_TUNING_TASK_H_
#define TUNING_TUNING_TASK_H_

#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "controllers/controller_type.h"
#include "models.h"
#include "statistics/WelfordAlgorithm.h"
#include "transfer_function/sundaresan.h"
#include "system/system_interface.h"

enum class TuningState {
  PLANT_IDENTIFICATION,
  CONTROLLER_TUNING,
  TEST_CONTROLLER,
  COMPLETED,
  ERROR
};

enum class TestPhase {
  Prepare,
  WaitTransient,
  WaitTouchRef,
  Analyze
};

enum class ControllerTestRound {
  INITIAL,
  REFINEMENT
};

class TuningTask {
 public:
  TuningTask(ControllerType controllerType, ISystem& system);

  void stepSimulation();
  void update();

  bool isCompleted() const;
  bool hasError() const;
  PIDParameters getResult();
  double getTestScore() const { return _testScore; }
  TuningState getState() const { return _state; }

  size_t debugUdpCandidateIndex() const {
    return _state == TuningState::TEST_CONTROLLER ? (_currentCandidateIndex + 1) : 0;
  }
  std::string debugTestPhaseName() const;

  struct DebugTransientState {
    double refBand{0.0};
    double acceptedDeviation{0.0};
    double deviation{0.0};
    double refError{0.0};
    bool nearReference{false};
    unsigned int stableStreak{0};
  };
  DebugTransientState debugTransientState() const;

  // In TEST_CONTROLLER / WaitTouchRef: accumulates max |jerk| for the current test (UDP subsampling).
  void noteControllerTestJerkSample(std::optional<double> jerk);
  // Max |jerk| for the current candidate; reset between tests.
  double debugUdpJerkMaxCurrentCandidate() const;

  void setSimulationStep(double dt) { _simulationStep = dt; }
  double getSimulationStep() const { return _simulationStep; }

 private:
  void tuneController();
  void runTestController();
  std::vector<double> buildInitialThreeTc() const;
  void appendLateralCandidatesFromBestOne();
  bool hasNearbyTc(double tc) const;
  void onCandidateTestFinished(double score, double iaeIfMeaningful,
                               double settlingTimeS, double overshootPercent,
                               int oscillations, double peakSpeedForOvershoot);
  void updateOscillationCountAnalyze(double output);
  unsigned int oscillationMinOutsideStreakRequired() const;
  double oscillationErrorDeadband() const;
  double scoreCurrentCandidate() const;

  ControllerType _controllerType;
  ISystem* _system;
  Sundaresan _sundaresan;
  double _simulationStep{0.001};
  double _noise{0.0};

  TuningState _state{TuningState::PLANT_IDENTIFICATION};
  PlantModel _plantModel{0.0, 0.0, 0.0, 0.0};
  PIDParameters _parameters{0.0, 0.0, 0.0};

  std::vector<PIDParameters> _candidates;
  size_t _currentCandidateIndex{0};
  TestPhase _testPhase{TestPhase::Prepare};
  unsigned int _prepareSteps{0};
  unsigned int _prepareNearZeroStreak{0};
  bool _resetSystemBeforePrepare{true};
  static constexpr double kIdentificationInputStep = 0.5;
  // Reference step height (driver.step.height) during closed-loop PID candidate tests.
  static constexpr double kClosedLoopStepReference = 36.0;
  double _closedLoopReference{0.0};
  WelfordAlgorithm _testWelford{10};

  double _maxOutput{0.0};
  bool _hasOvershoot5{false};
  int _oscillationCount{0};
  int _lastSign{0};
  unsigned int _oscOutsideStreak{0};
  int _oscOutsideDir{0};
  bool _oscOutsideLobeConfirmed{false};
  int _oscHystOutside{0};
  double _analyzeSumAbsError{0.0};
  unsigned int _analyzeStepCount{0};
  static constexpr unsigned int kAnalyzeDurationSteps = 5000;
  // Prepare (open loop): stricter near-zero tolerance and longer budget than WaitTransient.
  static constexpr unsigned int kPrepareZeroSpeedMaxSteps = 250000;
  static constexpr double kPrepareZeroSpeedAbsTol = 5e-5;
  static constexpr double kPrepareZeroSpeedNoiseFactor = 2.0;
  static constexpr unsigned int kPrepareZeroStableSteps = 80;
  static constexpr double kZeroSpeedAbsTol = 1e-4;
  static constexpr double kZeroSpeedNoiseFactor = 5.0;
  //static constexpr unsigned int kWaitTouchRefTimeoutSteps = 45000;
  static constexpr unsigned int kWaitTouchRefTimeoutSteps = 10000;

  static constexpr double kOvershootThreshold = 1.05;
  static constexpr double kSettlingBandFrac = 0.02;
  static constexpr double kSettlingBandNoiseMult = 5.0;
  static constexpr unsigned int kOscillationMinOutsideSteps = 18;
  static constexpr double kOscillationMinOutsideMs = 80.0;
  static constexpr double kOscillationDeadbandAbsFloor = 1e-9;
  static constexpr double kOscillationDeadbandHysteresisRatio = 0.58;
  static constexpr double kWaitTransientDriftNoiseMult = 10.0;
  static constexpr double kWaitTransientDriftAbsMin = 0.008;
  static constexpr double kWaitTransientRefProgressFrac = 0.0004;
  static constexpr double kWaitTransientRefProgressMaxStartFrac = 0.05;
  static constexpr double kWaitTransientStepJumpNoiseMult = 5.0;
  static constexpr unsigned int kWaitTransientStableSamples = 10;
  static constexpr unsigned int kWaitTransientMaxSteps = 30000;
  // After plant ID (Sundaresan), require open-loop output near zero for a stable window
  // before starting gain tests.
  static constexpr unsigned int kPostIdentificationZeroStableSteps = 7000;
  static constexpr double kPostIdentificationZeroAbsTol = 5e-5;
  static constexpr double kPostIdentificationZeroNoiseFactor = 2.0;
  // Phase 1: tau_c grid = kInitialTcProbeMinS + i * kInitialTcProbeStepS, i = 0..2.
  static constexpr double kInitialTcProbeMinS = 6.0;
  static constexpr double kInitialTcProbeStepS = 3.0;
  // Phase 2: lateral candidates tau_c* * (1 +/- kTcLateralFraction).
  static constexpr double kTcLateralFraction = 0.12;
  static constexpr double kTcNearRelativeTol = 0.04;
  ControllerTestRound _controllerTestRound{ControllerTestRound::INITIAL};
  size_t _round1Count{0};
  std::vector<double> _candidateTc;
  std::vector<double> _candidateScores;
  std::vector<double> _candidateJerkAbsMax;
  unsigned int _waitTouchRefSteps{0};
  double _waitTransientStartOutput{0.0};
  double _waitTransientPrevOutput{0.0};
  unsigned int _waitTransientStepCount{0};
  unsigned int _waitTransientStableStreak{0};
  bool _waitTransientStallLogged{false};
  bool _problematicoDebugPrinted{false};
  unsigned int _postIdentificationWaitCounter{0};
  bool _postIdentificationWaitLogged{false};
  unsigned int _postIdentificationZeroStreak{0};

  size_t _bestCandidateIndex{0};
  double _bestScore{-1e9};
  double _bestJerkAbsMax{std::numeric_limits<double>::quiet_NaN()};
  double _testScore{0.0};

  double _settlingTimeS{std::numeric_limits<double>::quiet_NaN()};
  double _bestSettlingTimeS{std::numeric_limits<double>::quiet_NaN()};
  double _bestOvershootPercent{0.0};
  int _bestOscillations{0};
  double _bestPeakSpeedForOvershoot{std::numeric_limits<double>::quiet_NaN()};

  double _jerkAbsMaxThisCandidate{std::numeric_limits<double>::quiet_NaN()};
  double _minJerkMaxAmongCompletedTests{std::numeric_limits<double>::quiet_NaN()};
  bool _hasMinJerkMaxAmongCompletedTests{false};
};

#endif
