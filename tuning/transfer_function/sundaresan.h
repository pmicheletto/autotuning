#ifndef TUNING_TRANSFER_FUNCTION_SUNDARESAN_H_
#define TUNING_TRANSFER_FUNCTION_SUNDARESAN_H_

#include "../models.h"
#include "statistics/MaxDeviationFinder.h"
#include "statistics/WelfordAlgorithm.h"
#include "../system/system_interface.h"

enum class SundaresanState {
  FindingMovement,
  FindingDeadBand,
  CalculateTransferFunction,
  Done,
  UnresponsivePlant,
};

// Fases internas do cálculo Sundaresan-Krishnaswamy
enum class CalcPhase {
  WaitZeroBeforeStep,
  DetectTransientStart,
  WaitSettled,
  WaitZeroAfterFirstRun,
  MeasureT1T2,
};

class ISystem;

class Sundaresan {
 public:
  explicit Sundaresan(ISystem& system);

  static PlantModel extractTransferFunction();
  //nodiscard C++17
  [[nodiscard]] double GetNoise() const;
  [[nodiscard]] double GetDeviation();      // usa WelfordAlgorithm (não-const)
  [[nodiscard]] double GetFilteredSpeed();  // usa WelfordAlgorithm (não-const)
  [[nodiscard]] double GetFinalValue() const;
  [[nodiscard]] SundaresanState GetState() const;
  [[nodiscard]] const PlantModel& GetPlantModel() const;
  [[nodiscard]] bool IsFinished() const;

  bool Execute();
  void Task();

 private:
  double readFilteredSpeed();
  void checkThereIsMovement();
  void checkDeadBand();
  void runCalculateTransferFunction();

  void runPhaseWaitZeroBeforeStep(double filteredSpeed);
  void runPhaseDetectTransientStart();
  void runPhaseWaitSettled(double filteredSpeed);
  void runPhaseWaitZeroAfterFirstRun(double filteredSpeed);
  void runPhaseMeasureT1T2(double filteredSpeed);
  void finishPlantModel();

  ISystem* _system;
  SundaresanState _state;
  int _timeout;
  double _step;
  double _finalValue{0.0};
  double _noise{0.0};
  unsigned int _stepCount{0};
  MaxDeviationFinder _finder;
  static PlantModel _plantModel;

  WelfordAlgorithm _welfordCalc{10};
  CalcPhase _calcPhase{CalcPhase::WaitZeroBeforeStep};
  unsigned int _calcStepCount{0};
  double _t1{0.0};
  double _t2{0.0};
  bool _t1Set{false};
  bool _t2Set{false};
  bool _secondStepApplied{false};
  double _stepHeightForCalc;
};

#endif
