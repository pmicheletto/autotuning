#include "sundaresan.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "../system/system_interface.h"

namespace {

constexpr double kMaxReference = 100.0;
constexpr double kPercentT1 = 0.353;
constexpr double kPercentT2 = 0.853;
constexpr double kSimulationDt = 0.001;

constexpr double kFirstStepRatio = 0.01;
constexpr double kStepHeightRatio = 0.5;
constexpr int kTimeoutIterations5s = 5000;
constexpr unsigned int kDefaultFinderStopTime = 20000;
constexpr unsigned int kMinStepsAtZero = 500;
constexpr double kMovementThreshold = 0.2;
constexpr double kSpeedNearZeroThreshold = 0.1;
constexpr unsigned int kWaitSettledStablePassesRequired = 10;

}  // namespace

PlantModel Sundaresan::_plantModel;

Sundaresan::Sundaresan(ISystem& system)
    : _system(&system),
      _state(SundaresanState::FindingMovement),
      _timeout(kTimeoutIterations5s),
      _step(kFirstStepRatio),
      _finder(kDefaultFinderStopTime),
      _stepHeightForCalc(kStepHeightRatio) {
  std::cout << "[Sundaresan] Etapa: início — procurando movimento (degrau inicial = "
            << _step << ")\n";
}

bool Sundaresan::Execute() {
  switch (_state) {
    case SundaresanState::FindingMovement:
      _system->setVariable("driver.step.height", _step);
      _timeout--;
      checkThereIsMovement();
      break;

    case SundaresanState::FindingDeadBand:
      _system->setVariable("driver.step.height", _step);
      _stepCount++;
      checkDeadBand();
      break;

    case SundaresanState::CalculateTransferFunction: {
      const bool phaseZeroOrThree =
          _calcPhase == CalcPhase::WaitZeroBeforeStep ||
          _calcPhase == CalcPhase::WaitZeroAfterFirstRun;
      _system->setVariable("driver.step.height",
                           phaseZeroOrThree ? 0.0 : _stepHeightForCalc);
      runCalculateTransferFunction();
      break;
    }

    default:
      break;
  }
  return _state != SundaresanState::Done &&
         _state != SundaresanState::UnresponsivePlant;
}

double Sundaresan::readFilteredSpeed() {
  const double speed = _system->readValue("out");
  _welfordCalc.AddSample(speed);
  return _welfordCalc.GetMean();
}

void Sundaresan::checkThereIsMovement() {
  const double filteredSpeed = readFilteredSpeed();
  if (filteredSpeed > kMovementThreshold) {
    std::cout << "[Sundaresan] Etapa: movimento detectado — medindo ruído / banda morta\n";
    _state = SundaresanState::FindingDeadBand;
    _finder.MaxDeviationTimerStarter(static_cast<unsigned int>(_stepCount));
    return;
  }

  if (_timeout == 0) {
    _step += kFirstStepRatio * kMaxReference;
    _timeout = kTimeoutIterations5s;
    std::cout << "[Sundaresan] Etapa: tempo esgotado sem movimento — aumentando degrau para "
              << _step << "\n";
  }

  if (_step > (kMaxReference / 2)) {
    std::cout << "[Sundaresan] Etapa: planta não responsiva (degrau além do limite)\n";
    _state = SundaresanState::UnresponsivePlant;
  }
}

void Sundaresan::checkDeadBand() {
  const double filteredSpeed = readFilteredSpeed();
  const bool timeout =
      _finder.AddData(filteredSpeed, static_cast<unsigned int>(_stepCount));
  if (timeout) {
    std::cout << "[Sundaresan] Etapa: medição de ruído concluída — iniciando função de transferência\n";
    _noise = _finder.GetMaxStdDeviation();
    std::cout << "[Sundaresan] Ruído estimado: " << _noise << "\n";
    _welfordCalc.Clear();
    _calcPhase = CalcPhase::WaitZeroBeforeStep;
    _calcStepCount = 0;
    _system->setVariable("driver.step.height", 0.0);
    _state = SundaresanState::CalculateTransferFunction;
  }
}

void Sundaresan::runCalculateTransferFunction() {
  const double filteredSpeed = readFilteredSpeed();

  switch (_calcPhase) {
    case CalcPhase::WaitZeroBeforeStep:
      runPhaseWaitZeroBeforeStep(filteredSpeed);
      break;
    case CalcPhase::DetectTransientStart:
      runPhaseDetectTransientStart();
      break;
    case CalcPhase::WaitSettled:
      runPhaseWaitSettled(filteredSpeed);
      break;
    case CalcPhase::WaitZeroAfterFirstRun:
      runPhaseWaitZeroAfterFirstRun(filteredSpeed);
      break;
    case CalcPhase::MeasureT1T2:
      runPhaseMeasureT1T2(filteredSpeed);
      break;
  }
}

void Sundaresan::runPhaseWaitZeroBeforeStep(double filteredSpeed) {
  if (filteredSpeed >= kSpeedNearZeroThreshold) {
    _calcStepCount = 0;
    return;
  }
  _calcStepCount++;
  if (_calcStepCount < kMinStepsAtZero) {
    return;
  }
  _calcStepCount = 0;
  _welfordCalc.Clear();
  _calcPhase = CalcPhase::DetectTransientStart;
  std::cout << "[Sundaresan] Etapa: saída em repouso próximo de zero — detectando início do transitório\n";
}

void Sundaresan::runPhaseDetectTransientStart() {
  if (_welfordCalc.GetDeviation() <= _noise) {
    return;
  }
  std::cout << "[Sundaresan] Etapa: transitório detectado — aguardando estabilização da resposta\n";
  _waitSettledStablePasses = 0;
  _calcPhase = CalcPhase::WaitSettled;
}

void Sundaresan::runPhaseWaitSettled(double filteredSpeed) {
  if (_welfordCalc.GetDeviation() > _noise) {
    _waitSettledStablePasses = 0;
    return;
  }
  _waitSettledStablePasses++;
  if (_waitSettledStablePasses < kWaitSettledStablePassesRequired) {
    return;
  }
  _finalValue = filteredSpeed;
  _waitSettledStablePasses = 0;
  std::cout << "[Sundaresan] Etapa: resposta estabilizada — valor final estimado = " << _finalValue
            << "\n";
  _calcPhase = CalcPhase::WaitZeroAfterFirstRun;
  _calcStepCount = 0;
  _system->setVariable("driver.step.height", 0.0);
}

void Sundaresan::runPhaseWaitZeroAfterFirstRun(double filteredSpeed) {
  // Same near-zero criterion as runPhaseWaitZeroBeforeStep.
  if (filteredSpeed >= kSpeedNearZeroThreshold) {
    _calcStepCount = 0;
    return;
  }
  _calcStepCount++;
  if (_calcStepCount < kMinStepsAtZero) {
    return;
  }
  std::cout << "[Sundaresan] Etapa: retorno após primeiro degrau — preparando medição de t1 e t2\n";
  _calcPhase = CalcPhase::MeasureT1T2;
  _secondStepApplied = false;
  _t1Set = false;
  _t2Set = false;
  _calcStepCount = 0;
  _welfordCalc.Clear();
}

void Sundaresan::runPhaseMeasureT1T2(double filteredSpeed) {
  if (!_secondStepApplied) {
    std::cout << "[Sundaresan] Etapa: segundo degrau aplicado — cronometrando "
              << (kPercentT1 * 100) << "% e " << (kPercentT2 * 100)
              << "% do valor final\n";
    _secondStepApplied = true;
    _calcStepCount = 0;
    return;
  }

  _calcStepCount++;
  const double t = _calcStepCount * kSimulationDt;
  const double targetT1 = kPercentT1 * _finalValue;
  const double targetT2 = kPercentT2 * _finalValue;

  if (!_t1Set && filteredSpeed >= targetT1) {
    _t1 = t;
    _t1Set = true;

    std::cout << "  " << (kPercentT1 * 100) << "% do valor final (alvo "
              << targetT1 << ") atingido em t1 = " << _t1 << " s\n";
  
  }
  if (!_t2Set && filteredSpeed >= targetT2) {
    _t2 = t;
    _t2Set = true;

    std::cout << "  " << (kPercentT2 * 100) << "% do valor final (alvo "
              << targetT2 << ") atingido em t2 = " << _t2 << " s\n";

  }

  if (_t1Set && _t2Set) {
    std::cout << "[Sundaresan] Etapa: tempos t1 e t2 obtidos — calculando parâmetros FOPDT\n";
    std::cout << "Sundaresan-Krishnaswamy: t1 = " << _t1 << " s, t2 = " << _t2
              << " s\n";

    finishPlantModel();
    _state = SundaresanState::Done;
  }
}

void Sundaresan::finishPlantModel() {
  _plantModel.theta = std::max(0.0, 1.3 * _t1 - 0.29 * _t2);
  const double deltaT = _t2 - _t1;
  _plantModel.tau1 = (deltaT > 0.0) ? 0.67 * deltaT : 0.0;
  _plantModel.K =
      (_stepHeightForCalc > 0.0) ? (_finalValue / _stepHeightForCalc) : 0.0;
  std::cout << "k: " << _plantModel.K << ", final_value: " << _finalValue << "step: " << _stepHeightForCalc << std::endl;
  _plantModel.tau2 = 0.0;
}

double Sundaresan::GetNoise() const { return _noise; }
double Sundaresan::GetDeviation() { return _welfordCalc.GetDeviation(); }
double Sundaresan::GetFilteredSpeed() { return _welfordCalc.GetMean(); }
double Sundaresan::GetFinalValue() const { return _finalValue; }
SundaresanState Sundaresan::GetState() const { return _state; }
bool Sundaresan::IsFinished() const {
  return _state == SundaresanState::Done ||
         _state == SundaresanState::UnresponsivePlant;
}
const PlantModel& Sundaresan::GetPlantModel() const { return _plantModel; }
