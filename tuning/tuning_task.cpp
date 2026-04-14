#include "tuning_task.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

#include "loop_control.h"
#include "parameters/parameter_validator.h"
#include "parameters/skogestad_tuner.h"

namespace {
constexpr const char* kOutputVar = "out";
}  // namespace

TuningTask::TuningTask(ControllerType controllerType, ISystem& system)
    : _controllerType(controllerType),
      _system(&system),
      _sundaresan(system) {}

void TuningTask::stepSimulation() {
  _system->advanceSimulation(_simulationStep);
}

void TuningTask::update() {
  if (_state == TuningState::COMPLETED || _state == TuningState::ERROR) {
    return;
  }

  if (_state == TuningState::PLANT_IDENTIFICATION) {
    loop_control::openLoop(*_system);
    _sundaresan.Execute();

    if (_sundaresan.IsFinished()) {
      if (_sundaresan.GetState() == SundaresanState::Done) {
        _plantModel = _sundaresan.GetPlantModel();
        _noise = _sundaresan.GetNoise();
        std::cout << "Função de transferência encontrada (Sundaresan-Krishnaswamy):\n"
                  << "  K = " << _plantModel.K << ", tau1 = " << _plantModel.tau1
                  << ", tau2 = " << _plantModel.tau2 << ", theta = " << _plantModel.theta << " s\n";
        _state = TuningState::CONTROLLER_TUNING;
      } else {
        _state = TuningState::ERROR;
      }
    }
    return;
  }

  if (_state == TuningState::CONTROLLER_TUNING) {
    tuneController();
    return;
  }

  if (_state == TuningState::TEST_CONTROLLER) {
    runTestController();
  }
}

void TuningTask::runTestController() {
  const double output = _system->readValue(kOutputVar);

  switch (_testPhase) {
    case TestPhase::Prepare: {
      if (_currentCandidateIndex > 0) {
        loop_control::openLoop(*_system);
        _system->setVariable("driver.step.height", 0.0);
        _prepareSteps++;
        if (_prepareSteps < kSettleStepsBetweenCandidates) {
          return;
        }
      }
      const PIDParameters& params = _candidates[_currentCandidateIndex];
      loop_control::applyPIDWithTimeConstants(*_system, params.Kp, params.Ti,
                                              params.Td);
      loop_control::closeLoop(*_system);
      _system->setVariable("driver.step.height", _closedLoopReference);
      std::cout << "manual: " << _system->readValue("driver.useManualControl") << std::endl;
      _testWelford.Clear();
      _prepareSteps = 0;
      _waitTouchRefSteps = 0;
      _testPhase = TestPhase::WaitTransient;
      _touchedRef = false;
      _maxOutput = 0.0;
      _hasOvershoot5 = false;
      _oscillationCount = 0;
      _lastSign = 0;
      _analyzeSumAbsError = 0.0;
      _analyzeStepCount = 0;
      break;
    }

    case TestPhase::WaitTransient: {
      _testWelford.AddSample(output);
      const double deviation = _testWelford.GetDeviation();
      if (deviation > _noise) {
        _testPhase = TestPhase::WaitTouchRef;
      }
      break;
    }

    case TestPhase::WaitTouchRef: {
      _waitTouchRefSteps++;
      if (output >= _closedLoopReference) {
        _touchedRef = true;
        _maxOutput = output;
        _lastSign = (output > _closedLoopReference ? 1 : 0);
        _analyzeSumAbsError = 0.0;
        _analyzeStepCount = 0;
        _waitTouchRefSteps = 0;
        _testPhase = TestPhase::Analyze;
      } else if (_waitTouchRefSteps >= kWaitTouchRefTimeoutSteps) {
        onCandidateTestFinished(-1e9, std::numeric_limits<double>::quiet_NaN());
      }
      break;
    }

    case TestPhase::Analyze: {
      _maxOutput = std::max(_maxOutput, output);
      if (_maxOutput > _closedLoopReference * kOvershootThreshold) {
        _hasOvershoot5 = true;
      }
      const double err = output - _closedLoopReference;
      const int sign = (err > 0 ? 1 : (err < 0 ? -1 : 0));
      if (_lastSign != 0 && sign != 0 && _lastSign != sign) {
        _oscillationCount++;
      }
      _lastSign = sign;
      _analyzeSumAbsError += std::fabs(err) * _simulationStep;
      _analyzeStepCount++;

      if (_analyzeStepCount >= kAnalyzeDurationSteps) {
        const double score = scoreCurrentCandidate();
        onCandidateTestFinished(score, _analyzeSumAbsError);
      }
      break;
    }
  }
}

double TuningTask::scoreCurrentCandidate() const {
  if (_hasOvershoot5) {
    return -1e6;
  }
  const double oscillationPenalty = static_cast<double>(_oscillationCount) * 100.0;
  const double followReward = -_analyzeSumAbsError;
  return followReward - oscillationPenalty;
}

std::vector<double> TuningTask::buildInitialFourTc() const {
  const double theta = _plantModel.theta;
  const double tau = _plantModel.tau1;
  const double baseTc = (theta > 0.001) ? theta : (tau * 0.1);
  const double tc_min = std::max(1e-9, baseTc * kInitialTcMinFactor);
  const double tc_max = std::max(tc_min * 1.01, baseTc * kInitialTcMaxFactor);
  std::vector<double> out;
  out.reserve(4);
  for (int i = 0; i < 4; ++i) {
    const double alpha = static_cast<double>(i) / 3.0;
    out.push_back(tc_min * std::pow(tc_max / tc_min, alpha));
  }
  return out;
}

bool TuningTask::hasNearbyTc(double tc) const {
  for (double t : _candidateTc) {
    const double scale = std::max(std::fabs(t), std::fabs(tc));
    if (scale < 1e-15) {
      continue;
    }
    if (std::fabs(t - tc) <= kTcNearRelativeTol * scale) {
      return true;
    }
  }
  return false;
}

void TuningTask::appendLateralCandidatesFromTopTwo() {
  const size_t n = _round1Count;
  if (n == 0) {
    return;
  }
  std::vector<size_t> order(n);
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [this](size_t a, size_t b) {
    return _candidateScores[a] > _candidateScores[b];
  });
  const size_t nTop = std::min<size_t>(2, n);
  std::vector<double> proposed;
  for (size_t k = 0; k < nTop; ++k) {
    const double tc_star = _candidateTc[order[k]];
    proposed.push_back(std::max(1e-12, tc_star * (1.0 - kTcLateralFraction)));
    proposed.push_back(tc_star * (1.0 + kTcLateralFraction));
  }
  size_t added = 0;
  for (double tc : proposed) {
    if (hasNearbyTc(tc)) {
      continue;
    }
    PIDParameters currentParams;
    switch (_controllerType) {
      case ControllerType::PI:
        currentParams = SkogestadTuner::calculatePI(_plantModel, tc);
        break;
      case ControllerType::PID:
        currentParams = SkogestadTuner::calculatePID(_plantModel, tc);
        break;
      default:
        _state = TuningState::ERROR;
        return;
    }
    if (ParameterValidator::validate(currentParams)) {
      _candidates.push_back(currentParams);
      _candidateTc.push_back(tc);
      ++added;
    }
  }
  _candidateScores.resize(_candidates.size(), -1e9);
  std::cout << "Fase 2 — refinamento: " << added
            << " candidato(s) lateral(is) em τ_c (vizinhos dos 2 melhores da fase 1).\n";
  for (size_t i = _round1Count; i < _candidates.size(); ++i) {
    const auto& p = _candidates[i];
    std::cout << "  [" << (i + 1) << "] τ_c = " << _candidateTc[i] << ", Kp = " << p.Kp
              << ", Ti = " << p.Ti << ", Td = " << p.Td << "\n";
  }
}

void TuningTask::onCandidateTestFinished(double score, double iaeForThisCandidate) {
  const size_t idx = _currentCandidateIndex;
  if (idx < _candidateScores.size()) {
    _candidateScores[idx] = score;
  }
  if (score > _bestScore) {
    _bestScore = score;
    _bestCandidateIndex = idx;
    _parameters = _candidates[idx];
    if (!std::isnan(iaeForThisCandidate)) {
      _testScore = iaeForThisCandidate;
    }
  }
  _currentCandidateIndex++;
  if (_controllerTestRound == ControllerTestRound::INITIAL && _round1Count > 0 &&
      _currentCandidateIndex == _round1Count) {
    appendLateralCandidatesFromTopTwo();
    _controllerTestRound = ControllerTestRound::REFINEMENT;
  }
  if (_currentCandidateIndex < _candidates.size()) {
    _testPhase = TestPhase::Prepare;
    _prepareSteps = 0;
  } else {
    _parameters = _candidates[_bestCandidateIndex];
    _state = TuningState::COMPLETED;
    _isReady = true;
  }
}

void TuningTask::tuneController() {
  const std::vector<double> tcList = buildInitialFourTc();
  _candidates.clear();
  _candidateTc.clear();

  for (double tc : tcList) {
    PIDParameters currentParams;
    switch (_controllerType) {
      case ControllerType::PI:
        currentParams = SkogestadTuner::calculatePI(_plantModel, tc);
        break;
      case ControllerType::PID:
        currentParams = SkogestadTuner::calculatePID(_plantModel, tc);
        break;
      default:
        _state = TuningState::ERROR;
        return;
    }
    if (ParameterValidator::validate(currentParams)) {
      _candidates.push_back(currentParams);
      _candidateTc.push_back(tc);
    }
  }

  if (_candidates.empty()) {
    _state = TuningState::ERROR;
    return;
  }

  std::cout << "Fase 1 — 4 ganhos espaçados (τ_c em escala log entre "
            << kInitialTcMinFactor << "× e " << kInitialTcMaxFactor << "× da base SIMC):\n";
  for (size_t i = 0; i < _candidates.size(); ++i) {
    const auto& p = _candidates[i];
    std::cout << "  [" << (i + 1) << "] τ_c = " << _candidateTc[i] << ", Kp = " << p.Kp
              << ", Ti = " << p.Ti << ", Td = " << p.Td << "\n";
  }

  _closedLoopReference = _plantModel.K * kIdentificationInputStep;

  _controllerTestRound = ControllerTestRound::INITIAL;
  _round1Count = _candidates.size();
  _candidateScores.assign(_round1Count, -1e9);
  _currentCandidateIndex = 0;
  _testPhase = TestPhase::Prepare;
  _prepareSteps = 0;
  _bestScore = -1e9;
  _bestCandidateIndex = 0;
  _state = TuningState::TEST_CONTROLLER;
}

bool TuningTask::isBetterParams(const PIDParameters& current,
                                const PIDParameters& best) {
  return current.Kp < best.Kp;
}

bool TuningTask::isReady() const {
  return _isReady && _state == TuningState::COMPLETED;
}

bool TuningTask::isCompleted() const {
  return _state == TuningState::COMPLETED;
}

bool TuningTask::hasError() const {
  return _state == TuningState::ERROR;
}

PIDParameters TuningTask::getResult() {
  if (_state != TuningState::COMPLETED) {
    throw std::runtime_error("Tuning task não está completa");
  }
  return _parameters;
}
