#include "tuning_task.h"

#include <algorithm>
#include <cmath>
#include <iostream>
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
        _system->setVariable("step.height", 0.0);
        _prepareSteps++;
        if (_prepareSteps < kSettleStepsBetweenCandidates) {
          return;
        }
      }
      const PIDParameters& params = _candidates[_currentCandidateIndex];
      loop_control::applyPID(*_system, params.Kp, params.Ti, params.Td);
      loop_control::closeLoop(*_system);
      _system->setVariable("step.height", kReference50);
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
      if (output >= kReference50) {
        _touchedRef = true;
        _maxOutput = output;
        _lastSign = (output > kReference50 ? 1 : 0);
        _analyzeSumAbsError = 0.0;
        _analyzeStepCount = 0;
        _waitTouchRefSteps = 0;
        _testPhase = TestPhase::Analyze;
      } else if (_waitTouchRefSteps >= kWaitTouchRefTimeoutSteps) {
        _currentCandidateIndex++;
        if (_currentCandidateIndex < _candidates.size()) {
          _testPhase = TestPhase::Prepare;
          _prepareSteps = 0;
          _waitTouchRefSteps = 0;
        } else {
          _parameters = _candidates[_bestCandidateIndex];
          _state = TuningState::COMPLETED;
          _isReady = true;
        }
      }
      break;
    }

    case TestPhase::Analyze: {
      _maxOutput = std::max(_maxOutput, output);
      if (_maxOutput > kReference50 * kOvershootThreshold) {
        _hasOvershoot5 = true;
      }
      const double err = output - kReference50;
      const int sign = (err > 0 ? 1 : (err < 0 ? -1 : 0));
      if (_lastSign != 0 && sign != 0 && _lastSign != sign) {
        _oscillationCount++;
      }
      _lastSign = sign;
      _analyzeSumAbsError += std::fabs(err) * _simulationStep;
      _analyzeStepCount++;

      if (_analyzeStepCount >= kAnalyzeDurationSteps) {
        const double score = scoreCurrentCandidate();
        if (score > _bestScore) {
          _bestScore = score;
          _bestCandidateIndex = _currentCandidateIndex;
          _parameters = _candidates[_currentCandidateIndex];
          _testScore = _analyzeSumAbsError;
        }
        _currentCandidateIndex++;
        if (_currentCandidateIndex < _candidates.size()) {
          _testPhase = TestPhase::Prepare;
          _prepareSteps = 0;
        } else {
          _parameters = _candidates[_bestCandidateIndex];
          _state = TuningState::COMPLETED;
          _isReady = true;
        }
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

std::vector<double> TuningTask::buildTcCandidates() const {
  const double theta = _plantModel.theta;
  const double step = theta * 0.5;
  std::vector<double> out;
  out.push_back(theta);
  for (int n = 1; n <= 5; ++n) {
    double left = theta - n * step;
    if (left > 0.0) out.push_back(left);
    out.push_back(theta + n * step);
  }
  return out;
}

void TuningTask::tuneController() {
  if (_plantModel.theta <= 0.0) {
    _state = TuningState::ERROR;
    return;
  }

  std::vector<double> tcCandidates = buildTcCandidates();
  _candidates.clear();

  for (double tc : tcCandidates) {
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
    }
  }

  if (_candidates.empty()) {
    _state = TuningState::ERROR;
    return;
  }

  std::cout << "Ganhos a testar (" << _candidates.size() << " candidatos):\n";
  for (size_t i = 0; i < _candidates.size(); ++i) {
    const auto& p = _candidates[i];
    std::cout << "  [" << (i + 1) << "] Kp = " << p.Kp << ", Ti = " << p.Ti << ", Td = " << p.Td << "\n";
  }

  _currentCandidateIndex = 0;
  _testPhase = TestPhase::Prepare;
  _prepareSteps = 0;
  _bestScore = -1e9;
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
