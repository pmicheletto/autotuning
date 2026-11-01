#include "tuning_task.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "loop_control.h"
#include "parameters/parameter_validator.h"
#include "parameters/skogestad_tuner.h"

namespace {
constexpr const char* kOutputVar = "out";

std::string formatPercentForLog(double value) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << value << '%';
  return oss.str();
}
}  // namespace

TuningTask::TuningTask(ControllerType controllerType, ISystem& system)
    : _controllerType(controllerType),
      _system(&system),
      _sundaresan(system) {}

void TuningTask::noteControllerTestJerkSample(std::optional<double> jerk) {
  if (_state != TuningState::TEST_CONTROLLER) {
    return;
  }
  if (_testPhase != TestPhase::WaitTouchRef) {
    return;
  }
  if (!jerk.has_value() || !std::isfinite(*jerk)) {
    return;
  }
  const double a = std::fabs(*jerk);
  if (std::isnan(_jerkAbsMaxThisCandidate)) {
    _jerkAbsMaxThisCandidate = a;
  } else {
    _jerkAbsMaxThisCandidate = std::max(_jerkAbsMaxThisCandidate, a);
  }
}

double TuningTask::debugUdpJerkMaxCurrentCandidate() const {
  return std::isfinite(_jerkAbsMaxThisCandidate)
             ? _jerkAbsMaxThisCandidate
             : std::numeric_limits<double>::quiet_NaN();
}

std::string TuningTask::debugTestPhaseName() const {
  if (_state != TuningState::TEST_CONTROLLER) {
    return "-";
  }
  switch (_testPhase) {
    case TestPhase::Prepare:
      return "Prepare";
    case TestPhase::WaitTransient:
      return "WaitTransient";
    case TestPhase::WaitTouchRef:
      return "WaitTouchRef";
    case TestPhase::Analyze:
      return "Analyze";
  }
  return "?";
}

TuningTask::DebugTransientState TuningTask::debugTransientState() const {
  DebugTransientState out;
  if (_state != TuningState::TEST_CONTROLLER || _testPhase != TestPhase::WaitTransient) {
    return out;
  }

  const double noiseFloor = std::max(_noise, 1e-12);
  out.acceptedDeviation = std::max(kZeroSpeedAbsTol, kZeroSpeedNoiseFactor * noiseFloor);
  out.refBand = std::max(kSettlingBandFrac * std::fabs(_closedLoopReference),
                        kSettlingBandNoiseMult * noiseFloor);
  out.refError = _system->readValue(kOutputVar) - _closedLoopReference;
  out.nearReference = std::fabs(out.refError) <= out.refBand;
  out.deviation = _testWelford.GetDeviation();
  out.stableStreak = _waitTransientStableStreak;
  return out;
}

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
    loop_control::openLoop(*_system);
    _system->setVariable("driver.step.height", 0.0);
    if (!_postIdentificationWaitLogged) {
      std::cout << "[Tuning] Pós-identificação: aguardando speed ~= 0 por "
                << kPostIdentificationZeroStableSteps
                << " amostras estáveis antes dos testes de ganhos.\n";
      _postIdentificationWaitLogged = true;
    }
    const double output = _system->readValue(kOutputVar);
    const double zeroTol =
        std::max(kPostIdentificationZeroAbsTol,
                 kPostIdentificationZeroNoiseFactor * _noise);
    if (std::fabs(output) <= zeroTol) {
      ++_postIdentificationZeroStreak;
    } else {
      _postIdentificationZeroStreak = 0;
    }
    ++_postIdentificationWaitCounter;
    if (_postIdentificationZeroStreak < kPostIdentificationZeroStableSteps) {
      return;
    }
    if (_postIdentificationWaitLogged) {
      std::cout << "[Tuning] Pós-identificação concluída (speed ~= 0 por "
                << _postIdentificationZeroStreak
                << " amostras, total aguardado: " << _postIdentificationWaitCounter
                << ") — iniciando testes de ganhos.\n";
      _postIdentificationWaitLogged = false;
    }
    tuneController();
    return;
  }

  if (_state == TuningState::TEST_CONTROLLER) {
    runTestController();
  }
}

void TuningTask::runTestController() {
  const double output = _system->readValue(kOutputVar);
  _testWelford.AddSample(output);

  switch (_testPhase) {
    case TestPhase::Prepare: {
      if (_prepareSteps == 0 && _resetSystemBeforePrepare) {
        _system->reset();
        _resetSystemBeforePrepare = false;
      }
      loop_control::openLoop(*_system);
      _system->setVariable("driver.step.height", 0.0);
      _prepareSteps++;
      if (_prepareSteps == 1u) {
        _prepareNearZeroStreak = 0;
      }
      const double prepareTol =
          std::max(kPrepareZeroSpeedAbsTol,
                   kPrepareZeroSpeedNoiseFactor * _noise);
      const bool atPrepareZero = std::fabs(output) <= prepareTol;
      if (atPrepareZero) {
        ++_prepareNearZeroStreak;
      } else {
        _prepareNearZeroStreak = 0;
      }
      const bool prepareSettled =
          atPrepareZero && _prepareNearZeroStreak >= kPrepareZeroStableSteps;
      if (!prepareSettled && _prepareSteps < kPrepareZeroSpeedMaxSteps) {
        return;
      }
      if (!prepareSettled) {
        std::cout << "Aviso: saida (speed) nao estabilizou perto de 0 antes do candidato "
                  << (_currentCandidateIndex + 1) << " (|out|=" << output
                  << ", tol=" << prepareTol << ", passos=" << _prepareSteps << ") — continua.\n";
      }
      const PIDParameters& params = _candidates[_currentCandidateIndex];
      double kpApply = params.Kp;
      double tiApply = params.Ti;
      double tdApply = params.Td;
      _closedLoopReference = kClosedLoopStepReference;
      loop_control::applyPIDWithTimeConstants(*_system, kpApply, tiApply, tdApply);
      loop_control::closeLoop(*_system);
      const double tcThis = (_currentCandidateIndex < _candidateTc.size())
                                ? _candidateTc[_currentCandidateIndex]
                                : std::numeric_limits<double>::quiet_NaN();
      std::cout << "[Tuning] Início candidato " << (_currentCandidateIndex + 1) << "/"
                << _candidates.size() << " (malha fechada): tau_c=" << tcThis
                << ", Kp=" << kpApply << ", Ti=" << tiApply << ", Td=" << tdApply
                << ", ref=" << _closedLoopReference << "\n";
      _testWelford.Clear();
      _prepareSteps = 0;
      _prepareNearZeroStreak = 0;
      _waitTouchRefSteps = 0;
      _waitTransientStepCount = 0;
      _waitTransientStableStreak = 0;
      _testPhase = TestPhase::WaitTransient;
      _maxOutput = 0.0;
      _hasOvershoot5 = false;
      _oscillationCount = 0;
      _lastSign = 0;
      _oscOutsideStreak = 0;
      _oscOutsideDir = 0;
      _oscOutsideLobeConfirmed = false;
      _oscHystOutside = 0;
      _analyzeSumAbsError = 0.0;
      _analyzeStepCount = 0;
      _settlingTimeS = std::numeric_limits<double>::quiet_NaN();
      _waitTransientStallLogged = false;
      _problematicoDebugPrinted = false;
      _jerkAbsMaxThisCandidate = std::numeric_limits<double>::quiet_NaN();
      break;
    }

    case TestPhase::WaitTransient: {
      _system->setVariable("driver.step.height", _closedLoopReference);
      if (_waitTransientStepCount == 0) {
        _waitTransientStartOutput = output;
        _waitTransientPrevOutput = output;
        std::cout << "[Tuning] WaitTransient inicio: out=" << output
                  << ", ref=" << _closedLoopReference << "\n";
      }
      _waitTransientStepCount++;
      const double noiseFloor = std::max(_noise, 1e-12);
      const double acceptedDeviation =
          std::max(kZeroSpeedAbsTol, kZeroSpeedNoiseFactor * noiseFloor);
      const double drift = std::fabs(output - _waitTransientStartOutput);
      const double stepJump = std::fabs(output - _waitTransientPrevOutput);
      const double refBand =
          std::max(kSettlingBandFrac * std::fabs(_closedLoopReference),
                   kSettlingBandNoiseMult * noiseFloor);
      const double refError = output - _closedLoopReference;
      const bool nearReference = std::fabs(refError) <= refBand;
      _waitTransientPrevOutput = output;

      if (nearReference) {
        _testWelford.AddSample(refError);
      } else {
        _testWelford.Clear();
      }

      const double deviation = _testWelford.GetDeviation();
      const bool byDeviation = nearReference && deviation <= acceptedDeviation;
      const bool byDrift = drift <= std::max(kWaitTransientDriftNoiseMult * noiseFloor,
                                            kWaitTransientDriftAbsMin);
      const bool byStepJump = stepJump <= kWaitTransientStepJumpNoiseMult * noiseFloor;

      const double nearZeroTol =
          std::max(kZeroSpeedAbsTol, kZeroSpeedNoiseFactor * _noise);
      const bool stillNearZero = std::fabs(output) <= nearZeroTol;

      if (byDeviation && byDrift && byStepJump) {
        ++_waitTransientStableStreak;
      } else {
        _waitTransientStableStreak = 0;
      }

      if (_waitTransientStableStreak >= kWaitTransientStableSamples) {
        std::cout << "[Tuning] WaitTransient -> WaitTouchRef (passos="
                  << _waitTransientStepCount << ", erro_ref=" << refError
                  << ", desvio=" << deviation << ", faixa_aceita="
                  << acceptedDeviation << ", amostras_estaveis="
                  << _waitTransientStableStreak << ")\n";
        _jerkAbsMaxThisCandidate = std::numeric_limits<double>::quiet_NaN();
        _testPhase = TestPhase::WaitTouchRef;
      } else if (_waitTransientStepCount >= kWaitTransientMaxSteps &&
                 !stillNearZero) {
        std::cout << "Aviso: WaitTransient timeout (" << kWaitTransientMaxSteps
                  << " passos) com saida fora de ~0 — segue para WaitTouchRef.\n";
        _jerkAbsMaxThisCandidate = std::numeric_limits<double>::quiet_NaN();
        _testPhase = TestPhase::WaitTouchRef;
      } else if (_waitTransientStepCount >= kWaitTransientMaxSteps &&
                 stillNearZero) {
        _waitTransientStepCount = 0;
        _waitTransientStableStreak = 0;
        if (!_waitTransientStallLogged) {
          std::cout << "Aviso: WaitTransient atingiu " << kWaitTransientMaxSteps
                    << " passos com saida ainda ~0 — continua em WaitTransient "
                       "até haver movimento.\n";
          _waitTransientStallLogged = true;
        }
      }
      break;
    }

    case TestPhase::WaitTouchRef: {
      _system->setVariable("driver.step.height", _closedLoopReference);
      _waitTouchRefSteps++;
      const double noiseFloorTouch = std::max(_noise, 1e-12);
      const double refTouchTol =
          std::max(kSettlingBandFrac * std::fabs(_closedLoopReference),
                   kSettlingBandNoiseMult * noiseFloorTouch);
      if (output >= _closedLoopReference - refTouchTol) {
        _maxOutput = output;
        _lastSign = 0;
        _oscOutsideStreak = 0;
        _oscOutsideDir = 0;
        _oscOutsideLobeConfirmed = false;
        _oscHystOutside = 0;
        _analyzeSumAbsError = 0.0;
        _analyzeStepCount = 0;
        _settlingTimeS =
            static_cast<double>(_waitTouchRefSteps) * _simulationStep;
        _waitTouchRefSteps = 0;
        _testPhase = TestPhase::Analyze;
      } else if (_waitTouchRefSteps >= kWaitTouchRefTimeoutSteps) {
        onCandidateTestFinished(-1e9, std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(), 0.0, 0,
                                std::numeric_limits<double>::quiet_NaN());
      }
      break;
    }

    case TestPhase::Analyze: {
      _system->setVariable("driver.step.height", _closedLoopReference);
      _maxOutput = std::max(_maxOutput, output);
      if (_maxOutput > _closedLoopReference * kOvershootThreshold) {
        _hasOvershoot5 = true;
      }
      updateOscillationCountAnalyze(output);
      _analyzeSumAbsError +=
          std::fabs(output - _closedLoopReference) * _simulationStep;
      _analyzeStepCount++;

      if (_analyzeStepCount >= kAnalyzeDurationSteps) {
        const double score = scoreCurrentCandidate();
        double overshootPct = 0.0;
        if (_closedLoopReference > 1e-12 && _maxOutput > _closedLoopReference) {
          overshootPct =
              100.0 * (_maxOutput - _closedLoopReference) / _closedLoopReference;
        }
        onCandidateTestFinished(score, _analyzeSumAbsError, _settlingTimeS,
                                overshootPct, _oscillationCount, _maxOutput);
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

unsigned int TuningTask::oscillationMinOutsideStreakRequired() const {
  unsigned int n = kOscillationMinOutsideSteps;
  if (kOscillationMinOutsideMs > 0.0) {
    const double stepMs = _simulationStep * 1000.0;
    if (stepMs > 1e-18) {
      const unsigned int fromT = static_cast<unsigned int>(
          std::ceil(kOscillationMinOutsideMs / stepMs));
      n = std::max(n, fromT);
    }
  }
  return std::max(n, 1u);
}

double TuningTask::oscillationErrorDeadband() const {
  const double noiseFloor = std::max(_noise, 1e-12);
  const double halfBand =
      std::max(kSettlingBandFrac * std::fabs(_closedLoopReference),
               kSettlingBandNoiseMult * noiseFloor);
  return std::max(halfBand, kOscillationDeadbandAbsFloor);
}

void TuningTask::updateOscillationCountAnalyze(double output) {
  const double err = output - _closedLoopReference;
  const double deadEnter = oscillationErrorDeadband();
  const double deadRelease =
      std::max(deadEnter * kOscillationDeadbandHysteresisRatio,
               kOscillationDeadbandAbsFloor);
  if (deadRelease >= deadEnter) {
    _oscHystOutside = (err > deadEnter) ? 1 : ((err < -deadEnter) ? -1 : 0);
  } else if (err > deadEnter) {
    _oscHystOutside = 1;
  } else if (err < -deadEnter) {
    _oscHystOutside = -1;
  } else if (std::fabs(err) < deadRelease) {
    _oscHystOutside = 0;
  }

  const unsigned int needStreak = oscillationMinOutsideStreakRequired();

  if (_oscHystOutside == 0) {
    _oscOutsideStreak = 0;
    _oscOutsideDir = 0;
    _oscOutsideLobeConfirmed = false;
    return;
  }

  if (_oscHystOutside == _oscOutsideDir) {
    ++_oscOutsideStreak;
  } else {
    _oscOutsideDir = _oscHystOutside;
    _oscOutsideStreak = 1;
    _oscOutsideLobeConfirmed = false;
  }

  if (!_oscOutsideLobeConfirmed && _oscOutsideStreak >= needStreak) {
    if (_lastSign != 0 && _lastSign != _oscOutsideDir) {
      ++_oscillationCount;
    }
    _lastSign = _oscOutsideDir;
    _oscOutsideLobeConfirmed = true;
  }
}

std::vector<double> TuningTask::buildInitialThreeTc() const {
  std::vector<double> out;
  out.reserve(3);
  for (int i = 0; i < 3; ++i) {
    out.push_back(kInitialTcProbeMinS +
                   static_cast<double>(i) * kInitialTcProbeStepS);
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

void TuningTask::appendLateralCandidatesFromBestOne() {
  const size_t n = _round1Count;
  if (n == 0) {
    return;
  }

  size_t bestIdx = 0;
  bool hasFiniteJerk = false;
  for (size_t i = 0; i < n; ++i) {
    if (i >= _candidateJerkAbsMax.size() || !std::isfinite(_candidateJerkAbsMax[i])) {
      continue;
    }
    if (!hasFiniteJerk || _candidateJerkAbsMax[i] < _candidateJerkAbsMax[bestIdx]) {
      bestIdx = i;
      hasFiniteJerk = true;
    }
  }
  if (!hasFiniteJerk) {
    bestIdx = std::distance(
        _candidateScores.begin(),
        std::max_element(_candidateScores.begin(), _candidateScores.begin() + n));
  }

  std::vector<double> proposed;
  const double tc_star = _candidateTc[bestIdx];
  proposed.push_back(std::max(1e-12, tc_star * (1.0 - kTcLateralFraction)));
  proposed.push_back(tc_star * (1.0 + kTcLateralFraction));

  size_t added = 0;
  for (double tc : proposed) {
    if (hasNearbyTc(tc)) {
      continue;
    }
    const PIDParameters currentParams =
        (_controllerType == ControllerType::PI)
            ? SkogestadTuner::calculatePI(_plantModel, tc)
            : SkogestadTuner::calculatePID(_plantModel, tc);
    if (ParameterValidator::validate(currentParams)) {
      _candidates.push_back(currentParams);
      _candidateTc.push_back(tc);
      ++added;
    }
  }
  _candidateScores.resize(_candidates.size(), -1e9);
  _candidateJerkAbsMax.resize(_candidates.size(),
                              std::numeric_limits<double>::quiet_NaN());
  std::cout << "Fase 2 — refinamento: " << added
            << " candidato(s) lateral(is) em τ_c (2 vizinhos do melhor da fase 1).\n";
  for (size_t i = _round1Count; i < _candidates.size(); ++i) {
    const auto& p = _candidates[i];
    std::cout << "  [" << (i + 1) << "] τ_c = " << _candidateTc[i] << ", Kp = " << p.Kp
              << ", Ti = " << p.Ti << ", Td = " << p.Td << "\n";
  }
}

void TuningTask::onCandidateTestFinished(double score, double iaeForThisCandidate,
                                         double settlingTimeS,
                                         double overshootPercent, int oscillations,
                                         double peakSpeedForOvershoot) {
  const size_t idx = _currentCandidateIndex;
  const double jerkAbsMaxThisCandidate = _jerkAbsMaxThisCandidate;
  if (!std::isnan(jerkAbsMaxThisCandidate)) {
    if (!_hasMinJerkMaxAmongCompletedTests) {
      _minJerkMaxAmongCompletedTests = jerkAbsMaxThisCandidate;
      _hasMinJerkMaxAmongCompletedTests = true;
    } else {
      _minJerkMaxAmongCompletedTests =
          std::min(_minJerkMaxAmongCompletedTests, jerkAbsMaxThisCandidate);
    }
  }
  _jerkAbsMaxThisCandidate = std::numeric_limits<double>::quiet_NaN();

  const double tcDone =
      (idx < _candidateTc.size()) ? _candidateTc[idx]
                                  : std::numeric_limits<double>::quiet_NaN();
  std::cout << "[Tuning] Teste candidato " << (idx + 1) << "/"
            << _candidates.size() << ": tau_c=" << tcDone
            << ", tempo_acomodacao_s=";
  if (std::isnan(settlingTimeS)) {
    std::cout << "n/a";
  } else {
    std::cout << settlingTimeS;
  }
  std::cout << ", ref=" << _closedLoopReference;
  if (!std::isnan(peakSpeedForOvershoot)) {
    std::cout << ", pico_speed=" << peakSpeedForOvershoot;
  } else {
    std::cout << ", pico_speed=n/a";
  }
  std::cout << ", sobressinal=" << formatPercentForLog(overshootPercent);
  std::cout << ", oscilacoes=" << oscillations;
  if (!std::isnan(iaeForThisCandidate)) {
    std::cout << ", IAE=" << iaeForThisCandidate;
  }
  if (std::isfinite(jerkAbsMaxThisCandidate)) {
    std::cout << ", jerk_abs_max_WaitTouchRef=" << jerkAbsMaxThisCandidate;
  } else {
    std::cout << ", jerk_abs_max_WaitTouchRef=n/a";
  }
  std::cout << ", score=" << score << "\n";

  if (idx < _candidateScores.size()) {
    _candidateScores[idx] = score;
  }
  if (idx < _candidateJerkAbsMax.size()) {
    _candidateJerkAbsMax[idx] = jerkAbsMaxThisCandidate;
  }

  const bool currentJerkValid = std::isfinite(jerkAbsMaxThisCandidate);
  const bool bestJerkValid = std::isfinite(_bestJerkAbsMax);
  bool replaceBest = false;
  if (currentJerkValid && !bestJerkValid) {
    replaceBest = true;
  } else if (currentJerkValid && bestJerkValid) {
    if (jerkAbsMaxThisCandidate < _bestJerkAbsMax) {
      replaceBest = true;
    } else if (std::fabs(jerkAbsMaxThisCandidate - _bestJerkAbsMax) <= 1e-12 &&
               score > _bestScore) {
      replaceBest = true;
    }
  } else if (!currentJerkValid && !bestJerkValid && score > _bestScore) {
    replaceBest = true;
  }

  if (replaceBest) {
    _bestScore = score;
    _bestJerkAbsMax = jerkAbsMaxThisCandidate;
    _bestCandidateIndex = idx;
    _parameters = _candidates[idx];
    if (!std::isnan(iaeForThisCandidate)) {
      _testScore = iaeForThisCandidate;
    }
    _bestSettlingTimeS = settlingTimeS;
    _bestOvershootPercent = overshootPercent;
    _bestOscillations = oscillations;
    _bestPeakSpeedForOvershoot = peakSpeedForOvershoot;
  }
  _currentCandidateIndex++;
  if (_controllerTestRound == ControllerTestRound::INITIAL && _round1Count > 0 &&
      _currentCandidateIndex == _round1Count) {
    appendLateralCandidatesFromBestOne();
    _controllerTestRound = ControllerTestRound::REFINEMENT;
  }
  if (_currentCandidateIndex < _candidates.size()) {
    _testPhase = TestPhase::Prepare;
    _prepareSteps = 0;
    _resetSystemBeforePrepare = true;
  } else {
    _parameters = _candidates[_bestCandidateIndex];
    _state = TuningState::COMPLETED;
    const double tcBest = _candidateTc[_bestCandidateIndex];
    std::cout << "[Tuning] Ganhos escolhidos: Kp=" << _parameters.Kp << ", Ti=" << _parameters.Ti
              << ", Td=" << _parameters.Td << ", tau_c=" << tcBest
              << " — tempo_acomodacao_s=";
    if (std::isnan(_bestSettlingTimeS)) {
      std::cout << "n/a";
    } else {
      std::cout << _bestSettlingTimeS;
    }
    std::cout << ", ref=" << _closedLoopReference;
    if (!std::isnan(_bestPeakSpeedForOvershoot)) {
      std::cout << ", pico_speed=" << _bestPeakSpeedForOvershoot;
    } else {
      std::cout << ", pico_speed=n/a";
    }
    std::cout << ", sobressinal=" << formatPercentForLog(_bestOvershootPercent);
    std::cout << ", oscilacoes=" << _bestOscillations;
    if (std::isfinite(_bestJerkAbsMax)) {
      std::cout << ", jerk_abs_max_WaitTouchRef=" << _bestJerkAbsMax;
    } else {
      std::cout << ", jerk_abs_max_WaitTouchRef=n/a";
    }
    std::cout << "\n";
  }
}

void TuningTask::tuneController() {
  _candidates.clear();
  _candidateTc.clear();

  for (double tc : buildInitialThreeTc()) {
    const PIDParameters currentParams =
        (_controllerType == ControllerType::PI)
            ? SkogestadTuner::calculatePI(_plantModel, tc)
            : SkogestadTuner::calculatePID(_plantModel, tc);
    if (ParameterValidator::validate(currentParams)) {
      _candidates.push_back(currentParams);
      _candidateTc.push_back(tc);
    }
  }

  if (_candidates.empty()) {
    _state = TuningState::ERROR;
    return;
  }

  const double tc_end = kInitialTcProbeMinS + 2.0 * kInitialTcProbeStepS;
  std::cout << "Fase 1 — 3 ganhos (τ_c de " << kInitialTcProbeMinS << " a " << tc_end
            << " s, passo " << kInitialTcProbeStepS << " s):\n";
  for (size_t i = 0; i < _candidates.size(); ++i) {
    const auto& p = _candidates[i];
    std::cout << "  [" << (i + 1) << "] τ_c = " << _candidateTc[i] << ", Kp = " << p.Kp
              << ", Ti = " << p.Ti << ", Td = " << p.Td << "\n";
  }

  _closedLoopReference = _plantModel.K * kIdentificationInputStep;

  _controllerTestRound = ControllerTestRound::INITIAL;
  _round1Count = _candidates.size();
  _candidateScores.assign(_round1Count, -1e9);
  _candidateJerkAbsMax.assign(_round1Count,
                              std::numeric_limits<double>::quiet_NaN());
  _currentCandidateIndex = 0;
  _testPhase = TestPhase::Prepare;
  _prepareSteps = 0;
  _resetSystemBeforePrepare = true;
  _bestScore = -1e9;
  _bestJerkAbsMax = std::numeric_limits<double>::quiet_NaN();
  _bestCandidateIndex = 0;
  _state = TuningState::TEST_CONTROLLER;
  _postIdentificationWaitCounter = 0;
  _postIdentificationZeroStreak = 0;
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
