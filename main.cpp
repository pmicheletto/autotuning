#include <iostream>
#include <limits>
#include <optional>
#include <string>

#include "simulation/SimulationVarsTxt.h"
#include "simulation/UDPSender.h"
#include "statistics/JerkEstimator.h"
#include "tuning/controllers/controller_type.h"
#include "tuning/system/system_interface.h"
#include "tuning/tuning_task.h"

namespace {

// Default FMU when no argv[1] is provided (relative to run working directory).
constexpr const char* kDefaultFmuPath = "../simulation/modelica_models/variantC.fmu";
// Co-simulation / tuning inner step (s); must match UDP time grid below.
constexpr double kSimulationStepS = 0.001;
constexpr const char* kUdpHost = "127.0.0.1";
constexpr int kUdpPort = 9870;
// Emit UDP (and optionally CSV) once per this many simulation steps.
constexpr int kUdpEveryNSteps = 100;
constexpr unsigned kJerkSpeedMeanWindow = 20;
constexpr unsigned kJerkMeanWindow = 20;
// Minimum spacing between jerk estimator speed samples (s).
constexpr double kJerkMinSampleIntervalS = 0.05;
// Append CSV on every call (each call is already aligned with UDP sends).
constexpr unsigned kSimTraceWriteEverySamples = 1;
constexpr int kMaxSimulationSteps = 1'000'000;
constexpr int kProgressLogEveryNSteps = 5000;

}  // namespace

int main(int argc, char** argv) {
  std::string fmuPath = kDefaultFmuPath;
  if (argc >= 2) {
    fmuPath = argv[1];
  }

  try {
    auto system = createSystem(true, fmuPath);
    if (!system) {
      std::cerr << "Error: could not create simulation system.\n";
      return 1;
    }

    TuningTask task(ControllerType::PID, *system);
    task.setSimulationStep(kSimulationStepS);

    UDPSender udp(kUdpHost, kUdpPort);
    JerkEstimator jerk(kJerkSpeedMeanWindow, kJerkMinSampleIntervalS, kJerkMeanWindow);
    SimulationVarsTxt simTxt("sim_trace.txt", kSimTraceWriteEverySamples);
    bool was_prepare_phase = false;
    size_t last_prepare_candidate = std::numeric_limits<size_t>::max();

    std::cout << "Tuning (Sundaresan ID + Skogestad PID). FMU: " << fmuPath << "\n";
    std::cout << "UDP: " << kUdpHost << ":" << kUdpPort
              << " (time, step, speed, jerk, speed_filt, "
                 "jerk_max_current_gain_test, test_phase). Max "
              << kMaxSimulationSteps << " steps.\n";

    for (int i = 0; i < kMaxSimulationSteps; ++i) {
      task.stepSimulation();
      task.update();

      const std::string phase_now = task.debugTestPhaseName();
      const size_t candidate_now = task.debugUdpCandidateIndex();
      if (phase_now == "Prepare") {
        if (!was_prepare_phase || candidate_now != last_prepare_candidate) {
          jerk.Clear();
          last_prepare_candidate = candidate_now;
        }
        was_prepare_phase = true;
      } else {
        was_prepare_phase = false;
      }

      const double t = (i + 1) * task.getSimulationStep();
      const double speed = system->readValue("out");
      const double step = system->readValue("driver.step.y");

      if ((i + 1) % kUdpEveryNSteps == 0) {
        const std::string phase = phase_now;
        std::optional<double> jerk_sample = std::nullopt;
        double speed_filt = speed;
        if (phase != "Prepare") {
          jerk_sample = jerk.AddSpeed(speed, t);
          speed_filt = jerk.GetLastFilteredSpeed();
        }
        task.noteControllerTestJerkSample(jerk_sample);
        const double jerk_max_current_gain_test =
            task.debugUdpJerkMaxCurrentCandidate();
        simTxt.append(t, step, speed, jerk_sample, speed_filt,
                      jerk_max_current_gain_test,
                      phase);
        std::string json =
            "{\"time\": " + std::to_string(t) +
            ", \"step\": " + std::to_string(step) +
            ", \"speed\": " + std::to_string(speed) +
            ", \"jerk\": ";
        if (jerk_sample.has_value()) {
          json += std::to_string(*jerk_sample);
        } else {
          json += "null";
        }
        json += ", \"speed_filt\": " + std::to_string(speed_filt) +
            ", \"jerk_max_current_gain_test\": " +
            std::to_string(jerk_max_current_gain_test) +
            ", \"test_phase\": \"" + phase + "\"}";
        udp.send(json);
      }

      if (task.isCompleted() || task.hasError()) {
        std::cout << "Done: " << (i + 1) << " steps.\n";
        break;
      }
      if ((i + 1) % kProgressLogEveryNSteps == 0) {
        std::cout << "  " << (i + 1) << " / " << kMaxSimulationSteps << " steps (" << t
                  << " s)\n";
      }
    }

    if (task.isCompleted()) {
      auto params = task.getResult();
      std::cout << "Tuning finished. PID parameters:\n";
      std::cout << "  Kp = " << params.Kp << ", Ti = " << params.Ti << ", Td = " << params.Td << "\n";
      std::cout << "  IAE (closed-loop test): " << task.getTestScore() << "\n";
      return 0;
    }

    if (task.hasError()) {
      std::cout << "Tuning failed (invalid identification or parameters).\n";
      return 1;
    }

    std::cout << "Timeout: maximum steps reached.\n";
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
}
