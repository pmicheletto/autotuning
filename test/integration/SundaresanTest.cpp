#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "simulation/UDPSender.h"
#include "tuning/system/system_interface.h"
#include "tuning/transfer_function/sundaresan.h"

namespace {

constexpr int kMaxSteps = 25000;
constexpr double kSimulationStepS = 0.001;
constexpr const char* kUdpHost = "127.0.0.1";
constexpr int kUdpPort = 9870;
constexpr int kUdpEveryNSteps = 100;
constexpr int kProgressLogEveryNSteps = 5000;
// Throttle loop so UDP is ~1 ms per FMU step for PlotJuggler.
constexpr int kWallClockSleepMsPerStep = 1;

}  // namespace

int main(int argc, char** argv) {
  std::string fmuPath = "../simulation/modelica_models/TransferFunction_control.fmu";
  if (argc >= 2) {
    fmuPath = argv[1];
  }

  try {
    auto system = createSystem(true, fmuPath);
    if (!system) {
      std::cerr << "Error: could not create simulation system.\n";
      return 1;
    }

    Sundaresan alg(*system);

    UDPSender udp(kUdpHost, kUdpPort);
    std::cout << "Finding movement then dead band (FMU: " << fmuPath << "). UDP: " << kUdpHost
              << ":" << kUdpPort << " every " << kUdpEveryNSteps << " steps. " << kMaxSteps
              << " steps (" << (kMaxSteps * kSimulationStepS) << " s sim).\n";

    for (int i = 0; i < kMaxSteps; ++i) {
      system->advanceSimulation(kSimulationStepS);
      alg.Execute();
      if (alg.IsFinished()) {
        std::cout << "Done: " << (i + 1) << " steps.\n";
        break;
      }
      double t = (i + 1) * kSimulationStepS;
      double speed = system->readValue("out");
      double stepY = system->readValue("step.y");
      bool manual_control = system->readValue("manual");

      if ((i + 1) % kUdpEveryNSteps == 0) {
        double noise = alg.GetNoise();
        double deviation = alg.GetDeviation();
        double filteredSpeed = alg.GetFilteredSpeed();
        std::string json =
            "{\"time\": " + std::to_string(t) +
            ", \"rpm\": " + std::to_string(speed) +
            ", \"speed_filtrado\": " + std::to_string(filteredSpeed) +
            ", \"step.y\": " + std::to_string(stepY) +
            ", \"noise\": " + std::to_string(noise) +
            ", \"manual_control\": " + std::to_string(manual_control) +
            ", \"desvio_padrao\": " + std::to_string(deviation) + "}";
        udp.send(json);
      }
      if ((i + 1) % kProgressLogEveryNSteps == 0) {
        std::cout << "  " << (i + 1) << " / " << kMaxSteps << " steps (" << t << " s)\n";
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(kWallClockSleepMsPerStep));
    }

    const SundaresanState state = alg.GetState();

    if (state == SundaresanState::Done) {
      std::cout << "Dead band found.\n";
      std::cout << "  Noise (max truncated deviation): " << alg.GetNoise() << "\n";
      std::cout << "  Final value (speed): " << alg.GetFinalValue() << "\n";
      const PlantModel& pm = alg.GetPlantModel();
      std::cout << "Transfer function (Sundaresan-Krishnaswamy):\n";
      std::cout << "  K (gain): " << pm.K << "\n";
      std::cout << "  theta (dead time): " << pm.theta << "\n";
      std::cout << "  tau1: " << pm.tau1 << "\n";
      std::cout << "  tau2: " << pm.tau2 << "\n";
      return 0;
    }

    if (state == SundaresanState::UnresponsivePlant) {
      std::cout << "Plant did not respond (step command exceeded limit).\n";
      return 1;
    }

    std::cout << "Final state: " << static_cast<int>(state) << "\n";
    return 0;

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
}
