#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "simulation/UDPSender.h"
#include "tuning/system/system_interface.h"
#include "tuning/transfer_function/sundaresan.h"

namespace {
// Estado para JSON (PlotJuggler): 0=FindingMovement, 1=FindingDeadBand, 2=CalculateTransferFunction, 3=Done, 4=UnresponsivePlant
int stateToInt(SundaresanState s) {
  return static_cast<int>(s);
}
}  // namespace

int main(int argc, char** argv) {
  std::string fmuPath = "../simulation/modelica_models/TransferFunction_control.fmu";
  if (argc >= 2) {
    fmuPath = argv[1];
  }

  try {
    auto system = createSystem(true, fmuPath);
    if (!system) {
      std::cerr << "Erro: nao foi possivel criar o sistema de simulacao.\n";
      return 1;
    }

    Sundaresan alg(*system);

    UDPSender udp("127.0.0.1", 9870);
    const int udpInterval = 100;

    const int maxSteps = 25000;
    const double simulationStep = 0.001;
    std::cout << "A procurar movimento e depois dead band (FMU: " << fmuPath
              << "). UDP: 127.0.0.1:9870 a cada " << udpInterval
              << " passos. " << maxSteps << " passos (" << (maxSteps * simulationStep) << " s sim).\n";

    for (int i = 0; i < maxSteps; ++i) {
      system->advanceSimulation(simulationStep);
      alg.Execute();
      if (alg.IsFinished()) {
        std::cout << "Fim: " << (i + 1) << " passos concluidos.\n";
        break;
      }
      double t = (i + 1) * simulationStep;
      double speed = system->readValue("out");
      double stepY = system->readValue("step.y");
      bool manual_control = system->readValue("manual");

      if ((i + 1) % udpInterval == 0) {
        int state = stateToInt(alg.GetState());
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
      if ((i + 1) % 5000 == 0) {
        std::cout << "  " << (i + 1) << " / " << maxSteps << " passos (" << t << " s)\n";
      }
      // Sem este sleep, os 25000 passos correm em segundos e o UDP chega tudo de uma vez;
      // o PlotJuggler (e a sensação de "a correr") precisam de tempo real ~1 ms por passo.
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const SundaresanState state = alg.GetState();

    if (state == SundaresanState::Done) {
      std::cout << "Dead band encontrada.\n";
      std::cout << "  Ruido (max desvio truncado): " << alg.GetNoise() << "\n";
      std::cout << "  Valor final (speed): " << alg.GetFinalValue() << "\n";
      const PlantModel& pm = alg.GetPlantModel();
      std::cout << "Funcao de transferencia (Sundaresan-Krishnaswamy):\n";
      std::cout << "  K (ganho): " << pm.K << "\n";
      std::cout << "  theta (dead time): " << pm.theta << "\n";
      std::cout << "  tau1: " << pm.tau1 << "\n";
      std::cout << "  tau2: " << pm.tau2 << "\n";
      return 0;
    }

    if (state == SundaresanState::UnresponsivePlant) {
      std::cout << "Planta nao respondeu (simulationStep > 0.5).\n";
      return 1;
    }

    std::cout << "Estado final: " << static_cast<int>(state) << "\n";
    return 0;

  } catch (const std::exception& e) {
    std::cerr << "Erro: " << e.what() << "\n";
    return 1;
  }
}
