#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "simulation/SimulationVarsTxt.h"
#include "simulation/UDPSender.h"
#include "tuning/controllers/controller_type.h"
#include "tuning/system/system_interface.h"
#include "tuning/tuning_task.h"

int main(int argc, char** argv) {
  //std::string fmuPath = "../simulation/modelica_models/TransferFunction_control.fmu";
  std::string fmuPath = "../simulation/modelica_models/teste.fmu";

  if (argc >= 2) {
    fmuPath = argv[1];
  }

  try {
    auto system = createSystem(true, fmuPath);
    if (!system) {
      std::cerr << "Erro: nao foi possivel criar o sistema de simulacao.\n";
      return 1;
    }

    TuningTask task(ControllerType::PID, *system);
    task.setSimulationStep(0.001);

    UDPSender udp("127.0.0.1", 9870);
    const int udpInterval = 100;
    SimulationVarsTxt simTxt("sim_trace.txt", static_cast<unsigned>(udpInterval));
    const int maxSteps = 1000000;

    std::cout << "Tuning (identificacao Sundaresan + Skogestad PID). FMU: " << fmuPath << "\n";
    std::cout << "UDP: 127.0.0.1:9870 (step, speed). Max " << maxSteps << " passos.\n";

    for (int i = 0; i < maxSteps; ++i) {
      task.stepSimulation();
      task.update();

      const double t = (i + 1) * 0.001;
      const double speed = system->readValue("out");
      const double step = system->readValue("driver.step.y");

      simTxt.append(t, step, speed);

      if ((i + 1) % udpInterval == 0) {
        std::string json =
            "{\"time\": " + std::to_string(t) +
            ", \"step\": " + std::to_string(step) +
            ", \"speed\": " + std::to_string(speed) + "}";
        udp.send(json);
      }

      if (task.isCompleted() || task.hasError()) {
        std::cout << "Fim: " << (i + 1) << " passos.\n";
        break;
      }
      if ((i + 1) % 5000 == 0) {
        std::cout << "  " << (i + 1) << " / " << maxSteps << " passos (" << t << " s)\n";
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (task.isCompleted()) {
      auto params = task.getResult();
      std::cout << "Tuning concluido. Parametros PID:\n";
      std::cout << "  Kp = " << params.Kp << ", Ti = " << params.Ti << ", Td = " << params.Td << "\n";
      std::cout << "  IAE (teste malha fechada): " << task.getTestScore() << "\n";
      return 0;
    }

    if (task.hasError()) {
      std::cout << "Tuning terminou com erro (identificacao ou parametros invalidos).\n";
      return 1;
    }

    std::cout << "Timeout: max passos atingido.\n";
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "Erro: " << e.what() << "\n";
    return 1;
  }
}
