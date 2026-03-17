#include <stdexcept>

#include "simulation/ModelicaWrapper.h"
#include "system_interface.h"

// Implementação para simulação: setVariable escreve no modelo (ex.: step.height = degrau de alimentação);
// advanceSimulation faz a simulação correr no tempo (stepFMU).
class SimulationSystem : public ISystem {
 public:
  explicit SimulationSystem(const std::string& fmuPath)
      : _wrapper(std::make_unique<ModelicaWrapper>(fmuPath)), _simulationTime(0.0) {
    _wrapper->init(0.0);
  }

  void advanceSimulation(double dt) override {
    _wrapper->stepFMU(dt);
    _simulationTime += dt;
  }

  double readValue(const std::string& varName) override {
    return _wrapper->getReal(varName);
  }

  void setVariable(const std::string& varName, double value) override {
    _wrapper->setReal(varName, value);
  }

  // Retorna o tempo de simulação correspondente a \p simulation_steps com passo \p dt.
  static double getSimulationTimeForSteps(int simulation_steps, double dt) {
    return static_cast<double>(simulation_steps) * dt;
  }

  // Retorna o tempo de simulação atual (soma dos dt de advanceSimulation desde a criação).
  double getCurrentSimulationTime() const { return _simulationTime; }

 private:
  std::unique_ptr<ModelicaWrapper> _wrapper;
  double _simulationTime;
};

std::unique_ptr<ISystem> createSimulationSystem(const std::string& fmuPath) {
  if (fmuPath.empty()) {
    throw std::invalid_argument("Caminho do FMU necessário para simulação");
  }
  return std::make_unique<SimulationSystem>(fmuPath);
}
