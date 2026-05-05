#include <stdexcept>

#include "simulation/ModelicaWrapper.h"
#include "system_interface.h"

namespace {

// Initial simulation time after FMU init/reset (s).
constexpr double kInitialSimulationTimeS = 0.0;

}  // namespace

class SimulationSystem : public ISystem {
 public:
  explicit SimulationSystem(const std::string& fmuPath)
      : _fmuPath(fmuPath),
        _wrapper(std::make_unique<ModelicaWrapper>(fmuPath)),
        _simulationTime(kInitialSimulationTimeS) {
    _wrapper->init(kInitialSimulationTimeS);
  }

  void reset() override {
    _wrapper = std::make_unique<ModelicaWrapper>(_fmuPath);
    _wrapper->init(kInitialSimulationTimeS);
    _simulationTime = kInitialSimulationTimeS;
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

  static double getSimulationTimeForSteps(int simulation_steps, double dt) {
    return static_cast<double>(simulation_steps) * dt;
  }

  double getCurrentSimulationTime() const { return _simulationTime; }

 private:
  std::string _fmuPath;
  std::unique_ptr<ModelicaWrapper> _wrapper;
  double _simulationTime;
};

std::unique_ptr<ISystem> createSimulationSystem(const std::string& fmuPath) {
  if (fmuPath.empty()) {
    throw std::invalid_argument("FMU path is required for simulation");
  }
  return std::make_unique<SimulationSystem>(fmuPath);
}
