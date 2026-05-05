#include "system_interface.h"

#include <stdexcept>

// Stub `ISystem` for future hardware I/O; every call throws until implemented.
class HardwareSystem : public ISystem {
 public:
  void reset() override {
    throw std::runtime_error("HardwareSystem: not implemented");
  }

  void advanceSimulation(double) override {
    throw std::runtime_error("HardwareSystem: not implemented");
  }

  double readValue(const std::string&) override {
    throw std::runtime_error("HardwareSystem: not implemented");
  }

  void setVariable(const std::string&, double) override {
    throw std::runtime_error("HardwareSystem: not implemented");
  }
};

std::unique_ptr<ISystem> createHardwareSystem() {
  return std::make_unique<HardwareSystem>();
}
