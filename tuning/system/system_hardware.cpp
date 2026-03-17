#include "system_interface.h"
#include <stdexcept>

// Implementação stub para hardware (vazia por enquanto)
class HardwareSystem : public ISystem {
 public:
  void advanceSimulation(double dt) override {
    // TODO: Implementar quando necessário (ex.: esperar dt segundos / próxima amostra)
    throw std::runtime_error("HardwareSystem não implementado ainda");
  }

  double readValue(const std::string& varName) override {
    // TODO: Implementar quando necessário
    throw std::runtime_error("HardwareSystem não implementado ainda");
  }

  void setVariable(const std::string& varName, double value) override {
    // TODO: Implementar quando necessário
    throw std::runtime_error("HardwareSystem não implementado ainda");
  }
};

std::unique_ptr<ISystem> createHardwareSystem() {
  return std::make_unique<HardwareSystem>();
}
