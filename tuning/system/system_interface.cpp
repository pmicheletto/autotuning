#include "system_interface.h"

std::unique_ptr<ISystem> createSimulationSystem(const std::string& fmuPath);
std::unique_ptr<ISystem> createHardwareSystem();

std::unique_ptr<ISystem> createSystem(bool useSimulation,
                                      const std::string& fmuPath) {
  if (useSimulation) {
    return createSimulationSystem(fmuPath);
  } else {
    return createHardwareSystem();
  }
}
