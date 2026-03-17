#include "autotuning_manager.h"

#include <stdexcept>

#include "tuning/controllers/controller_interface.h"
#include "tuning/tuning_task.h"

AutotuningManager::AutotuningManager() : _currentController(nullptr) {}

void AutotuningManager::requestTuning(Controller* controller, ISystem& system) {
  if (controller == nullptr) {
    throw std::invalid_argument("Controller não pode ser nulo");
  }

  _currentController = controller;
  ControllerType controllerType = controller->getType();

  _tuningTask = std::make_unique<TuningTask>(controllerType, system);
}

bool AutotuningManager::isTuningReady() const {
  if (_tuningTask == nullptr) {
    return false;
  }
  return _tuningTask->isReady();
}

PIDParameters AutotuningManager::getTuningResult() {
  if (_tuningTask == nullptr) {
    throw std::runtime_error("Nenhuma tarefa de tuning foi iniciada");
  }
  return _tuningTask->getResult();
}
