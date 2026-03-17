#ifndef AUTOTUNING_MANAGER_H_
#define AUTOTUNING_MANAGER_H_

#include <memory>

#include "tuning/controllers/controller_interface.h"
#include "tuning/parameters/skogestad_tuner.h"
#include "tuning/system/system_interface.h"
#include "tuning/tuning_task.h"

class AutotuningManager {
 public:
  AutotuningManager();

  void requestTuning(Controller* controller, ISystem& system);
  bool isTuningReady() const;
  PIDParameters getTuningResult();

 private:
  std::unique_ptr<TuningTask> _tuningTask;
  Controller* _currentController;
};

#endif
