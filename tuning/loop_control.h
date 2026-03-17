#ifndef TUNING_LOOP_CONTROL_H_
#define TUNING_LOOP_CONTROL_H_

#include <string>

#include "system/system_interface.h"

namespace loop_control {

constexpr const char* kManualVar = "manual";

void closeLoop(ISystem& system,
               const std::string& manualVar = kManualVar);

void openLoop(ISystem& system,
              const std::string& manualVar = kManualVar);

struct ControllerVarNames {
  std::string k{"pid.k"};
  std::string Ti{"pid.Ti"};
  std::string Td{"pid.Td"};
};

void applyPID(ISystem& system, double Kp, double Ki, double Kd,
              const ControllerVarNames& names = ControllerVarNames{});

}  // namespace loop_control

#endif
