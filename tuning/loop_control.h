#ifndef TUNING_LOOP_CONTROL_H_
#define TUNING_LOOP_CONTROL_H_

#include <string>

#include "system/system_interface.h"

namespace loop_control {

constexpr const char* kManualVar = "driver.useManualControl";

void closeLoop(ISystem& system,
               const std::string& manualVar = kManualVar);

void openLoop(ISystem& system,
              const std::string& manualVar = kManualVar);

struct ControllerVarNames {
  std::string k{"driver.pid.k"};
  std::string Ti{"driver.pid.Ti"};
  std::string Td{"driver.pid.Td"};
};

// Parallel form: Ki = Kp/Ti, Kd = Kp*Td; maps to Gain.k, I.k, D.k, D.T, P.k in OpenModelica.
void applyPID(ISystem& system, double Kp, double Ki, double Kd,
              const ControllerVarNames& names = ControllerVarNames{});

void applyPIDWithTimeConstants(ISystem& system, double Kp, double Ti, double Td,
                               const ControllerVarNames& names = ControllerVarNames{});

}  // namespace loop_control

#endif
