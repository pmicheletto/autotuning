#include "loop_control.h"

namespace loop_control {

void closeLoop(ISystem& system, const std::string& manualVar) {
  system.setVariable(manualVar, 0.0);  // manual=false
}

void openLoop(ISystem& system, const std::string& manualVar) {
  system.setVariable(manualVar, 1.0);  // manual=true
}

void applyPID(ISystem& system, double Kp, double Ki, double Kd,
              const ControllerVarNames& names) {

  double k  = Kp;
  double Ti = (Ki > 0.0) ? Kp / Ki : 1e10;
  double Td = (Kp > 0.0) ? Kd / Kp : 0.0;

  system.setVariable(names.k, k);
  system.setVariable(names.Ti, Ti);
  system.setVariable(names.Td, Td);
}

void applyPIDWithTimeConstants(ISystem& system, double Kp, double Ti, double Td,
                               const ControllerVarNames& names) {
  const double Ki = (Ti > 1e-12) ? Kp / Ti : 0.0;
  const double Kd = Kp * Td;
  applyPID(system, Kp, Ki, Kd, names);
}

}  // namespace loop_control
