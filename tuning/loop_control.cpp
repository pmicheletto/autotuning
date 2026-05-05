#include "loop_control.h"

#include <algorithm>
#include <exception>

namespace loop_control {

namespace {

void syncOpenModelicaDerivedPidParameters(ISystem& system, double k, double Ti, double Td) {
  constexpr double kNdFallback = 10.0;
  constexpr double kDtFloor = 2.220446049250313e-14;
  double nd = kNdFallback;
  try {
    const double v = system.readValue("driver.pid.Nd");
    if (v > 1e-300) {
      nd = v;
    }
  } catch (const std::exception&) {
  }

  try {
    system.setVariable("driver.pid.Gain.k", k);
    const double i_k = (Ti > 1e-12) ? (1.0 / Ti) : 0.0;
    system.setVariable("driver.pid.I.k", i_k);
    system.setVariable("driver.pid.D.k", Td);
    system.setVariable("driver.pid.D.T", std::max(Td / nd, kDtFloor));
    system.setVariable("driver.pid.P.k", 1.0);
  } catch (const std::exception&) {
  }
}

}  // namespace

void closeLoop(ISystem& system, const std::string& manualVar) {
  system.setVariable(manualVar, 0.0);
}

void openLoop(ISystem& system, const std::string& manualVar) {
  system.setVariable(manualVar, 1.0);
}

void applyPID(ISystem& system, double Kp, double Ki, double Kd,
              const ControllerVarNames& names) {

  double k  = Kp;
  double Ti = (Ki > 0.0) ? Kp / Ki : 1e10;
  double Td = (Kp > 0.0) ? Kd / Kp : 0.0;

  system.setVariable(names.k, k);
  system.setVariable(names.Ti, Ti);
  system.setVariable(names.Td, Td);

  syncOpenModelicaDerivedPidParameters(system, k, Ti, Td);
}

void applyPIDWithTimeConstants(ISystem& system, double Kp, double Ti, double Td,
                               const ControllerVarNames& names) {
  const double Ki = (Ti > 1e-12) ? Kp / Ti : 0.0;
  const double Kd = Kp * Td;
  applyPID(system, Kp, Ki, Kd, names);
}

}  // namespace loop_control
